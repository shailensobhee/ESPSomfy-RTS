#include "HCSR04.h"
#include "ConfigSettings.h"
#include "MQTT.h"
#include "Sockets.h"
#include "Somfy.h"
#include "esp_log.h"
#include <esp_rom_sys.h>

static const char *TAG = "HCSR04";

extern ConfigSettings settings;
extern MQTTClass mqtt;
extern SocketEmitter sockEmit;
extern SomfyShadeController somfy;

// --- ISR state (must be in IRAM, accessed from interrupt context) ---
static volatile uint64_t s_echoRiseUs = 0;
static volatile uint64_t s_echoFallUs = 0;
static volatile bool     s_echoReady  = false;
static volatile gpio_num_t s_echoGpio  = GPIO_NUM_NC;

static void IRAM_ATTR echoISR(void *) {
    if (gpio_get_level((gpio_num_t)s_echoGpio)) {
        s_echoRiseUs = (uint64_t)esp_timer_get_time();
    } else {
        s_echoFallUs = (uint64_t)esp_timer_get_time();
        s_echoReady  = true;
    }
}

// --- Timer callback: fire TRIG pulse every intervalSec seconds ---
static void triggerCallback(void *) {
    if (s_echoReady == false && s_echoRiseUs != 0) return; // previous echo still pending
    s_echoReady  = false;
    s_echoRiseUs = 0;
    gpio_set_level((gpio_num_t)settings.HCSR04.trigPin, 1);
    esp_rom_delay_us(10);
    gpio_set_level((gpio_num_t)settings.HCSR04.trigPin, 0);
}

bool HCSR04Class::begin() {
    if (!settings.HCSR04.enabled ||
        settings.HCSR04.trigPin == 255 ||
        settings.HCSR04.echoPin == 255) {
        ESP_LOGI(TAG, "HC-SR04 disabled or pins not configured — skipping");
        return false;
    }
    if (settings.HCSR04.trigPin == settings.HCSR04.echoPin) {
        ESP_LOGE(TAG, "HC-SR04 TRIG and ECHO cannot be the same pin (%d)",
                 settings.HCSR04.trigPin);
        return false;
    }
    if (somfy.transceiver.usesPin(settings.HCSR04.trigPin) ||
        somfy.transceiver.usesPin(settings.HCSR04.echoPin)) {
        ESP_LOGE(TAG, "HC-SR04 pins conflict with CC1101 (TRIG=%d ECHO=%d)",
                 settings.HCSR04.trigPin, settings.HCSR04.echoPin);
        return false;
    }
    // Reset any stale ISR state from a previous measurement cycle
    s_echoReady  = false;
    s_echoRiseUs = 0;
    s_echoFallUs = 0;

    s_echoGpio = (gpio_num_t)settings.HCSR04.echoPin;

    // Configure TRIG as output
    gpio_config_t trig_conf = {};
    trig_conf.pin_bit_mask = (1ULL << settings.HCSR04.trigPin);
    trig_conf.mode         = GPIO_MODE_OUTPUT;
    trig_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
    trig_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    trig_conf.intr_type    = GPIO_INTR_DISABLE;
    gpio_config(&trig_conf);
    gpio_set_level((gpio_num_t)settings.HCSR04.trigPin, 0);

    // Configure ECHO as input with both-edge interrupt
    gpio_config_t echo_conf = {};
    echo_conf.pin_bit_mask = (1ULL << settings.HCSR04.echoPin);
    echo_conf.mode         = GPIO_MODE_INPUT;
    echo_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
    echo_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    echo_conf.intr_type    = GPIO_INTR_ANYEDGE;
    gpio_config(&echo_conf);
    esp_err_t isr_err = gpio_install_isr_service(0);
    if (isr_err != ESP_OK && isr_err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to install ISR service: %d", isr_err);
        return false;
    }
    gpio_isr_handler_add((gpio_num_t)s_echoGpio, echoISR, nullptr);

    // Create periodic timer
    uint64_t intervalUs = (uint64_t)settings.HCSR04.intervalSec * 1000000ULL;
    esp_timer_create_args_t args = {};
    args.callback = triggerCallback;
    args.name     = "hcsr04_trig";
    if (esp_timer_create(&args, &_timer) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create HC-SR04 timer");
        gpio_isr_handler_remove((gpio_num_t)s_echoGpio);
        return false;
    }
    if (esp_timer_start_periodic(_timer, intervalUs) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HC-SR04 timer");
        esp_timer_delete(_timer);
        _timer = nullptr;
        gpio_isr_handler_remove((gpio_num_t)s_echoGpio);
        return false;
    }

    _active = true;
    ESP_LOGI(TAG, "HC-SR04 started: TRIG=%d ECHO=%d interval=%ds",
             settings.HCSR04.trigPin, settings.HCSR04.echoPin, settings.HCSR04.intervalSec);
    return true;
}

void HCSR04Class::end() {
    if (!_active) return;
    if (_timer) {
        esp_timer_stop(_timer);
        esp_timer_delete(_timer);
        _timer = nullptr;
    }
    gpio_isr_handler_remove((gpio_num_t)s_echoGpio);
    _active = false;
    ESP_LOGI(TAG, "HC-SR04 stopped");
}

void HCSR04Class::loop() {
    if (!_active) return;

    portDISABLE_INTERRUPTS();
    if (!s_echoReady) {
        portENABLE_INTERRUPTS();
        return;
    }
    uint64_t rise = s_echoRiseUs;
    uint64_t fall = s_echoFallUs;
    s_echoReady  = false;
    s_echoRiseUs = 0;  // clear so next triggerCallback isn't blocked by the "pending echo" guard
    s_echoFallUs = 0;
    portENABLE_INTERRUPTS();

    uint64_t durationUs = fall - rise;
    // Sanity check: HC-SR04 max range ~4m = ~23ms; min ~2cm = ~116µs
    if (durationUs < 116 || durationUs > 23200) {
        ESP_LOGI(TAG, "HC-SR04 out-of-range pulse: %lluµs", durationUs);
        return;
    }
    float cm = (float)durationUs * 0.01715f; // µs × (0.0343 cm/µs ÷ 2)
    lastDistanceCm = cm;

    ESP_LOGI(TAG, "Distance: %.1f cm", cm);

    // Publish via MQTT
    if (mqtt.connected()) {
        char payload[12];
        snprintf(payload, sizeof(payload), "%.1f", cm);
        mqtt.publish("sensors/distance", payload, false);
    }

    // Emit to WebSocket clients
    JsonSockEvent *json = sockEmit.beginEmit("distance");
    json->beginObject();
    json->addElem("distanceCm", cm);
    json->endObject();
    sockEmit.endEmit();
}

bool HCSR04Class::usesPin(uint8_t pin) {
    if (!settings.HCSR04.enabled) return false;
    return (pin == settings.HCSR04.trigPin || pin == settings.HCSR04.echoPin);
}

void HCSR04Class::publishDisco() {
    if (!mqtt.connected() || !settings.MQTT.pubDisco || !settings.HCSR04.enabled) return;

    char topic[128];
    snprintf(topic, sizeof(topic), "%s/sensor/espsomfy_%s_distance/config",
             settings.MQTT.discoTopic, settings.serverId);

    JsonDocument doc;
    JsonObject obj = doc.to<JsonObject>();

    // Device block — links to same device as the shades
    JsonObject dobj = obj["device"].to<JsonObject>();
    dobj["hw_version"] = settings.fwVersion.name;
    dobj["name"]       = settings.hostname;
    dobj["mf"]         = "rstrouse";
    dobj["model"]      = "ESPSomfy-RTS MQTT";
    char devId[48]; // must outlive JsonObject serialisation — same stack frame
    snprintf(devId, sizeof(devId), "mqtt_espsomfyrts_%s", settings.serverId);
    JsonArray ids = dobj["identifiers"].to<JsonArray>();
    ids.add(devId);
    dobj["via_device"] = devId;

    // Sensor entity
    char uniqueId[48];
    snprintf(uniqueId, sizeof(uniqueId), "espsomfy_%s_distance", settings.serverId);
    obj["unique_id"]           = uniqueId;
    obj["name"]                = "Distance";
    obj["device_class"]        = "distance";
    obj["unit_of_measurement"] = "cm";

    char stateTopic[128];
    snprintf(stateTopic, sizeof(stateTopic), "%s/sensors/distance",
             settings.MQTT.rootTopic);
    obj["state_topic"] = stateTopic;

    char availTopic[128];
    snprintf(availTopic, sizeof(availTopic), "%s/status", settings.MQTT.rootTopic);
    obj["availability_topic"]    = availTopic;
    obj["payload_available"]     = "online";
    obj["payload_not_available"] = "offline";

    if (doc.overflowed()) {
        ESP_LOGE(TAG, "HC-SR04 discovery JSON overflowed — not publishing");
        return;
    }
    mqtt.publishDisco(topic, obj, true);
    ESP_LOGI(TAG, "Published HC-SR04 HA discovery");
}

void HCSR04Class::unpublishDisco() {
    if (!mqtt.connected() || !settings.MQTT.pubDisco) return;
    char topic[128];
    snprintf(topic, sizeof(topic), "%s/sensor/espsomfy_%s_distance/config",
             settings.MQTT.discoTopic, settings.serverId);
    mqtt.publishBuffer(topic, (uint8_t *)"", 0, true);
}

HCSR04Class hcsr04;
