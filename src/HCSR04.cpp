#include "HCSR04.h"
#include "ConfigSettings.h"
#include "MQTT.h"
#include "Sockets.h"
#include "esp_log.h"
#include <esp_rom_sys.h>

static const char *TAG = "HCSR04";

extern ConfigSettings settings;
extern MQTTClass mqtt;
extern SocketEmitter sockEmit;

// --- ISR state (must be in IRAM, accessed from interrupt context) ---
static volatile uint64_t s_echoRiseUs = 0;
static volatile uint64_t s_echoFallUs = 0;
static volatile bool     s_echoReady  = false;
static gpio_num_t        s_echoGpio   = GPIO_NUM_NC;

static void IRAM_ATTR echoISR(void *) {
    if (gpio_get_level(s_echoGpio)) {
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
    gpio_install_isr_service(0);  // safe to call multiple times — returns ESP_ERR_INVALID_STATE if already installed
    gpio_isr_handler_add(s_echoGpio, echoISR, nullptr);

    // Create periodic timer
    uint64_t intervalUs = (uint64_t)settings.HCSR04.intervalSec * 1000000ULL;
    esp_timer_create_args_t args = {};
    args.callback = triggerCallback;
    args.name     = "hcsr04_trig";
    esp_timer_create(&args, &_timer);
    esp_timer_start_periodic(_timer, intervalUs);

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
    gpio_isr_handler_remove(s_echoGpio);
    _active = false;
    ESP_LOGI(TAG, "HC-SR04 stopped");
}

void HCSR04Class::loop() {
    if (!_active || !s_echoReady) return;
    s_echoReady = false;

    uint64_t durationUs = s_echoFallUs - s_echoRiseUs;
    // Sanity check: HC-SR04 max range ~4m = ~23ms; min ~2cm = ~116µs
    if (durationUs < 116 || durationUs > 23200) {
        ESP_LOGD(TAG, "HC-SR04 out-of-range pulse: %lluµs", durationUs);
        return;
    }
    float cm = (float)durationUs * 0.01715f; // µs × (0.0343 cm/µs ÷ 2)
    lastDistanceCm = cm;

    ESP_LOGD(TAG, "Distance: %.1f cm", cm);

    // Publish via MQTT
    if (mqtt.connected()) {
        char payload[12];
        snprintf(payload, sizeof(payload), "%.1f", cm);
        mqtt.publish("sensors/distance", payload, false);
    }

    // Emit to WebSocket clients
    JsonSockEvent *json = sockEmit.beginEmit("distance");
    json->addElem("distanceCm", cm);
    sockEmit.endEmit();
}

bool HCSR04Class::usesPin(uint8_t pin) {
    if (!settings.HCSR04.enabled) return false;
    return (pin == settings.HCSR04.trigPin || pin == settings.HCSR04.echoPin);
}
