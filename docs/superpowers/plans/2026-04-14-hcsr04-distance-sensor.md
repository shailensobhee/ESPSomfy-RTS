# HC-SR04 Distance Sensor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add optional HC-SR04 ultrasonic distance sensor support — hardware-interrupt driven so it cannot interfere with CC1101 RF reception — with MQTT Home Assistant discovery and a web config UI.

**Architecture:** A new `HCSR04Settings` class (mirroring `MQTTSettings`) stores config in NVS Preferences and is a member of `ConfigSettings`. A new `HCSR04` module uses an `esp_timer` callback to fire the TRIG pulse and a `IRAM_ATTR` GPIO ISR to timestamp the ECHO edges — no `pulseIn()`, no blocking. The `loop()` checks a volatile flag in the main Arduino loop and publishes via existing MQTT infrastructure. The feature is passive (skips `begin()` silently) when disabled or pins unset.

**Tech Stack:** ESP-IDF `esp_timer`, `esp_rom_delay_us`, GPIO interrupts, ArduinoJson 7, PubSubClient (existing MQTT), LittleFS/NVS Preferences (existing config), ESPAsyncWebServer (existing Web layer), vanilla JS (existing UI pattern).

---

## MQTT Topic Contract (important for ESPSomfy-RTS-HA adapter later)

| Topic | Direction | Format |
|---|---|---|
| `{rootTopic}/sensors/distance` | publish | `"123.4"` (cm, 1 decimal place, float as string) |
| `{discoTopic}/sensor/espsomfy_{serverId}_distance/config` | publish (retain) | HA MQTT discovery JSON (see Task 3) |

The HA project at https://github.com/shailensobhee/ESPSomfy-RTS-HA will need to be updated to subscribe to `sensors/distance` and expose it as a sensor entity. That is out of scope for this plan but the topic contract above is fixed.

---

## File Map

| File | Action | Responsibility |
|---|---|---|
| `src/HCSR04.h` | **Create** | Class declaration, volatile ISR state |
| `src/HCSR04.cpp` | **Create** | Timer, ISR, distance calc, MQTT publish, HA discovery |
| `src/ConfigSettings.h` | **Modify** | Add `HCSR04Settings` class + `HCSR04` member to `ConfigSettings` |
| `src/ConfigSettings.cpp` | **Modify** | Implement `HCSR04Settings` load/save/fromJSON/toJSON/begin; wire `begin()` |
| `src/MQTT.h` | **Modify** | Add `publish(const char*, float, bool)` overload declaration |
| `src/MQTT.cpp` | **Modify** | Implement float publish |
| `src/Web.cpp` | **Modify** | Add `/hcsr04settings` GET + `/connecthcsr04` POST routes |
| `src/SomfyController.ino` | **Modify** | Include, declare global, `begin()`, `loop()` |
| `data-src/index.html` | **Modify** | Add "Sensors" top-level tab + HC-SR04 config form |
| `data-src/index.js` | **Modify** | Add `hcsr04` JS object with `init()` / `loadSettings()` / `saveSettings()` |

---

## Task 1: `HCSR04Settings` — config class

**Files:**
- Modify: `src/ConfigSettings.h`
- Modify: `src/ConfigSettings.cpp`

- [ ] **Step 1: Add `HCSR04Settings` class to ConfigSettings.h**

Add this block immediately before the `ConfigSettings` class (after `MQTTSettings`):

```cpp
class HCSR04Settings: BaseSettings {
  public:
    bool enabled = false;
    uint8_t trigPin = 255;   // 255 = not configured
    uint8_t echoPin = 255;
    uint8_t intervalSec = 5; // publish interval in seconds
    bool begin();
    bool save();
    bool load();
    bool toJSON(JsonObject &obj);
    bool fromJSON(JsonObject &obj);
};
```

Add `HCSR04Settings HCSR04;` as a member of `ConfigSettings` (after `MQTTSettings MQTT;`):

```cpp
    MQTTSettings MQTT;
    HCSR04Settings HCSR04;  // <-- add this line
    SecuritySettings Security;
```

- [ ] **Step 2: Implement `HCSR04Settings` methods in ConfigSettings.cpp**

Add this block at the end of ConfigSettings.cpp (before the final blank line):

```cpp
bool HCSR04Settings::begin() {
  this->load();
  return true;
}
bool HCSR04Settings::save() {
  pref.begin("HCSR04");
  pref.putBool("enabled", this->enabled);
  pref.putUChar("trigPin", this->trigPin);
  pref.putUChar("echoPin", this->echoPin);
  pref.putUChar("intervalSec", this->intervalSec);
  pref.end();
  return true;
}
bool HCSR04Settings::load() {
  pref.begin("HCSR04");
  this->enabled = pref.getBool("enabled", false);
  this->trigPin = pref.getUChar("trigPin", 255);
  this->echoPin = pref.getUChar("echoPin", 255);
  this->intervalSec = pref.getUChar("intervalSec", 5);
  pref.end();
  return true;
}
bool HCSR04Settings::toJSON(JsonObject &obj) {
  obj["enabled"] = this->enabled;
  obj["trigPin"] = this->trigPin;
  obj["echoPin"] = this->echoPin;
  obj["intervalSec"] = this->intervalSec;
  return true;
}
bool HCSR04Settings::fromJSON(JsonObject &obj) {
  if(!obj["enabled"].isNull())     this->enabled = obj["enabled"];
  if(!obj["trigPin"].isNull())     this->trigPin = obj["trigPin"].as<uint8_t>();
  if(!obj["echoPin"].isNull())     this->echoPin = obj["echoPin"].as<uint8_t>();
  if(!obj["intervalSec"].isNull()) this->intervalSec = obj["intervalSec"].as<uint8_t>();
  return true;
}
```

- [ ] **Step 3: Wire `HCSR04.begin()` into `ConfigSettings::begin()`**

In `ConfigSettings::begin()` in ConfigSettings.cpp, add after `this->MQTT.begin();`:

```cpp
  this->MQTT.begin();
  this->HCSR04.begin();  // <-- add this line
```

- [ ] **Step 4: Verify it compiles**

```bash
pio run -e esp32c6 2>&1 | grep -E "error:|FAILED|SUCCESS"
```

Expected: `SUCCESS`

- [ ] **Step 5: Commit**

```bash
git add src/ConfigSettings.h src/ConfigSettings.cpp
git commit -m "feat: add HCSR04Settings config class"
```

---

## Task 2: `HCSR04` sensor module — timer + ISR

**Files:**
- Create: `src/HCSR04.h`
- Create: `src/HCSR04.cpp`

- [ ] **Step 1: Create `src/HCSR04.h`**

```cpp
#ifndef HCSR04_H
#define HCSR04_H
#include <Arduino.h>
#include <esp_timer.h>
#include "driver/gpio.h"

class HCSR04Class {
  public:
    bool begin();
    void end();
    void loop();
    bool usesPin(uint8_t pin);
    float lastDistanceCm = -1.0f;  // -1 = no valid reading yet
  private:
    bool _active = false;
    esp_timer_handle_t _timer = nullptr;
    uint32_t _lastPublishMs = 0;
};

extern HCSR04Class hcsr04;
#endif
```

- [ ] **Step 2: Create `src/HCSR04.cpp`**

```cpp
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
```

- [ ] **Step 3: Verify it compiles**

```bash
pio run -e esp32c6 2>&1 | grep -E "error:|FAILED|SUCCESS"
```

Expected: `SUCCESS`

- [ ] **Step 4: Commit**

```bash
git add src/HCSR04.h src/HCSR04.cpp
git commit -m "feat: add HCSR04 sensor module with non-blocking timer+ISR"
```

---

## Task 3: MQTT Home Assistant discovery

**Files:**
- Modify: `src/HCSR04.cpp`
- Modify: `src/MQTT.h`
- Modify: `src/MQTT.cpp`

- [ ] **Step 1: Add `publishDisco()` declaration to `src/HCSR04.h`**

Add inside `HCSR04Class`:

```cpp
    void publishDisco();
    void unpublishDisco();
```

- [ ] **Step 2: Implement `publishDisco()` / `unpublishDisco()` in `src/HCSR04.cpp`**

Add after the `usesPin()` function:

```cpp
void HCSR04Class::publishDisco() {
    if (!mqtt.connected() || !settings.MQTT.pubDisco) return;

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
    char devId[32];
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
    obj["availability_topic"]      = availTopic;
    obj["payload_available"]       = "online";
    obj["payload_not_available"]   = "offline";

    mqtt.publishDisco(topic, obj, true);
    ESP_LOGI(TAG, "Published HC-SR04 HA discovery");
}

void HCSR04Class::unpublishDisco() {
    if (!mqtt.connected()) return;
    char topic[128];
    snprintf(topic, sizeof(topic), "%s/sensor/espsomfy_%s_distance/config",
             settings.MQTT.discoTopic, settings.serverId);
    mqtt.unpublish(topic);
}
```

- [ ] **Step 3: Call `publishDisco()` when MQTT connects**

In `src/MQTT.cpp`, in the `MQTTClass::connect()` function, find where `somfy.publish()` is called after a successful connect. Add `hcsr04.publishDisco()` on the next line:

```cpp
        somfy.publish();
        hcsr04.publishDisco();  // <-- add this line
```

Add the include at the top of `src/MQTT.cpp`:

```cpp
#include "HCSR04.h"
```

And declare the extern:

```cpp
extern HCSR04Class hcsr04;
```

- [ ] **Step 4: Verify it compiles**

```bash
pio run -e esp32c6 2>&1 | grep -E "error:|FAILED|SUCCESS"
```

Expected: `SUCCESS`

- [ ] **Step 5: Commit**

```bash
git add src/HCSR04.h src/HCSR04.cpp src/MQTT.cpp
git commit -m "feat: HCSR04 MQTT HA discovery on connect"
```

---

## Task 4: Web API endpoints

**Files:**
- Modify: `src/Web.cpp`

The pattern follows the existing `/connectmqtt` + `/mqttsettings` pair exactly.

- [ ] **Step 1: Add include and extern to Web.cpp**

At the top of `src/Web.cpp`, add after the existing includes:

```cpp
#include "HCSR04.h"
```

And after the existing `extern` declarations:

```cpp
extern HCSR04Class hcsr04;
```

- [ ] **Step 2: Add `/hcsr04settings` GET + `/connecthcsr04` POST to Web.cpp**

Find the `/mqttsettings` GET route (around line 2343) and add the two HC-SR04 routes immediately after the closing `});` of the mqttsettings handler:

```cpp
  // hcsr04settings
  asyncServer.on("/hcsr04settings", HTTP_GET, [](AsyncWebServerRequest *request) {
    JsonDocument doc;
    JsonObject obj = doc.to<JsonObject>();
    settings.HCSR04.toJSON(obj);
    if (hcsr04.lastDistanceCm >= 0)
      obj["lastDistanceCm"] = hcsr04.lastDistanceCm;
    serializeJson(doc, g_async_content, sizeof(g_async_content));
    request->send(200, _encoding_json, g_async_content);
  });

  asyncServer.addHandler(new AsyncCallbackJsonWebHandler("/connecthcsr04",
    [](AsyncWebServerRequest *request, JsonVariant &json) {
      if (json.isNull()) {
        request->send(500, "application/json", "{\"status\":\"ERROR\",\"desc\":\"JSON parse error\"}");
        return;
      }
      JsonObject obj = json.as<JsonObject>();
      hcsr04.end();
      settings.HCSR04.fromJSON(obj);
      settings.HCSR04.save();
      hcsr04.begin();
      JsonDocument sdoc;
      JsonObject sobj = sdoc.to<JsonObject>();
      settings.HCSR04.toJSON(sobj);
      serializeJson(sdoc, g_async_content, sizeof(g_async_content));
      request->send(200, _encoding_json, g_async_content);
    }));
```

- [ ] **Step 3: Verify it compiles**

```bash
pio run -e esp32c6 2>&1 | grep -E "error:|FAILED|SUCCESS"
```

Expected: `SUCCESS`

- [ ] **Step 4: Commit**

```bash
git add src/Web.cpp
git commit -m "feat: add /hcsr04settings and /connecthcsr04 web API routes"
```

---

## Task 5: Wire up in SomfyController.ino

**Files:**
- Modify: `src/SomfyController.ino`

- [ ] **Step 1: Add include and global declaration**

At the top of `src/SomfyController.ino`, after `#include "GitOTA.h"`:

```cpp
#include "HCSR04.h"
```

After `GitUpdater git;` in the globals section:

```cpp
HCSR04Class hcsr04;
```

- [ ] **Step 2: Call `hcsr04.begin()` in `setup()`**

After `somfy.begin();` in `setup()`:

```cpp
  somfy.begin();
  hcsr04.begin();  // <-- add this line (no-op if disabled or pins not set)
```

- [ ] **Step 3: Call `hcsr04.loop()` in `loop()`**

After `somfy.loop();` in the main `loop()` function:

```cpp
  somfy.loop();
  hcsr04.loop();  // <-- add this line (no-op if not active)
```

- [ ] **Step 4: Verify it compiles**

```bash
pio run -e esp32c6 2>&1 | grep -E "error:|FAILED|SUCCESS"
```

Expected: `SUCCESS`

- [ ] **Step 5: Commit**

```bash
git add src/SomfyController.ino
git commit -m "feat: wire HCSR04 begin/loop into main sketch"
```

---

## Task 6: Web UI — Sensors tab

**Files:**
- Modify: `data-src/index.html`
- Modify: `data-src/index.js`

- [ ] **Step 1: Add "Sensors" top-level tab to `index.html`**

Find the tab-container line (line 143):

```html
<div class="tab-container"><span class="selected" data-grpid="divSystemSettings">System</span><span data-grpid="divNetworkSettings">Network</span><span data-grpid="divSomfySettings">Somfy</span><span data-grpid="divRadioSettings">Radio</span></div>
```

Replace with:

```html
<div class="tab-container"><span class="selected" data-grpid="divSystemSettings">System</span><span data-grpid="divNetworkSettings">Network</span><span data-grpid="divSomfySettings">Somfy</span><span data-grpid="divRadioSettings">Radio</span><span data-grpid="divSensorsSettings">Sensors</span></div>
```

- [ ] **Step 2: Add Sensors panel HTML to `index.html`**

Find `<div id="divRadioSettings" style="display:none;">` and add the new Sensors panel immediately before it:

```html
                <div id="divSensorsSettings" style="display:none;">
                    <div class="subtab-container"><span class="selected" data-grpid="divHCSR04">HC-SR04 Distance</span></div>
                    <div id="divHCSR04" class="subtab-content">
                        <div class="field-group" style="vertical-align:middle;margin-top:-20px;">
                            <input id="cbHCSR04Enabled" name="hcsr04-enabled" type="checkbox" data-bind="hcsr04.enabled" style="display:inline-block;" />
                            <label for="cbHCSR04Enabled" style="display:inline-block;cursor:pointer;">Enable HC-SR04 Distance Sensor</label>
                        </div>
                        <div id="divHCSR04Config">
                            <div style="font-size:0.8em;color:#888;margin-bottom:8px;">Use a 3.3V-compatible HC-SR04P. Standard HC-SR04 ECHO pin outputs 5V and requires a voltage divider.</div>
                            <div class="field-group" style="width:45%;display:inline-block;margin-right:7px;">
                                <select id="selHCSR04Trig" name="hcsr04-trig" data-bind="hcsr04.trigPin" data-datatype="int" style="width:100%;"></select>
                                <label for="selHCSR04Trig">TRIG Pin</label>
                            </div>
                            <div class="field-group" style="width:45%;display:inline-block;">
                                <select id="selHCSR04Echo" name="hcsr04-echo" data-bind="hcsr04.echoPin" data-datatype="int" style="width:100%;"></select>
                                <label for="selHCSR04Echo">ECHO Pin</label>
                            </div>
                            <div class="field-group" style="width:45%;display:inline-block;margin-right:7px;">
                                <select id="selHCSR04Interval" name="hcsr04-interval" data-bind="hcsr04.intervalSec" data-datatype="int" style="width:100%;">
                                    <option value="1">1 second</option>
                                    <option value="2">2 seconds</option>
                                    <option value="5" selected>5 seconds</option>
                                    <option value="10">10 seconds</option>
                                    <option value="30">30 seconds</option>
                                    <option value="60">60 seconds</option>
                                </select>
                                <label for="selHCSR04Interval">Read Interval</label>
                            </div>
                            <div id="divHCSR04LastReading" style="display:none;margin-top:8px;color:#00bcd4;font-size:0.9em;"></div>
                        </div>
                        <div class="button-container">
                            <button id="btnSaveHCSR04" type="button" onclick="hcsr04.saveSettings();">Save Sensor Settings</button>
                        </div>
                    </div>
                </div>
```

- [ ] **Step 3: Add `hcsr04` JavaScript module to `index.js`**

Find the block `mqtt.init();` near the end of `index.js` (around line 641) and add before it:

```javascript
hcsr04.init();
```

Then, near the end of the file after the `mqtt` object definition, add the new module. Find a similar pattern (e.g. the line just before `mqtt.init()` at the bottom) and insert:

```javascript
var hcsr04 = {
    init: function () {
        this.loadSettings();
    },
    loadSettings: function () {
        getJSONSync('/hcsr04settings', (err, s) => {
            if (err) return;
            // Populate pin selectors with available GPIO numbers
            const pins = [255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21];
            ['selHCSR04Trig', 'selHCSR04Echo'].forEach(id => {
                const sel = document.getElementById(id);
                if (!sel) return;
                sel.innerHTML = '';
                pins.forEach(p => {
                    const opt = document.createElement('option');
                    opt.value = p;
                    opt.text = p === 255 ? '-- not set --' : 'GPIO ' + p;
                    sel.appendChild(opt);
                });
            });
            firmware.bindData({ hcsr04: s });
            if (typeof s.lastDistanceCm === 'number') {
                const div = document.getElementById('divHCSR04LastReading');
                if (div) {
                    div.style.display = '';
                    div.textContent = 'Last reading: ' + s.lastDistanceCm.toFixed(1) + ' cm';
                }
            }
        });
    },
    saveSettings: function () {
        var obj = { hcsr04: {} };
        firmware.serializeForm(obj);
        putJSONSync('/connecthcsr04', obj.hcsr04, (err, response) => {
            if (err) {
                firmware.showError('Failed to save sensor settings');
                return;
            }
            firmware.showMessage('Sensor settings saved');
        });
    }
};
```

- [ ] **Step 4: Handle the new "Sensors" tab in the tab-switch logic**

Find the `switch` statement in `index.js` that handles `divNetworkSettings` (around line 1069) and add a case for the new tab:

```javascript
            case 'divSensorsSettings':
                hcsr04.loadSettings();
                break;
```

- [ ] **Step 5: Verify build compiles (data files get minified)**

```bash
pio run -e esp32c6 2>&1 | grep -E "error:|FAILED|SUCCESS|\[minify\]"
```

Expected: `SUCCESS` with minify output showing all files processed.

- [ ] **Step 6: Commit**

```bash
git add data-src/index.html data-src/index.js
git commit -m "feat: add Sensors tab with HC-SR04 config UI"
```

---

## Task 7: Pin conflict guard

**Files:**
- Modify: `src/Somfy.cpp` (the pin conflict checker)

- [ ] **Step 1: Add HCSR04 pin conflict check**

In `src/Somfy.cpp`, find `Transceiver::usesPin()` (around line 4524) and note how shade pin validation works around line 3162. Find where `somfy.transceiver.usesPin(upPin)` is called in the shade's pin assignment validation and add HCSR04 checks alongside:

```cpp
        if((upPin != 255 && somfy.transceiver.usesPin(upPin)) ||
          (downPin != 255 && somfy.transceiver.usesPin(downPin)) ||
          (myPin != 255 && somfy.transceiver.usesPin(myPin)) ||
          (upPin != 255 && hcsr04.usesPin(upPin)) ||       // <-- add
          (downPin != 255 && hcsr04.usesPin(downPin)) ||   // <-- add
          (myPin != 255 && hcsr04.usesPin(myPin)))          // <-- add
```

Add the include at the top of `Somfy.cpp`:

```cpp
#include "HCSR04.h"
```

- [ ] **Step 2: Verify it compiles**

```bash
pio run -e esp32c6 2>&1 | grep -E "error:|FAILED|SUCCESS"
```

Expected: `SUCCESS`

- [ ] **Step 3: Commit**

```bash
git add src/Somfy.cpp
git commit -m "feat: add HCSR04 pin conflict guard to shade pin validation"
```

---

## Task 8: Full build verification — all targets

- [ ] **Step 1: Build all 4 environments**

```bash
pio run -e esp32dev -e esp32c3 -e esp32s3 -e esp32c6 2>&1 | grep -E "Environment|Status|error:|fatal"
```

Expected output:
```
Environment    Status    Duration
-------------  --------  ------------
esp32dev       SUCCESS   ...
esp32c3        SUCCESS   ...
esp32s3        SUCCESS   ...
esp32c6        SUCCESS   ...
```

- [ ] **Step 2: Confirm no new warnings introduced**

```bash
pio run -e esp32c6 2>&1 | grep "warning:" | grep -v "deprecated\|volatile" | head -20
```

Expected: no new warnings beyond the pre-existing ArduinoJson deprecation warnings.

- [ ] **Step 3: Commit any final tweaks and tag for testing**

```bash
git add -p   # review any remaining changes
git commit -m "chore: verify all targets build with HCSR04 addon"
```

---

## Notes for ESPSomfy-RTS-HA adapter (out of scope, for later)

When updating https://github.com/shailensobhee/ESPSomfy-RTS-HA to expose distance metrics in Home Assistant, the integration will need to:

1. Subscribe to `{rootTopic}/sensors/distance` — value is a string like `"123.4"` (cm, 1 decimal)
2. Optionally rely on MQTT discovery (if `pubDisco` is enabled on the device) — the sensor entity will auto-appear under the same HA device as the shades
3. Or register the sensor entity manually in the HA integration using the REST endpoint `GET /hcsr04settings` to check if `enabled == true` and `trigPin != 255` before registering the entity

The WebSocket `distance` event emitted by `sockEmit` can also be used for real-time updates in the HA integration's local polling path — payload: `{ "distanceCm": 123.4 }`.
