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
