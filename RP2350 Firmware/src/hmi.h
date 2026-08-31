#pragma once
#include <Arduino.h>
#include "pins.h"

namespace Hmi {

#if STATUS_LED_ACTIVE_LOW
static constexpr uint8_t LED_ON  = LOW;
static constexpr uint8_t LED_OFF = HIGH;
#else
static constexpr uint8_t LED_ON  = HIGH;
static constexpr uint8_t LED_OFF = LOW;
#endif

inline void begin() {
    pinMode(PIN_LED_A, OUTPUT);
    pinMode(PIN_LED_B, OUTPUT);
    digitalWrite(PIN_LED_A, LED_OFF);
    digitalWrite(PIN_LED_B, LED_OFF);

    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);
}

inline void ledA(bool on) { digitalWrite(PIN_LED_A, on ? LED_ON : LED_OFF); }
inline void ledB(bool on) { digitalWrite(PIN_LED_B, on ? LED_ON : LED_OFF); }

inline void blink(uint8_t pin, uint16_t period_ms) {
    if (period_ms == 0) { digitalWrite(pin, LED_ON); return; }
    bool on = ((millis() / period_ms) & 1) == 0;
    digitalWrite(pin, on ? LED_ON : LED_OFF);
}

inline void beep(uint16_t ms = 80, uint16_t freq = 2700) {
    tone(PIN_BUZZER, freq, ms);
}

}

namespace Power {

inline void begin() {
    analogReadResolution(12);
    for (uint8_t i = 0; i < 3; i++) pinMode(PIN_J4_ADC[i], INPUT);
}

inline float batteryVolts() {
    uint32_t acc = 0;
    for (uint8_t i = 0; i < 16; i++) acc += analogRead(PIN_VBAT_ADC);
    return (acc / 16.0f / 4095.0f) * ADC_VREF * VBAT_DIVIDER;
}

inline float readAux(uint8_t idx) {
    if (idx > 2) return 0.0f;
    return (analogRead(PIN_J4_ADC[idx]) / 4095.0f) * ADC_VREF;
}

inline bool usbPresent() { return (bool)Serial; }

static constexpr float VBAT_FULL = 4.10f;
static constexpr float VBAT_WARN = 3.40f;
static constexpr float VBAT_CRITICAL = 3.10f;

}