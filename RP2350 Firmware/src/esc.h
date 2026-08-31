#pragma once
#include <Arduino.h>
#include "pins.h"

// SAFE STUB — no motor output.
// Holds all ESC signal pins LOW so no PWM is ever emitted; motors cannot spin.
// Replace with a real driver (e.g. Servo 1000-2000us PWM) before flying.
namespace Esc {

inline void begin() {
    for (uint8_t i = 0; i < 4; i++) {
        pinMode(PIN_ESC[i], OUTPUT);
        digitalWrite(PIN_ESC[i], LOW);
    }
}

inline void disarm() {
    for (uint8_t i = 0; i < 4; i++) digitalWrite(PIN_ESC[i], LOW);
}

inline void arm() {
    // No-op stub: real driver would output min throttle for ~2 s here.
    disarm();
}

inline void setThrottle(uint8_t idx, float throttle) {
    (void)idx;
    (void)throttle;
    // No-op stub: motors never spin.
}

}
