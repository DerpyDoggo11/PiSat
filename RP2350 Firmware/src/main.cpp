
#include <Arduino.h>
#include "pins.h"
#include "gnss.h"
#include "esc.h"
#include "hmi.h"
#include "esp_link.h"

#if defined(ESP_PASSTHROUGH_ONLY)

void setup() {
    Serial.begin(115200);
    Hmi::begin();
    Esc::begin();
    Esc::disarm();
    EspLink::begin();
    EspLink::passthrough();
}
void loop() {}

#else

static volatile float g_throttle[4] = {0, 0, 0, 0};
static volatile bool  g_armed = false;
static float g_vbat  = 0.0f;

// CORE 0 
void setup() {
    Serial.begin(115200);

    Hmi::begin();
    Power::begin();
    EspLink::begin();
    Gnss::begin();

    for (uint8_t i = 0; i < 5; i++) pinMode(PIN_J6[i], INPUT);

    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 2000) { }

    Serial.println("=== GNSS / BLDC board (RP2350A) ===");
    Serial.printf("VBAT: %.2f V\n", Power::batteryVolts());
    Serial.println("Type 'help' for commands, 'esp_bridge' to flash the ESP32.");

    Hmi::beep(60);
}

static void handleCommand(char* cmd) {
    if (!strcmp(cmd, "help")) {
        Serial.println("arm | disarm | t <0-3> <0.0-1.0> | tall <0.0-1.0> |");
        Serial.println("gps | aux | stat | gnss_reset | ant on|off |");
        Serial.println("esp_reset | esp_boot | esp_bridge");
    }
    else if (!strcmp(cmd, "arm")) {
        if (g_vbat < Power::VBAT_CRITICAL) {
            Serial.println("refused: battery too low");
            return;
        }
        Serial.println("arming ESCs (2 s of min throttle)...");
        Esc::arm();
        g_armed = true;
        Hmi::beep(150);
        Serial.println("ARMED");
    }
    else if (!strcmp(cmd, "disarm")) {
        g_armed = false;
        for (uint8_t i = 0; i < 4; i++) g_throttle[i] = 0.0f;
        Esc::disarm();
        Serial.println("disarmed");
    }
    else if (!strncmp(cmd, "tall ", 5)) {
        float v = atof(cmd + 5);
        for (uint8_t i = 0; i < 4; i++) g_throttle[i] = constrain(v, 0.f, 1.f);
        Serial.printf("all = %.2f\n", v);
    }
    else if (!strncmp(cmd, "t ", 2)) {
        int idx; float v;
        if (sscanf(cmd + 2, "%d %f", &idx, &v) == 2 && idx >= 0 && idx < 4) {
            g_throttle[idx] = constrain(v, 0.0f, 1.0f);
            Serial.printf("esc %d = %.2f\n", idx, g_throttle[idx]);
        }
    }
    else if (!strcmp(cmd, "gps")) {
        const GnssFix& f = Gnss::fix();
        if (Gnss::hasFix()) {
            Serial.printf("fix q%u  %.6f, %.6f  alt %.1f m  %.1f m/s  " "hdg %.0f  sats %u  hdop %.1f  %02u:%02u:%02uZ\n", f.fixQuality, f.latitude, f.longitude, f.altitude_m, f.speed_mps, f.course_deg, f.satellites, f.hdop, f.hour, f.minute, f.second);
        } else {
            Serial.printf("no fix (sats %u, pps %lu)\n", f.satellites, (unsigned long)Gnss::ppsCount());
        }
    }
    else if (!strcmp(cmd, "aux")) {
        Serial.printf("J4: A1 %.3fV  A2 %.3fV  A3 %.3fV\n", Power::readAux(0), Power::readAux(1), Power::readAux(2));
    }
    else if (!strcmp(cmd, "stat")) {
        Serial.printf("vbat %.2f V  usb %d  armed %d  pps %lu  fix %d\n", g_vbat, (int)Power::usbPresent(), (int)g_armed, (unsigned long)Gnss::ppsCount(), (int)Gnss::hasFix());
    }
    else if (!strcmp(cmd, "gnss_reset")) { Gnss::reset(); Serial.println("gnss reset"); }
    else if (!strcmp(cmd, "ant on")) { Gnss::antennaPower(true);  Serial.println("ant on"); }
    else if (!strcmp(cmd, "ant off")) { Gnss::antennaPower(false); Serial.println("ant off"); }
    else if (!strcmp(cmd, "esp_reset")) { EspLink::reset(); Serial.println("esp reset"); }
    else if (!strcmp(cmd, "esp_boot")) { EspLink::enterBootloader(); Serial.println("esp bootloader"); }
    else if (!strcmp(cmd, "esp_bridge")) {
        g_armed = false;
        Esc::disarm();
        Serial.println("entering programmer mode - reset the RP2350 to exit");
        Serial.flush();
        EspLink::passthrough();
    }
}

void loop() {
    Gnss::poll();

    static char line[64];
    static uint8_t n = 0;
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (n) { line[n] = 0; handleCommand(line); n = 0; }
        } else if (n < sizeof(line) - 1) {
            line[n++] = c;
        }
    }

    while (Serial1.available()) Serial.write((char)Serial1.read());

    static uint32_t tLast = 0;
    if (millis() - tLast >= 100) {
        tLast = millis();
        g_vbat = Power::batteryVolts();

        if (Gnss::hasFix()) Hmi::ledA(true);
        else Hmi::blink(PIN_LED_A, 500);

        if (g_vbat < Power::VBAT_WARN) Hmi::blink(PIN_LED_B, 125);
        else Hmi::ledB(g_armed);

        if (g_armed && g_vbat < Power::VBAT_CRITICAL) {
            g_armed = false;
            for (uint8_t i = 0; i < 4; i++) g_throttle[i] = 0.0f;
            Esc::disarm();
            Hmi::beep(400);
            Serial.println("!! low voltage cutoff - disarmed");
        }
    }
}

// CORE 1

void setup1() {
    Esc::begin();
}

void loop1() {
    static uint32_t next = 0;
    uint32_t now = micros();
    if ((int32_t)(now - next) < 0) return;
    next = now + 10000;

    if (!g_armed) { Esc::disarm(); return; }

    for (uint8_t i = 0; i < 4; i++) Esc::setThrottle(i, g_throttle[i]);
}

#endif