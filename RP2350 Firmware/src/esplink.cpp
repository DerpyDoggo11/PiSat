#include "esp_link.h"
#include "esc.h"
#include "hmi.h"

namespace EspLink {

void begin() {
    pinMode(PIN_ESP_EN, OUTPUT);
    pinMode(PIN_ESP_BOOT, OUTPUT);
    digitalWrite(PIN_ESP_EN, HIGH); 
    digitalWrite(PIN_ESP_BOOT, HIGH);

    Serial1.setTX(PIN_ESP_TX);
    Serial1.setRX(PIN_ESP_RX);
    Serial1.setFIFOSize(512);
    Serial1.begin(LINK_BAUD);
}

void reset() {
    digitalWrite(PIN_ESP_BOOT, HIGH);
    digitalWrite(PIN_ESP_EN, LOW);
    delay(50);
    digitalWrite(PIN_ESP_EN, HIGH);
    delay(200);
}

void enterBootloader() {
    digitalWrite(PIN_ESP_BOOT, LOW); 
    digitalWrite(PIN_ESP_EN, LOW);
    delay(50);
    digitalWrite(PIN_ESP_EN, HIGH); 
    delay(100);
    digitalWrite(PIN_ESP_BOOT, HIGH);
}

void hold() { digitalWrite(PIN_ESP_EN, LOW); }

[[noreturn]] void passthrough() {
    Esc::disarm();
    Hmi::ledA(true);
    Hmi::ledB(true);

    uint32_t baud = LINK_BAUD;
    Serial1.begin(baud);

    bool lastDtr = false, lastRts = false;

    for (;;) {
        uint32_t hostBaud = Serial.baud();
        if (hostBaud >= 1200 && hostBaud != baud) {
            baud = hostBaud;
            Serial1.flush();
            Serial1.end();
            Serial1.setTX(PIN_ESP_TX);
            Serial1.setRX(PIN_ESP_RX);
            Serial1.begin(baud);
        }

        bool dtr = Serial.dtr();
        bool rts = Serial.rts();
        if (dtr != lastDtr || rts != lastRts) {
            if (dtr == rts) {
                digitalWrite(PIN_ESP_EN, HIGH);
                digitalWrite(PIN_ESP_BOOT, HIGH);
            } else if (rts) {
                digitalWrite(PIN_ESP_EN, LOW);
                digitalWrite(PIN_ESP_BOOT, HIGH);
            } else {
                digitalWrite(PIN_ESP_EN, HIGH);
                digitalWrite(PIN_ESP_BOOT, LOW);
            }
            lastDtr = dtr;
            lastRts = rts;
        }

        while (Serial.available() && Serial1.availableForWrite())
            Serial1.write((uint8_t)Serial.read());
        while (Serial1.available() && Serial.availableForWrite())
            Serial.write((uint8_t)Serial1.read());
    }
}

void pollForBridgeRequest() {
    static char buf[16];
    static uint8_t n = 0;
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            buf[n] = 0;
            if (strcmp(buf, "esp_bridge") == 0) {
                Serial.println("[rp2350] entering ESP32 programmer mode");
                Serial.flush();
                delay(50);
                passthrough();
            }
            n = 0;
        } else if (n < sizeof(buf) - 1) {
            buf[n++] = c;
        }
    }
}

}