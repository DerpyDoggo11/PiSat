// ============================================================================
//  PiSat RTK ROVER WiFi bridge  --  ESP32-C3 Super Mini
//
//  The rover's brain (GNSS, RTK, pathfinding, motors) now lives on the PiSat
//  RP2350, because the two ZS-X111H motor controllers are wired to the PiSat's
//  own breakout GPIO. This board is a TRANSPARENT WiFi <-> UART bridge:
//
//    1. Hosts the WiFi SoftAP "PiSat-RTK" (the base relay and the Processing
//       client both join it).
//    2. TCP server :2101 -- the BASE connects and pushes RTCM3 corrections; we
//       write those bytes straight to the RP2350 (GP13), which feeds the GNSS.
//    3. TCP server :8888 -- Processing connects; we forward the RP2350's
//       telemetry text (from GP12) straight to it.
//
//  It parses nothing and makes no decisions.
//
//  WIRING  --  ESP32-C3 Super Mini  <->  PiSat rover RP2350 (J6 breakout):
//    C3 GPIO 20 (RX) <- RP2350 GP12 (J6 pin 1)   : telemetry text in @115200
//    C3 GPIO 21 (TX) -> RP2350 GP13 (J6 pin 2)   : RTCM3 corrections out
//    C3 GND          -- RP2350 GND               : common ground
//  (The motors do NOT connect here -- they are on the PiSat. See
//   RTK_SETUP_INSTRUCTIONS.txt section 4.)
// ============================================================================

#include <Arduino.h>
#include <WiFi.h>

const int      GPS_RX_PIN = 20;   // <- RP2350 GP12 (telemetry text)
const int      GPS_TX_PIN = 21;   // -> RP2350 GP13 (RTCM3 corrections)
const uint32_t LINK_BAUD  = 115200;
HardwareSerial Link(1);           // UART1 to the RP2350

const char*    AP_SSID   = "PiSat-RTK";
const char*    AP_PASS   = "pisat-rtk-1234";   // >= 8 chars
const uint16_t RTCM_PORT = 2101;   // base -> here -> RP2350
const uint16_t TELE_PORT = 8888;   // RP2350 -> here -> Processing

WiFiServer rtcmServer(RTCM_PORT);
WiFiServer teleServer(TELE_PORT);
WiFiClient rtcmClient;             // the base station
WiFiClient teleClient;             // the Processing visualiser

unsigned long lastRtcmMs = 0, bytesToGnss = 0, bytesToTele = 0;

void setup() {
    Serial.begin(115200);
    Link.begin(LINK_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    delay(50);
    Serial.println("PiSat RTK ROVER WiFi bridge (ESP32-C3) boot");

    WiFi.mode(WIFI_AP);
    bool ok = WiFi.softAP(AP_SSID, AP_PASS);
    Serial.printf("SoftAP %s  ssid=%s  ip=%s\n",
                  ok ? "up" : "FAILED", AP_SSID, WiFi.softAPIP().toString().c_str());
    rtcmServer.begin();
    teleServer.begin();
    Serial.printf("RTCM server :%u   telemetry server :%u\n", RTCM_PORT, TELE_PORT);
}

void loop() {
    // --- accept clients -----------------------------------------------------
    if (!rtcmClient || !rtcmClient.connected()) {
        WiFiClient c = rtcmServer.available();
        if (c) { rtcmClient = c; Serial.println("base station connected (RTCM)"); }
    }
    if (!teleClient || !teleClient.connected()) {
        WiFiClient c = teleServer.available();
        if (c) { teleClient = c; Serial.println("telemetry client connected"); }
    }

    // --- RTCM in (base -> RP2350 -> GNSS) -----------------------------------
    if (rtcmClient && rtcmClient.connected()) {
        uint8_t buf[256];
        int n = rtcmClient.available();
        while (n > 0) {
            int m = rtcmClient.read(buf, min(n, (int)sizeof(buf)));
            if (m <= 0) break;
            Link.write(buf, m);
            bytesToGnss += m; lastRtcmMs = millis();
            n -= m;
        }
    }

    // --- telemetry out (RP2350 -> Processing) -------------------------------
    if (Link.available()) {
        uint8_t buf[256];
        int n = 0;
        while (Link.available() && n < (int)sizeof(buf)) buf[n++] = Link.read();
        if (n > 0 && teleClient && teleClient.connected()) {
            teleClient.write(buf, n);
            bytesToTele += n;
        }
    }

    static uint32_t tLog = 0;
    if (millis() - tLog >= 3000) {
        tLog = millis();
        unsigned long age = lastRtcmMs ? (millis() - lastRtcmMs) : 999999;
        Serial.printf("clients: base=%d tele=%d | ->gnss=%lu ->tele=%lu | rtcm %lums\n",
                      rtcmClient.connected(), teleClient.connected(),
                      bytesToGnss, bytesToTele, age);
    }
}
