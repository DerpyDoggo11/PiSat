// ============================================================================
//  PiSat STA uplink bridge  --  ESP32-C3 Super Mini
//
//  A one-way UART -> WiFi -> TCP forwarder. It joins the MAIN rover's SoftAP
//  ("PiSat-RTK") as a station and streams whatever its RP2350 sends on the UART
//  straight to a TCP port on the rover (192.168.4.1). Two build roles pick the
//  port (platformio.ini):
//
//    [env:client]  -DUPLINK_PORT=8889  -- CLIENT PiSat: forwards $GP2 position
//                                          lines to the rover's command server,
//                                          so the rover averages the two GPS.
//    [env:base]    -DUPLINK_PORT=2101  -- BASE PiSat: forwards RTCM3 / $PSDG
//                                          corrections to the rover.
//
//  WIRING (both roles identical): C3 GPIO20 (RX) <- PiSat header TX (RP2350 GP0),
//  common GND. The link is one-way; GPIO21 is spare. Power the C3 from USB or the
//  PiSat 3V3 pin.
// ============================================================================
#include <Arduino.h>
#include <WiFi.h>

#ifndef UPLINK_PORT
#define UPLINK_PORT 8889          // default: client role
#endif

static const char*    ROVER_SSID = "PiSat-RTK";
static const char*    ROVER_PASS = "pisat-rtk-1234";
static IPAddress      ROVER_IP(192, 168, 4, 1);   // main rover SoftAP default
static const uint16_t PORT       = UPLINK_PORT;

static const int      LINK_RX_PIN = 20;   // <- PiSat header TX (RP2350 GP0)
static const int      LINK_TX_PIN = 21;   // spare (one-way link)
static const uint32_t LINK_BAUD   = 115200;
HardwareSerial LinkSerial(1);

WiFiClient rover;
unsigned long lastByteMs = 0, bytesFwd = 0;

static void connectWifi() {
    if (WiFi.status() == WL_CONNECTED) return;
    Serial.printf("joining %s ...\n", ROVER_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ROVER_SSID, ROVER_PASS);
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) { delay(250); Serial.print('.'); }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) Serial.printf("joined, ip %s\n", WiFi.localIP().toString().c_str());
    else                               Serial.println("wifi join failed, will retry");
}

static void connectRover() {
    if (rover.connected()) return;
    Serial.printf("connecting to rover %s:%u ...\n", ROVER_IP.toString().c_str(), PORT);
    if (rover.connect(ROVER_IP, PORT)) Serial.println("rover link up");
    else                               Serial.println("rover connect failed");
}

void setup() {
    Serial.begin(115200);
    LinkSerial.begin(LINK_BAUD, SERIAL_8N1, LINK_RX_PIN, LINK_TX_PIN);
    delay(200);
    Serial.printf("PiSat STA uplink bridge (ESP32-C3) boot -> rover :%u\n", PORT);
    connectWifi();
    connectRover();
}

void loop() {
    if (WiFi.status() != WL_CONNECTED) { connectWifi(); return; }
    if (!rover.connected())            { connectRover(); delay(200); return; }

    uint8_t buf[256];
    int n = 0;
    while (LinkSerial.available() && n < (int)sizeof(buf)) buf[n++] = LinkSerial.read();
    if (n > 0) { rover.write(buf, n); bytesFwd += n; lastByteMs = millis(); }

    static uint32_t tLog = 0;
    if (millis() - tLog >= 2000) {
        tLog = millis();
        unsigned long age = lastByteMs ? (millis() - lastByteMs) : 999999;
        Serial.printf("fwd=%lu bytes  last %lums ago  rover=%d\n", bytesFwd, age, rover.connected());
    }
}
