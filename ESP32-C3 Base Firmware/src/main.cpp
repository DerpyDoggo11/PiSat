// ============================================================================
//  PiSat RTK BASE bridge  --  ESP32-C3 Super Mini
//
//  Sits next to the base PiSat. The base PiSat's RP2350 (flashed with the
//  [env:base] firmware) configures its Teseo III GNSS to output RTCM3 and
//  streams those bytes out on GP12. This board reads them on a UART, joins the
//  rover's WiFi SoftAP, and forwards them over TCP to the rover, which injects
//  them into its own GNSS to obtain an RTK / DGPS fix.
//
//  WIRING (ESP32-C3 super mini <-> base PiSat RP2350 J6 breakout, common GND):
//    C3 GPIO 4 (RX) <- RP2350 GP12 (J6 pin 1)   : RTCM3 stream in @115200
//    C3 GND         <- RP2350 GND
//    (No TX needed; the base link is one-way.)
//
//  Power the C3 from its own USB or from the PiSat 3V3 breakout pin.
// ============================================================================

#include <Arduino.h>
#include <WiFi.h>

// ---- Must match the rover's SoftAP + RTCM server --------------------------
static const char*    ROVER_SSID = "PiSat-RTK";
static const char*    ROVER_PASS = "pisat-rtk-1234";
static IPAddress      ROVER_IP(192, 168, 4, 1);   // ESP32 SoftAP default
static const uint16_t RTCM_PORT  = 2101;

// ---- UART from the base RP2350 --------------------------------------------
static const int      LINK_RX_PIN = 4;    // <- RP2350 GP12
static const int      LINK_TX_PIN = 5;    // unused (spare)
static const uint32_t LINK_BAUD   = 115200;
HardwareSerial LinkSerial(1);

WiFiClient rover;
unsigned long lastByteMs = 0;
unsigned long bytesFwd   = 0;

static void connectWifi() {
    if (WiFi.status() == WL_CONNECTED) return;
    Serial.printf("joining %s ...\n", ROVER_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ROVER_SSID, ROVER_PASS);
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
        delay(250);
        Serial.print('.');
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED)
        Serial.printf("joined, my ip %s\n", WiFi.localIP().toString().c_str());
    else
        Serial.println("wifi join failed, will retry");
}

static void connectRover() {
    if (rover.connected()) return;
    Serial.printf("connecting to rover %s:%u ...\n",
                  ROVER_IP.toString().c_str(), RTCM_PORT);
    if (rover.connect(ROVER_IP, RTCM_PORT)) Serial.println("rover link up");
    else                                    Serial.println("rover connect failed");
}

void setup() {
    Serial.begin(115200);
    LinkSerial.begin(LINK_BAUD, SERIAL_8N1, LINK_RX_PIN, LINK_TX_PIN);
    delay(200);
    Serial.println("PiSat RTK BASE bridge (ESP32-C3) boot");
    connectWifi();
    connectRover();
}

void loop() {
    if (WiFi.status() != WL_CONNECTED) { connectWifi(); return; }
    if (!rover.connected())            { connectRover(); delay(200); return; }

    // Forward every RTCM3 byte from the base GNSS straight to the rover.
    uint8_t buf[256];
    int n = 0;
    while (LinkSerial.available() && n < (int)sizeof(buf)) buf[n++] = LinkSerial.read();
    if (n > 0) {
        rover.write(buf, n);
        bytesFwd += n;
        lastByteMs = millis();
    }

    static uint32_t tLog = 0;
    if (millis() - tLog >= 2000) {
        tLog = millis();
        unsigned long age = lastByteMs ? (millis() - lastByteMs) : 999999;
        Serial.printf("fwd=%lu bytes  last rtcm %lums ago  rover=%d\n",
                      bytesFwd, age, rover.connected());
    }
}
