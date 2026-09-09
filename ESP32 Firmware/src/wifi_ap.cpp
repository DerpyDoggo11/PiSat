// Basic WiFi Access Point test firmware for the ESP32-C3.
//
// Brings up a SoftAP you can connect to from a phone/laptop, and serves a
// tiny status page at http://192.168.4.1/ . Status is also printed over the
// UART link to the RP2350, which echoes it to USB (visible in the RP2350
// serial monitor). Built only in the [env:esp_wifi_ap] environment.

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "pins.h"

static const char* AP_SSID = "PiSat-Test";
static const char* AP_PASS = "pisat1234";   // >= 8 chars, or use "" for open

static WebServer server(80);

// UART0 -> RP2350 link (its Serial1). Prints here are echoed to USB by the
// RP2350 in normal mode, so they show up in the RP2350 serial monitor.
static HardwareSerial Link(0);

static void handleRoot() {
    int n = WiFi.softAPgetStationNum();
    String html = "<!doctype html><meta name=viewport content='width=device-width,initial-scale=1'>"
                  "<title>PiSat AP</title><body style='font-family:sans-serif'>"
                  "<h2>PiSat ESP32-C3 WiFi AP</h2>"
                  "<p>SSID: " + String(AP_SSID) + "</p>"
                  "<p>AP IP: " + WiFi.softAPIP().toString() + "</p>"
                  "<p>Clients connected: " + String(n) + "</p>"
                  "<p>Uptime: " + String(millis() / 1000) + " s</p>"
                  "</body>";
    server.send(200, "text/html", html);
}

void setup() {
    // Serial0 = UART0 on the link pins -> RP2350 (echoed to USB).
    Link.begin(LINK_BAUD, SERIAL_8N1, PIN_LINK_RX, PIN_LINK_TX);
    delay(50);
    Link.println("S,esp32-c3 wifi-ap boot");

    WiFi.mode(WIFI_AP);
    bool ok = WiFi.softAP(AP_SSID, AP_PASS);
    Link.printf("S,softAP %s ssid=%s ip=%s\n",
                   ok ? "up" : "FAILED",
                   AP_SSID, WiFi.softAPIP().toString().c_str());

    server.on("/", handleRoot);
    server.begin();
    Link.println("S,http server on http://192.168.4.1/");
}

void loop() {
    server.handleClient();

    static uint32_t t = 0;
    static int lastN = -1;
    if (millis() - t >= 1000) {
        t = millis();
        int n = WiFi.softAPgetStationNum();
        if (n != lastN) {
            lastN = n;
            Link.printf("S,clients=%d\n", n);
        }
    }
}
