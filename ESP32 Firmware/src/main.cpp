// ============================================================================
//  PiSat RTK ROVER WiFi bridge  --  ESP32-C3 Super Mini
//
//  This is the MAIN rover bridge. Two build modes (chosen in platformio.ini):
//
//   [env:main]            DEFAULT / SoftAP  ------------------------------------
//     Hosts the "PiSat-RTK" access point. A local BASE PiSat pushes position-
//     domain corrections ($PSDG) on TCP :2101; a CLIENT PiSat pushes its $GP2
//     position on :8889; Processing gets telemetry on :8888 and sends path/WASD
//     on :8889. Everything is a transparent bridge to the RP2350 (which does
//     GNSS/RTK/pathfinding/motors). (The client/base PiSats run src/bridge_sta.cpp.)
//
//   [env:main_ntrip]      NTRIP client  (-DUSE_NTRIP=1) -----------------------
//     Joins your INTERNET WiFi (station) and runs an NTRIP client: it pulls
//     real RTCM3 corrections from a caster (a base within ~10-30 km, or a
//     service) and streams them to the RP2350, whose GNSS then computes a real
//     DGPS/RTK fix on-chip. No local base PiSat needed. Telemetry :8888 and
//     command :8889 still run (find the rover at pisat-rover.local or its IP,
//     printed on this serial). The laptop joins the SAME WiFi -> imagery works.
//
//  WIRING (unchanged): C3 GPIO20 (RX) <- RP2350 GP0 ; C3 GPIO21 (TX) -> GP1 ;
//  common GND. The RP2350 demuxes RTCM3 (-> GNSS) from "$" commands on the link.
// ============================================================================

#include <Arduino.h>
#include <WiFi.h>

#ifndef USE_NTRIP
#define USE_NTRIP 0
#endif
#if USE_NTRIP
#include <ESPmDNS.h>
#endif

const int      GPS_RX_PIN = 20;   // <- RP2350 GP0 (telemetry text)
const int      GPS_TX_PIN = 21;   // -> RP2350 GP1 (corrections + commands)
const uint32_t LINK_BAUD  = 115200;
HardwareSerial Link(1);           // UART1 to the RP2350

const uint16_t TELE_PORT = 8888;  // RP2350 -> Processing (telemetry)
const uint16_t CMD_PORT  = 8889;  // Processing -> RP2350 (path + WASD)
WiFiServer teleServer(TELE_PORT);
WiFiServer cmdServer(CMD_PORT);
WiFiClient teleClient, cmdClient;
unsigned long bytesToTele = 0, bytesFromCmd = 0, rtcmBytes = 0, lastRtcmMs = 0;

#if USE_NTRIP
// ============================ NTRIP CONFIG -- EDIT ==========================
const char*    STA_SSID    = "your-wifi";        // internet WiFi (phone hotspot ok)
const char*    STA_PASS    = "your-password";
const char*    NTRIP_HOST  = "rtk2go.com";       // caster host (e.g. rtk2go.com)
const uint16_t NTRIP_PORT  = 2101;
const char*    NTRIP_MOUNT = "MOUNTPOINT";       // a base near you
const char*    NTRIP_USER  = "vrs18451@email.com";    // rtk2go: your email
const char*    NTRIP_PASS  = "none";             // rtk2go: "none"
double NTRIP_GGA_LAT = 48.7597, NTRIP_GGA_LON = -122.3620;  // fallback for 1st GGA
// ============================================================================

WiFiClient ntrip;
bool   ntripOk = false;
double lastLat = 0, lastLon = 0; bool haveFix = false;

String b64(String in) {
  const char* T = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  String out = ""; int val = 0, bits = -6;
  for (int i = 0; i < in.length(); i++) {
    val = (val << 8) + (uint8_t)in.charAt(i); bits += 8;
    while (bits >= 0) { out += T[(val >> bits) & 0x3F]; bits -= 6; }
  }
  if (bits > -6) out += T[((val << 8) >> (bits + 8)) & 0x3F];
  while (out.length() % 4) out += "=";
  return out;
}
String buildGGA(double lat, double lon) {
  int    dLa = (int)fabs(lat); double mLa = (fabs(lat) - dLa) * 60.0; char ns = lat >= 0 ? 'N' : 'S';
  int    dLo = (int)fabs(lon); double mLo = (fabs(lon) - dLo) * 60.0; char ew = lon >= 0 ? 'E' : 'W';
  char body[128];
  snprintf(body, sizeof(body), "GPGGA,000000.00,%02d%09.6f,%c,%03d%09.6f,%c,1,08,1.0,10.0,M,0.0,M,,",
           dLa, mLa, ns, dLo, mLo, ew);
  uint8_t cs = 0; for (char* p = body; *p; p++) cs ^= (uint8_t)*p;
  char out[140]; snprintf(out, sizeof(out), "$%s*%02X\r\n", body, cs);
  return String(out);
}
void sendGGA() {
  ntrip.print(buildGGA(haveFix ? lastLat : NTRIP_GGA_LAT, haveFix ? lastLon : NTRIP_GGA_LON));
}
bool ntripConnect() {
  if (WiFi.status() != WL_CONNECTED) return false;
  Serial.printf("NTRIP: connecting %s:%u /%s ...\n", NTRIP_HOST, NTRIP_PORT, NTRIP_MOUNT);
  if (!ntrip.connect(NTRIP_HOST, NTRIP_PORT)) { Serial.println("NTRIP: TCP connect failed"); return false; }
  ntrip.print(String("GET /") + NTRIP_MOUNT + " HTTP/1.0\r\n"
              + "User-Agent: NTRIP PiSat/1.0\r\n"
              + "Authorization: Basic " + b64(String(NTRIP_USER) + ":" + NTRIP_PASS) + "\r\n\r\n");
  uint32_t t0 = millis(); String line = "";
  while (millis() - t0 < 6000) {
    if (!ntrip.available()) { delay(5); continue; }
    char c = ntrip.read();
    if (c == '\n') {
      line.trim();
      if (line.startsWith("ICY 200"))  { Serial.println("NTRIP: streaming (ICY)"); return true; }
      if (line.indexOf("200") >= 0 && line.startsWith("HTTP")) {   // HTTP: skip headers
        while (millis() - t0 < 6000) {
          if (!ntrip.available()) { delay(5); continue; }
          char d = ntrip.read();
          if (d == '\n') { String h=""; return true; }   // first \n after status: good enough
        }
        return true;
      }
      if (line.length() > 0) { Serial.println("NTRIP: bad response: " + line); return false; }
      line = "";
    } else line += c;
  }
  Serial.println("NTRIP: no response");
  return false;
}
void serviceNtrip() {
  if (WiFi.status() != WL_CONNECTED) {
    static uint32_t t = 0; if (millis()-t > 3000) { t = millis(); WiFi.begin(STA_SSID, STA_PASS); }
    return;
  }
  if (!ntrip.connected()) {
    static uint32_t t = 0; if (millis()-t < 3000) return; t = millis();
    ntripOk = ntripConnect(); if (ntripOk) sendGGA();
    return;
  }
  uint8_t buf[512];
  int n = ntrip.available();
  while (n > 0) {
    int m = ntrip.read(buf, min(n, (int)sizeof(buf)));
    if (m <= 0) break;
    Link.write(buf, m);                 // RTCM3 -> RP2350 -> GNSS
    rtcmBytes += m; lastRtcmMs = millis();
    n -= m;
  }
  static uint32_t g = 0; if (millis()-g > 10000) { g = millis(); sendGGA(); }  // keep-alive GGA
}
// Peek telemetry to keep a recent lat/lon for the NTRIP GGA (VRS casters).
void tapTelemetry(uint8_t* buf, int n) {
  static char ln[160]; static int p = 0;
  for (int i = 0; i < n; i++) {
    char c = buf[i];
    if (c == '\n' || c == '\r') {
      ln[p] = 0;
      if (p > 3 && ln[0]=='T' && ln[1]==',') {
        // T,ms,fixQual,lat,lon,...
        char tmp[160]; strncpy(tmp, ln, sizeof(tmp)); tmp[sizeof(tmp)-1]=0;
        char* f[6]; int nf=0; f[nf++]=tmp;
        for (char* q=tmp; *q && nf<6; q++) if (*q==','){ *q=0; f[nf++]=q+1; }
        if (nf >= 5) { double la=atof(f[3]), lo=atof(f[4]); if (atoi(f[2])>=1 && la!=0) { lastLat=la; lastLon=lo; haveFix=true; } }
      }
      p = 0;
    } else if (p < (int)sizeof(ln)-1) ln[p++] = c;
  }
}
#else
// ---- SoftAP mode: local base relays $PSDG on :2101 -------------------------
const char*    AP_SSID   = "PiSat-RTK";
const char*    AP_PASS   = "pisat-rtk-1234";
const uint16_t RTCM_PORT = 2101;
WiFiServer rtcmServer(RTCM_PORT);
WiFiClient rtcmClient;
unsigned long bytesToGnss = 0;
#endif

void setup() {
  Serial.begin(115200);
  Link.begin(LINK_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  delay(50);
#if USE_NTRIP
  Serial.println("PiSat rover bridge (ESP32-C3) -- NTRIP mode");
  WiFi.mode(WIFI_STA);
  WiFi.begin(STA_SSID, STA_PASS);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis()-t0 < 15000) { delay(250); Serial.print('.'); }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("joined %s  ip %s\n", STA_SSID, WiFi.localIP().toString().c_str());
    if (MDNS.begin("pisat-rover")) Serial.println("mDNS: pisat-rover.local");
  } else Serial.println("WiFi join failed, will retry");
#else
  Serial.println("PiSat rover bridge (ESP32-C3) -- SoftAP mode");
  WiFi.mode(WIFI_AP);
  bool ok = WiFi.softAP(AP_SSID, AP_PASS);
  Serial.printf("SoftAP %s  ssid=%s  ip=%s\n", ok?"up":"FAILED", AP_SSID, WiFi.softAPIP().toString().c_str());
  rtcmServer.begin();
#endif
  teleServer.begin();
  cmdServer.begin();
  Serial.printf("telemetry :%u   command :%u\n", TELE_PORT, CMD_PORT);
}

void loop() {
#if USE_NTRIP
  serviceNtrip();                       // pull RTCM3 from the caster -> Link
#else
  { WiFiClient c = rtcmServer.available();
    if (c) { if (rtcmClient) rtcmClient.stop(); rtcmClient = c; Serial.println("base connected (RTCM)"); } }
  if (rtcmClient && rtcmClient.connected()) {
    uint8_t buf[256]; int n = rtcmClient.available();
    while (n > 0) { int m = rtcmClient.read(buf, min(n,(int)sizeof(buf))); if (m<=0) break;
                    Link.write(buf, m); bytesToGnss += m; lastRtcmMs = millis(); n -= m; }
  }
#endif

  // accept the Processing telemetry + command clients (replace on reconnect)
  { WiFiClient c = teleServer.available();
    if (c) { if (teleClient) teleClient.stop(); teleClient = c; Serial.println("telemetry client connected"); } }
  { WiFiClient c = cmdServer.available();
    if (c) { if (cmdClient) cmdClient.stop(); cmdClient = c; Serial.println("command client connected"); } }

  // commands (path + WASD) : Processing -> Link
  if (cmdClient && cmdClient.connected()) {
    uint8_t buf[256]; int n = cmdClient.available();
    while (n > 0) { int m = cmdClient.read(buf, min(n,(int)sizeof(buf))); if (m<=0) break;
                    Link.write(buf, m); bytesFromCmd += m; n -= m; }
  }

  // telemetry : Link -> Processing
  if (Link.available()) {
    uint8_t buf[256]; int n = 0;
    while (Link.available() && n < (int)sizeof(buf)) buf[n++] = Link.read();
    if (n > 0) {
      if (teleClient && teleClient.connected()) { teleClient.write(buf, n); bytesToTele += n; }
#if USE_NTRIP
      tapTelemetry(buf, n);             // keep lat/lon for the NTRIP GGA
#endif
    }
  }

  static uint32_t tLog = 0;
  if (millis() - tLog >= 3000) {
    tLog = millis();
    unsigned long age = lastRtcmMs ? (millis() - lastRtcmMs) : 999999;
#if USE_NTRIP
    Serial.printf("wifi=%d ntrip=%d | rtcm=%lu bytes (%lums)  ->tele=%lu cmd=%lu\n",
                  WiFi.status()==WL_CONNECTED, ntrip.connected(), rtcmBytes, age, bytesToTele, bytesFromCmd);
#else
    Serial.printf("clients: base=%d tele=%d cmd=%d | ->gnss=%lu ->tele=%lu cmd=%lu | rtcm %lums\n",
                  rtcmClient.connected(), teleClient.connected(), cmdClient.connected(),
                  bytesToGnss, bytesToTele, bytesFromCmd, age);
#endif
  }
}
