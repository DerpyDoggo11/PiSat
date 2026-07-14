#include "gnss.h"
#include "pins.h"
#include <string.h>
#include <stdlib.h>

static GnssFix           s_fix;
static volatile uint32_t s_ppsCount = 0;
static volatile uint32_t s_ppsMs    = 0;

static char    s_line[100];
static uint8_t s_len = 0;

static void ppsIsr() {
    s_ppsCount++;
    s_ppsMs = millis();
}

static double nmeaToDegrees(const char* field, char hemi) {
    if (!field || !*field) return 0.0;
    double raw     = atof(field);
    int    degrees = (int)(raw / 100.0);
    double minutes = raw - degrees * 100.0;
    double out     = degrees + minutes / 60.0;
    if (hemi == 'S' || hemi == 'W') out = -out;
    return out;
}

static uint8_t split(char* s, char* out[], uint8_t maxFields) {
    uint8_t n = 0;
    out[n++] = s;
    for (char* p = s; *p && n < maxFields; p++) {
        if (*p == ',') { *p = '\0'; out[n++] = p + 1; }
        else if (*p == '*') { *p = '\0'; break; }
    }
    return n;
}

static bool checksumOk(const char* s) {
    const char* star = strchr(s, '*');
    if (!star || strlen(star) < 3) return false;
    uint8_t sum = 0;
    for (const char* p = s + 1; p < star; p++) sum ^= (uint8_t)*p;
    return sum == (uint8_t)strtol(star + 1, nullptr, 16);
}

static void parseSentence(char* s) {
    char* f[20];
    uint8_t n = split(s, f, 20);
    if (n < 2) return;

    const char* type = f[0] + 3;

    if (!strncmp(type, "GGA", 3) && n >= 10) {
        s_fix.fixQuality = (uint8_t)atoi(f[6]);
        s_fix.satellites = (uint8_t)atoi(f[7]);
        s_fix.hdop       = (float)atof(f[8]);
        s_fix.altitude_m = (float)atof(f[9]);

        if (s_fix.fixQuality > 0 && *f[2]) {
            s_fix.latitude  = nmeaToDegrees(f[2], f[3][0]);
            s_fix.longitude = nmeaToDegrees(f[4], f[5][0]);
            s_fix.valid     = true;
            s_fix.lastFixMs = millis();
        } else {
            s_fix.valid = false;
        }

        if (strlen(f[1]) >= 6) {
            s_fix.hour   = (uint8_t)((f[1][0] - '0') * 10 + (f[1][1] - '0'));
            s_fix.minute = (uint8_t)((f[1][2] - '0') * 10 + (f[1][3] - '0'));
            s_fix.second = (uint8_t)((f[1][4] - '0') * 10 + (f[1][5] - '0'));
        }
    }
    else if (!strncmp(type, "RMC", 3) && n >= 9) {
        s_fix.speed_mps  = (float)atof(f[7]) * 0.514444f;
        s_fix.course_deg = (float)atof(f[8]);
        if (f[2][0] != 'A') s_fix.valid = false;
    }
}

void Gnss::begin() {
    pinMode(PIN_GNSS_NRESET, OUTPUT);
    digitalWrite(PIN_GNSS_NRESET, HIGH);

    pinMode(PIN_GNSS_WAKEUP, OUTPUT);
    digitalWrite(PIN_GNSS_WAKEUP, HIGH);

    pinMode(PIN_GNSS_ANTOFF, OUTPUT);
    digitalWrite(PIN_GNSS_ANTOFF, LOW); 

    pinMode(PIN_GNSS_PPS, INPUT); 
    attachInterrupt(digitalPinToInterrupt(PIN_GNSS_PPS), ppsIsr, RISING);

    Serial2.setTX(PIN_GNSS_TX);
    Serial2.setRX(PIN_GNSS_RX);
    Serial2.setFIFOSize(256);
    Serial2.begin(GNSS_BAUD);

    reset();
}

void Gnss::reset() {
    digitalWrite(PIN_GNSS_NRESET, LOW);
    delay(10);
    digitalWrite(PIN_GNSS_NRESET, HIGH);
    delay(100);
    s_len = 0;
    s_fix = GnssFix();
}

void Gnss::antennaPower(bool on) {
    digitalWrite(PIN_GNSS_ANTOFF, on ? LOW : HIGH);
}

void Gnss::wake() {
    digitalWrite(PIN_GNSS_WAKEUP, HIGH);
}

bool Gnss::poll() {
    bool completed = false;
    while (Serial2.available()) {
        char c = (char)Serial2.read();

        if (c == '$') { s_len = 0; s_line[s_len++] = c; continue; }
        if (s_len == 0) continue;

        if (c == '\r' || c == '\n') {
            s_line[s_len] = '\0';
            if (s_len > 6 && checksumOk(s_line)) {
                parseSentence(s_line);
                completed = true;
            }
            s_len = 0;
        } else if (s_len < sizeof(s_line) - 1) {
            s_line[s_len++] = c;
        } else {
            s_len = 0;                          
        }
    }
    return completed;
}

const GnssFix& Gnss::fix() { return s_fix; }
uint32_t Gnss::ppsCount() { return s_ppsCount; }
uint32_t Gnss::lastPpsMs() { return s_ppsMs; }

bool Gnss::hasFix() {
    return s_fix.valid && (millis() - s_fix.lastFixMs) < 3000;
}

void Gnss::sendCommand(const char* body) {
    uint8_t sum = 0;
    for (const char* p = body; *p; p++) sum ^= (uint8_t)*p;
    Serial2.printf("$%s*%02X\r\n", body, sum);
}