
#include <Arduino.h>
#include "pins.h"
#include "gnss.h"
#include "esc.h"
#include "hmi.h"
#include "esplink.h"
#include <SerialPIO.h>
#include <math.h>

// Which pins carry the link to the external ESP32-C3:
//   1 (default) = the onboard "ESP32" header = RP2350 hardware UART Serial1
//                 (GP0 = TX, GP1 = RX). Cleaner, and frees GP12/GP13. Use this
//                 because the onboard ESP32-C3 is not populated.
//   0           = the J6 breakout GP12 (TX) / GP13 (RX) via a PIO soft-UART.
#ifndef PISAT_LINK_HW_UART
#define PISAT_LINK_HW_UART 1
#endif

// ============================================================================
//  PiSat RP2350 firmware -- three build roles (selected in platformio.ini):
//
//   [env:gnss]          ROVER  (default) -- the rover's brain. Parses GNSS,
//                              feeds in RTCM3 corrections (RTK), follows a GPS
//                              waypoint path, and DRIVES TWO ZS-X111H brushless
//                              motor controllers wired to the PiSat breakout.
//                              Streams telemetry text out to the rover's ESP32-C3,
//                              which is just a transparent WiFi bridge.
//   [env:base]          BASE   (-DPISAT_BASE) -- puts the GNSS in base mode and
//                              streams the RTCM3 it produces out to the small
//                              ESP32-C3 super-mini, which broadcasts it on WiFi.
//   [env:esp_passthrough] flashes the onboard ESP32-C3 through this board.
//
//  Link to the external ESP32-C3 (115200 baud). By default it uses the onboard
//  "ESP32" header = RP2350 hardware UART Serial1 (GP0 = TX, GP1 = RX), which is
//  free because the onboard C3 is not populated. Set -DPISAT_LINK_HW_UART=0 to
//  use the J6 breakout GP12 (TX) / GP13 (RX) soft-UART instead.
//    TX carries telemetry text out on the rover / RTCM out on the base.
//    RX carries RTCM corrections in (ROVER only).
//    Always share a common GND with the external ESP32-C3.
// ============================================================================

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

// ============================================================================
#elif defined(PISAT_BASE)
// ---------------------------------------------------------------------------
//  BASE STATION
//  The Teseo III is configured to emit RTCM3. We frame those RTCM3 messages
//  off the GNSS UART and forward each complete frame to the ESP32-C3 super
//  mini on GP12, which pushes them over WiFi to the rover.
// ---------------------------------------------------------------------------
static const uint32_t ESP_LINK_BAUD = 115200;
#if PISAT_LINK_HW_UART
static Stream& EspOut = Serial1;                       // GP0 TX ("ESP32" header)
#else
static SerialPIO EspOutPio(PIN_J6[0] /* GP12 TX */, NOPIN);
static Stream& EspOut = EspOutPio;
#endif

// RTCM3 framer: 0xD3, 2-byte header (10-bit length), payload, 3-byte CRC24Q.
static uint8_t  rtcm[1029];
static uint16_t rtcmIdx = 0;
static uint16_t rtcmLen = 0;      // full frame length once known
static uint32_t framesOut = 0;

static void feedRtcmByte(uint8_t b) {
    if (rtcmIdx == 0) {
        if (b != 0xD3) return;              // hunt for preamble
        rtcm[rtcmIdx++] = b;
    } else if (rtcmIdx < 3) {
        rtcm[rtcmIdx++] = b;
        if (rtcmIdx == 3) {
            uint16_t payload = ((rtcm[1] & 0x03) << 8) | rtcm[2];
            if (payload > 1023) { rtcmIdx = 0; return; }   // bad -> resync
            rtcmLen = payload + 6;          // 3 header + payload + 3 CRC
        }
    } else {
        rtcm[rtcmIdx++] = b;
        if (rtcmIdx >= rtcmLen) {
            EspOut.write(rtcm, rtcmLen);    // whole frame -> WiFi bridge
            framesOut++;
            rtcmIdx = 0;
        }
    }
}

void setup() {
    Serial.begin(115200);
    Hmi::begin();
    Power::begin();
    Gnss::begin();
    Gnss::configureBase();          // survey-in + RTCM3 output (see gnss.cpp)
#if PISAT_LINK_HW_UART
    Serial1.setTX(0); Serial1.setRX(1); Serial1.setFIFOSize(512); Serial1.begin(ESP_LINK_BAUD);
#else
    EspOutPio.begin(ESP_LINK_BAUD);
#endif

    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 2000) { }
    Serial.println("=== PiSat RTK BASE (RP2350A) ===");
    Serial.println("Configuring Teseo III for RTCM3 base output...");
#if PISAT_LINK_HW_UART
    Serial.println("Streaming RTCM3 to ESP32-C3 on the ESP32 header (GP0).");
#else
    Serial.println("Streaming RTCM3 to ESP32-C3 on J6 GP12.");
#endif
    Serial.println("Type 'help' for commands ('gps' for fix/sat status).");
    Hmi::beep(60);
}

void loop() {
    // Read each GNSS byte once: frame RTCM out AND parse NMEA for debug/status.
    int b;
    while ((b = Gnss::readRaw()) >= 0) {
        feedRtcmByte((uint8_t)b);
        Gnss::feedByte((uint8_t)b);
    }

    static char line[32]; static uint8_t n = 0;
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (n) {
                line[n] = 0;
                if (!strcmp(line, "help"))
                    Serial.println("gps | sats | stat | nmea | gnss_reset");
                else if (!strcmp(line, "gps") || !strcmp(line, "sats"))
                    Gnss::printStatus();
                else if (!strcmp(line, "stat")) {
                    const GnssFix& gf = Gnss::fix();
                    Serial.printf("rtcm frames %lu  vbat %.2fV  sats %u  fix q%u  1pps %lu\n",
                                  (unsigned long)framesOut, Power::batteryVolts(),
                                  gf.satellites, gf.fixQuality, (unsigned long)Gnss::ppsCount());
                }
                else if (!strcmp(line, "nmea")) {
                    bool on = !Gnss::rawDebug();
                    Gnss::setRawDebug(on);
                    Serial.printf("raw NMEA echo %s\n", on ? "ON" : "OFF");
                }
                else if (!strcmp(line, "gnss_reset")) { Gnss::reset(); Gnss::configureBase(); Serial.println("gnss re-configured"); }
                n = 0;
            }
        } else if (n < sizeof(line) - 1) line[n++] = c;
    }

    static uint32_t tLed = 0, lastFrames = 0;
    if (millis() - tLed >= 500) {
        tLed = millis();
        if (framesOut != lastFrames) { Hmi::ledA(true); lastFrames = framesOut; }
        else Hmi::blink(PIN_LED_A, 500);       // no RTCM yet -> blink
        if (Power::batteryVolts() < Power::VBAT_WARN) Hmi::blink(PIN_LED_B, 125);
    }
}

void setup1() {}
void loop1() {}

// ============================================================================
#else
// ---------------------------------------------------------------------------
//  ROVER  --  GNSS + RTK + pathfinding + two ZS-X111H motor controllers.
//
//  The motors are wired to the PiSat's own breakout GPIO (below); all steering
//  decisions run here. The rover's external ESP32-C3 is a transparent WiFi
//  bridge: it pushes RTCM corrections (from the base, over WiFi) into GP13, and
//  relays our telemetry text from GP12 out to the Processing client.
// ---------------------------------------------------------------------------

// The PiSat's ONBOARD ESP32-C3 is NOT populated: the WiFi RTK link runs on the
// EXTERNAL ESP32-C3 super mini wired to the J6 breakout. So we skip the onboard
// UART bridge (EspLink). Set to 1 only if you actually solder the onboard C3.
#ifndef PISAT_ONBOARD_ESP
#define PISAT_ONBOARD_ESP 0
#endif

// ---- external C3 bridge link -----------------------------------------------
// Default: hardware UART Serial1 on the onboard "ESP32" header (GP0 TX, GP1 RX).
// Set -DPISAT_LINK_HW_UART=0 to use the J6 soft-UART on GP12/GP13 instead.
static const uint32_t ESP_LINK_BAUD = 115200;
#if PISAT_LINK_HW_UART
static Stream& link = Serial1;
#else
static SerialPIO EspLinkIO(PIN_J6[0] /* GP12 TX telemetry out */,
                          PIN_J6[1] /* GP13 RX RTCM in */);
static Stream& link = EspLinkIO;
#endif

// ---- ZS-X111H motor controllers, wired to the PiSat breakout ---------------
// Four wires per wheel: P (PWM speed, out), ZF (rotation/direction, out),
// BRAKE (out), M (speed-feedback pulse, in). The C3 link now lives on the
// "ESP32" header (GP0/GP1), so J6 GP12/GP13 are free for the LEFT wheel.
// RIGHT wheel (ZS-X111H #1)
static const uint8_t M_PWM_R   = 20;   // J5 -> P     (PWM speed)         GPIO 20
static const uint8_t M_SPEED_R = 21;   // J5 <- M     (speed feedback)    GPIO 21
static const uint8_t M_BRAKE_R = 22;   // J5 -> BRAKE                     GPIO 22
static const uint8_t M_DIR_R   = 23;   // J5 -> ZF    (rotation)          GPIO 23
// LEFT wheel (ZS-X111H #2)
static const uint8_t M_PWM_L   = 15;   // J6 -> P     (PWM speed)         GPIO 15
static const uint8_t M_SPEED_L = 19;   // J6 <- M     (speed feedback)    GPIO 19
static const uint8_t M_BRAKE_L = 13;   // J6 -> BRAKE                     GPIO 13
static const uint8_t M_DIR_L   = 12;   // J6 -> ZF    (rotation)          GPIO 12

// LEFT brake/rotation use GP12/GP13, free only because the link is on the
// hardware UART. If you switch the link back to the J6 soft-UART, they clash.
#if !PISAT_LINK_HW_UART
#error "LEFT motor uses GP12/GP13, which the soft-UART link (PISAT_LINK_HW_UART=0) also needs. Keep the HW-UART link, or move M_BRAKE_L/M_DIR_L to other free pins (e.g. GP14/GP24)."
#endif

// DIR level that makes each wheel drive the robot FORWARD. Flip a constant here
// (rather than swapping motor phase wires) if a wheel spins the wrong way.
static const int DIR_R_FORWARD = HIGH;
static const int DIR_L_FORWARD = HIGH;
static const int BRAKE_ON  = HIGH;     // ZS-X111H brake engages on high
static const int BRAKE_OFF = LOW;

// Wheel-speed feedback: the ZS-X111H M pin pulses as the wheel turns. We count
// edges and report the pulse frequency (Hz) in telemetry. Divide by your
// motor's pulses-per-revolution to get RPM.
static volatile uint32_t s_pulsesR = 0, s_pulsesL = 0;
static float g_spdHzR = 0, g_spdHzL = 0;
static void isrSpeedR() { s_pulsesR++; }
static void isrSpeedL() { s_pulsesL++; }

// ---- PATH  ---  EDIT THIS for your field -----------------------------------
// Waypoints are real GPS coordinates {lat, lon} in degrees, driven in order
// (>= 2 needed). BOUNDARY is an optional safety polygon; < 3 points disables it.
struct LatLon { double lat; double lon; };

static const LatLon WAYPOINTS[] = {
    // { 40.000000, -75.000000 },
    // { 40.000100, -75.000000 },
    // { 40.000100, -74.999900 },
};
static const int WAYPOINT_COUNT = sizeof(WAYPOINTS) / sizeof(WAYPOINTS[0]);

static const LatLon BOUNDARY[] = {
    // { 40.000050, -75.000050 },
    // { 40.000050, -74.999850 },
    // { 40.000150, -74.999850 },
    // { 40.000150, -75.000050 },
};
static const int BOUNDARY_COUNT = sizeof(BOUNDARY) / sizeof(BOUNDARY[0]);

// ---- TUNING ----------------------------------------------------------------
static float MAX_PWM = 40;                        // 0..255, base wheel speed
static const float STRAIGHT_DEG = 5.0f;           // below this |err| = straight
static const float PIVOT_DEG    = 60.0f;          // above this |err| = pivot
static const float TURN_FACTOR  = 0.5f;           // inside-wheel speed multiplier
static const float LOOKAHEAD_FRAC = 0.2f;         // aim this far along the segment
static const float WAYPOINT_RADIUS_M = 1.5f;      // waypoint reached within (m)
static const float MIN_SPEED_FOR_COURSE = 0.4f;   // m/s below which course is bad

// ---- STATE -----------------------------------------------------------------
static float currentCourse = 0, currentSpeed = 0;
static float targetBearing = 0, headingError = 0;
static float lastPwmR = 0, lastPwmL = 0;
static int   currentSegmentIndex = 0;
static bool  pathCompleted = false;
static bool  hadFix = false;
static bool  g_hold = false;        // manual safety hold (brakes motors)
static float g_vbat = 0.0f;
static uint32_t lastRtcmMs = 0;

// local metre frame (equirectangular, origin = first valid fix)
static bool   originSet = false;
static double originLat = 0, originLon = 0;
static double mPerDegLat = 111320.0, mPerDegLon = 111320.0;

struct Vec2 { double e; double n; };

static void setOrigin(double lat, double lon) {
    originLat = lat; originLon = lon;
    mPerDegLat = 111320.0;
    mPerDegLon = 111320.0 * cos(lat * M_PI / 180.0);
    originSet = true;
}
static Vec2 toLocal(double lat, double lon) {
    return { (lon - originLon) * mPerDegLon, (lat - originLat) * mPerDegLat };
}

// ---- UTILS -----------------------------------------------------------------
static float normalizeAngle(float a) {          // -> [-180, 180)
    a = fmodf(a + 180.0f, 360.0f);
    if (a < 0) a += 360.0f;
    return a - 180.0f;
}
static float bearingBetween(const Vec2& a, const Vec2& b) {
    float deg = (float)(atan2(b.e - a.e, b.n - a.n) * 180.0 / M_PI);
    if (deg < 0) deg += 360.0f;
    return deg;
}
static double dist2(const Vec2& a, const Vec2& b) {
    double dE = b.e - a.e, dN = b.n - a.n;
    return dE * dE + dN * dN;
}
static bool isInsidePolygon(const Vec2& p, const LatLon poly[], int count) {
    bool inside = false;
    for (int i = 0, j = count - 1; i < count; j = i++) {
        Vec2 pi = toLocal(poly[i].lat, poly[i].lon);
        Vec2 pj = toLocal(poly[j].lat, poly[j].lon);
        if (((pi.n > p.n) != (pj.n > p.n)) &&
            (p.e < (pj.e - pi.e) * (p.n - pi.n) / (pj.n - pi.n) + pi.e))
            inside = !inside;
    }
    return inside;
}

// ---- MOTORS  (two ZS-X111H: PWM speed + DIR + BRAKE per wheel) --------------
static void motorsInit() {
    analogWriteFreq(1000);      // 1 kHz PWM, inside the ZS-X111H 50Hz-20kHz window
    analogWriteRange(255);
    pinMode(M_DIR_R, OUTPUT);   pinMode(M_BRAKE_R, OUTPUT);
    pinMode(M_DIR_L, OUTPUT);   pinMode(M_BRAKE_L, OUTPUT);
    digitalWrite(M_DIR_R, DIR_R_FORWARD);
    digitalWrite(M_DIR_L, DIR_L_FORWARD);
    digitalWrite(M_BRAKE_R, BRAKE_OFF);
    digitalWrite(M_BRAKE_L, BRAKE_OFF);
    analogWrite(M_PWM_R, 0);
    analogWrite(M_PWM_L, 0);

    // Speed-feedback inputs (M pin is typically open-collector -> pull-up).
    pinMode(M_SPEED_R, INPUT_PULLUP);
    pinMode(M_SPEED_L, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(M_SPEED_R), isrSpeedR, FALLING);
    attachInterrupt(digitalPinToInterrupt(M_SPEED_L), isrSpeedL, FALLING);
}

// Convert the pulse counts to a frequency (Hz) over a ~250 ms window.
static void updateWheelSpeeds() {
    static uint32_t prevMs = 0, prevR = 0, prevL = 0;
    uint32_t now = millis(), dt = now - prevMs;
    if (dt >= 250) {
        uint32_t cR = s_pulsesR, cL = s_pulsesL;
        g_spdHzR = (cR - prevR) * 1000.0f / dt;
        g_spdHzL = (cL - prevL) * 1000.0f / dt;
        prevR = cR; prevL = cL; prevMs = now;
    }
}
static void setWheel(uint8_t pwmPin, uint8_t brakePin, float pwm) {
    digitalWrite(brakePin, BRAKE_OFF);
    analogWrite(pwmPin, (int)constrain(pwm, 0.0f, 255.0f));
}
static void motorsStop() {              // coast: PWM 0, brakes released
    lastPwmR = lastPwmL = 0;
    analogWrite(M_PWM_R, 0);  analogWrite(M_PWM_L, 0);
    digitalWrite(M_BRAKE_R, BRAKE_OFF);  digitalWrite(M_BRAKE_L, BRAKE_OFF);
}
static void motorsBrake() {             // hard stop: PWM 0, brakes engaged
    lastPwmR = lastPwmL = 0;
    analogWrite(M_PWM_R, 0);  analogWrite(M_PWM_L, 0);
    digitalWrite(M_BRAKE_R, BRAKE_ON);   digitalWrite(M_BRAKE_L, BRAKE_ON);
}
static void driveTowardHeading(float bearingTarget, bool courseReliable) {
    float pwmRight = MAX_PWM, pwmLeft = MAX_PWM;
    if (!courseReliable) {
        setWheel(M_PWM_R, M_BRAKE_R, pwmRight);
        setWheel(M_PWM_L, M_BRAKE_L, pwmLeft);
        headingError = 0;
        lastPwmR = pwmRight; lastPwmL = pwmLeft;
        return;
    }
    headingError = normalizeAngle(bearingTarget - currentCourse);
    float mag = fabsf(headingError);
    float inside = MAX_PWM * TURN_FACTOR;
    if (mag >= PIVOT_DEG) inside = 0;
    if (mag < STRAIGHT_DEG)      { /* straight */ }
    else if (headingError > 0)   pwmRight = inside;
    else                         pwmLeft  = inside;
    setWheel(M_PWM_R, M_BRAKE_R, pwmRight);
    setWheel(M_PWM_L, M_BRAKE_L, pwmLeft);
    lastPwmR = pwmRight; lastPwmL = pwmLeft;
}

// ---- PATHFINDING -----------------------------------------------------------
static void pathfind() {
    const GnssFix& f = Gnss::fix();
    if (g_hold)             { motorsBrake(); return; }
    if (WAYPOINT_COUNT < 2) { motorsStop();  return; }
    if (pathCompleted)      { motorsBrake(); return; }
    if (!Gnss::hasFix())    { motorsStop();  return; }

    Vec2 pos = toLocal(f.latitude, f.longitude);

    if (BOUNDARY_COUNT >= 3 && !isInsidePolygon(pos, BOUNDARY, BOUNDARY_COUNT)) {
        Serial.println("LEFT boundary zone -> emergency stop");
        motorsBrake(); Hmi::beep(300); pathCompleted = true; return;
    }

    Vec2 finalWp = toLocal(WAYPOINTS[WAYPOINT_COUNT - 1].lat,
                           WAYPOINTS[WAYPOINT_COUNT - 1].lon);
    if (currentSegmentIndex >= WAYPOINT_COUNT - 1 ||
        dist2(pos, finalWp) < WAYPOINT_RADIUS_M * WAYPOINT_RADIUS_M) {
        Serial.println("Final waypoint reached -> path complete");
        motorsBrake(); Hmi::beep(120); pathCompleted = true; return;
    }

    Vec2 curr = toLocal(WAYPOINTS[currentSegmentIndex].lat,
                        WAYPOINTS[currentSegmentIndex].lon);
    Vec2 next = toLocal(WAYPOINTS[currentSegmentIndex + 1].lat,
                        WAYPOINTS[currentSegmentIndex + 1].lon);
    Vec2 seg = { next.e - curr.e, next.n - curr.n };
    double segLen2 = seg.e * seg.e + seg.n * seg.n;

    double t = 0;
    if (segLen2 > 1e-9)
        t = ((pos.e - curr.e) * seg.e + (pos.n - curr.n) * seg.n) / segLen2;

    if (t > 1.0 || dist2(pos, next) < WAYPOINT_RADIUS_M * WAYPOINT_RADIUS_M) {
        currentSegmentIndex++;
        Serial.printf("Advancing to segment %d\n", currentSegmentIndex);
        return;
    }

    double tl = t + LOOKAHEAD_FRAC;
    if (tl < 0) tl = 0; if (tl > 1) tl = 1;
    Vec2 lookahead = { curr.e + seg.e * tl, curr.n + seg.n * tl };
    targetBearing = bearingBetween(pos, lookahead);
    driveTowardHeading(targetBearing, currentSpeed >= MIN_SPEED_FOR_COURSE);
}

// ---- RTCM in: bridge -> GP13 -> GNSS ---------------------------------------
static void pumpRtcmToGnss() {
    uint8_t buf[64]; size_t n = 0;
    while (link.available() && n < sizeof(buf)) buf[n++] = (uint8_t)link.read();
    if (n) { Gnss::write(buf, n); lastRtcmMs = millis(); }
}

// ---- TELEMETRY out: text -> GP12 -> C3 bridge -> Processing -----------------
// One-time-ish header (path/boundary/origin) resent periodically so a late
// Processing client still gets it; then live T lines at 10 Hz.
static void sendPathHeader() {
    link.printf("M,%.0f,%.2f,%d,%d\n", MAX_PWM, WAYPOINT_RADIUS_M,
                WAYPOINT_COUNT, BOUNDARY_COUNT);
    for (int i = 0; i < WAYPOINT_COUNT; i++)
        link.printf("W,%d,%.7f,%.7f\n", i, WAYPOINTS[i].lat, WAYPOINTS[i].lon);
    for (int i = 0; i < BOUNDARY_COUNT; i++)
        link.printf("B,%d,%.7f,%.7f\n", i, BOUNDARY[i].lat, BOUNDARY[i].lon);
    if (originSet) link.printf("O,%.7f,%.7f\n", originLat, originLon);
}
static void sendTelemetry() {
    const GnssFix& f = Gnss::fix();
    uint8_t fq = Gnss::hasFix() ? f.fixQuality : 0;
    unsigned long age = lastRtcmMs ? (millis() - lastRtcmMs) : 999999;
    // ...,<rtcmAgeMs>,<pwmR>,<pwmL>,<spdHzR>,<spdHzL>
    link.printf(
        "T,%lu,%u,%.7f,%.7f,%.1f,%.2f,%u,%.1f,%d,%.1f,%.1f,%d,%lu,%.0f,%.0f,%.1f,%.1f\n",
        (unsigned long)millis(), fq, f.latitude, f.longitude,
        currentCourse, currentSpeed, f.satellites, f.hdop,
        currentSegmentIndex, targetBearing, headingError, pathCompleted ? 1 : 0,
        age, lastPwmR, lastPwmL, g_spdHzR, g_spdHzL);
}

// ---- USB console -----------------------------------------------------------
static void handleCommand(char* cmd) {
    const GnssFix& f = Gnss::fix();
    if (!strcmp(cmd, "help")) {
        Serial.println("gps | sats | aux | stat | hold | gnss_reset | nmea | ant on|off");
    }
    else if (!strcmp(cmd, "hold")) {
        g_hold = !g_hold;
        Serial.printf("motor HOLD %s\n", g_hold ? "ON (braked)" : "OFF");
    }
    else if (!strcmp(cmd, "gps") || !strcmp(cmd, "sats")) {
        Gnss::printStatus();
    }
    else if (!strcmp(cmd, "aux"))
        Serial.printf("J4: A1 %.3fV  A2 %.3fV  A3 %.3fV\n",
                      Power::readAux(0), Power::readAux(1), Power::readAux(2));
    else if (!strcmp(cmd, "stat"))
        Serial.printf("vbat %.2f V  usb %d  hold %d  seg %d  fix %d q%u  rtcm %lums  "
                      "pwm R%.0f/L%.0f  spd R%.1f/L%.1f Hz\n",
                      g_vbat, (int)Power::usbPresent(), (int)g_hold,
                      currentSegmentIndex, (int)Gnss::hasFix(), f.fixQuality,
                      lastRtcmMs ? (unsigned long)(millis() - lastRtcmMs) : 999999UL,
                      lastPwmR, lastPwmL, g_spdHzR, g_spdHzL);
    else if (!strcmp(cmd, "gnss_reset")) { Gnss::reset(); Gnss::configureRover(); Serial.println("gnss reset"); }
    else if (!strcmp(cmd, "nmea")) {
        bool on = !Gnss::rawDebug();
        Gnss::setRawDebug(on);
        Serial.printf("raw NMEA echo %s\n", on ? "ON" : "OFF");
    }
    else if (!strcmp(cmd, "ant on"))  { Gnss::antennaPower(true);  Serial.println("ant on"); }
    else if (!strcmp(cmd, "ant off")) { Gnss::antennaPower(false); Serial.println("ant off"); }
#if PISAT_ONBOARD_ESP
    else if (!strcmp(cmd, "esp_reset")) { EspLink::reset(); Serial.println("esp reset"); }
    else if (!strcmp(cmd, "esp_bridge")) { Serial.flush(); EspLink::passthrough(); }
#else
    else if (!strncmp(cmd, "esp_", 4))
        Serial.println("onboard ESP32-C3 not populated (RTK uses the external C3)");
#endif
}

// ---- SETUP / LOOP ----------------------------------------------------------
void setup() {
    Serial.begin(115200);
    Hmi::begin();
    Power::begin();
#if PISAT_ONBOARD_ESP
    EspLink::begin();
#endif
    Gnss::begin();
    Gnss::configureRover();     // accept RTCM3 in, report RTK quality in GGA

    motorsInit();   // owns GP12/13/15/19/20/21/22/23 (both ZS-X111H)
    motorsStop();

#if PISAT_LINK_HW_UART
    Serial1.setTX(0); Serial1.setRX(1); Serial1.setFIFOSize(512); Serial1.begin(ESP_LINK_BAUD);
#else
    EspLinkIO.begin(ESP_LINK_BAUD);   // GP12 TX / GP13 RX soft-UART
#endif

    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 2000) { }
    Serial.println("=== PiSat RTK ROVER (RP2350A) ===");
    Serial.printf("VBAT: %.2f V\n", Power::batteryVolts());
    if (WAYPOINT_COUNT < 2)
        Serial.println("WARNING: no path set. Edit WAYPOINTS[] before driving.");
    Serial.println("Type 'help' for commands, 'hold' to brake/release motors.");
    Hmi::beep(60);
}

void loop() {
    Gnss::poll();
    pumpRtcmToGnss();

    static char line[64]; static uint8_t n = 0;
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') { if (n) { line[n] = 0; handleCommand(line); n = 0; } }
        else if (n < sizeof(line) - 1) line[n++] = c;
    }

    // capture course/speed and set the local-frame origin on first fix
    const GnssFix& f = Gnss::fix();
    if (Gnss::hasFix()) {
        currentCourse = f.course_deg;
        currentSpeed  = f.speed_mps;
        if (!originSet) setOrigin(f.latitude, f.longitude);
    }
    if (!hadFix && Gnss::hasFix()) { hadFix = true; Serial.println("GPS fix acquired"); Hmi::beep(120); }

    updateWheelSpeeds();

    static uint32_t tLast = 0, tHdr = 0;
    if (millis() - tLast >= 100) {          // 10 Hz control + telemetry
        tLast = millis();
        g_vbat = Power::batteryVolts();

        pathfind();

        if (millis() - tHdr >= 2000) { tHdr = millis(); sendPathHeader(); }
        sendTelemetry();

        if (Gnss::hasFix()) Hmi::ledA(true); else Hmi::blink(PIN_LED_A, 500);
        if (g_vbat < Power::VBAT_WARN) Hmi::blink(PIN_LED_B, 125); else Hmi::ledB(!g_hold);

        if (g_vbat < Power::VBAT_CRITICAL && !g_hold) {
            g_hold = true;
            motorsBrake();
            Hmi::beep(400);
            Serial.println("!! low voltage cutoff - motors held");
        }
    }
}

void setup1() {}
void loop1() {}

#endif
