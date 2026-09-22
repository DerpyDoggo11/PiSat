#pragma once
#include <Arduino.h>

struct GnssFix {
    bool valid  = false;
    double latitude = 0.0;  
    double longitude = 0.0;
    float altitude_m = 0.0f; 
    float speed_mps = 0.0f;
    float course_deg = 0.0f;
    uint8_t satellites = 0;
    float hdop = 99.9f;
    uint8_t fixQuality = 0; 
    uint8_t hour = 0, minute = 0, second = 0;
    uint32_t lastFixMs = 0;
};

namespace Gnss {
    void begin();
    void reset();
    void antennaPower(bool on);
    void wake();

    bool poll();

    void setRawDebug(bool on);   // echo raw NMEA bytes to USB Serial
    bool rawDebug();

    // Feed one already-read byte into the NMEA parser (for the BASE, which reads
    // the UART itself to frame RTCM and can't also call poll()).
    bool feedByte(uint8_t b);

    // Print a human-friendly status to USB Serial: fix quality, satellite count,
    // HDOP, 1PPS count, and -- before a fix -- how close it is / what to check.
    void printStatus();

    const GnssFix& fix();
    uint32_t ppsCount();
    uint32_t lastPpsMs();
    bool hasFix();

    void sendCommand(const char* body);

    // --- RTK support -------------------------------------------------------
    // Inject raw correction bytes (RTCM3) straight into the GNSS UART. Used on
    // the ROVER to feed base-station corrections received over WiFi into the
    // Teseo III so it can compute a differential / RTK solution.
    void write(const uint8_t* data, size_t len);

    // Non-blocking access to the raw GNSS UART stream, so the BASE role can
    // frame RTCM3 messages coming out of the receiver without the NMEA parser
    // in poll() eating the bytes first. Returns -1 when no byte is waiting.
    int  readRaw();
    int  availableRaw();

    // Configure the receiver as an RTK BASE: put it into a mode where it emits
    // RTCM3 corrections (and/or raw measurements) on its UART. These are ST
    // Teseo III $PSTM proprietary commands -- confirm the exact parameter IDs
    // against your module's "Teseo III Software Command Reference" before
    // relying on them; the transport around them is correct regardless.
    void configureBase();
    // Configure the receiver as an RTK ROVER: make sure it will accept RTCM3
    // input on the UART and report the differential/RTK fix quality in GGA.
    void configureRover();

    // Enable/disable ST Teseo raw-measurement output (carrier phase +
    // pseudorange), which a host RTK engine (RTKLIB on a Pi Zero 2 W) consumes.
    // Uses $PSTMSETPAR,1228 (PSTMPVRAW) per ST UM2229 -- VERIFY against your
    // module's firmware; it may require the measurement-engine (ME) build.
    void rawMeasurements(bool on);
}