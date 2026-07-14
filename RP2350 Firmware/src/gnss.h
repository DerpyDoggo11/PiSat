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

    const GnssFix& fix();
    uint32_t ppsCount();
    uint32_t lastPpsMs();
    bool hasFix(); 

    void sendCommand(const char* body);
}