#pragma once
#include <Arduino.h>
#include "pins.h"

namespace EspLink {

static constexpr uint32_t LINK_BAUD = 115200;

void begin(); 

void reset(); 
void enterBootloader(); 
void hold();  

[[noreturn]] void passthrough();

void pollForBridgeRequest();

}