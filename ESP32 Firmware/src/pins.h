#pragma once
#include <cstdint>

static constexpr int PIN_VMON_ADC = 0;

static constexpr int PIN_DW_WAKEUP = 1;
static constexpr int PIN_DW_IRQ    = 3;
static constexpr int PIN_DW_MISO   = 4;
static constexpr int PIN_DW_MOSI   = 5;
static constexpr int PIN_DW_CLK    = 6;
static constexpr int PIN_DW_CS     = 7;
static constexpr int PIN_DW_RST    = 10;

static constexpr int PIN_LINK_RX = 20;
static constexpr int PIN_LINK_TX = 21;
static constexpr uint32_t LINK_BAUD = 115200;