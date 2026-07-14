#include <Arduino.h>
#include <SPI.h>
#include "pins.h"

static SPIClass dwSpi(FSPI);

static void dwHardReset() {
  pinMode(PIN_DW_RST, OUTPUT);
  digitalWrite(PIN_DW_RST, LOW);
  delay(2);
  pinMode(PIN_DW_RST, INPUT);
  delay(5);
}

static bool dwBegin() {
  pinMode(PIN_DW_CS, OUTPUT);
  digitalWrite(PIN_DW_CS, HIGH);
  pinMode(PIN_DW_WAKEUP, OUTPUT);
  digitalWrite(PIN_DW_WAKEUP, LOW);
  pinMode(PIN_DW_IRQ, INPUT);

  dwSpi.begin(PIN_DW_CLK, PIN_DW_MISO, PIN_DW_MOSI, PIN_DW_CS);
  dwHardReset();

  uint8_t hdr[2] = {0x00, 0x00};
  uint8_t id[4]  = {0};
  dwSpi.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_DW_CS, LOW);
  dwSpi.transfer(hdr, 2);
  dwSpi.transfer(id, 4);
  digitalWrite(PIN_DW_CS, HIGH);
  dwSpi.endTransaction();

  uint32_t devid = (uint32_t)id[3] << 24 | (uint32_t)id[2] << 16 | (uint32_t)id[1] << 8 | id[0];
  Serial0.printf("S,UWB devid=0x%08lX\n", (unsigned long)devid);
  return devid != 0x00000000 && devid != 0xFFFFFFFF;
}

void setup() {
  Serial0.begin(LINK_BAUD, SERIAL_8N1, PIN_LINK_RX, PIN_LINK_TX);
  delay(50);
  Serial0.println("S,esp32-c3 boot");

  analogReadResolution(12);

}


void loop() {--
    static char line[64];
    static uint8_t n = 0;
 
    static uint32_t t = 0;
    if (millis() - t >= 100) {
        t = millis();
 
        float x = 0, y = 0, z = 0;
        int quality = 0;
 
        Serial0.printf("P,%.3f,%.3f,%.3f,%d\n", x, y, z, quality);
    }
 
    static uint32_t tv = 0;
    if (millis() - tv >= 2000) {
        tv = millis();
        int raw = analogRead(PIN_VMON_ADC);
        Serial0.printf("S,vmon=%d\n", raw);
    }
}