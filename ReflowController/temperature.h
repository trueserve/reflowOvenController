#ifndef TEMPERATURE_H
#define TEMPERATURE_H

#include <Arduino.h>
#include <SPI.h>

typedef union {
  uint32_t value;
  uint8_t bytes[4];
  struct {
    uint8_t b31:1;
    uint8_t b30:1;
    uint8_t b29:1;
    uint8_t b28:1;
    uint8_t b27:1;
    uint8_t b26:1;
    uint8_t b25:1;
    uint8_t b24:1;
    uint8_t b23:1;
    uint8_t b22:1;
    uint8_t b21:1;
    uint8_t b20:1;
    uint8_t b19:1;
    uint8_t b18:1;
    uint8_t Reserved2:1;
    uint8_t Fault:1;
    uint8_t b15:1;
    uint8_t b14:1;
    uint8_t b13:1;
    uint8_t b12:1;
    uint8_t b11:1;
    uint8_t b10:1;
    uint8_t b9:1;
    uint8_t b8:1;
    uint8_t b7:1;
    uint8_t b6:1;
    uint8_t b5:1;
    uint8_t b4:1;
    uint8_t Reserved1:1;
    uint8_t FaultShortSupply:1;
    uint8_t FaultShortGround:1;
    uint8_t FaultOpen:1;
  };
} __attribute__((packed)) MAX31855_t;

typedef struct Thermocouple {
  double temperature;
  uint8_t stat;
  uint8_t chipSelect;
};

void readThermocouple(struct Thermocouple* input) {
  MAX31855_t sensor;

  uint8_t lcdState = digitalRead(LCD_CS);
  digitalWrite(LCD_CS, HIGH);
  digitalWrite(input->chipSelect, LOW);
  delay(1);
  
  for (int8_t i = 3; i >= 0; i--) {
    sensor.bytes[i] = SPI.transfer(0x00);
  }

  input->stat = sensor.bytes[0] & 0b111;

  uint16_t value = (sensor.value >> 18) & 0x3FFF; // mask off the sign bit and shit to the correct alignment for the temp data  
  input->temperature = value * 0.25;

  digitalWrite(input->chipSelect, HIGH);
  digitalWrite(LCD_CS, lcdState);
}

#endif
