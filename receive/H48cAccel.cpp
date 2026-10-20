#include "H48cAccel.h"

void H48cAccel::begin() {
  pinMode(clk_, OUTPUT);
  pinMode(cs_, OUTPUT);
  digitalWrite(clk_, LOW);
  digitalWrite(cs_, HIGH);
}

// MCP3204: start + single-ended + channel (5 bits), then a null bit and 12 data bits.
// Each bit is sampled after the clock pulse (clock high, then low, then read).
int H48cAccel::readChannel(uint8_t channel) {
  digitalWrite(cs_, LOW);
  pinMode(dio_, OUTPUT);
  uint8_t cmd = 0b11000 | (channel & 0x03);
  for (int8_t i = 4; i >= 0; i--) {
    digitalWrite(dio_, (cmd >> i) & 1);
    digitalWrite(clk_, HIGH);
    if (i == 0) pinMode(dio_, INPUT);
    digitalWrite(clk_, LOW);
  }
  unsigned int value = 0;
  for (uint8_t i = 0; i < 13; i++) {
    digitalWrite(clk_, HIGH);
    digitalWrite(clk_, LOW);
    value = (value << 1) | digitalRead(dio_);
  }
  digitalWrite(cs_, HIGH);
  return value & 0x0FFF;
}

// Axis reading relative to the reference channel (Vref = 0 g).
float H48cAccel::readAxis(uint8_t channel) {
  int vref = readChannel(3);
  int axis = readChannel(channel);
  return (axis - vref) * G_PER_COUNT;
}

void H48cAccel::update() {
  unsigned long now = millis();
  if (now - lastRead_ < READ_INTERVAL_MS) return;
  lastRead_ = now;
  x_ = readAxis(0);
  y_ = readAxis(1);
  z_ = readAxis(2);
}
