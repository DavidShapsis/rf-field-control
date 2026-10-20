#include "LisyGyro.h"

void LisyGyro::begin() {
  pinMode(sclk_, OUTPUT);
  pinMode(cs_, OUTPUT);
  pinMode(dout_, INPUT);
  digitalWrite(sclk_, HIGH);              // clock idles high
  digitalWrite(cs_, HIGH);
  lastMicros_ = micros();
}

// 16 clocks, sampled on the rising edge. Frame = 3 leading zeros, 10 data bits, trailing bits.
int LisyGyro::readRaw() {
  unsigned int raw = 0;
  digitalWrite(cs_, LOW);
  for (uint8_t i = 0; i < 16; i++) {
    digitalWrite(sclk_, LOW);
    delayMicroseconds(1);
    digitalWrite(sclk_, HIGH);
    raw = (raw << 1) | digitalRead(dout_);
  }
  digitalWrite(cs_, HIGH);
  return (raw >> 3) & 0x03FF;
}

void LisyGyro::calibrate(uint8_t samples) {
  long sum = 0;
  for (uint8_t i = 0; i < samples; i++) {
    sum += readRaw();
    delay(12);
  }
  offset_ = sum / samples;
  angle_ = 0;
  lastMicros_ = micros();
}

void LisyGyro::update() {
  int diff = readRaw() - offset_;
  if (abs(diff) <= DEADBAND) diff = 0;
  rate_ = diff * DPS_PER_COUNT;
  unsigned long now = micros();
  angle_ += rate_ * (now - lastMicros_) / 1000000.0;
  lastMicros_ = now;
}
