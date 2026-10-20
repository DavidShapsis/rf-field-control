#include "Hm55bCompass.h"

void Hm55bCompass::begin() {
  pinMode(clk_, OUTPUT);
  pinMode(en_, OUTPUT);
  digitalWrite(clk_, LOW);
  digitalWrite(en_, HIGH);
}

void Hm55bCompass::shiftOut(uint8_t value, uint8_t bits) {
  pinMode(dio_, OUTPUT);
  for (int8_t i = bits - 1; i >= 0; i--) {
    digitalWrite(dio_, (value >> i) & 1);
    digitalWrite(clk_, HIGH);
    digitalWrite(clk_, LOW);
  }
}

unsigned int Hm55bCompass::shiftIn(uint8_t bits) {
  pinMode(dio_, INPUT);
  unsigned int value = 0;
  for (uint8_t i = 0; i < bits; i++) {
    digitalWrite(clk_, HIGH);
    delayMicroseconds(2);
    value = (value << 1) | digitalRead(dio_);
    digitalWrite(clk_, LOW);
  }
  return value;
}

void Hm55bCompass::pulseEnable() {
  digitalWrite(en_, HIGH);
  digitalWrite(en_, LOW);
}

void Hm55bCompass::startMeasurement() {
  pulseEnable();
  shiftOut(CMD_RESET, 4);
  pulseEnable();
  shiftOut(CMD_MEASURE, 4);
  startMs_ = millis();
  measuring_ = true;
}

void Hm55bCompass::update() {
  if (!measuring_) {
    startMeasurement();
    return;
  }
  unsigned long elapsed = millis() - startMs_;
  if (elapsed < FIRST_CHECK_MS) return;

  pulseEnable();
  shiftOut(CMD_REPORT, 4);
  if (shiftIn(4) != STATUS_READY) {
    if (elapsed > TIMEOUT_MS) {                // not answering: start over
      digitalWrite(en_, HIGH);
      measuring_ = false;
    }
    return;                                    // check again on a later call
  }

  unsigned int rx = shiftIn(11);
  unsigned int ry = shiftIn(11);
  digitalWrite(en_, HIGH);
  measuring_ = false;

  if (rx & 0x0400) rx |= 0xF800;               // sign extend 11 bit to 16 bit
  if (ry & 0x0400) ry |= 0xF800;
  x_ = (int16_t)rx;
  y_ = (int16_t)ry;

  float deg = atan2(-(float)y_, (float)x_) * 180.0 / PI;   // arctan(-y / x)
  if (deg < 0) deg += 360.0;
  heading_ = deg;
  lastGoodMs_ = millis();
  everGood_ = true;
}

bool Hm55bCompass::ok() const {
  return everGood_ && millis() - lastGoodMs_ < OK_WINDOW_MS;
}
