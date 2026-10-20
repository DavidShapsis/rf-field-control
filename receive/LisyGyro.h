// LisyGyro: Parallax LISY300 yaw gyro (ADC101S021 inside). Gives rate and integrated angle.
#pragma once
#include <Arduino.h>

class LisyGyro {
public:
  LisyGyro(uint8_t doutPin, uint8_t sclkPin, uint8_t csPin)
    : dout_(doutPin), sclk_(sclkPin), cs_(csPin) {}

  void begin();
  void calibrate(uint8_t samples = 100);   // keep the robot still; about 12 ms per sample
  void update();                           // call often; integrates the angle

  float rateDps() const { return rate_; }  // positive = counter-clockwise
  float angle() const { return angle_; }   // degrees turned since boot or the last zero
  void zeroAngle() { angle_ = 0; }
  int offset() const { return offset_; }

private:
  static constexpr float DPS_PER_COUNT = 0.9767;
  static const int DEADBAND = 1;           // counts treated as zero (noise)
  int readRaw();

  uint8_t dout_, sclk_, cs_;
  int offset_ = 512;
  float rate_ = 0, angle_ = 0;
  unsigned long lastMicros_ = 0;
};
