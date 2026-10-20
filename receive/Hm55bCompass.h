// Hm55bCompass: Hitachi HM55B compass module, read without blocking.
// A measurement takes 30 to 40 ms; update() starts one, returns, and collects the result on a
// later call, so the robot can answer the controller in the meantime.
#pragma once
#include <Arduino.h>

class Hm55bCompass {
public:
  // Din and Dout are tied together on one pin.
  Hm55bCompass(uint8_t dioPin, uint8_t clkPin, uint8_t enPin)
    : dio_(dioPin), clk_(clkPin), en_(enPin) {}

  void begin();
  void update();                       // call every loop

  bool ok() const;                     // a reading arrived in the last second
  float heading() const { return heading_; }   // deg clockwise from north, uncalibrated
  int rawX() const { return x_; }
  int rawY() const { return y_; }

private:
  static const uint8_t CMD_RESET = 0b0000;
  static const uint8_t CMD_MEASURE = 0b1000;
  static const uint8_t CMD_REPORT = 0b1100;
  static const uint8_t STATUS_READY = 0b1100;
  static const unsigned long FIRST_CHECK_MS = 40;
  static const unsigned long TIMEOUT_MS = 200;
  static const unsigned long OK_WINDOW_MS = 1000;

  void shiftOut(uint8_t value, uint8_t bits);
  unsigned int shiftIn(uint8_t bits);
  void pulseEnable();
  void startMeasurement();

  uint8_t dio_, clk_, en_;
  bool measuring_ = false;
  unsigned long startMs_ = 0;
  unsigned long lastGoodMs_ = 0;
  bool everGood_ = false;
  int x_ = 0, y_ = 0;
  float heading_ = 0;
};
