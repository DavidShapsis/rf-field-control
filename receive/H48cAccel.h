// H48cAccel: Hitachi H48C 3-axis accelerometer (MCP3204 ADC inside), in g.
#pragma once
#include <Arduino.h>

class H48cAccel {
public:
  H48cAccel(uint8_t clkPin, uint8_t dioPin, uint8_t csPin)
    : clk_(clkPin), dio_(dioPin), cs_(csPin) {}

  void begin();
  bool update();                      // call often; reads at most every READ_INTERVAL_MS,
                                      // returns true when it took a new reading

  float x() const { return x_; }
  float y() const { return y_; }
  float z() const { return z_; }

private:
  static const unsigned long READ_INTERVAL_MS = 20;
  static constexpr float G_PER_COUNT = 0.0022;   // (3.3 / 0.3663) / 4095
  int readChannel(uint8_t channel);
  float readAxis(uint8_t channel);

  uint8_t clk_, dio_, cs_;
  float x_ = 0, y_ = 0, z_ = 0;
  unsigned long lastRead_ = 0;
};
