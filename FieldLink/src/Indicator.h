// FieldLink - Indicator: an LED that can be on, off, or flashed for a moment.
#pragma once
#include <Arduino.h>

class Indicator {
public:
  // activeLow = true when the LED is wired from 5V (through a resistor) to the pin.
  Indicator(int8_t pin, bool activeLow = false) : pin_(pin), activeLow_(activeLow) {}

  void begin() {
    if (pin_ < 0) return;
    pinMode(pin_, OUTPUT);
    write(false);
  }
  void set(bool on) {
    if (on == steady_) return;
    steady_ = on;
    if (!flashUntil_) write(on);   // a flash in progress finishes first
  }
  void flash(uint16_t ms) { flashUntil_ = millis() + ms; write(!steady_); }
  void update() {
    if (flashUntil_ && (long)(millis() - flashUntil_) >= 0) {
      flashUntil_ = 0;
      write(steady_);
    }
  }

private:
  void write(bool on) {
    if (pin_ >= 0) digitalWrite(pin_, (on != activeLow_) ? HIGH : LOW);
  }
  int8_t pin_;
  bool activeLow_;
  bool steady_ = false;
  unsigned long flashUntil_ = 0;
};
