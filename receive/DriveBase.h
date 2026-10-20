// DriveBase: what the command handlers drive. Swap DebugDrive for a real motor class later
// (for example an L298N or a servo drive) without touching anything else.
#pragma once
#include <Arduino.h>

class DriveBase {
public:
  virtual ~DriveBase() {}
  virtual void begin() {}
  virtual void move(int8_t left, int8_t right) = 0;   // -100 to 100 each
  virtual void stop() = 0;
  virtual bool moving() const = 0;
};

// Placeholder: prints what it would do. Remove once real motors are wired.
class DebugDrive : public DriveBase {
public:
  void move(int8_t left, int8_t right) override {
    left_ = left;
    right_ = right;
    Serial.print(F("[DRIVE] move left "));
    Serial.print(left);
    Serial.print(F(" right "));
    Serial.println(right);
  }
  void stop() override {
    if (moving()) Serial.println(F("[DRIVE] stop"));
    left_ = right_ = 0;
  }
  bool moving() const override { return left_ != 0 || right_ != 0; }

private:
  int8_t left_ = 0, right_ = 0;
};
