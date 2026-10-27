// RobotRegistry: the paired robots, saved in EEPROM so they survive a reset.
#pragma once
#include <FieldLink.h>
#include "RobotProxy.h"

class RobotRegistry {
public:
  static const uint8_t CAPACITY = 4;

  explicit RobotRegistry(int eepromAddr = 0) : addr_(eepromAddr) {}

  void load();                       // read the saved pairings
  void save() const;

  RobotProxy *find(uint8_t code);
  RobotProxy *add(uint8_t code);     // nullptr if full; returns the existing one if present
  bool remove(uint8_t code);

  uint8_t count() const;
  bool full() const { return count() >= CAPACITY; }

  // Slot access for iteration: check inUse() on each.
  RobotProxy &slot(uint8_t i) { return robots_[i]; }
  const RobotProxy &slot(uint8_t i) const { return robots_[i]; }

  // Round-robin: the next robot in use after the last one returned, or nullptr if none.
  RobotProxy *next();
  // Round-robin as next(), but only robots due for a poll (see RobotProxy::duePoll).
  RobotProxy *nextDue(unsigned long now, unsigned long intervalMs);

private:
  static const uint8_t MAGIC = 0x46;
  static const uint8_t VERSION = 1;
  static const uint8_t EMPTY = 0xFF;

  int addr_;
  RobotProxy robots_[CAPACITY];
  uint8_t cursor_ = 0;
};
