#include "RobotRegistry.h"
#include <EEPROM.h>

// EEPROM layout: [MAGIC] [VERSION] [code 0] [code 1] [code 2] [code 3], 0xFF = empty slot
void RobotRegistry::load() {
  for (uint8_t i = 0; i < CAPACITY; i++) robots_[i].release();
  if (EEPROM.read(addr_) != MAGIC || EEPROM.read(addr_ + 1) != VERSION) return;
  for (uint8_t i = 0; i < CAPACITY; i++) {
    uint8_t code = EEPROM.read(addr_ + 2 + i);
    if (fl::isRobotCode(code) && !find(code)) robots_[i].assign(code);
  }
}

void RobotRegistry::save() const {
  EEPROM.update(addr_, MAGIC);
  EEPROM.update(addr_ + 1, VERSION);
  for (uint8_t i = 0; i < CAPACITY; i++) {
    EEPROM.update(addr_ + 2 + i, robots_[i].inUse() ? robots_[i].code() : EMPTY);
  }
}

RobotProxy *RobotRegistry::find(uint8_t code) {
  for (uint8_t i = 0; i < CAPACITY; i++) {
    if (robots_[i].inUse() && robots_[i].code() == code) return &robots_[i];
  }
  return nullptr;
}

RobotProxy *RobotRegistry::add(uint8_t code) {
  RobotProxy *r = find(code);
  if (r) return r;
  for (uint8_t i = 0; i < CAPACITY; i++) {
    if (!robots_[i].inUse()) {
      robots_[i].assign(code);
      save();
      return &robots_[i];
    }
  }
  return nullptr;
}

bool RobotRegistry::remove(uint8_t code) {
  RobotProxy *r = find(code);
  if (!r) return false;
  r->release();
  save();
  return true;
}

uint8_t RobotRegistry::count() const {
  uint8_t n = 0;
  for (uint8_t i = 0; i < CAPACITY; i++) n += robots_[i].inUse();
  return n;
}

RobotProxy *RobotRegistry::next() {
  for (uint8_t k = 0; k < CAPACITY; k++) {
    cursor_ = (cursor_ + 1) % CAPACITY;
    if (robots_[cursor_].inUse()) return &robots_[cursor_];
  }
  return nullptr;
}
