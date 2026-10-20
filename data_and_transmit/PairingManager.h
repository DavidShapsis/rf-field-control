// PairingManager: robots that asked to pair and are waiting for the operator to approve.
#pragma once
#include <FieldLink.h>

class PairingManager {
public:
  static const uint8_t CAPACITY = 4;
  static const unsigned long REQUEST_TTL_MS = 30000;   // a request is forgotten after this

  // Records a request. Returns true if it is new (not already waiting), so the caller prompts.
  bool onRequest(uint8_t code, unsigned long now) {
    Entry *e = find(code);
    if (e) { e->lastSeen = now; return false; }
    for (uint8_t i = 0; i < CAPACITY; i++) {
      if (!entries_[i].used) {
        entries_[i] = {true, code, now};
        return true;
      }
    }
    return false;                                     // list full, ignore until one expires
  }

  bool isWaiting(uint8_t code) const { return findConst(code) != nullptr; }

  // Removes the request (after approve or deny). Returns false if it was not waiting.
  bool take(uint8_t code) {
    Entry *e = find(code);
    if (!e) return false;
    e->used = false;
    return true;
  }

  void expire(unsigned long now) {
    for (uint8_t i = 0; i < CAPACITY; i++) {
      if (entries_[i].used && now - entries_[i].lastSeen > REQUEST_TTL_MS) entries_[i].used = false;
    }
  }

  uint8_t count() const {
    uint8_t n = 0;
    for (uint8_t i = 0; i < CAPACITY; i++) n += entries_[i].used;
    return n;
  }
  // i-th waiting code (0 <= i < count())
  uint8_t codeAt(uint8_t i) const {
    for (uint8_t k = 0; k < CAPACITY; k++) {
      if (entries_[k].used && i-- == 0) return entries_[k].code;
    }
    return 0;
  }

private:
  struct Entry {
    bool used;
    uint8_t code;
    unsigned long lastSeen;
  };
  Entry *find(uint8_t code) {
    for (uint8_t i = 0; i < CAPACITY; i++) {
      if (entries_[i].used && entries_[i].code == code) return &entries_[i];
    }
    return nullptr;
  }
  const Entry *findConst(uint8_t code) const {
    for (uint8_t i = 0; i < CAPACITY; i++) {
      if (entries_[i].used && entries_[i].code == code) return &entries_[i];
    }
    return nullptr;
  }
  Entry entries_[CAPACITY] = {};
};
