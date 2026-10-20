// Command handlers: one class per command type. The dispatcher looks up the handler by the
// command's type number (Commands.h) and calls handle().
//
// To add a movement: write a class like MoveHandler with a new CMD_ number and register it in
// field_robot.ino with dispatcher.add(&yourHandler).
#pragma once
#include <FieldLink.h>
#include "DriveBase.h"
#include "LisyGyro.h"

class CommandHandler {
public:
  virtual ~CommandHandler() {}
  virtual uint8_t type() const = 0;
  virtual void handle(const fl::CommandFrame &cmd) = 0;
};

class StopHandler : public CommandHandler {
public:
  explicit StopHandler(DriveBase &drive) : drive_(drive) {}
  uint8_t type() const override { return fl::CMD_STOP; }
  void handle(const fl::CommandFrame &) override { drive_.stop(); }
private:
  DriveBase &drive_;
};

class MoveHandler : public CommandHandler {
public:
  explicit MoveHandler(DriveBase &drive) : drive_(drive) {}
  uint8_t type() const override { return fl::CMD_MOVE; }
  void handle(const fl::CommandFrame &cmd) override {
    drive_.move(constrain(cmd.argI8(0), -100, 100), constrain(cmd.argI8(1), -100, 100));
  }
private:
  DriveBase &drive_;
};

class ZeroGyroHandler : public CommandHandler {
public:
  explicit ZeroGyroHandler(LisyGyro &gyro) : gyro_(gyro) {}
  uint8_t type() const override { return fl::CMD_ZERO_GYRO; }
  void handle(const fl::CommandFrame &) override {
    gyro_.zeroAngle();
    Serial.println(F("[GYRO] angle zeroed"));
  }
private:
  LisyGyro &gyro_;
};

// Runs each command once. Retries from the controller carry the same sequence number, so a
// repeat is recognised and skipped (it is still acknowledged).
class CommandDispatcher {
public:
  static const uint8_t MAX_HANDLERS = 12;

  bool add(CommandHandler *h) {
    if (count_ >= MAX_HANDLERS) return false;
    handlers_[count_++] = h;
    return true;
  }

  // Returns true if the command was new and was run.
  bool execute(const fl::CommandFrame &cmd) {
    if (cmd.empty() || cmd.seq == 0 || cmd.seq == lastSeq_) return false;
    lastSeq_ = cmd.seq;
    for (uint8_t i = 0; i < count_; i++) {
      if (handlers_[i]->type() == cmd.type) {
        handlers_[i]->handle(cmd);
        return true;
      }
    }
    Serial.print(F("[CMD] no handler for type "));
    Serial.println(cmd.type);
    return true;
  }

  uint8_t lastSeq() const { return lastSeq_; }

private:
  CommandHandler *handlers_[MAX_HANDLERS];
  uint8_t count_ = 0;
  uint8_t lastSeq_ = 0;
};
