// RobotProxy: everything the controller knows about one paired robot.
#pragma once
#include <FieldLink.h>

class RobotProxy {
public:
  enum CmdState : uint8_t { CMD_IDLE, CMD_PENDING, CMD_ACKED, CMD_FAILED };
  enum Event : uint8_t { EV_NONE, EV_ACKED, EV_FAILED, EV_ONLINE, EV_OFFLINE, EV_REBOOTED };

  static const uint8_t MAX_TRIES = 5;             // polls carrying a command before giving up
  static const uint8_t OFFLINE_AFTER_MISSES = 3;  // unanswered polls in a row

  void assign(uint8_t code) {
    *this = RobotProxy();
    code_ = code;
    used_ = true;
  }
  void release() { used_ = false; }
  bool inUse() const { return used_; }
  uint8_t code() const { return code_; }

  // ---- commands ----
  // A new command replaces one that has not been confirmed yet (latest wins).
  void queue(const fl::CommandFrame &cmd, const __FlashStringHelper *label) {
    cmd_ = cmd;
    lastSeq_ = fl::nextSeq(lastSeq_);
    if (statusKnown_ && lastSeq_ == status_.ackSeq) lastSeq_ = fl::nextSeq(lastSeq_);
    cmd_.seq = lastSeq_;
    label_ = label;
    tries_ = 0;
    cmdState_ = CMD_PENDING;
  }
  // Commands are held back until the robot has answered once this session, so a sequence
  // number left over from before a reset can never look like a confirmation.
  bool hasCommandToSend() const { return cmdState_ == CMD_PENDING && statusKnown_; }
  const fl::CommandFrame &command() const { return cmd_; }
  void onCommandSent() { tries_++; }

  // ---- poll results (each returns at most one event, the most important) ----
  Event onStatus(const fl::StatusReport &s, unsigned long now) {
    Event ev = EV_NONE;
    bool wasOnline = online();
    if (statusKnown_ && s.uptimeS + 2 < status_.uptimeS) ev = EV_REBOOTED;
    if (!statusKnown_ && cmdState_ == CMD_PENDING && cmd_.seq == s.ackSeq) {
      lastSeq_ = fl::nextSeq(lastSeq_);           // avoid a stale match
      cmd_.seq = lastSeq_;
    }
    status_ = s;
    statusKnown_ = true;
    heard_ = true;
    misses_ = 0;
    okCount_++;
    lastHeardMs_ = now;

    if (cmdState_ == CMD_PENDING && tries_ > 0 && s.ackSeq == cmd_.seq) {
      cmdState_ = CMD_ACKED;
      return EV_ACKED;
    }
    if (cmdState_ == CMD_PENDING && tries_ >= MAX_TRIES) {
      cmdState_ = CMD_FAILED;
      return EV_FAILED;
    }
    if (!wasOnline) return EV_ONLINE;
    return ev;
  }

  Event onTimeout() {
    bool wasOnline = online();
    misses_++;
    missCount_++;
    if (cmdState_ == CMD_PENDING && tries_ >= MAX_TRIES) {
      cmdState_ = CMD_FAILED;
      return EV_FAILED;
    }
    if (wasOnline && !online()) {
      statusKnown_ = false;                        // it may reboot before it comes back
      return EV_OFFLINE;
    }
    return EV_NONE;
  }

  // ---- read-only state for the dashboard ----
  bool online() const { return heard_ && misses_ < OFFLINE_AFTER_MISSES; }
  bool heardEver() const { return heard_; }
  const fl::StatusReport &status() const { return status_; }
  CmdState cmdState() const { return cmdState_; }
  uint8_t cmdSeq() const { return cmd_.seq; }
  uint8_t tries() const { return tries_; }
  const __FlashStringHelper *label() const { return label_; }
  unsigned long okCount() const { return okCount_; }
  unsigned long missCount() const { return missCount_; }
  unsigned long lastHeardMs() const { return lastHeardMs_; }

private:
  bool used_ = false;
  uint8_t code_ = 0;
  bool heard_ = false, statusKnown_ = false;
  uint8_t misses_ = 0;
  fl::StatusReport status_;
  unsigned long okCount_ = 0, missCount_ = 0, lastHeardMs_ = 0;

  fl::CommandFrame cmd_;
  const __FlashStringHelper *label_ = nullptr;
  uint8_t lastSeq_ = 0, tries_ = 0;
  CmdState cmdState_ = CMD_IDLE;
};
