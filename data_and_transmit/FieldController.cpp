#include "FieldController.h"

void FieldController::begin() {
  radio_.begin();
  lastBeaconMs_ = millis() - BEACON_INTERVAL_MS;    // first beacon right away
}

void FieldController::update() {
  unsigned long now = millis();
  pairing_.expire(now);
  Packet p;

  switch (state_) {
    case IDLE: {
      while (radio_.receive(p)) {}                   // nothing expected, discard
      if (outboxCount_ > 0) {
        Control c = outbox_[0];
        for (uint8_t i = 1; i < outboxCount_; i++) outbox_[i - 1] = outbox_[i];
        outboxCount_--;
        radio_.send(Packet(c.type, id_, c.code));
        return;
      }
      if (scanning_ && now - lastBeaconMs_ >= BEACON_INTERVAL_MS) {
        startBeacon();
        return;
      }
      RobotProxy *robot = registry_.nextDue(now, POLL_INTERVAL_MS);
      if (robot) startPoll(*robot);
      break;
    }

    case WAIT_REPLY: {
      while (radio_.receive(p)) {
        if (p.type == fl::MSG_STATUS && p.dst == id_ && p.src == waitingFor_) {
          RobotProxy *robot = registry_.find(waitingFor_);
          fl::StatusReport s;
          if (robot && s.readFrom(p)) emit(*robot, robot->onStatus(s, millis()));
          state_ = IDLE;
          return;
        }
      }
      if ((long)(now - deadline_) >= 0) {
        RobotProxy *robot = registry_.find(waitingFor_);
        if (robot) emit(*robot, robot->onTimeout());
        state_ = IDLE;
      }
      break;
    }

    case BEACON_WINDOW: {
      while (radio_.receive(p)) handleBeaconPacket(p);
      if ((long)(now - deadline_) >= 0) state_ = IDLE;
      break;
    }
  }
}

void FieldController::startPoll(RobotProxy &robot) {
  Packet p(fl::MSG_POLL, id_, robot.code());
  p.seq = ++pollSeq_;
  if (robot.hasCommandToSend()) {
    robot.command().writeTo(p);
    robot.onCommandSent();
  }                                                  // otherwise len 0 = plain poll
  robot.onPolled(millis());
  radio_.send(p);
  waitingFor_ = robot.code();
  deadline_ = millis() + fl::REPLY_TIMEOUT_MS;
  state_ = WAIT_REPLY;
}

void FieldController::startBeacon() {
  Packet p(fl::MSG_BEACON, id_, fl::ADDR_BROADCAST);
  radio_.send(p);
  lastBeaconMs_ = millis();
  deadline_ = lastBeaconMs_ + fl::BEACON_WINDOW_MS;
  state_ = BEACON_WINDOW;
}

void FieldController::handleBeaconPacket(const Packet &p) {
  if (p.type != fl::MSG_PAIR_REQUEST || p.dst != id_ || !fl::isRobotCode(p.src)) return;
  if (registry_.find(p.src)) {
    // Already approved (the robot rebooted or timed out): let it straight back in.
    sendControl(fl::MSG_PAIR_ACCEPT, p.src);
    return;
  }
  if (pairing_.onRequest(p.src, millis()) && listener_) listener_->onPairRequest(p.src);
}

void FieldController::sendControl(uint8_t type, uint8_t code) {
  for (uint8_t i = 0; i < outboxCount_; i++) {
    if (outbox_[i].type == type && outbox_[i].code == code) return;   // already queued
  }
  if (outboxCount_ < OUTBOX_SIZE) outbox_[outboxCount_++] = {type, code};
}

FieldController::PairResult FieldController::approve(uint8_t code) {
  if (registry_.find(code)) { pairing_.take(code); return PAIR_ALREADY; }
  if (!pairing_.isWaiting(code)) return PAIR_NOT_WAITING;
  if (registry_.full()) return PAIR_FULL;
  pairing_.take(code);
  registry_.add(code);
  sendControl(fl::MSG_PAIR_ACCEPT, code);
  return PAIR_OK;
}

bool FieldController::deny(uint8_t code) {
  if (!pairing_.take(code)) return false;
  sendControl(fl::MSG_PAIR_DENY, code);
  return true;
}

bool FieldController::unpair(uint8_t code) {
  if (!registry_.remove(code)) return false;
  sendControl(fl::MSG_UNPAIR, code);   // best effort; the robot also times out by itself
  return true;
}

uint8_t FieldController::queueCommand(int code, const fl::CommandFrame &cmd,
                                      const __FlashStringHelper *label) {
  uint8_t n = 0;
  for (uint8_t i = 0; i < RobotRegistry::CAPACITY; i++) {
    RobotProxy &r = registry_.slot(i);
    if (!r.inUse()) continue;
    if (code != ALL_ROBOTS && r.code() != code) continue;
    r.queue(cmd, label);
    n++;
  }
  return n;
}
