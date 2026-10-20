#include "RobotNode.h"

void RobotNode::begin() {
  radio_.begin();
  Serial.print(F("Robot "));
  if (code_ < 10) Serial.print('0');
  Serial.print(code_);
  Serial.println(F(" ready, waiting for a controller to pair."));
}

void RobotNode::update() {
  unsigned long now = millis();

  Packet p;
  while (radio_.receive(p)) handle(p);

  if (replyPending_ && (long)(now - replyAt_) >= 0) sendScheduled();

  checkTimeouts(millis());
  updateLed(millis());
}

void RobotNode::handle(const Packet &p) {
  unsigned long now = millis();

  switch (p.type) {
    case fl::MSG_BEACON:
      if (p.dst != fl::ADDR_BROADCAST || paired_) return;
      if ((long)(quietUntil_ - now) > 0) return;
      // Random slot so two unpaired robots are unlikely to answer on top of each other.
      schedule(fl::MSG_PAIR_REQUEST, p.src,
               fl::REPLY_DELAY_MS + random(fl::BEACON_SLOTS) * fl::BEACON_SLOT_MS);
      return;

    case fl::MSG_PAIR_ACCEPT:
      if (p.dst != code_) return;
      if (!paired_ || controller_ == p.src) {
        setPaired(true, p.src);
        lastPollMs_ = now;
      }
      return;

    case fl::MSG_PAIR_DENY:
      if (p.dst != code_ || paired_) return;
      quietUntil_ = now + DENIED_QUIET_MS;
      Serial.println(F("[LINK] pair request denied"));
      return;

    case fl::MSG_UNPAIR:
      if (p.dst != code_ || !paired_ || p.src != controller_) return;
      setPaired(false, 0);
      return;

    case fl::MSG_POLL: {
      if (p.dst != code_) return;
      // A controller only polls robots it has approved, so a poll also means "paired".
      if (!paired_) setPaired(true, p.src);
      if (p.src != controller_) return;     // belongs to someone else
      lastPollMs_ = now;
      if (p.len >= 2) {
        fl::CommandFrame cmd;
        if (cmd.readFrom(p) && dispatcher_.execute(cmd)) {
          failsafe_ = false;
          led_.flash(80);
        }
      }
      schedule(fl::MSG_STATUS, controller_, fl::REPLY_DELAY_MS);
      return;
    }

    default:
      return;
  }
}

void RobotNode::schedule(uint8_t type, uint8_t dst, uint16_t delayMs) {
  replyType_ = type;
  replyDst_ = dst;
  replyAt_ = millis() + delayMs;
  replyPending_ = true;
}

void RobotNode::sendScheduled() {
  replyPending_ = false;
  Packet p(replyType_, code_, replyDst_);
  if (replyType_ == fl::MSG_STATUS) {
    fl::StatusReport s;
    sensors_.fill(s);
    s.ackSeq = dispatcher_.lastSeq();
    if (drive_.moving()) s.flags |= fl::STATUS_FLAG_DRIVING;
    if (failsafe_) s.flags |= fl::STATUS_FLAG_FAILSAFE;
    s.uptimeS = millis() / 1000;
    s.writeTo(p);
  }
  radio_.send(p);
}

void RobotNode::setPaired(bool on, uint8_t controller) {
  paired_ = on;
  controller_ = controller;
  if (on) {
    Serial.print(F("[LINK] paired with controller "));
    Serial.println(controller);
  } else {
    drive_.stop();
    replyPending_ = false;
    Serial.println(F("[LINK] unpaired"));
  }
}

void RobotNode::checkTimeouts(unsigned long now) {
  if (!paired_) return;
  unsigned long silent = now - lastPollMs_;
  if (silent > fl::ROBOT_FAILSAFE_MS && drive_.moving()) {
    drive_.stop();
    failsafe_ = true;
    Serial.println(F("[LINK] no polls, failsafe stop"));
  }
  if (silent > fl::ROBOT_LINK_TIMEOUT_MS) {
    Serial.println(F("[LINK] controller lost"));
    setPaired(false, 0);
  }
}

// Paired: steady on (blinks off briefly for each command). Unpaired: slow blink.
void RobotNode::updateLed(unsigned long now) {
  if (paired_) {
    led_.set(true);
  } else if (now - ledToggleMs_ >= 500) {
    ledToggleMs_ = now;
    ledState_ = !ledState_;
    led_.set(ledState_);
  }
  led_.update();
}
