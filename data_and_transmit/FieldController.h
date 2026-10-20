// FieldController: runs the radio side of the controller.
// It alternates between polling paired robots (one at a time, carrying any queued command) and,
// while scanning is on, broadcasting a beacon and listening for pair requests.
#pragma once
#include <FieldLink.h>
#include "RobotRegistry.h"
#include "PairingManager.h"

// Anything that wants to hear about link events (the dashboard) implements this.
class ControllerEvents {
public:
  virtual void onPairRequest(uint8_t code) = 0;
  virtual void onRobotEvent(const RobotProxy &robot, RobotProxy::Event ev) = 0;
};

class FieldController {
public:
  static const unsigned long BEACON_INTERVAL_MS = 3000;

  FieldController(uint8_t id, PacketRadio &radio, RobotRegistry &registry, PairingManager &pairing)
    : id_(id), radio_(radio), registry_(registry), pairing_(pairing) {}

  void setListener(ControllerEvents *listener) { listener_ = listener; }
  void begin();
  void update();                         // call every loop

  uint8_t id() const { return id_; }
  bool scanning() const { return scanning_; }
  void setScanning(bool on) { scanning_ = on; }

  // Operator actions. Each returns false if it could not be done.
  enum PairResult : uint8_t { PAIR_OK, PAIR_NOT_WAITING, PAIR_FULL, PAIR_ALREADY };
  PairResult approve(uint8_t code);
  bool deny(uint8_t code);
  bool unpair(uint8_t code);

  // Queues a command for one robot, or for every paired robot when code is ALL_ROBOTS.
  // Returns how many robots it was queued for.
  static const int ALL_ROBOTS = -1;
  uint8_t queueCommand(int code, const fl::CommandFrame &cmd, const __FlashStringHelper *label);

private:
  enum State : uint8_t { IDLE, WAIT_REPLY, BEACON_WINDOW };

  void startPoll(RobotProxy &robot);
  void startBeacon();
  void handleBeaconPacket(const Packet &p);
  void sendControl(uint8_t type, uint8_t code);     // queued, sent when the link is idle
  void emit(const RobotProxy &r, RobotProxy::Event ev) {
    if (ev != RobotProxy::EV_NONE && listener_) listener_->onRobotEvent(r, ev);
  }

  uint8_t id_;
  PacketRadio &radio_;
  RobotRegistry &registry_;
  PairingManager &pairing_;
  ControllerEvents *listener_ = nullptr;

  State state_ = IDLE;
  bool scanning_ = true;
  unsigned long deadline_ = 0;
  unsigned long lastBeaconMs_ = 0;
  uint8_t waitingFor_ = 0;
  uint8_t pollSeq_ = 0;

  // small outbox for PAIR_ACCEPT / PAIR_DENY / UNPAIR
  static const uint8_t OUTBOX_SIZE = 4;
  struct Control { uint8_t type, code; };
  Control outbox_[OUTBOX_SIZE];
  uint8_t outboxCount_ = 0;
};
