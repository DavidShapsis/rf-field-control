// RobotNode: the robot's side of the link.
//   Unpaired: answers controller beacons with a pair request (in a random time slot).
//   Paired:   answers polls from its controller with a STATUS packet and runs any command the
//             poll carries. Stops driving if the controller goes quiet (failsafe) and forgets
//             the controller after a longer silence, so it can be paired again.
#pragma once
#include <FieldLink.h>
#include "SensorSuite.h"
#include "DriveBase.h"
#include "CommandHandlers.h"

class RobotNode {
public:
  RobotNode(uint8_t code, PacketRadio &radio, CommandDispatcher &dispatcher,
            SensorSuite &sensors, DriveBase &drive, Indicator &linkLed)
    : code_(code), radio_(radio), dispatcher_(dispatcher),
      sensors_(sensors), drive_(drive), led_(linkLed) {}

  void begin();
  void update();                           // call every loop

  bool paired() const { return paired_; }
  uint8_t code() const { return code_; }

private:
  static const unsigned long DENIED_QUIET_MS = 30000;

  void handle(const Packet &p);
  void schedule(uint8_t type, uint8_t dst, uint16_t delayMs);
  void sendScheduled();
  void setPaired(bool on, uint8_t controller);
  void checkTimeouts(unsigned long now);
  void updateLed(unsigned long now);

  uint8_t code_;
  PacketRadio &radio_;
  CommandDispatcher &dispatcher_;
  SensorSuite &sensors_;
  DriveBase &drive_;
  Indicator &led_;

  bool paired_ = false;
  uint8_t controller_ = 0;
  unsigned long lastPollMs_ = 0;
  unsigned long quietUntil_ = 0;           // after a deny
  bool failsafe_ = false;

  bool replyPending_ = false;              // one scheduled transmission
  uint8_t replyType_ = 0, replyDst_ = 0;
  unsigned long replyAt_ = 0;

  unsigned long ledToggleMs_ = 0;
  bool ledState_ = false;
};
