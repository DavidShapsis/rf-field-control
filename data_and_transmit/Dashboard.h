// Dashboard: everything printed to the Serial Monitor. Prints link events as they happen and,
// while "watch" is on, a status table every couple of seconds.
#pragma once
#include <FieldLink.h>
#include "FieldController.h"

class Dashboard : public ControllerEvents {
public:
  static const unsigned long TABLE_INTERVAL_MS = 2000;

  Dashboard(FieldController &ctl, RobotRegistry &registry, PairingManager &pairing, PacketRadio &radio)
    : ctl_(ctl), registry_(registry), pairing_(pairing), radio_(radio) {}

  void begin();
  void update();                      // call every loop
  void setWatching(bool on) { watching_ = on; }
  bool watching() const { return watching_; }
  void printTable();

  // ControllerEvents
  void onPairRequest(uint8_t code) override;
  void onRobotEvent(const RobotProxy &robot, RobotProxy::Event ev) override;

  // Helpers also used by the console commands
  static void printCode(uint8_t code);                  // always two digits
  static void printCmdState(const RobotProxy &r);

private:
  void printRobot(const RobotProxy &r, unsigned long now);
  static void pad(uint8_t used, uint8_t width);
  static uint8_t printTenths(int16_t v10, uint8_t width);
  static uint8_t printNum(unsigned long v);

  FieldController &ctl_;
  RobotRegistry &registry_;
  PairingManager &pairing_;
  PacketRadio &radio_;
  bool watching_ = true;
  unsigned long lastTableMs_ = 0;
};
