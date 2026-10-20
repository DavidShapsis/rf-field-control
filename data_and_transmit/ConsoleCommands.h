// Console commands. Two kinds:
//   - Operator commands (pair, deny, unpair, list, scan, watch) that act on the controller.
//   - Robot commands (move, stop, zero) that are sent over the radio. These derive from
//     RobotCommand, which handles the "<code|all>" target and delivery; a subclass only says
//     how to turn the typed words into a CommandFrame.
//
// To add a movement, copy MoveCommand, give it a new name and CMD_ number (see Commands.h),
// and add it to the console in field_controller.ino.
#pragma once
#include <FieldLink.h>
#include "Console.h"
#include "FieldController.h"
#include "Dashboard.h"

// Reads "on" or "off".
bool parseOnOff(ArgList &args, bool &out);

// ================= operator commands =================
class PairCommand : public ConsoleCommand {
public:
  explicit PairCommand(FieldController &ctl) : ctl_(ctl) {}
  const __FlashStringHelper *name() const override { return F("pair"); }
  const __FlashStringHelper *usage() const override { return F("pair <code>          approve a robot's pair request"); }
  bool run(ArgList &args) override;
private:
  FieldController &ctl_;
};

class DenyCommand : public ConsoleCommand {
public:
  explicit DenyCommand(FieldController &ctl) : ctl_(ctl) {}
  const __FlashStringHelper *name() const override { return F("deny"); }
  const __FlashStringHelper *usage() const override { return F("deny <code>          reject a robot's pair request"); }
  bool run(ArgList &args) override;
private:
  FieldController &ctl_;
};

class UnpairCommand : public ConsoleCommand {
public:
  explicit UnpairCommand(FieldController &ctl) : ctl_(ctl) {}
  const __FlashStringHelper *name() const override { return F("unpair"); }
  const __FlashStringHelper *usage() const override { return F("unpair <code>        forget a paired robot"); }
  bool run(ArgList &args) override;
private:
  FieldController &ctl_;
};

class ListCommand : public ConsoleCommand {
public:
  explicit ListCommand(Dashboard &dash) : dash_(dash) {}
  const __FlashStringHelper *name() const override { return F("list"); }
  const __FlashStringHelper *usage() const override { return F("list                 print the status table now"); }
  bool run(ArgList &args) override;
private:
  Dashboard &dash_;
};

class ScanCommand : public ConsoleCommand {
public:
  explicit ScanCommand(FieldController &ctl) : ctl_(ctl) {}
  const __FlashStringHelper *name() const override { return F("scan"); }
  const __FlashStringHelper *usage() const override { return F("scan on|off          look for new robots (off = faster polling)"); }
  bool run(ArgList &args) override;
private:
  FieldController &ctl_;
};

class WatchCommand : public ConsoleCommand {
public:
  explicit WatchCommand(Dashboard &dash) : dash_(dash) {}
  const __FlashStringHelper *name() const override { return F("watch"); }
  const __FlashStringHelper *usage() const override { return F("watch on|off         print the status table every 2 s"); }
  bool run(ArgList &args) override;
private:
  Dashboard &dash_;
};

// ================= robot commands =================
class RobotCommand : public ConsoleCommand {
public:
  explicit RobotCommand(FieldController &ctl) : ctl_(ctl) {}
  bool run(ArgList &args) override;     // parses the target, calls build(), queues the result
protected:
  // Fill in cmd.type and its arguments from the remaining words. Return false on bad input.
  virtual bool build(ArgList &args, fl::CommandFrame &cmd) = 0;
private:
  FieldController &ctl_;
};

class MoveCommand : public RobotCommand {
public:
  using RobotCommand::RobotCommand;
  const __FlashStringHelper *name() const override { return F("move"); }
  const __FlashStringHelper *usage() const override { return F("move <code|all> <left> <right>   wheel power -100 to 100"); }
protected:
  bool build(ArgList &args, fl::CommandFrame &cmd) override;
};

class StopCommand : public RobotCommand {
public:
  using RobotCommand::RobotCommand;
  const __FlashStringHelper *name() const override { return F("stop"); }
  const __FlashStringHelper *usage() const override { return F("stop <code|all>      stop moving"); }
protected:
  bool build(ArgList &args, fl::CommandFrame &cmd) override;
};

class ZeroCommand : public RobotCommand {
public:
  using RobotCommand::RobotCommand;
  const __FlashStringHelper *name() const override { return F("zero"); }
  const __FlashStringHelper *usage() const override { return F("zero <code|all>      set the gyro angle to 0"); }
protected:
  bool build(ArgList &args, fl::CommandFrame &cmd) override;
};
