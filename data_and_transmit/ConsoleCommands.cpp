#include "ConsoleCommands.h"
#include <string.h>

bool parseOnOff(ArgList &args, bool &out) {
  const char *w = args.next();
  if (!w || !args.done()) return false;
  if (strcmp_P(w, PSTR("on")) == 0)  { out = true;  return true; }
  if (strcmp_P(w, PSTR("off")) == 0) { out = false; return true; }
  return false;
}

static bool parseCode(ArgList &args, int &code) {
  return args.nextInt(0, fl::ROBOT_CODE_MAX, code) && args.done();
}

// ================= operator commands =================
bool PairCommand::run(ArgList &args) {
  int code;
  if (!parseCode(args, code)) return false;
  switch (ctl_.approve(code)) {
    case FieldController::PAIR_OK:
      Serial.print(F("Paired with robot "));
      Dashboard::printCode(code);
      Serial.println(F(". Saved, it reconnects by itself after a reset."));
      break;
    case FieldController::PAIR_ALREADY:
      Serial.println(F("That robot is already paired."));
      break;
    case FieldController::PAIR_FULL:
      Serial.print(F("All "));
      Serial.print(RobotRegistry::CAPACITY);
      Serial.println(F(" slots are used. Unpair a robot first."));
      break;
    case FieldController::PAIR_NOT_WAITING:
      Serial.println(F("No pair request from that code. Turn the robot on and wait for its request (scan must be on)."));
      break;
  }
  return true;
}

bool DenyCommand::run(ArgList &args) {
  int code;
  if (!parseCode(args, code)) return false;
  if (ctl_.deny(code)) Serial.println(F("Request denied. That robot stays quiet for 30 s."));
  else                 Serial.println(F("No pair request from that code."));
  return true;
}

bool UnpairCommand::run(ArgList &args) {
  int code;
  if (!parseCode(args, code)) return false;
  if (ctl_.unpair(code)) Serial.println(F("Unpaired and removed from memory."));
  else                   Serial.println(F("That robot is not paired."));
  return true;
}

bool ListCommand::run(ArgList &args) {
  if (!args.done()) return false;
  dash_.printTable();
  return true;
}

bool ScanCommand::run(ArgList &args) {
  bool on;
  if (!parseOnOff(args, on)) return false;
  ctl_.setScanning(on);
  Serial.println(on ? F("Scanning for new robots.") : F("Scanning off."));
  return true;
}

bool WatchCommand::run(ArgList &args) {
  bool on;
  if (!parseOnOff(args, on)) return false;
  dash_.setWatching(on);
  Serial.println(on ? F("Status table on.") : F("Status table off (type list to see it)."));
  return true;
}

// ================= robot commands =================
bool RobotCommand::run(ArgList &args) {
  const char *target = args.next();
  if (!target) return false;

  int code;
  if (strcmp_P(target, PSTR("all")) == 0) {
    code = FieldController::ALL_ROBOTS;
  } else {
    char *end;
    long v = strtol(target, &end, 10);
    if (end == target || *end != 0 || !fl::isRobotCode(v)) return false;
    code = (int)v;
  }

  fl::CommandFrame cmd;
  if (!build(args, cmd) || !args.done()) return false;

  uint8_t n = ctl_.queueCommand(code, cmd, name());
  if (n == 0) {
    Serial.println(code == FieldController::ALL_ROBOTS ? F("No robots are paired.")
                                                       : F("That robot is not paired."));
  } else {
    Serial.print(F("Queued for "));
    Serial.print(n);
    Serial.println(n == 1 ? F(" robot.") : F(" robots."));
  }
  return true;
}

bool MoveCommand::build(ArgList &args, fl::CommandFrame &cmd) {
  int left, right;
  if (!args.nextInt(-100, 100, left) || !args.nextInt(-100, 100, right)) return false;
  cmd.type = fl::CMD_MOVE;
  cmd.addArgI8(left);
  cmd.addArgI8(right);
  return true;
}

bool StopCommand::build(ArgList &, fl::CommandFrame &cmd) {
  cmd.type = fl::CMD_STOP;
  return true;
}

bool ZeroCommand::build(ArgList &, fl::CommandFrame &cmd) {
  cmd.type = fl::CMD_ZERO_GYRO;
  return true;
}
