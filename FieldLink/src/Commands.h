// FieldLink - command type numbers.
// The controller builds commands with these numbers and the robots dispatch on them, so both
// sides must be compiled with the same list. To add a movement:
//   1. add a number here
//   2. controller: write a RobotCommand subclass that fills in the arguments, register it
//   3. robot: write a CommandHandler subclass that carries it out, register it
#pragma once
#include <Arduino.h>

namespace fl {

enum CommandType : uint8_t {
  CMD_NONE         = 0,
  CMD_STOP         = 1,   // no arguments
  CMD_MOVE         = 2,   // args: left (int8, -100 to 100), right (int8, -100 to 100)
  CMD_ZERO_GYRO    = 3,   // no arguments: set the integrated gyro angle to 0
  // add new commands below, numbers 4 to 255
};

}  // namespace fl
