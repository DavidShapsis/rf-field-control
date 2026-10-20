// FieldLink - protocol shared by the field controller and the robots.
//
// The controller is the only device that talks unprompted. Robots answer only when asked, so
// two robots never transmit at the same time (except unpaired robots answering a beacon, which
// pick random time slots).
//
//   Pairing:  controller --BEACON (broadcast)--> unpaired robots
//             robot --PAIR_REQUEST--> controller        (in a random slot after the beacon)
//             operator types "pair <code>" on the controller
//             controller --PAIR_ACCEPT--> robot
//   Running:  controller --POLL (+ optional command)--> robot
//             robot --STATUS (gyro, accelerometer, compass, last command seq)--> controller
#pragma once
#include <Arduino.h>
#include "Packet.h"

namespace fl {

// ---------- Addresses ----------
// Robots use their two-digit code (0 to 99) as their address.
// Controllers use 100 to 199. 255 means everyone.
const uint8_t ROBOT_CODE_MAX = 99;
const uint8_t CONTROLLER_ID_MIN = 100;
const uint8_t CONTROLLER_ID_MAX = 199;
const uint8_t ADDR_BROADCAST = 255;

inline bool isRobotCode(int v) { return v >= 0 && v <= ROBOT_CODE_MAX; }

// ---------- Message types ----------
enum MsgType : uint8_t {
  MSG_BEACON       = 1,   // controller -> all       "unpaired robots, introduce yourselves"
  MSG_PAIR_REQUEST = 2,   // robot -> controller      "I am robot <src>"
  MSG_PAIR_ACCEPT  = 3,   // controller -> robot
  MSG_PAIR_DENY    = 4,   // controller -> robot
  MSG_UNPAIR       = 5,   // controller -> robot
  MSG_POLL         = 6,   // controller -> robot      payload: CommandFrame (type 0 = none)
  MSG_STATUS       = 7,   // robot -> controller      payload: StatusReport
};

// ---------- Timing (ms) ----------
const uint16_t REPLY_DELAY_MS = 15;      // robot waits this long before answering, so the
                                         // controller has switched back to receive
const uint16_t REPLY_TIMEOUT_MS = 260;   // controller gives up on an answer after this
                                         // (a STATUS reply takes about 160 ms)
const uint8_t  BEACON_SLOTS = 4;         // unpaired robots answer in a random slot
const uint16_t BEACON_SLOT_MS = 130;     // long enough for one PAIR_REQUEST burst
const uint16_t BEACON_WINDOW_MS = REPLY_DELAY_MS + BEACON_SLOTS * BEACON_SLOT_MS + 120;
const uint16_t ROBOT_FAILSAFE_MS = 3000; // robot stops driving if it is not polled this long
const uint16_t ROBOT_LINK_TIMEOUT_MS = 10000;  // robot forgets its controller after this

// ---------- Commands carried inside a POLL ----------
// Command type numbers live in Commands.h so the controller and robots agree on them.
const uint8_t CMD_MAX_ARGS = 8;

struct CommandFrame {
  uint8_t seq = 0;                // 1 to 255, never 0
  uint8_t type = 0;               // 0 = no command
  uint8_t argc = 0;
  uint8_t args[CMD_MAX_ARGS];

  void clear() { seq = 0; type = 0; argc = 0; }
  bool empty() const { return type == 0; }
  bool addArg(uint8_t v) { if (argc >= CMD_MAX_ARGS) return false; args[argc++] = v; return true; }
  bool addArgI8(int8_t v) { return addArg((uint8_t)v); }
  bool addArgI16(int16_t v) { return addArg((uint16_t)v >> 8) && addArg(v & 0xFF); }
  int8_t argI8(uint8_t i) const { return i < argc ? (int8_t)args[i] : 0; }
  int16_t argI16(uint8_t i) const {
    return (i + 1 < argc) ? (int16_t)(((uint16_t)args[i] << 8) | args[i + 1]) : 0;
  }

  void writeTo(Packet &p) const {
    PayloadWriter w(p);
    w.u8(seq);
    w.u8(type);
    for (uint8_t i = 0; i < argc; i++) w.u8(args[i]);
  }
  bool readFrom(const Packet &p) {
    PayloadReader r(p);
    seq = r.u8();
    type = r.u8();
    argc = 0;
    while (r.remaining() && argc < CMD_MAX_ARGS) args[argc++] = r.u8();
    return r.ok();
  }
};

// ---------- Robot status sent back with every STATUS (16 bytes) ----------
const uint8_t STATUS_FLAG_DRIVING    = 0x01;
const uint8_t STATUS_FLAG_FAILSAFE   = 0x02;   // robot stopped itself after losing the link
const uint8_t STATUS_FLAG_COMPASS_OK = 0x04;   // compass answered recently

struct StatusReport {
  int16_t rate10 = 0;       // gyro rate, 0.1 deg/s (positive = counter-clockwise)
  int16_t gyroAngle10 = 0;  // integrated gyro angle, 0.1 deg
  int16_t ax = 0;           // acceleration, milli-g
  int16_t ay = 0;
  int16_t az = 0;
  int16_t compass10 = 0;    // compass heading, 0.1 deg clockwise from north (0 to 3599)
  uint8_t ackSeq = 0;       // last command sequence number executed
  uint8_t flags = 0;
  uint16_t uptimeS = 0;     // seconds since the robot booted (shows resets)

  void writeTo(Packet &p) const {
    PayloadWriter w(p);
    w.i16(rate10);
    w.i16(gyroAngle10);
    w.i16(ax);
    w.i16(ay);
    w.i16(az);
    w.i16(compass10);
    w.u8(ackSeq);
    w.u8(flags);
    w.u16(uptimeS);
  }
  bool readFrom(const Packet &p) {
    PayloadReader r(p);
    rate10 = r.i16();
    gyroAngle10 = r.i16();
    ax = r.i16();
    ay = r.i16();
    az = r.i16();
    compass10 = r.i16();
    ackSeq = r.u8();
    flags = r.u8();
    uptimeS = r.u16();
    return r.ok();
  }
};

inline uint8_t nextSeq(uint8_t s) { return s >= 255 ? 1 : s + 1; }

}  // namespace fl
