#include "Dashboard.h"
#include <stdlib.h>

void Dashboard::begin() {
  Serial.println();
  Serial.print(F("Field controller "));
  Serial.print(ctl_.id());
  Serial.print(F(" ready. "));
  Serial.print(registry_.count());
  Serial.println(F(" robot(s) remembered from last time. Type help for commands."));
  lastTableMs_ = millis();
}

void Dashboard::update() {
  if (watching_ && millis() - lastTableMs_ >= TABLE_INTERVAL_MS) {
    lastTableMs_ = millis();
    printTable();
  }
}

// ---------- events ----------
void Dashboard::onPairRequest(uint8_t code) {
  Serial.print(F("[PAIR] Robot "));
  printCode(code);
  Serial.print(F(" wants to pair. Type: pair "));
  printCode(code);
  Serial.print(F("   (or: deny "));
  printCode(code);
  Serial.println(')');
}

void Dashboard::onRobotEvent(const RobotProxy &r, RobotProxy::Event ev) {
  Serial.print(F("[ROBOT "));
  printCode(r.code());
  Serial.print(F("] "));
  switch (ev) {
    case RobotProxy::EV_ONLINE:   Serial.println(F("online")); break;
    case RobotProxy::EV_OFFLINE:  Serial.println(F("OFFLINE (stopped answering)")); break;
    case RobotProxy::EV_REBOOTED: Serial.println(F("rebooted")); break;
    case RobotProxy::EV_ACKED:
    case RobotProxy::EV_FAILED:
      printCmdState(r);
      Serial.println();
      break;
    default: Serial.println(); break;
  }
}

// ---------- table ----------
void Dashboard::printTable() {
  unsigned long now = millis();
  lastTableMs_ = now;
  Serial.println();
  Serial.print(F("==== FIELD CONTROLLER "));
  Serial.print(ctl_.id());
  Serial.print(F(" ====  t="));
  char buf[12];
  Serial.print(dtostrf(now / 1000.0, 1, 1, buf));
  Serial.print(F("s  scan "));
  Serial.print(ctl_.scanning() ? F("ON") : F("OFF"));
  Serial.print(F("  paired "));
  Serial.print(registry_.count());
  Serial.print('/');
  Serial.print(RobotRegistry::CAPACITY);
  Serial.print(F("  radio tx "));
  Serial.print(radio_.sent());
  Serial.print(F(" rx "));
  Serial.print(radio_.received());
  Serial.print(F(" bad "));
  Serial.println(radio_.badFrames());

  if (registry_.count() == 0) {
    Serial.println(F("No robots paired. Power a robot on; its pair request will show up here."));
  }
  for (uint8_t i = 0; i < RobotRegistry::CAPACITY; i++) {
    const RobotProxy &r = registry_.slot(i);
    if (r.inUse()) printRobot(r, now);
  }

  if (pairing_.count() > 0) {
    Serial.print(F("Waiting to pair:"));
    for (uint8_t i = 0; i < pairing_.count(); i++) {
      Serial.print(' ');
      printCode(pairing_.codeAt(i));
    }
    Serial.println(F("   (pair <code> / deny <code>)"));
  }
}

// Two lines per robot:
//   ROBOT 42  ONLINE   moving    polls 58/0    seen 0.2s   cmd #2 move DONE
//      gyro   12.3 dps  angle  -45.0 deg   accel  0.01 -0.02  1.00 g   compass 271.5 W   tilt side   0.6 front  -1.1 deg
void Dashboard::printRobot(const RobotProxy &r, unsigned long now) {
  static const char *const names[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
  const fl::StatusReport &s = r.status();
  char buf[12];

  Serial.print(F("ROBOT "));
  printCode(r.code());
  Serial.print(F("  "));
  if (!r.heardEver())    Serial.print(F("WAITING "));
  else if (r.online())   Serial.print(F("ONLINE  "));
  else                   Serial.print(F("OFFLINE "));
  Serial.print(' ');
  if (!r.heardEver())                         Serial.print(F("--       "));
  else if (s.flags & fl::STATUS_FLAG_FAILSAFE) Serial.print(F("FAILSAFE "));
  else if (s.flags & fl::STATUS_FLAG_DRIVING)  Serial.print(F("moving   "));
  else                                         Serial.print(F("idle     "));

  Serial.print(F(" polls "));
  uint8_t n = printNum(r.okCount());
  Serial.print('/');
  n += 1 + printNum(r.missCount());
  pad(n, 8);
  Serial.print(F("seen "));
  if (r.heardEver()) {
    Serial.print(dtostrf((now - r.lastHeardMs()) / 1000.0, 4, 1, buf));
    Serial.print('s');
  } else {
    Serial.print(F("  --"));
    Serial.print(' ');
  }
  Serial.print(F("   cmd "));
  printCmdState(r);
  Serial.println();

  Serial.print(F("     "));
  if (!r.heardEver()) {
    Serial.println(F("no data yet"));
    return;
  }
  Serial.print(F("gyro "));
  printTenths(s.rate10, 6);
  Serial.print(F(" dps  angle "));
  printTenths(s.gyroAngle10, 6);
  Serial.print(F(" deg   accel "));
  Serial.print(dtostrf(s.ax / 1000.0, 5, 2, buf));
  Serial.print(dtostrf(s.ay / 1000.0, 6, 2, buf));
  Serial.print(dtostrf(s.az / 1000.0, 6, 2, buf));
  Serial.print(F(" g   compass "));
  if (s.flags & fl::STATUS_FLAG_COMPASS_OK) {
    printTenths(s.compass10, 5);
    Serial.print(' ');
    Serial.print(names[((s.compass10 + 225) / 450) % 8]);
  } else {
    Serial.print(F("  --  (no reading)"));
  }
  // Tilt from the accelerometer (does not drift): side = X axis, front = Y axis, from level.
  float ax = s.ax, ay = s.ay, az = s.az;
  Serial.print(F("   tilt side "));
  Serial.print(dtostrf(atan2(ax, sqrt(ay * ay + az * az)) * 180.0 / PI, 5, 1, buf));
  Serial.print(F(" front "));
  Serial.print(dtostrf(atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI, 5, 1, buf));
  Serial.print(F(" deg"));
  Serial.println();
}

// ---------- helpers ----------
void Dashboard::printCode(uint8_t code) {
  if (code < 10) Serial.print('0');
  Serial.print(code);
}

void Dashboard::printCmdState(const RobotProxy &r) {
  if (r.cmdState() == RobotProxy::CMD_IDLE) {
    Serial.print(F("none"));
    return;
  }
  Serial.print('#');
  Serial.print(r.cmdSeq());
  Serial.print(' ');
  Serial.print(r.label());
  switch (r.cmdState()) {
    case RobotProxy::CMD_PENDING:
      Serial.print(F(" PENDING"));
      if (r.tries() > 0) {
        Serial.print(F(" (try "));
        Serial.print(r.tries());
        Serial.print('/');
        Serial.print(RobotProxy::MAX_TRIES);
        Serial.print(')');
      }
      break;
    case RobotProxy::CMD_ACKED:  Serial.print(F(" DONE")); break;
    case RobotProxy::CMD_FAILED: Serial.print(F(" FAILED (no confirmation)")); break;
    default: break;
  }
}

void Dashboard::pad(uint8_t used, uint8_t width) {
  for (uint8_t i = used; i < width; i++) Serial.print(' ');
}

uint8_t Dashboard::printTenths(int16_t v10, uint8_t width) {
  char buf[12];
  dtostrf(v10 / 10.0, width, 1, buf);
  Serial.print(buf);
  return strlen(buf);
}

uint8_t Dashboard::printNum(unsigned long v) {
  char buf[12];
  ultoa(v, buf, 10);
  Serial.print(buf);
  return strlen(buf);
}
