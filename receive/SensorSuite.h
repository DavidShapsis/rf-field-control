// SensorSuite: owns the robot's sensors and packs their readings into a StatusReport.
// To add a sensor, give it begin()/update(), add it here, and add a field to StatusReport.
#pragma once
#include <FieldLink.h>
#include "LisyGyro.h"
#include "H48cAccel.h"
#include "Hm55bCompass.h"

class SensorSuite {
public:
  SensorSuite(LisyGyro &gyro, H48cAccel &accel, Hm55bCompass &compass)
    : gyro_(gyro), accel_(accel), compass_(compass) {}

  void begin() {
    gyro_.begin();
    accel_.begin();
    compass_.begin();
  }

  void startCalibration() { gyro_.startCalibration(); }   // keep the robot still

  // Returns true when the accelerometer took a new reading.
  bool update() {
    gyro_.update();
    bool newAccel = accel_.update();
    compass_.update();
    return newAccel;
  }

  // Fills the sensor fields; the link fields (ack, flags, uptime) are added by RobotNode.
  void fill(fl::StatusReport &s) const {
    s.rate10 = clamp16(gyro_.takeAverageRate() * 10.0);   // average since the last report
    s.gyroAngle10 = clamp16(gyro_.angle() * 10.0);
    s.ax = clamp16(accel_.x() * 1000.0);
    s.ay = clamp16(accel_.y() * 1000.0);
    s.az = clamp16(accel_.z() * 1000.0);
    s.compass10 = clamp16(compass_.heading() * 10.0);
    if (compass_.ok()) s.flags |= fl::STATUS_FLAG_COMPASS_OK;
  }

  LisyGyro &gyro() { return gyro_; }
  const H48cAccel &accel() const { return accel_; }

private:
  static int16_t clamp16(float v) {
    if (v > 32767.0) return 32767;
    if (v < -32768.0) return -32768;
    return (int16_t)(v >= 0 ? v + 0.5 : v - 0.5);   // round to nearest
  }
  LisyGyro &gyro_;
  H48cAccel &accel_;
  Hm55bCompass &compass_;
};
