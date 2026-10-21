// LisyGyro: Parallax LISY300 yaw gyro (TI ADC101S021 inside). Gives rate and integrated angle.
//
// Reading: the same digitalWrite() read as the original motion sketch ((raw >> 3) & 0x3FF).
// Calibration: runs in the background from the normal readings, starting 3 s after boot (readings
// in the first second or so after boot were wrong on the real gyro). It finds the most common
// reading first and averages only readings near it, so occasional odd readings can't skew it.
// Running: readings are grouped into 20 ms batches and the median of each batch is used, so the
// scattered readings seen on the real gyro can't bias the rate. A +/-3 count zero band and slow
// re-zeroing while the robot is known to be still stop the angle from drifting at rest.
// Spike rejection: a rate only counts once two batches in a row agree on its direction. Single
// 20 ms spikes (about -21 dps, seen regularly on the real robot at rest) are ignored; a real turn
// lasts many batches and loses nothing because the first batch is added once the second confirms.
#pragma once
#include <Arduino.h>

class LisyGyro {
public:
  LisyGyro(uint8_t doutPin, uint8_t sclkPin, uint8_t csPin)
    : dout_(doutPin), sclk_(sclkPin), cs_(csPin) {}

  void begin();
  // Starts a background calibration: waits delayMs, then measures for durationMs using the
  // normal update() readings. Keep the robot still. Rate and angle read 0 until it finishes.
  void startCalibration(unsigned long delayMs = 3000, unsigned long durationMs = 2000);
  bool calibrating() const { return calState_ != CAL_DONE; }
  bool takeCalibrationReport();            // true once, right after a calibration finishes
  void update();                           // call often; integrates the angle

  float rateDps() const { return rate_; }  // latest confirmed rate; positive = counter-clockwise
  // Average rate since the last call (degrees actually added to the angle / time). This is what
  // the robot reports, so the dashboard rate matches the angle instead of one 20 ms snapshot.
  float takeAverageRate();
  float angle() const { return angle_; }   // degrees turned since boot or the last zero
  void zeroAngle() { angle_ = 0; }
  // Tell the gyro whether the robot is commanded to stand still. While it is, and the gyro reads
  // close to zero, the offset slowly follows the resting value.
  void setStationary(bool still) { stationary_ = still; }

  // Calibration report
  int offset() const { return (int)(offsetF_ + 0.5); }
  int calMin() const { return calMin_; }   // range of the readings that were averaged
  int calMax() const { return calMax_; }
  long calUsed() const { return calUsed_; }
  long calFrames() const { return calFrames_; }
  bool calibrated() const { return calibrated_; }

private:
  static constexpr float DPS_PER_COUNT = 0.9767;
  static const int DEADBAND = 3;           // counts treated as zero (noise)
  static const unsigned long BATCH_US = 20000;   // readings per rate update: one 20 ms batch
  static const uint8_t BATCH_MAX = 32;
  static const int REZERO_BAND = 6;        // re-zero only when this close to zero (about 6 dps)
  static constexpr float REZERO_RATE = 0.02;     // per batch: follows over about 1 s
  static const uint8_t HIST_BINS = 64;     // 16 counts per bin
  static const int CAL_WINDOW = 32;        // average only readings within this of the typical one

  // PASS1 finds the typical reading (histogram peak), PASS2 averages readings near it.
  enum CalState : uint8_t { CAL_WAITING, CAL_PASS1, CAL_PASS2, CAL_DONE };

  int readRaw();
  void calFinish();
  void addDegrees(float deg);

  uint8_t dout_, sclk_, cs_;
  float offsetF_ = 512;
  float rate_ = 0, angle_ = 0;
  unsigned long lastMicros_ = 0;
  bool stationary_ = true;
  int8_t runSign_ = 0;                     // direction of the confirmed turn in progress, 0 = none
  bool pending_ = false;                   // one batch over the zero band, waiting for a second
  int8_t pendingSign_ = 0;
  float pendingDeg_ = 0;                   // degrees that batch would add
  float travelDeg_ = 0;                    // degrees added since the last takeAverageRate()
  unsigned long travelStartUs_ = 0;
  int batch_[BATCH_MAX];
  uint8_t batchN_ = 0;
  unsigned long batchStart_ = 0;

  CalState calState_ = CAL_DONE;
  bool calibrated_ = false, calReport_ = false;
  unsigned long calStartMs_ = 0, calEndMs_ = 0, calDurationMs_ = 0;
  uint8_t calTries_ = 0;
  uint16_t calHist_[HIST_BINS];
  int calCenter_ = 512;
  long calSum_ = 0, calUsed_ = 0, calFrames_ = 0;
  int calMin_ = 0, calMax_ = 0;
};
