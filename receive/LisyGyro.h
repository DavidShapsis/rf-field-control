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
  // Prints gyro debug lines to Serial (see printDebug in LisyGyro.cpp). For testing only.
  void setDebug(bool on) { debug_ = on; }

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
  // While the robot is not commanded to drive, the offset follows any reading within this band.
  // (An earlier "7 count shift" in the resting value turned out to be the transmit disturbance:
  // -13 counts for 140 of every 250 ms averages about -7. With that time skipped, the resting
  // value stayed within 0.3 counts for 90 s, so the band is back to 6.) Hand turns slower than
  // about 6 dps with no drive command are partly absorbed; turns while driving never are.
  static const int REZERO_BAND = 6;
  static constexpr float REZERO_RATE = 0.01;     // per 20 ms batch: follows over about 2 s
  // The robot's radio stops the loop for about 140 ms on every reply, so the gyro is not read
  // then, and on the real robot the gyro reads about 13 counts low for about 90 ms after each
  // transmit (measured with the debug output). A pause longer than GAP_US between readings is
  // treated as a transmit: readings for SETTLE_US after it are skipped, and the whole pause is
  // covered with the average of the confirmed rates before and after it (0 at rest).
  static const unsigned long GAP_US = 20000;     // only a radio transmit stops the loop this long
  static const unsigned long SETTLE_US = 150000; // measured recovery about 90 ms, plus margin
  static const uint8_t HIST_BINS = 64;     // 16 counts per bin
  static const int CAL_WINDOW = 32;        // average only readings within this of the typical one

  // PASS1 finds the typical reading (histogram peak), PASS2 averages readings near it.
  enum CalState : uint8_t { CAL_WAITING, CAL_PASS1, CAL_PASS2, CAL_DONE };

  int readRaw();
  void calFinish();
  void addDegrees(float deg);
  void closeGap();
  void processBatch(float diff, unsigned long now);
  void printDebug(unsigned long ms, uint8_t n, int m, int lo, int hi, float diff, float added);

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
  unsigned long lastReadUs_ = 0;           // time of the previous reading, to spot pauses
  unsigned long settleStartUs_ = 0;
  bool settling_ = false;                  // skipping readings right after a pause
  bool gapOpen_ = false;                   // a pause waiting for the rate after it
  float gapSec_ = 0;
  float gapRateBefore_ = 0;
  bool afterPause_ = false;                // next batch is the first after a pause (debug)
  bool debug_ = false;
  unsigned long debugLastMs_ = 0;

  CalState calState_ = CAL_DONE;
  bool calibrated_ = false, calReport_ = false;
  unsigned long calStartMs_ = 0, calEndMs_ = 0, calDurationMs_ = 0;
  uint8_t calTries_ = 0;
  uint16_t calHist_[HIST_BINS];
  int calCenter_ = 512;
  long calSum_ = 0, calUsed_ = 0, calFrames_ = 0;
  int calMin_ = 0, calMax_ = 0;
};
