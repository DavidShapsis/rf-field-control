#include "LisyGyro.h"

void LisyGyro::begin() {
  pinMode(sclk_, OUTPUT);
  pinMode(cs_, OUTPUT);
  pinMode(dout_, INPUT);
  digitalWrite(sclk_, HIGH);              // clock idles high
  digitalWrite(cs_, HIGH);
  lastMicros_ = micros();
}

// Same read as the original motion sketch: clock idles high, 16 clocks, sampled on the rising
// edge, data = (raw >> 3) & 0x3FF.
int LisyGyro::readRaw() {
  unsigned int raw = 0;
  digitalWrite(cs_, LOW);
  for (uint8_t i = 0; i < 16; i++) {
    digitalWrite(sclk_, LOW);
    delayMicroseconds(1);
    digitalWrite(sclk_, HIGH);
    raw = (raw << 1) | digitalRead(dout_);
  }
  digitalWrite(cs_, HIGH);
  return (raw >> 3) & 0x03FF;
}

void LisyGyro::startCalibration(unsigned long delayMs, unsigned long durationMs) {
  calState_ = CAL_WAITING;
  calStartMs_ = millis() + delayMs;
  calDurationMs_ = durationMs;
  calTries_ = 0;
  rate_ = 0;
  angle_ = 0;
}

bool LisyGyro::takeCalibrationReport() {
  if (!calReport_) return false;
  calReport_ = false;
  return true;
}

void LisyGyro::calFinish() {
  const long minUsed = 50;
  bool ok = calUsed_ >= minUsed;
  if (!ok && ++calTries_ < 3) {           // too few readings near the typical one: try again
    calState_ = CAL_WAITING;
    calStartMs_ = millis();
    return;
  }
  calibrated_ = ok;
  if (ok) offsetF_ = (float)calSum_ / calUsed_;
  calState_ = CAL_DONE;
  calReport_ = true;
  rate_ = 0;
  angle_ = 0;
  batchN_ = 0;
  runSign_ = 0;
  pending_ = false;
  gapOpen_ = false;
  travelDeg_ = 0;
  lastMicros_ = micros();
  batchStart_ = lastMicros_;
  travelStartUs_ = lastMicros_;
}

// Median of a small array (sorted in place).
static int medianOf(int *v, uint8_t n) {
  for (uint8_t i = 1; i < n; i++) {
    int x = v[i];
    int8_t j = i - 1;
    while (j >= 0 && v[j] > x) { v[j + 1] = v[j]; j--; }
    v[j + 1] = x;
  }
  return (n & 1) ? v[n / 2] : (v[n / 2 - 1] + v[n / 2] + 1) / 2;
}

void LisyGyro::update() {
  int v = readRaw();
  unsigned long nowUs = micros();
  unsigned long nowMs = millis();

  // Pause since the previous reading (the radio blocks the loop while it transmits): skip the
  // readings for SETTLE_US, then cover the whole pause with the average rate across it.
  if (nowUs - lastReadUs_ > GAP_US) {
    settling_ = true;
    settleStartUs_ = nowUs;
  }
  lastReadUs_ = nowUs;
  if (settling_) {
    if (nowUs - settleStartUs_ < SETTLE_US) return;
    settling_ = false;
    if (calState_ == CAL_DONE) {
      // The pause is added once the rate after it is known (see closeGap): averaging the rates
      // before and after it splits the difference when a turn starts or stops during the pause.
      closeGap();                         // an earlier pause still open: settle it first
      gapOpen_ = true;
      gapRateBefore_ = rate_;
      gapSec_ = 0;
      gapSec_ += (nowUs - lastMicros_) / 1000000.0;
      lastMicros_ = nowUs;
      batchN_ = 0;
      batchStart_ = nowUs;
      pending_ = false;
      afterPause_ = true;
    }
  }

  if (calState_ != CAL_DONE) {
    if (calState_ == CAL_WAITING) {
      if ((long)(nowMs - calStartMs_) >= 0) {
        calState_ = CAL_PASS1;
        calEndMs_ = nowMs + calDurationMs_ / 2;
        for (uint8_t i = 0; i < HIST_BINS; i++) calHist_[i] = 0;
      }
    } else if (calState_ == CAL_PASS1) {
      calHist_[v >> 4]++;
      if ((long)(nowMs - calEndMs_) >= 0) {
        uint8_t peak = 0;
        for (uint8_t i = 1; i < HIST_BINS; i++) {
          if (calHist_[i] > calHist_[peak]) peak = i;
        }
        calCenter_ = peak * 16 + 8;
        calState_ = CAL_PASS2;
        calEndMs_ = nowMs + calDurationMs_ / 2;
        calSum_ = calUsed_ = calFrames_ = 0;
        calMin_ = 1023;
        calMax_ = 0;
      }
    } else {                              // CAL_PASS2
      calFrames_++;
      if (abs(v - calCenter_) <= CAL_WINDOW) {
        calSum_ += v;
        calUsed_++;
        if (v < calMin_) calMin_ = v;
        if (v > calMax_) calMax_ = v;
      }
      if ((long)(nowMs - calEndMs_) >= 0) calFinish();
    }
    lastMicros_ = micros();
    return;
  }

  // Collect a 20 ms batch, then use its median.
  if (batchN_ < BATCH_MAX) batch_[batchN_++] = v;
  unsigned long now = micros();
  if (now - batchStart_ < BATCH_US) return;

  uint8_t n = batchN_;
  int m = medianOf(batch_, n);            // also sorts the batch: batch_[0] = lowest
  int lo = batch_[0], hi = batch_[n - 1];
  batchN_ = 0;
  batchStart_ = now;

  float before = angle_;
  float diff = m - offsetF_;
  processBatch(diff, now);
  if (debug_) printDebug(nowMs, n, m, lo, hi, diff, angle_ - before);
  afterPause_ = false;
}

void LisyGyro::processBatch(float diff, unsigned long now) {
  // Robot standing still and gyro close to zero: let the offset follow the resting value.
  if (stationary_ && fabs(diff) <= REZERO_BAND) offsetF_ += REZERO_RATE * diff;

  float dt = (now - lastMicros_) / 1000000.0;
  lastMicros_ = now;

  if (fabs(diff) <= DEADBAND) {           // nothing happening
    rate_ = 0;
    runSign_ = 0;
    pending_ = false;
    closeGap();
    return;
  }
  float r = diff * DPS_PER_COUNT;
  int8_t sign = (r > 0) ? 1 : -1;
  float deg = r * dt;

  if (runSign_ == sign) {                 // turn already confirmed: keep adding
    addDegrees(deg);
    rate_ = r;
    closeGap();
  } else if (pending_ && pendingSign_ == sign) {   // second batch in a row: it's a real turn
    addDegrees(pendingDeg_ + deg);        // include the first batch too
    pending_ = false;
    runSign_ = sign;
    rate_ = r;
    closeGap();
  } else {                                // first batch over the band: wait for confirmation
    pending_ = true;
    pendingSign_ = sign;
    pendingDeg_ = deg;
    runSign_ = 0;
    rate_ = 0;
  }
}

// Debug line, only for batches that are not plain rest (outside the zero band, or that changed
// the angle), plus one status line per second. Kept short so Serial never blocks the loop.
//   g <ms> <readings> <median> <lowest> <highest> <counts from rest> <deg added> [P]
//   (P = first batch after a radio pause)
void LisyGyro::printDebug(unsigned long ms, uint8_t n, int m, int lo, int hi, float diff,
                          float added) {
  if (fabs(diff) > DEADBAND || added != 0) {
    Serial.print(F("g "));
    Serial.print(ms);
    Serial.print(' ');
    Serial.print(n);
    Serial.print(' ');
    Serial.print(m);
    Serial.print(' ');
    Serial.print(lo);
    Serial.print(' ');
    Serial.print(hi);
    Serial.print(' ');
    Serial.print(diff, 1);
    Serial.print(' ');
    Serial.print(added, 2);
    if (afterPause_) Serial.print(F(" P"));
    Serial.println();
  }
  if (ms - debugLastMs_ >= 1000) {
    debugLastMs_ = ms;
    Serial.print(F("s "));
    Serial.print(ms);
    Serial.print(F(" angle "));
    Serial.print(angle_, 1);
    Serial.print(F(" rest "));
    Serial.println(offsetF_, 1);
  }
}

// Adds a pause once the confirmed rate after it is known: the average of the rates before and
// after it (both 0 at rest, so a pause at rest adds nothing).
void LisyGyro::closeGap() {
  if (!gapOpen_) return;
  addDegrees((gapRateBefore_ + rate_) * 0.5 * gapSec_);
  gapOpen_ = false;
}

void LisyGyro::addDegrees(float deg) {
  angle_ += deg;
  travelDeg_ += deg;
}

float LisyGyro::takeAverageRate() {
  unsigned long now = micros();
  float dt = (now - travelStartUs_) / 1000000.0;
  float r = (dt > 0.001) ? travelDeg_ / dt : 0;
  travelDeg_ = 0;
  travelStartUs_ = now;
  return r;
}
