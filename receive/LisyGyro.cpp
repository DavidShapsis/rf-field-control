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
  unsigned long nowMs = millis();

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

  int m = medianOf(batch_, batchN_);
  batchN_ = 0;
  batchStart_ = now;

  float diff = m - offsetF_;
  // Robot standing still and gyro close to zero: let the offset follow the resting value.
  if (stationary_ && fabs(diff) <= REZERO_BAND) offsetF_ += REZERO_RATE * diff;

  float dt = (now - lastMicros_) / 1000000.0;
  lastMicros_ = now;

  if (fabs(diff) <= DEADBAND) {           // nothing happening
    rate_ = 0;
    runSign_ = 0;
    pending_ = false;
    return;
  }
  float r = diff * DPS_PER_COUNT;
  int8_t sign = (r > 0) ? 1 : -1;
  float deg = r * dt;

  if (runSign_ == sign) {                 // turn already confirmed: keep adding
    addDegrees(deg);
    rate_ = r;
  } else if (pending_ && pendingSign_ == sign) {   // second batch in a row: it's a real turn
    addDegrees(pendingDeg_ + deg);        // include the first batch too
    pending_ = false;
    runSign_ = sign;
    rate_ = r;
  } else {                                // first batch over the band: wait for confirmation
    pending_ = true;
    pendingSign_ = sign;
    pendingDeg_ = deg;
    runSign_ = 0;
    rate_ = 0;
  }
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
