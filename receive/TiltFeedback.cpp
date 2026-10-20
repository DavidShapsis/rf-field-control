#include "TiltFeedback.h"

volatile uint8_t *TiltFeedback::ledPort_[4];
uint8_t TiltFeedback::ledMask_[4];
volatile uint8_t TiltFeedback::duty_[4] = {0, 0, 0, 0};
volatile uint8_t *TiltFeedback::buzzPort_ = nullptr;
uint8_t TiltFeedback::buzzMask_ = 0;
volatile uint16_t TiltFeedback::buzzStep_ = 0;

TiltFeedback::TiltFeedback(uint8_t leftPin, uint8_t rightPin, uint8_t frontPin, uint8_t backPin,
                           uint8_t buzzerPin, bool buzzerOn)
  : pins_{leftPin, rightPin, frontPin, backPin}, buzzerPin_(buzzerPin), buzzerOn_(buzzerOn) {}

void TiltFeedback::begin() {
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(pins_[i], OUTPUT);
    digitalWrite(pins_[i], LOW);
    ledPort_[i] = portOutputRegister(digitalPinToPort(pins_[i]));
    ledMask_[i] = digitalPinToBitMask(pins_[i]);
  }
  pinMode(buzzerPin_, OUTPUT);
  digitalWrite(buzzerPin_, LOW);
  buzzPort_ = portOutputRegister(digitalPinToPort(buzzerPin_));
  buzzMask_ = digitalPinToBitMask(buzzerPin_);
  RfLink::setTickHook(tick);
}

// 16 kHz: LED software PWM (64 steps, 250 Hz) and the buzzer square wave.
void TiltFeedback::tick() {
  static uint8_t phase = 0;
  static uint16_t buzzPhase = 0;

  phase = (phase + 1) & 63;
  for (uint8_t i = 0; i < 4; i++) {
    if (phase < duty_[i]) *ledPort_[i] |= ledMask_[i];
    else                  *ledPort_[i] &= ~ledMask_[i];
  }

  if (!buzzPort_) return;
  uint16_t step = buzzStep_;
  if (step) {
    buzzPhase += step;
    if (buzzPhase & 0x8000) *buzzPort_ |= buzzMask_;
    else                    *buzzPort_ &= ~buzzMask_;
  } else {
    *buzzPort_ &= ~buzzMask_;
  }
}

// 0 below TILT_ON_G, 255 at or above TILT_FULL_G, linear between.
int TiltFeedback::toBrightness(float value) {
  if (value <= TILT_ON_G) return 0;
  float f = (value - TILT_ON_G) / (TILT_FULL_G - TILT_ON_G);
  if (f > 1.0) f = 1.0;
  return (int)(f * 255.0);
}

void TiltFeedback::update(float gx, float gy) {
  if (!filterReady_) {                       // start from the first reading, no ramp-up
    filtX_ = gx;
    filtY_ = gy;
    filterReady_ = true;
  } else {
    filtX_ += FILTER_ALPHA * (gx - filtX_);
    filtY_ += FILTER_ALPHA * (gy - filtY_);
  }

  float sideways = LEFT_IS_POS_X ? filtX_ : -filtX_;    // positive = tilting left
  float forward = FRONT_IS_POS_Y ? filtY_ : -filtY_;    // positive = tilting forward

  level_[0] = (sideways > 0) ? toBrightness(sideways) : 0;    // left
  level_[1] = (sideways < 0) ? toBrightness(-sideways) : 0;   // right
  level_[2] = (forward > 0) ? toBrightness(forward) : 0;      // front
  level_[3] = (forward < 0) ? toBrightness(-forward) : 0;     // back
  for (uint8_t i = 0; i < 4; i++) duty_[i] = level_[i] >> 2;  // 0..255 down to 0..63

  // Buzzer pitch follows the total tilt. Silent below TILT_ON_G.
  float tilt = sqrt(sideways * sideways + forward * forward);
  if (!buzzerOn_ || tilt <= TILT_ON_G) {
    buzzHz_ = 0;
  } else {
    float f = (tilt - TILT_ON_G) / (TILT_FULL_G - TILT_ON_G);
    if (f > 1.0) f = 1.0;
    buzzHz_ = BUZZ_MIN_HZ + (int)(f * (BUZZ_MAX_HZ - BUZZ_MIN_HZ));
  }
  uint16_t step = buzzHz_ > 0 ? (uint16_t)(buzzHz_ * 4.096) : 0;   // 65536 / 16000
  noInterrupts();                            // 16-bit value shared with the interrupt
  buzzStep_ = step;
  interrupts();
}
