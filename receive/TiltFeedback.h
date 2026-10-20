// TiltFeedback: the 4 tilt LEDs and the tilt buzzer from the motion sketch.
// Each LED gets brighter the further the robot tilts its way; the buzzer pitch rises with the
// total tilt (300 to 2000 Hz). Driven by the accelerometer, low-pass filtered.
// LED PWM and the buzzer tone run on the radio's 16 kHz timer tick (RfLink::setTickHook).
// Testing aid: remove this class (and its pins) when the robot gets motors.
#pragma once
#include <FieldLink.h>

class TiltFeedback {
public:
  TiltFeedback(uint8_t leftPin, uint8_t rightPin, uint8_t frontPin, uint8_t backPin,
               uint8_t buzzerPin, bool buzzerOn = true);

  void begin();                          // after the radio has started
  void update(float gx, float gy);       // accelerometer X and Y in g

  int levelLeft() const { return level_[0]; }    // 0 to 255, for printing
  int levelRight() const { return level_[1]; }
  int levelFront() const { return level_[2]; }
  int levelBack() const { return level_[3]; }
  int buzzHz() const { return buzzHz_; }

  // Tuning (same values as the motion sketch)
  static constexpr float TILT_ON_G = 0.12;     // below this tilt the LEDs stay off
  static constexpr float TILT_FULL_G = 0.80;   // at or above this an LED is fully bright
  static constexpr float FILTER_ALPHA = 0.15;
  static const bool LEFT_IS_POS_X = true;      // +X reading means tilting left
  static const bool FRONT_IS_POS_Y = false;    // +Y reading means tilting back
  static const int BUZZ_MIN_HZ = 300;
  static const int BUZZ_MAX_HZ = 2000;

private:
  static void tick();                    // runs in the timer interrupt
  static int toBrightness(float value);

  uint8_t pins_[4];
  uint8_t buzzerPin_;
  bool buzzerOn_;
  float filtX_ = 0, filtY_ = 0;
  bool filterReady_ = false;
  int level_[4] = {0, 0, 0, 0};
  int buzzHz_ = 0;

  // shared with the interrupt
  static volatile uint8_t *ledPort_[4];
  static uint8_t ledMask_[4];
  static volatile uint8_t duty_[4];      // 0 to 63
  static volatile uint8_t *buzzPort_;
  static uint8_t buzzMask_;
  static volatile uint16_t buzzStep_;
};
