// David Shapsis
// 10/20/26
//
// FIELD ROBOT (Arduino Uno)
// Pairs with the field controller over the 433 MHz RF link, runs the commands it receives and
// reports its gyro, accelerometer and compass readings every time it is polled.
// For testing it also keeps the tilt LEDs and tilt buzzer from the motion sketch.
//
// Needs the FieldLink library (Sketch > Include Library > Add .ZIP Library > FieldLink.zip).
//
// Before uploading, set ROBOT_CODE below. Every robot needs a different code (00 to 99).
//
// Wiring (same sensor, LED and buzzer pins as the motion sketch)
//   RF module      DATA -> D5    T/R -> D12   VIN -> 5V   GND -> GND   PDN, RSSI unconnected
//   H48C accel     CLK -> D2     DIO -> D3    CS -> D4    VDD -> 5V    VSS -> GND   (0G unconnected)
//   HM55B compass  DIN and DOUT (tied) -> D6  CLK -> D7   EN -> D8     VDD -> 5V    VSS -> GND
//   LISY300 gyro   DOUT -> D9    SCLK -> D10  CS -> D11   VDD -> 5V    GND -> GND   ST -> GND
//   Tilt LEDs      left A0, right A1, front A2, back A3 (resistor, LED, GND)
//   Buzzer         D13
//   Link LED       optional, A4, active low: 5V -> resistor -> LED -> A4.
//                  Slow blink = not paired, on = paired, off flicker = command received.
//   A5 is left unconnected: its noise seeds the random pairing slot.
//
// Serial Monitor (optional, for debugging): 115200 baud.
//
// Code layout:
//   RobotNode          link side: pairing, polls, status replies, failsafe
//   SensorSuite        gyro + accelerometer + compass, packed into the status reply
//   TiltFeedback       tilt LEDs and buzzer (testing aid, remove later)
//   CommandDispatcher  runs each command once, by type number
//   CommandHandlers    one class per command (add new movements there)
//   DriveBase          motor interface; DebugDrive just prints for now

#include <FieldLink.h>
#include "LisyGyro.h"
#include "H48cAccel.h"
#include "Hm55bCompass.h"
#include "SensorSuite.h"
#include "TiltFeedback.h"
#include "DriveBase.h"
#include "CommandHandlers.h"
#include "RobotNode.h"

// ---------- Settings ----------
const uint8_t ROBOT_CODE = 42;          // 0 to 99, unique per robot
static_assert(ROBOT_CODE <= fl::ROBOT_CODE_MAX, "ROBOT_CODE must be 0 to 99");
const bool BUZZER_ON = true;            // false = tilt LEDs only, no sound

// RF module
const int RF_DATA_PIN = 5;
const int RF_TR_PIN = 12;
// H48C accelerometer
const int H48C_CLK = 2;
const int H48C_DIO = 3;
const int H48C_CS = 4;
// HM55B compass (Din and Dout tied together on one pin)
const int HM55B_DIO = 6;
const int HM55B_CLK = 7;
const int HM55B_EN = 8;
// LISY300 gyro
const int GYRO_DOUT = 9;
const int GYRO_SCLK = 10;
const int GYRO_CS = 11;
// Tilt LEDs and buzzer
const int LED_LEFT = A0;
const int LED_RIGHT = A1;
const int LED_FRONT = A2;
const int LED_BACK = A3;
const int BUZZER_PIN = 13;
// Link LED (active low) and random seed
const int LINK_LED_PIN = A4;
const int SEED_PIN = A5;

// ---------- Objects ----------
RfLink rfLink(RF_DATA_PIN, RF_TR_PIN);
PacketRadio radio(rfLink);
Indicator linkLed(LINK_LED_PIN, true);

LisyGyro gyro(GYRO_DOUT, GYRO_SCLK, GYRO_CS);
H48cAccel accel(H48C_CLK, H48C_DIO, H48C_CS);
Hm55bCompass compass(HM55B_DIO, HM55B_CLK, HM55B_EN);
SensorSuite sensors(gyro, accel, compass);
TiltFeedback tilt(LED_LEFT, LED_RIGHT, LED_FRONT, LED_BACK, BUZZER_PIN, BUZZER_ON);

DebugDrive drive;                        // replace with your motor class

StopHandler stopHandler(drive);
MoveHandler moveHandler(drive);
ZeroGyroHandler zeroGyroHandler(gyro);
CommandDispatcher dispatcher;

RobotNode node(ROBOT_CODE, radio, dispatcher, sensors, drive, linkLed);

void setup() {
  Serial.begin(115200);
  randomSeed(analogRead(SEED_PIN) ^ ((unsigned long)ROBOT_CODE << 8) ^ micros());

  linkLed.begin();
  drive.begin();
  sensors.begin();
  Serial.println(F("Calibrating gyro, keep the robot still..."));
  sensors.calibrate();
  Serial.print(F("Gyro offset: "));
  Serial.println(gyro.offset());

  dispatcher.add(&stopHandler);
  dispatcher.add(&moveHandler);
  dispatcher.add(&zeroGyroHandler);

  node.begin();                          // starts the radio (and its 16 kHz timer)
  tilt.begin();                          // LEDs and buzzer run on that timer
}

void loop() {
  if (sensors.update()) tilt.update(accel.x(), accel.y());
  node.update();
}
