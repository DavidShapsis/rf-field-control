// David Shapsis
// 10/20/26
//
// FIELD ROBOT (Arduino Uno)
// Pairs with the field controller over the 433 MHz RF link, runs the commands it receives and
// reports its gyro, accelerometer and compass readings every time it is polled.
//
// Needs the FieldLink library (Sketch > Include Library > Add .ZIP Library > FieldLink.zip).
//
// Before uploading, set ROBOT_CODE below. Every robot needs a different code (00 to 99).
//
// Wiring
//   RF module      DATA -> D2    T/R -> D4    VIN -> 5V   GND -> GND   PDN, RSSI unconnected
//   LISY300 gyro   DOUT -> D7    SCLK -> D8   CS -> D12   VDD -> 5V    GND -> GND   ST -> GND
//   H48C accel     CLK -> A0     DIO -> A1    CS -> A2    VDD -> 5V    VSS -> GND   (0G unconnected)
//   HM55B compass  DIN and DOUT (tied) -> A3  CLK -> A4   EN -> D3     VDD -> 5V    VSS -> GND
//   Status LED     built-in LED on D13. Slow blink = not paired, on = paired, flicker = command.
//   Free for motors later: PWM pins 5, 6, 9, 10 and digital pin 11.
//   (Pins 3 and 11 cannot do PWM because the radio uses Timer2.)
//   A5 is left unconnected: its noise seeds the random pairing slot.
//
// Serial Monitor (optional, for debugging): 115200 baud.
//
// Code layout:
//   RobotNode          link side: pairing, polls, status replies, failsafe
//   SensorSuite        gyro + accelerometer + compass, packed into the status reply
//   CommandDispatcher  runs each command once, by type number
//   CommandHandlers    one class per command (add new movements there)
//   DriveBase          motor interface; DebugDrive just prints for now

#include <FieldLink.h>
#include "LisyGyro.h"
#include "H48cAccel.h"
#include "Hm55bCompass.h"
#include "SensorSuite.h"
#include "DriveBase.h"
#include "CommandHandlers.h"
#include "RobotNode.h"

// ---------- Settings ----------
const uint8_t ROBOT_CODE = 42;          // 0 to 99, unique per robot
static_assert(ROBOT_CODE <= fl::ROBOT_CODE_MAX, "ROBOT_CODE must be 0 to 99");

const int RF_DATA_PIN = 2;
const int RF_TR_PIN = 4;
const int GYRO_DOUT_PIN = 7;
const int GYRO_SCLK_PIN = 8;
const int GYRO_CS_PIN = 12;
const int ACCEL_CLK_PIN = A0;
const int ACCEL_DIO_PIN = A1;
const int ACCEL_CS_PIN = A2;
const int COMPASS_DIO_PIN = A3;
const int COMPASS_CLK_PIN = A4;
const int COMPASS_EN_PIN = 3;
const int LED_PIN = 13;
const int SEED_PIN = A5;

// ---------- Objects ----------
RfLink rfLink(RF_DATA_PIN, RF_TR_PIN);
PacketRadio radio(rfLink);
Indicator linkLed(LED_PIN);

LisyGyro gyro(GYRO_DOUT_PIN, GYRO_SCLK_PIN, GYRO_CS_PIN);
H48cAccel accel(ACCEL_CLK_PIN, ACCEL_DIO_PIN, ACCEL_CS_PIN);
Hm55bCompass compass(COMPASS_DIO_PIN, COMPASS_CLK_PIN, COMPASS_EN_PIN);
SensorSuite sensors(gyro, accel, compass);

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

  node.begin();
}

void loop() {
  sensors.update();
  node.update();
}
