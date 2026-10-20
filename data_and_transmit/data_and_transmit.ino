// David Shapsis
// 10/20/26
//
// FIELD CONTROLLER (Arduino Uno)
// The course dashboard. Pairs with up to 4 robots over the 433 MHz RF link, polls each one for
// its gyro, accelerometer and compass data, and sends the commands typed into the Serial Monitor.
//
// Needs the FieldLink library (Sketch > Include Library > Add .ZIP Library > FieldLink.zip).
//
// RF wiring: module DATA -> pin 8, T/R -> pin 9, GND -> GND, VIN -> 5V, PDN and RSSI unconnected
// LEDs (optional, active low, resistor from 5V to the anode):  A4 = packet received, A5 = transmitting
//
// Serial Monitor: 115200 baud, line ending "Newline". Type help for the command list.
//
// Code layout:
//   FieldController   radio side: beacons, pairing, polling, command delivery
//   RobotRegistry     paired robots (EEPROM)        RobotProxy      one robot's state
//   PairingManager    pair requests waiting for approval
//   Dashboard         everything printed            Console         typed commands
//   ConsoleCommands   one class per command (add new robot movements there)

#include <EEPROM.h>
#include <FieldLink.h>
#include "FieldController.h"
#include "Dashboard.h"
#include "Console.h"
#include "ConsoleCommands.h"

// ---------- Settings ----------
const uint8_t CONTROLLER_ID = 100;      // 100 to 199; give each controller its own if you run several
const int RF_DATA_PIN = 8;
const int RF_TR_PIN = 9;
const int LED_RX_PIN = A4;
const int LED_TX_PIN = A5;
const bool LEDS_ACTIVE_LOW = true;

// ---------- Objects ----------
RfLink rfLink(RF_DATA_PIN, RF_TR_PIN);
PacketRadio radio(rfLink);
Indicator rxLed(LED_RX_PIN, LEDS_ACTIVE_LOW);
Indicator txLed(LED_TX_PIN, LEDS_ACTIVE_LOW);

RobotRegistry registry;
PairingManager pairing;
FieldController controller(CONTROLLER_ID, radio, registry, pairing);
Dashboard dashboard(controller, registry, pairing, radio);
Console console;

PairCommand   pairCmd(controller);
DenyCommand   denyCmd(controller);
UnpairCommand unpairCmd(controller);
ListCommand   listCmd(dashboard);
ScanCommand   scanCmd(controller);
WatchCommand  watchCmd(dashboard);
MoveCommand   moveCmd(controller);
StopCommand   stopCmd(controller);
ZeroCommand   zeroCmd(controller);

void setup() {
  Serial.begin(115200);

  rxLed.begin();
  txLed.begin();
  radio.setIndicators(&rxLed, &txLed);

  registry.load();
  controller.setListener(&dashboard);
  controller.begin();

  console.add(&moveCmd);
  console.add(&stopCmd);
  console.add(&zeroCmd);
  console.add(&pairCmd);
  console.add(&denyCmd);
  console.add(&unpairCmd);
  console.add(&listCmd);
  console.add(&scanCmd);
  console.add(&watchCmd);

  dashboard.begin();
}

void loop() {
  console.update();
  controller.update();
  dashboard.update();
  rxLed.update();
}
