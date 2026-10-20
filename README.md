# rf-field-control

![Status](https://img.shields.io/badge/status-active%20development-brightgreen)
![Updated](https://img.shields.io/badge/updated-October%202026-blue)
![Board](https://img.shields.io/badge/board-Arduino%20Uno-00979D?logo=arduino&logoColor=white)
![Language](https://img.shields.io/badge/language-C%2B%2B-00599C?logo=cplusplus&logoColor=white)
![Radio](https://img.shields.io/badge/radio-433%20MHz-orange)
![Robots](https://img.shields.io/badge/robots-up%20to%204-purple)

> [!NOTE]
> This project is under active development and is being updated regularly as of October 2026. Features, wiring and the radio protocol may change between versions.

A two-way 433 MHz radio system for running robots on a course. One Arduino Uno is the **field controller**: it pairs with up to 4 robots, sends them commands typed into the Serial Monitor, and shows a live dashboard of each robot's gyro, accelerometer and compass data. Each robot is an Arduino Uno that runs the commands and reports its sensors back.

The code is object-oriented so new movements can be added as small classes without touching the radio or pairing logic.

## Features

- **Pairing with approval.** Each robot has a two-digit code. A new robot announces itself, and the controller only accepts it after the operator types `pair <code>`.
- **Pairings survive resets.** Approved robots are saved in the controller's EEPROM and reconnect automatically after either side reboots.
- **Up to 4 robots** on one channel. The controller polls one robot at a time, so robots never transmit over each other.
- **Reliable commands.** Every command carries a sequence number. The robot confirms it in its next reply, the controller retries up to 5 times, and the robot runs each command only once even if it is received twice.
- **Live telemetry.** Gyro rate and angle, 3-axis acceleration and compass heading from every robot, about once per second each with 4 robots.
- **Failsafe.** A robot stops its motors if it has not heard from the controller for 3 seconds.
- **Noise-tolerant framing.** CRC-8 checked packets, and a decoder that finds real packets inside receiver noise.

## Hardware

| Part | Controller | Each robot |
|---|---|---|
| Arduino Uno | 1 | 1 |
| Parallax 433 MHz RF Transceiver (#27982) | 1 | 1 |
| Parallax LISY300 gyro | | 1 |
| Hitachi H48C 3-axis accelerometer | | 1 |
| Hitachi HM55B compass | | 1 |
| LEDs + 220 to 330 ohm resistors (optional) | 2 | |

## Wiring

### Field controller

| Module pin | Arduino pin |
|---|---|
| RF DATA | 8 |
| RF T/R | 9 |
| RF VIN / GND | 5V / GND |
| RF PDN, RSSI | not connected |
| Receive LED (optional, active low) | A4 |
| Transmit LED (optional, active low) | A5 |

The LEDs are wired 5V, resistor, LED anode, LED cathode, then the Arduino pin, so the pin pulls LOW to light them.

### Robot

| Module pin | Arduino pin |
|---|---|
| RF DATA | D2 |
| RF T/R | D4 |
| RF VIN / GND | 5V / GND |
| RF PDN, RSSI | not connected |
| LISY300 DOUT / SCLK / CS | D7 / D8 / D12 |
| LISY300 ST | GND |
| H48C CLK / DIO / CS | A0 / A1 / A2 |
| HM55B DIN + DOUT (tied together) | A3 |
| HM55B CLK / EN | A4 / D3 |
| All sensor VDD / GND | 5V / GND |
| Status LED | built-in LED on D13 |

> [!TIP]
> Leave A5 unconnected: its noise seeds the random time slot used for pairing.

> [!TIP]
> Pins free for motors: PWM on **5, 6, 9, 10**, plus digital pin 11. Pins 3 and 11 cannot do PWM because the radio uses Timer2.

## Repository layout

```
rf-field-control/
├── FieldLink/              shared radio library (source)
│   ├── library.properties
│   └── src/
├── FieldLink.zip           the same library, ready for "Add .ZIP Library"
├── data_and_transmit/      field controller sketch
└── receive/                robot sketch
```

## Setup

> [!IMPORTANT]
> Always upload the controller and every robot from the same version of this repo. Older and newer versions cannot talk to each other.

1. **Install the library once.** In the Arduino IDE: *Sketch > Include Library > Add .ZIP Library* and pick `FieldLink.zip`. (Or copy the `FieldLink` folder into `Documents/Arduino/libraries`.)
2. **Controller:** open `data_and_transmit/data_and_transmit.ino`, select *Arduino Uno*, upload.
3. **Each robot:** open `receive/receive.ino`, set `ROBOT_CODE` to a number from 0 to 99 (different for every robot), upload.
4. Open the controller's Serial Monitor at **115200 baud** with line ending **Newline**.
5. Power on a robot and keep it still for about 1.2 seconds while the gyro calibrates. The controller prints:
```
   [PAIR] Robot 42 wants to pair. Type: pair 42   (or: deny 42)
```
   Type `pair 42`.

## Controller commands

| Command | What it does |
|---|---|
| `move <code\|all> <left> <right>` | Set wheel power, -100 to 100 each |
| `stop <code\|all>` | Stop moving |
| `zero <code\|all>` | Reset the gyro angle to 0 |
| `pair <code>` | Approve a robot's pair request |
| `deny <code>` | Reject a request (the robot stays quiet for 30 s) |
| `unpair <code>` | Forget a robot |
| `list` | Print the status table now |
| `scan on\|off` | Look for new robots. Turn off once everyone is paired for faster polling. |
| `watch on\|off` | Print the status table every 2 s |
| `help` | List commands |

Example dashboard (values are illustrative):

```
==== FIELD CONTROLLER 100 ====  t=24.0s  scan OFF  paired 2/4  radio tx 78 rx 75 bad 0
ROBOT 07  ONLINE   moving    polls 51/0    seen  0.1s   cmd #2 move DONE
     gyro   12.3 dps  angle  -45.0 deg   accel  0.01 -0.02  1.00 g   compass  26.6 NE
ROBOT 13  ONLINE   idle      polls 16/0    seen  0.3s   cmd #1 stop DONE
     gyro    0.0 dps  angle    0.0 deg   accel  0.00  0.01  0.99 g   compass 271.5 W
Waiting to pair: 42   (pair <code> / deny <code>)
```

## How it works

The 433 MHz modules are half-duplex: one DATA pin carries both directions, and the T/R pin selects transmit (HIGH) or receive (LOW). Serial data runs at **2000 baud**, inverted, bit-banged by a 16 kHz Timer2 interrupt.

Only the controller starts conversations:

```
Pairing   controller --BEACON (broadcast)--------> unpaired robots
          robot ------PAIR_REQUEST (random slot)--> controller
          operator types "pair <code>"
          controller --PAIR_ACCEPT---------------> robot

Running   controller --POLL (+ optional command)--> robot
          robot ------STATUS (sensors, last command)--> controller
```

Every burst starts with a sync pulse and a short idle gap so the receiver locks on cleanly. A frame on the air is:

```
0xA5 0x5A | len | type | src | dst | seq | payload (0 to 16 bytes) | CRC-8
```

Robots use their code (0 to 99) as their address, controllers use 100 to 199, and 255 is broadcast.

| Timing | Value |
|---|---|
| One poll and reply | about 220 ms |
| Beacon (while scanning) | every 3 s, 655 ms listen window |
| Controller waits for a reply | 260 ms |
| Robot marked offline after | 3 missed polls |
| Robot failsafe stop | 3 s without a poll |
| Robot forgets its controller | 10 s without a poll |

## Code structure

**FieldLink** (shared library)

| Class / file | Role |
|---|---|
| `RfLink` | Half-duplex software serial on Timer2, T/R switching |
| `PacketRadio` | Sends and receives whole packets |
| `FrameCodec`, `FrameDecoder` | Framing, CRC-8, noise-tolerant decoding |
| `Protocol.h` | Message types, addresses, timing, `CommandFrame`, `StatusReport` |
| `Commands.h` | Command type numbers shared by both sides |
| `Indicator` | Status LED helper |

**Controller** (`data_and_transmit/`)

| Class | Role |
|---|---|
| `FieldController` | Beacons, polling, command delivery |
| `RobotRegistry` | Paired robots, stored in EEPROM |
| `RobotProxy` | One robot's state: link, last status, pending command |
| `PairingManager` | Pair requests waiting for approval |
| `Console`, `ConsoleCommand` | Serial Monitor input, one class per command |
| `RobotCommand` | Base class for commands sent to robots |
| `Dashboard` | All Serial Monitor output |

**Robot** (`receive/`)

| Class | Role |
|---|---|
| `RobotNode` | Pairing, answering polls, failsafe |
| `SensorSuite` | Gyro + accelerometer + compass, packed into each reply |
| `LisyGyro`, `H48cAccel`, `Hm55bCompass` | Sensor drivers (the compass is read without blocking) |
| `CommandDispatcher`, `CommandHandler` | Runs each command once by type |
| `DriveBase` | Motor interface. `DebugDrive` only prints until real motors are added. |

## Adding a movement

1. **`FieldLink/src/Commands.h`:** add a command number, for example `CMD_TURN = 4`.
2. **Controller:** in `ConsoleCommands`, copy `MoveCommand`, give it a new name and fill in its arguments in `build()`. Register it in `data_and_transmit.ino` with `console.add(&turnCmd)`.
3. **Robot:** in `CommandHandlers.h`, copy `MoveHandler`, return the new type from `type()` and do the work in `handle()`. Register it in `receive.ino` with `dispatcher.add(&turnHandler)`.
4. Reinstall FieldLink (or edit the installed copy) and upload both sketches.

To drive real motors, write a class that inherits from `DriveBase` and implements `move()`, `stop()` and `moving()`, then use it in place of `DebugDrive` in `receive.ino`.

## Notes

> [!WARNING]
> `tone()` and PWM on pins 3 and 11 do not work on either board, because the radio uses Timer2.

> [!WARNING]
> Any change to `FieldLink` must be uploaded to the controller **and** every robot, or they stop understanding each other.

- The compass heading is uncalibrated, and the gyro angle drifts over time. Use `zero` to reset it.
- If you run more than one controller, give each a different `CONTROLLER_ID` (100 to 199) in `data_and_transmit.ino`.

## Author

David Shapsis