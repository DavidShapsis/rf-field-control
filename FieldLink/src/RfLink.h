// FieldLink - RfLink
// Half-duplex software serial port for the Parallax 433 MHz transceiver (#27982).
// One DATA pin carries both directions and the T/R pin selects the direction
// (HIGH = transmit, LOW = receive). Bits are clocked by a 16 kHz Timer2 interrupt.
//
// Timer2 is taken over, so tone() and analogWrite() on pins 3 and 11 stop working.
// PWM on pins 5, 6 (Timer0) and 9, 10 (Timer1) is unaffected.
#pragma once
#include <Arduino.h>

class RfLink {
public:
  // 16 kHz / 8 ticks per bit = 2000 baud. Must match on every device.
  static const uint8_t TICKS_PER_BIT = 8;
  static const uint8_t SWITCH_SETTLE_MS = 5;   // after flipping T/R (datasheet gives no figure)
  static const uint8_t SYNC_GAP_MS = 6;        // idle after the sync pulse, longer than one byte

  RfLink(uint8_t dataPin, uint8_t trPin, bool inverted = true);

  void begin();                    // starts Timer2 and leaves the module receiving

  // Sends a complete burst: switch to transmit, sync pulse, data, wait until it is out,
  // switch back to receive. Blocks for about 16 ms + 5 ms per byte.
  void transmit(const uint8_t *data, uint8_t len);

  int read();                      // next received byte, or -1
  void flushRx();
  bool transmitting() const { return txMode_; }

  void onTick();                   // called by the Timer2 interrupt only

private:
  static const uint8_t BUF_SIZE = 64;           // power of two
  static const uint8_t BUF_MASK = BUF_SIZE - 1;

  void enterTx();
  void enterRx();
  void push(uint8_t b);
  bool drained() const { return txHead_ == txTail_ && !txActive_; }
  inline void lineWrite(bool logicOne);
  inline bool lineRead() const;

  uint8_t dataPin_, trPin_;
  bool inverted_;
  volatile uint8_t *out_, *in_, *ddr_;
  uint8_t mask_;

  volatile uint8_t txBuf_[BUF_SIZE];
  volatile uint8_t txHead_ = 0, txTail_ = 0;
  volatile uint8_t rxBuf_[BUF_SIZE];
  volatile uint8_t rxHead_ = 0, rxTail_ = 0;
  volatile bool txMode_ = false, txActive_ = false, rxReset_ = false;

  // interrupt state
  uint8_t txTick_ = 0, txBit_ = 0, txShift_ = 0;
  bool rxBusy_ = false;
  uint8_t rxTick_ = 0, rxNext_ = 0, rxBit_ = 0, rxShift_ = 0;
};
