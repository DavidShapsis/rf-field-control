#include "RfLink.h"
#include <avr/interrupt.h>

static RfLink *activeLink = nullptr;
static volatile RfLink::TickHook tickHook = nullptr;

void RfLink::setTickHook(TickHook hook) {
  noInterrupts();
  tickHook = hook;
  interrupts();
}

RfLink::RfLink(uint8_t dataPin, uint8_t trPin, bool inverted)
  : dataPin_(dataPin), trPin_(trPin), inverted_(inverted) {}

void RfLink::begin() {
  uint8_t port = digitalPinToPort(dataPin_);
  out_ = portOutputRegister(port);
  in_ = portInputRegister(port);
  ddr_ = portModeRegister(port);
  mask_ = digitalPinToBitMask(dataPin_);

  pinMode(trPin_, OUTPUT);
  digitalWrite(trPin_, LOW);
  pinMode(dataPin_, INPUT);
  activeLink = this;

  // Timer2 CTC: 16 MHz / 8 / 125 = 16 kHz
  noInterrupts();
  TCCR2A = (1 << WGM21);
  TCCR2B = (1 << CS21);
  OCR2A = 124;
  TCNT2 = 0;
  TIMSK2 = (1 << OCIE2A);
  interrupts();

  enterRx();
}

inline void RfLink::lineWrite(bool logicOne) {
  if (logicOne != inverted_) *out_ |= mask_;
  else                       *out_ &= ~mask_;
}

inline bool RfLink::lineRead() const {
  bool high = *in_ & mask_;
  return high != inverted_;
}

// DATA becomes an output at idle level first, then T/R goes HIGH.
void RfLink::enterTx() {
  noInterrupts();
  *ddr_ |= mask_;
  lineWrite(true);
  txMode_ = true;
  interrupts();
  digitalWrite(trPin_, HIGH);
  delay(SWITCH_SETTLE_MS);
}

// DATA becomes an input (no pull-up) first, then T/R goes LOW.
void RfLink::enterRx() {
  noInterrupts();
  txMode_ = false;
  *ddr_ &= ~mask_;
  *out_ &= ~mask_;
  rxReset_ = true;
  interrupts();
  digitalWrite(trPin_, LOW);
  delay(SWITCH_SETTLE_MS);
  flushRx();                       // drop whatever was heard while switching
}

void RfLink::push(uint8_t b) {
  uint8_t next = (txHead_ + 1) & BUF_MASK;
  while (next == txTail_) {}       // wait for room
  txBuf_[txHead_] = b;
  txHead_ = next;
}

void RfLink::transmit(const uint8_t *data, uint8_t len) {
  enterTx();
  // Sync pulse (about 4.5 ms of continuous HIGH) so the receiver locks on, then an idle gap
  // longer than one byte so a receiver that started a false byte on noise finishes it before
  // the real data begins.
  push(inverted_ ? 0x00 : 0xFF);
  while (!drained()) {}
  delay(SYNC_GAP_MS);
  for (uint8_t i = 0; i < len; i++) push(data[i]);
  while (!drained()) {}
  enterRx();
}

int RfLink::read() {
  if (rxHead_ == rxTail_) return -1;
  uint8_t b = rxBuf_[rxTail_];
  rxTail_ = (rxTail_ + 1) & BUF_MASK;
  return b;
}

void RfLink::flushRx() {
  noInterrupts();
  rxTail_ = rxHead_;
  interrupts();
}

void RfLink::onTick() {
  if (txMode_) {
    // transmit: start bit, 8 data bits LSB first, stop bit
    if (!txActive_) {
      if (txHead_ != txTail_) {
        txShift_ = txBuf_[txTail_];
        txTail_ = (txTail_ + 1) & BUF_MASK;
        txActive_ = true;
        txTick_ = 0;
        txBit_ = 0;
        lineWrite(false);
      } else {
        lineWrite(true);
      }
    } else if (++txTick_ >= TICKS_PER_BIT) {
      txTick_ = 0;
      txBit_++;
      if (txBit_ <= 8) {
        lineWrite(txShift_ & 1);
        txShift_ >>= 1;
      } else if (txBit_ == 9) {
        lineWrite(true);
      } else {
        txActive_ = false;
      }
    }
    rxBusy_ = false;
    return;
  }

  // receive: sample in the middle of each bit
  if (rxReset_) {
    rxBusy_ = false;
    rxReset_ = false;
  }
  bool logic = lineRead();
  if (!rxBusy_) {
    if (!logic) {
      rxBusy_ = true;
      rxTick_ = 0;
      rxNext_ = TICKS_PER_BIT / 2;
      rxBit_ = 0;
      rxShift_ = 0;
    }
    return;
  }
  if (++rxTick_ != rxNext_) return;
  rxNext_ += TICKS_PER_BIT;
  if (rxBit_ == 0) {
    if (logic) rxBusy_ = false;              // glitch, not a start bit
  } else if (rxBit_ <= 8) {
    rxShift_ >>= 1;
    if (logic) rxShift_ |= 0x80;
  } else {
    if (logic) {                             // valid stop bit
      uint8_t next = (rxHead_ + 1) & BUF_MASK;
      if (next != rxTail_) {
        rxBuf_[rxHead_] = rxShift_;
        rxHead_ = next;
      }
    }
    rxBusy_ = false;
  }
  rxBit_++;
}

ISR(TIMER2_COMPA_vect) {
  if (activeLink) activeLink->onTick();
  if (tickHook) tickHook();
}
