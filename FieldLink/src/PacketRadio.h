// FieldLink - PacketRadio: sends and receives whole packets over an RfLink.
#pragma once
#include <Arduino.h>
#include "RfLink.h"
#include "Packet.h"
#include "Indicator.h"

class PacketRadio {
public:
  explicit PacketRadio(RfLink &link) : link_(link) {}

  void begin() { link_.begin(); }
  void setIndicators(Indicator *rx, Indicator *tx) { rxLed_ = rx; txLed_ = tx; }

  // Blocking: about 16 ms + 5 ms per byte (a 16 byte payload takes about 136 ms).
  void send(const Packet &p) {
    uint8_t frame[FL_MAX_FRAME];
    uint8_t n = FrameCodec::encode(p, frame);
    if (txLed_) txLed_->set(true);
    link_.transmit(frame, n);
    if (txLed_) txLed_->set(false);
    sent_++;
  }

  // Processes received bytes. Returns true (and fills p) when a valid packet arrived.
  bool receive(Packet &p) {
    int c;
    while ((c = link_.read()) >= 0) {
      if (decoder_.push((uint8_t)c, p)) {
        received_++;
        if (rxLed_) rxLed_->flash(60);
        return true;
      }
    }
    return false;
  }

  void flush() { link_.flushRx(); }

  unsigned long sent() const { return sent_; }
  unsigned long received() const { return received_; }
  unsigned long badFrames() const { return decoder_.badFrames(); }

private:
  RfLink &link_;
  FrameDecoder decoder_;
  Indicator *rxLed_ = nullptr;
  Indicator *txLed_ = nullptr;
  unsigned long sent_ = 0, received_ = 0;
};
