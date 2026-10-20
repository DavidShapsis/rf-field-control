// FieldLink - packets
// Frame on the air:  0xA5 0x5A | len | type | src | dst | seq | payload[len] | crc8
// The CRC covers len through the end of the payload.
#pragma once
#include <Arduino.h>

const uint8_t FL_MAX_PAYLOAD = 16;
const uint8_t FL_FRAME_OVERHEAD = 8;
const uint8_t FL_MAX_FRAME = FL_MAX_PAYLOAD + FL_FRAME_OVERHEAD;

struct Packet {
  uint8_t type = 0;
  uint8_t src = 0;
  uint8_t dst = 0;
  uint8_t seq = 0;
  uint8_t len = 0;
  uint8_t data[FL_MAX_PAYLOAD];

  Packet() {}
  Packet(uint8_t type_, uint8_t src_, uint8_t dst_) : type(type_), src(src_), dst(dst_) {}
};

// Appends values to a packet payload (big endian). ok() turns false if it overflows.
class PayloadWriter {
public:
  explicit PayloadWriter(Packet &p) : p_(p) { p_.len = 0; }
  void u8(uint8_t v);
  void i8(int8_t v) { u8((uint8_t)v); }
  void u16(uint16_t v) { u8(v >> 8); u8(v & 0xFF); }
  void i16(int16_t v) { u16((uint16_t)v); }
  bool ok() const { return ok_; }
private:
  Packet &p_;
  bool ok_ = true;
};

// Reads values back out of a payload. ok() turns false if it reads past the end.
class PayloadReader {
public:
  explicit PayloadReader(const Packet &p) : p_(p) {}
  uint8_t u8();
  int8_t i8() { return (int8_t)u8(); }
  uint16_t u16() { uint16_t h = u8(); return (h << 8) | u8(); }
  int16_t i16() { return (int16_t)u16(); }
  uint8_t remaining() const { return pos_ < p_.len ? p_.len - pos_ : 0; }
  bool ok() const { return ok_; }
private:
  const Packet &p_;
  uint8_t pos_ = 0;
  bool ok_ = true;
};

class FrameCodec {
public:
  static const uint8_t H1 = 0xA5;
  static const uint8_t H2 = 0x5A;
  static uint8_t crc8(const uint8_t *data, uint8_t len);
  static uint8_t encode(const Packet &p, uint8_t *out);   // returns the frame length
};

// Streaming decoder. Feed it every received byte; it finds frames inside receiver noise and
// re-scans after a false header, so a noise byte cannot hide a real frame.
class FrameDecoder {
public:
  bool push(uint8_t b, Packet &out);       // true when a valid frame completed
  unsigned long badFrames() const { return bad_; }
private:
  void drop(uint8_t n);
  uint8_t buf_[FL_MAX_FRAME];
  uint8_t n_ = 0;
  unsigned long bad_ = 0;
};
