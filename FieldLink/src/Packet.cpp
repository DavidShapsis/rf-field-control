#include "Packet.h"

void PayloadWriter::u8(uint8_t v) {
  if (p_.len >= FL_MAX_PAYLOAD) { ok_ = false; return; }
  p_.data[p_.len++] = v;
}

uint8_t PayloadReader::u8() {
  if (pos_ >= p_.len) { ok_ = false; return 0; }
  return p_.data[pos_++];
}

// CRC-8, polynomial 0x07
uint8_t FrameCodec::crc8(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
  }
  return crc;
}

uint8_t FrameCodec::encode(const Packet &p, uint8_t *out) {
  uint8_t len = p.len > FL_MAX_PAYLOAD ? FL_MAX_PAYLOAD : p.len;
  out[0] = H1;
  out[1] = H2;
  out[2] = len;
  out[3] = p.type;
  out[4] = p.src;
  out[5] = p.dst;
  out[6] = p.seq;
  for (uint8_t i = 0; i < len; i++) out[7 + i] = p.data[i];
  out[7 + len] = crc8(out + 2, 5 + len);
  return FL_FRAME_OVERHEAD + len;
}

void FrameDecoder::drop(uint8_t n) {
  if (n >= n_) { n_ = 0; return; }
  for (uint8_t i = n; i < n_; i++) buf_[i - n] = buf_[i];
  n_ -= n;
}

bool FrameDecoder::push(uint8_t b, Packet &out) {
  if (n_ == FL_MAX_FRAME) drop(1);
  buf_[n_++] = b;

  while (n_ > 0) {
    if (buf_[0] != FrameCodec::H1) { drop(1); continue; }
    if (n_ < 2) return false;
    if (buf_[1] != FrameCodec::H2) { drop(1); continue; }
    if (n_ < 3) return false;
    uint8_t len = buf_[2];
    if (len > FL_MAX_PAYLOAD) { drop(1); continue; }
    uint8_t total = FL_FRAME_OVERHEAD + len;
    if (n_ < total) return false;
    if (FrameCodec::crc8(buf_ + 2, 5 + len) != buf_[total - 1]) {
      bad_++;
      drop(1);                     // false header: look again from the next byte
      continue;
    }
    out.len = len;
    out.type = buf_[3];
    out.src = buf_[4];
    out.dst = buf_[5];
    out.seq = buf_[6];
    for (uint8_t i = 0; i < len; i++) out.data[i] = buf_[7 + i];
    drop(total);
    return true;
  }
  return false;
}
