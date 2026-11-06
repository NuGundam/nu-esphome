#pragma once
//
// AC Infinity UIS controller (67 / 69 / 69 Pro) BLE frame builder + advertisement
// parser for ESPHome.
//
// Protocol ported from hunterjm/ac-infinity-ble (MIT):
//   https://github.com/hunterjm/ac-infinity-ble
//
// Drop this file next to your .yaml and reference it with:
//   esphome:
//     includes: [aci.h]
//
#include <vector>
#include <cstdint>
#include <cstddef>

namespace aci {

// ---------------------------------------------------------------------------
// Advertisement layout
// ---------------------------------------------------------------------------
// If your logged advertisement bytes look shifted relative to the offsets below
// (e.g. temperature reads as a wild number), bump ADV_OFF by +/- 2. Different
// BLE stacks disagree about whether the 2-byte company ID stays in the payload.
static const int ADV_OFF = 0;

static const size_t ADV_MIN_LEN = 19 + ADV_OFF;

// ---------------------------------------------------------------------------
// CRC
// ---------------------------------------------------------------------------
// Byte-swapped CRC-16/CCITT variant. This is NOT a stock CRC16 -- do not
// substitute one from a library, the controller will silently ignore the frame.
inline uint16_t crc16(const uint8_t *d, int start, int count) {
  uint32_t b = 0xFFFF;
  for (int i = start; i < start + count; i++) {
    uint32_t b2 = (((b << 8) | (b >> 8)) & 0xFFFF) ^ (uint32_t)(d[i] & 0xFF);
    uint32_t b3 = b2 ^ ((b2 & 0xFF) >> 4);
    uint32_t b4 = b3 ^ ((b3 << 12) & 0xFFFF);
    b = b4 ^ (((b4 & 0xFF) << 5) & 0xFFFF);
  }
  return (uint16_t)(b & 0xFFFF);
}

inline uint16_t next_seq() {
  static uint16_t seq = 0;
  return ++seq;
}

// ---------------------------------------------------------------------------
// Frame builder
// ---------------------------------------------------------------------------
//  [0..1]  A5 00                 header
//  [2..3]  payload length        big endian
//  [4..5]  sequence number       big endian, increments per command
//  [6..7]  crc16 over bytes 0..5
//  [8]     00
//  [9]     command type
//  [10..]  payload
//  [last2] crc16 over bytes 8 .. 9+payload_len
inline std::vector<uint8_t> frame(uint8_t cmd, const std::vector<uint8_t> &payload) {
  size_t n = payload.size();
  std::vector<uint8_t> f(n + 12, 0);

  f[0] = 0xA5;
  f[1] = 0x00;
  f[2] = (uint8_t)((n >> 8) & 0xFF);
  f[3] = (uint8_t)(n & 0xFF);

  uint16_t seq = next_seq();
  f[4] = (uint8_t)((seq >> 8) & 0xFF);
  f[5] = (uint8_t)(seq & 0xFF);

  uint16_t c1 = crc16(f.data(), 0, 6);
  f[6] = (uint8_t)((c1 >> 8) & 0xFF);
  f[7] = (uint8_t)(c1 & 0xFF);

  f[8] = 0x00;
  f[9] = cmd;
  for (size_t i = 0; i < n; i++) f[10 + i] = payload[i];

  uint16_t c2 = crc16(f.data(), 8, (int)(n + 2));
  f[n + 10] = (uint8_t)((c2 >> 8) & 0xFF);
  f[n + 11] = (uint8_t)(c2 & 0xFF);

  return f;
}

// ---------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------
// mode:  1 = off, 2 = on
// level: 0..10
// port:  1-based UIS port on the controller
inline std::vector<uint8_t> set_level(uint8_t mode, uint8_t level, uint8_t port) {
  if (mode != 1 && mode != 2) mode = 1;
  if (level > 10) level = 10;
  std::vector<uint8_t> p = {
      0x10, 0x01, mode, (uint8_t)(mode + 0x10), 0x01, level, 0xFF, port};
  return frame(3, p);
}

// Set the controller's clock. Mirrors ProtocolResolution.setTimeData() in the
// decompiled Android app: register 0x01, payload length 7, then
// yy MM dd dow HH mm ss.
//   year  - full year, only the last two digits are sent
//   month - 1..12
//   dow   - 1..7 with Sunday = 1, which matches ESPHome's ESPTime::day_of_week
// This is a device-level setting, so no trailing [0xFF, port] pair. If the
// controller rejects it, try appending 0xFF and the port.
inline std::vector<uint8_t> set_time(int year, int month, int day, int dow,
                                     int hour, int minute, int second) {
  std::vector<uint8_t> p = {0x01,
                            0x07,
                            (uint8_t)(year % 100),
                            (uint8_t)month,
                            (uint8_t)day,
                            (uint8_t)dow,
                            (uint8_t)hour,
                            (uint8_t)minute,
                            (uint8_t)second};
  return frame(3, p);
}

// Ask the controller to dump its current model/state over the notify char.
inline std::vector<uint8_t> query(uint8_t port) {
  std::vector<uint8_t> p = {0x10, 0x11, 0x12, 0x13, 0x14, 0x15,
                            0x16, 0x17, 0xFF, port};
  return frame(1, p);
}

// ---------------------------------------------------------------------------
// Advertisement parsing (works without ever connecting)
// ---------------------------------------------------------------------------
inline int16_t be16(const std::vector<uint8_t> &d, size_t i) {
  return (int16_t)(((uint16_t)d[i] << 8) | (uint16_t)d[i + 1]);
}

// The controller sends 0x8000 (INT16_MIN) for "no reading" -- probe unplugged,
// or the field simply isn't populated in the advertisement. Publishing that
// raw gives you -327.68 in Home Assistant.
static const int16_t NO_READING = (int16_t)0x8000;

inline int   adv_version(const std::vector<uint8_t> &d) { return d[11 + ADV_OFF]; }
inline int   adv_type(const std::vector<uint8_t> &d)    { return d[12 + ADV_OFF]; }
inline float adv_temp(const std::vector<uint8_t> &d)    { return be16(d, 14 + ADV_OFF) / 100.0f; }
inline float adv_hum(const std::vector<uint8_t> &d)     { return be16(d, 16 + ADV_OFF) / 100.0f; }
inline int   adv_level(const std::vector<uint8_t> &d)   { return d[18 + ADV_OFF]; }
inline float adv_vpd(const std::vector<uint8_t> &d)     { return be16(d, 21 + ADV_OFF) / 100.0f; }

inline bool adv_temp_ok(const std::vector<uint8_t> &d) { return be16(d, 14 + ADV_OFF) != NO_READING; }
inline bool adv_hum_ok(const std::vector<uint8_t> &d)  { return be16(d, 16 + ADV_OFF) != NO_READING; }

// VPD is only present on newer multi-port controllers.
inline bool adv_has_vpd(const std::vector<uint8_t> &d) {
  if (d.size() < (size_t)(23 + ADV_OFF)) return false;
  int t = adv_type(d);
  if (!(adv_version(d) >= 3 && (t == 7 || t == 9 || t == 11 || t == 12))) return false;
  return be16(d, 21 + ADV_OFF) != NO_READING;
}

// True for controller types that expect the trailing [0xFF, port] byte pair.
inline bool is_multiport(const std::vector<uint8_t> &d) {
  int t = adv_type(d);
  return t == 7 || t == 9 || t == 11 || t == 12;
}

inline bool adv_valid(const std::vector<uint8_t> &d) {
  return d.size() >= ADV_MIN_LEN;
}

// ---------------------------------------------------------------------------
// Notify frames
// ---------------------------------------------------------------------------
// The controller stops advertising while it holds a connection, so once
// ble_client is attached the advertisement parsers above go silent and
// everything has to come from the notify characteristic instead.
//
// Two frame shapes arrive on 70D51002:
//   1E FF ...  unsolicited status, roughly once a second
//   A5 ...     response to a command we sent
//
// Status frame layout, verified against live captures:
//   [0..1]   1E FF          marker
//   [8..9]   temperature    signed, hundredths of a degree C
//   [10..11] humidity       hundredths of a percent
//   [12..13] vpd            hundredths of a kPa
//   [22..]   per-port blocks, 8 bytes each, port 1 first

inline bool nfy_is_status(const std::vector<uint8_t> &d) {
  return d.size() >= 26 && d[0] == 0x1E && d[1] == 0xFF;
}

inline float nfy_temp(const std::vector<uint8_t> &d) { return be16(d, 8) / 100.0f; }
inline float nfy_hum(const std::vector<uint8_t> &d) { return be16(d, 10) / 100.0f; }
inline float nfy_vpd(const std::vector<uint8_t> &d) { return be16(d, 12) / 100.0f; }

inline bool nfy_temp_ok(const std::vector<uint8_t> &d) { return be16(d, 8) != NO_READING; }
inline bool nfy_hum_ok(const std::vector<uint8_t> &d) { return be16(d, 10) != NO_READING; }
inline bool nfy_vpd_ok(const std::vector<uint8_t> &d) { return be16(d, 12) != NO_READING; }

static const size_t PORT_BASE = 22;
static const size_t PORT_STRIDE = 8;

// Work type for a 1-based port: 1 = off, 2 = on, higher values are the
// controller's own program modes. Returns -1 if the frame is too short.
inline int nfy_port_mode(const std::vector<uint8_t> &d, int port) {
  size_t off = PORT_BASE + (size_t)(port - 1) * PORT_STRIDE;
  if (d.size() < off + 4) return -1;
  return d[off + 3] & 0x0F;
}

// 0xFFFF in the fan-type field means nothing is plugged into that port.
inline bool nfy_port_occupied(const std::vector<uint8_t> &d, int port) {
  size_t off = PORT_BASE + (size_t)(port - 1) * PORT_STRIDE;
  if (d.size() < off + 2) return false;
  return !(d[off] == 0xFF && d[off + 1] == 0xFF);
}

// ---------------------------------------------------------------------------
// Command response frames
// ---------------------------------------------------------------------------
// Payload is a register / length / value list starting at offset 10, ending
// at an 0xFF marker followed by the port number. Registers of interest:
//   0x10  work type   1 = off, 2 = on
//   0x11  off level
//   0x12  on level
inline bool rsp_is_frame(const std::vector<uint8_t> &d) {
  return d.size() >= 12 && d[0] == 0xA5;
}

inline int rsp_reg(const std::vector<uint8_t> &d, uint8_t reg) {
  if (d.size() < 14) return -1;
  size_t end = d.size() - 2;
  size_t i = 10;
  while (i + 1 < end) {
    uint8_t r = d[i];
    if (r == 0xFF) break;
    uint8_t len = d[i + 1];
    if (len == 0 || i + 2 + len > end) break;
    if (r == reg) return d[i + 2];
    i += 2 + len;
  }
  return -1;
}

}  // namespace aci