#pragma once
// Generic M-Bus (EN 13757-3) variable data record decoder, used by the probe mode.
// Header-only and free of ESPHome dependencies so it can be unit-tested on a host.

#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace esphome {
namespace sensostar {

// Describe a primary VIF (without extension bit). Writes unit text and returns decimal exponent.
// Returns false if the VIF is not one of the common heat-meter VIFs.
inline bool mbus_describe_vif(uint8_t vif, const char **name, int *exp10) {
  vif &= 0x7f;
  if ((vif & 0x78) == 0x00) { *name = "Energy Wh"; *exp10 = (vif & 0x07) - 3; return true; }
  if ((vif & 0x78) == 0x08) { *name = "Energy J"; *exp10 = (vif & 0x07); return true; }
  if ((vif & 0x78) == 0x10) { *name = "Volume m3"; *exp10 = (vif & 0x07) - 6; return true; }
  if ((vif & 0x78) == 0x28) { *name = "Power W"; *exp10 = (vif & 0x07) - 3; return true; }
  if ((vif & 0x78) == 0x38) { *name = "Flow m3/h"; *exp10 = (vif & 0x07) - 6; return true; }
  if ((vif & 0x7C) == 0x58) { *name = "FlowTemp C"; *exp10 = (vif & 0x03) - 3; return true; }
  if ((vif & 0x7C) == 0x5C) { *name = "ReturnTemp C"; *exp10 = (vif & 0x03) - 3; return true; }
  if ((vif & 0x7C) == 0x60) { *name = "TempDiff K"; *exp10 = (vif & 0x03) - 3; return true; }
  if ((vif & 0x7C) == 0x64) { *name = "ExtTemp C"; *exp10 = (vif & 0x03) - 3; return true; }
  if ((vif & 0x7C) == 0x20) { *name = "OnTime"; *exp10 = 0; return true; }
  if ((vif & 0x7C) == 0x24) { *name = "OpTime"; *exp10 = 0; return true; }
  if (vif == 0x6C) { *name = "Date"; *exp10 = 0; return true; }
  if (vif == 0x6D) { *name = "DateTime"; *exp10 = 0; return true; }
  if (vif == 0x78) { *name = "FabNo"; *exp10 = 0; return true; }
  if (vif == 0x7F || vif == 0x7E) { *name = "ManufSpecific"; *exp10 = 0; return true; }
  return false;
}

// Decode all records of a long frame (68 L L 68 C A CI ...). Returns one text line per record.
inline std::vector<std::string> mbus_decode_records(const std::vector<uint8_t> &f) {
  std::vector<std::string> out;
  char buf[200];
  if (f.size() < 9 || f[0] != 0x68) { out.push_back("not a long frame"); return out; }
  const size_t end = f.size() - 2;  // exclude checksum and stop
  uint8_t ci = f[6];
  size_t i;
  if (ci == 0x72 || ci == 0x76) i = 19;       // 12-byte fixed header
  else if (ci == 0x7A) i = 11;                // 4-byte short header
  else if (ci == 0x78) i = 7;                 // no header
  else {
    snprintf(buf, sizeof buf, "CI=0x%02X (no standard header), payload decoded from offset 7", ci);
    out.push_back(buf);
    i = 7;
  }
  int rec = 0;
  while (i < end) {
    size_t i_dif = i;
    uint8_t dif = f[i];
    if (dif == 0x0F || dif == 0x1F) {  // manufacturer specific data follows
      std::string s = "MDH: ";
      for (size_t k = i; k < end; k++) { snprintf(buf, sizeof buf, "%02X ", f[k]); s += buf; }
      out.push_back(s);
      break;
    }
    if (dif == 0x2F) { i++; continue; }  // idle filler
    while (i < end && (f[i] & 0x80)) i++;  // DIFE
    i++;
    if (i >= end) break;
    size_t i_vif = i;
    while (i < end && (f[i] & 0x80)) i++;  // VIFE
    i++;
    uint8_t lc = dif & 0x0F;
    int len;
    bool bcd = false;
    switch (lc) {
      case 0x0: len = 0; break;
      case 0x1: len = 1; break;
      case 0x2: len = 2; break;
      case 0x3: len = 3; break;
      case 0x4: len = 4; break;
      case 0x5: len = 4; break;  // real
      case 0x6: len = 6; break;
      case 0x7: len = 8; break;
      case 0x9: len = 1; bcd = true; break;
      case 0xA: len = 2; bcd = true; break;
      case 0xB: len = 3; bcd = true; break;
      case 0xC: len = 4; bcd = true; break;
      case 0xE: len = 6; bcd = true; break;
      default: len = -1;
    }
    std::string hdr;
    for (size_t k = i_dif; k < i; k++) { snprintf(buf, sizeof buf, "%02X", f[k]); hdr += buf; }
    if (len < 0 || i + len > end) {
      snprintf(buf, sizeof buf, "#%d hdr=%s unsupported/truncated (lc=0x%X)", rec, hdr.c_str(), lc);
      out.push_back(buf);
      break;
    }
    std::string raw;
    for (int k = 0; k < len; k++) { snprintf(buf, sizeof buf, "%02X", f[i + k]); raw += buf; }
    double value = 0;
    bool have = len > 0 && lc != 0x5;
    if (bcd) {
      double mul = 1;
      for (int k = 0; k < len; k++) {
        uint8_t b = f[i + k];
        value += (b & 0x0F) * mul; mul *= 10;
        value += (b >> 4) * mul; mul *= 10;
      }
    } else if (have) {
      int64_t v = 0;
      for (int k = len - 1; k >= 0; k--) v = (v << 8) | f[i + k];
      if (len < 8 && (f[i + len - 1] & 0x80)) v -= (int64_t) 1 << (8 * len);
      value = (double) v;
    }
    const char *name = "?";
    int e = 0;
    uint8_t vif = f[i_vif];
    bool known = (vif != 0xFD && vif != 0xFB) && mbus_describe_vif(vif, &name, &e);
    uint8_t func = (dif >> 4) & 0x03;
    uint8_t storage = (dif >> 6) & 0x01;
    if (known && have) {
      double scaled = value;
      for (int k = 0; k < e; k++) scaled *= 10;
      for (int k = e; k < 0; k++) scaled /= 10;
      snprintf(buf, sizeof buf, "#%d hdr=%s data=%s f=%u st=%u -> %s = %.4f (res 1e%d)", rec, hdr.c_str(),
               raw.c_str(), func, storage, name, scaled, e);
    } else {
      snprintf(buf, sizeof buf, "#%d hdr=%s data=%s f=%u st=%u -> raw %.0f", rec, hdr.c_str(), raw.c_str(), func,
               storage, value);
    }
    out.push_back(buf);
    i += len;
    rec++;
  }
  return out;
}

}  // namespace sensostar
}  // namespace esphome
