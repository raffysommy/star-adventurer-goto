#include "skywatcher.h"

#include "mount_usb.h"

namespace sw {

// 24-bit values travel as little-endian hex: 0x123456 -> "563412"
static void encode24(uint32_t v, char *out) { snprintf(out, 7, "%02X%02X%02X", v & 0xff, (v >> 8) & 0xff, (v >> 16) & 0xff); }

static long decodeHex(const String &s) {
  String be;
  for (int i = s.length(); i >= 2; i -= 2) be += s.substring(i - 2, i);
  return strtol(be.c_str(), nullptr, 16);
}

bool cmd(char c, int axis, const char *data, String *payload) {
  char req[16], resp[32];
  int n = snprintf(req, sizeof(req), ":%c%d%s\r", c, axis, data);
  if (mountCmd(req, n, resp, sizeof(resp)) < 2 || resp[0] != '=') return false;
  if (payload) {
    resp[strcspn(resp, "\r")] = 0;
    *payload = resp + 1;
  }
  return true;
}

bool readParams(int axis, Params &p) {
  String a, b, g;
  p.valid = cmd('a', axis, "", &a) && cmd('b', axis, "", &b) && cmd('g', axis, "", &g);
  if (p.valid) {
    p.cpr = decodeHex(a);
    p.timerHz = decodeHex(b);
    p.highSpeedRatio = decodeHex(g);
    p.valid = p.cpr > 0 && p.timerHz > 0;
  }
  return p.valid;
}

bool getPos(int axis, long &counts) {
  String r;
  if (!cmd('j', axis, "", &r) || r.length() != 6) return false;
  counts = decodeHex(r) - 0x800000;
  return true;
}

bool setPos(int axis, long counts) {
  if (counts <= -0x800000 || counts >= 0x800000) return false;  // 24-bit register would wrap
  char d[7];
  encode24((uint32_t)(counts + 0x800000), d);
  return cmd('E', axis, d);
}

bool status(int axis, Status &s) {
  String r;
  if (!cmd('f', axis, "", &r) || r.length() != 3) return false;
  int a = strtol(r.substring(0, 1).c_str(), nullptr, 16);
  int b = strtol(r.substring(1, 2).c_str(), nullptr, 16);
  s.tracking = a & 1;
  s.ccw = a & 2;
  s.fast = a & 4;
  s.running = b & 1;
  return true;
}

bool stopSoft(int axis) { return cmd('K', axis); }
bool stopHard(int axis) { return cmd('L', axis); }

bool waitStopped(int axis, uint32_t timeoutMs) {
  for (uint32_t t0 = millis(); millis() - t0 < timeoutMs; delay(20)) {
    Status s;
    if (status(axis, s) && !s.running) return true;
  }
  return false;
}

bool setMode(int axis, bool fast, bool ccw) {
  const char mode[3] = {fast ? '3' : '1', ccw ? '1' : '0', 0};
  return cmd('G', axis, mode);
}

bool setT1(int axis, uint32_t t1) {
  char d[7];
  encode24(t1, d);
  return cmd('I', axis, d);
}

bool start(int axis) { return cmd('J', axis); }
bool initDone(int axis) { return cmd('F', axis); }

uint32_t t1ForRate(const Params &p, double degPerSec) {
  double countsPerSec = fabs(degPerSec) * p.cpr / 360.0;
  if (countsPerSec <= 0) return p.timerHz;
  return (uint32_t)(p.timerHz / countsPerSec);
}

double countsToDeg(const Params &p, long counts) { return p.cpr ? counts * 360.0 / p.cpr : 0; }
long degToCounts(const Params &p, double deg) { return (long)(deg * p.cpr / 360.0); }

}  // namespace sw
