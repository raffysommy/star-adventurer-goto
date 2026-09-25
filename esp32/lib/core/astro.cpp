#include "astro.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace astro {

double wrap360(double deg) {
  double r = fmod(deg, 360.0);
  if (r < 0) r += 360.0;
  if (r >= 360.0) r -= 360.0;  // fmod(-1e-17) + 360 rounds to 360
  return r;
}

// Python's % for a positive divisor
static double pyMod(double a, double b) {
  double r = fmod(a, b);
  return r < 0 ? r + b : r;
}

double julianDate(double unixUtc) { return unixUtc / 86400.0 + 2440587.5; }

double gmstDeg(double unixUtc) {
  double d = julianDate(unixUtc) - 2451545.0;
  double t = d / 36525.0;
  return wrap360(280.46061837 + 360.98564736629 * d + 0.000387933 * t * t - t * t * t / 38710000.0);
}

double lstDeg(double unixUtc, double lonEast) { return wrap360(gmstDeg(unixUtc) + lonEast); }

// get_hour_angle(), generalized: with offset == MARGIN this is lx200.py's
// (LST - RA) % 360 + offset
double hourAngle(double raDeg, double lst, double offset) { return wrap360(lst - raDeg + offset - MARGIN) + MARGIN; }

double rightAscension(double haDeg, double lst, double offset) { return wrap360(lst - (haDeg - offset)); }

RaTarget selectTarget(double requestedRa, double lst, double offset) {
  RaTarget t;
  if (hourAngle(requestedRa, lst, offset) > WINDOW_MAX) {
    t.flipped = true;
    t.ra = wrap360(requestedRa + 180);
  } else {
    t.flipped = false;
    t.ra = requestedRa;
  }
  return t;
}

RaTarget selectSyncTarget(double requestedRa, double lst, double offset, double currentRegister, double trackMax,
                          bool &ok) {
  RaTarget c[2] = {{requestedRa, false}, {wrap360(requestedRa + 180), true}};
  ok = false;
  RaTarget best = c[0];
  double bestDist = 1e9;
  for (const RaTarget &t : c) {
    double reg = hourAngle(t.ra, lst, offset);
    if (reg < MARGIN || reg > trackMax) continue;
    double d = fabs(reg - currentRegister);
    if (d < bestDist) {
      bestDist = d;
      best = t;
      ok = true;
    }
  }
  return best;
}

double reportedRa(double axisRa, bool flipped, double lst, double offset) {
  if (!flipped) return axisRa;
  return rightAscension(wrap360(hourAngle(axisRa, lst, offset) - 180), lst, offset);
}

// ---------------------------------------------------------------- DEC steps

long decToSteps(double deg) {
  return (long)(((deg * 3600.0 + 3600.0 * 180.0) / (3600.0 * 360.0)) * DEC_STEPS_PER_REV);
}

double stepsToDec(long steps, bool reverse) {
  double d = ((double)steps / DEC_STEPS_PER_REV) * 360.0 - 180.0;
  if (reverse) d = pyMod((180 - d) + 180, 360) - 180;  // flip back the meridian dec coordinates
  return d;
}

double decForSteps(double requestedDeg, bool reverse) { return reverse ? 180 - requestedDeg : requestedDeg; }

// ---------------------------------------------------------------- formatting

void degToHms(double deg, int &h, int &m, int &s) {
  deg = wrap360(deg);
  double totalHours = deg / 15.0;
  h = (int)totalHours;
  double remMinutes = (totalHours - h) * 60.0;
  m = (int)remMinutes;
  s = (int)((remMinutes - m) * 60.0);
}

double hmsToDeg(int h, int m, int s) {
  long total = (long)h * 3600 + (long)m * 60 + s;
  long norm = total % 86400;
  if (norm < 0) norm += 86400;
  return norm / 86400.0 * 360.0;
}

void formatRa(double deg, char *out, int outLen) {
  int h, m, s;
  degToHms(deg, h, m, s);
  snprintf(out, outLen, "%02d:%02d:%02d", h, m, s);
}

void formatDec(double deg, char *out, int outLen) {
  char sign = deg < 0 ? '-' : '+';
  long arcsec = lround(fabs(deg) * 3600.0);  // rounding here can't produce ":60"
  snprintf(out, outLen, "%c%02ld*%02ld:%02ld", sign, arcsec / 3600, (arcsec / 60) % 60, arcsec % 60);
}

// ---------------------------------------------------------------- parsing

// Reads exactly n decimal digits
static bool digits(const char *&p, int n, int &value) {
  value = 0;
  for (int i = 0; i < n; i++) {
    if (p[i] < '0' || p[i] > '9') return false;
    value = value * 10 + (p[i] - '0');
  }
  p += n;
  return true;
}

static bool expect(const char *&p, char c) {
  if (*p != c) return false;
  p++;
  return true;
}

// Degree separator: '*' as in lx200.py, or the 0xDF degree sign some clients send
static bool degSep(const char *&p) {
  if (*p == '*' || (unsigned char)*p == 0xDF) {
    p++;
    return true;
  }
  return false;
}

static int optSign(const char *&p) {
  if (*p == '-') {
    p++;
    return -1;
  }
  if (*p == '+') p++;
  return 1;
}

bool parseSr(const char *cmd, double &raDeg) {
  const char *p = cmd;
  int h, m, s;
  if (strncmp(p, ":Sr", 3)) return false;
  p += 3;
  if (!digits(p, 2, h) || !expect(p, ':') || !digits(p, 2, m) || !expect(p, ':') || !digits(p, 2, s) ||
      !expect(p, '#'))
    return false;
  raDeg = hmsToDeg(h, m, s);
  return true;
}

bool parseSd(const char *cmd, double &decDeg) {
  const char *p = cmd;
  int d, m, s;
  if (strncmp(p, ":Sd", 3)) return false;
  p += 3;
  int sign = optSign(p);
  if (!digits(p, 2, d) || !degSep(p) || !digits(p, 2, m) || !expect(p, ':') || !digits(p, 2, s) ||
      !expect(p, '#'))
    return false;
  decDeg = sign * (d + m / 60.0 + s / 3600.0);
  return true;
}

bool parseSg(const char *cmd, double &lonEast) {
  const char *p = cmd;
  int d, m;
  if (strncmp(p, ":Sg", 3)) return false;
  p += 3;
  int sign = optSign(p);
  const char *start = p;
  if (!digits(p, 3, d)) {
    p = start;
    if (!digits(p, 2, d)) return false;
  }
  if (!degSep(p) || !digits(p, 2, m) || !expect(p, '#')) return false;
  // Meade longitudes are west-positive (0..360); return east-positive in (-180, 180]
  double east = -sign * (d + m / 60.0);
  east = wrap360(east);
  if (east > 180) east -= 360;
  lonEast = east;
  return true;
}

bool parseSt(const char *cmd, double &lat) {
  const char *p = cmd;
  int d, m;
  if (strncmp(p, ":St", 3)) return false;
  p += 3;
  int sign = optSign(p);
  if (!digits(p, 2, d) || !degSep(p) || !digits(p, 2, m) || !expect(p, '#')) return false;
  lat = sign * (d + m / 60.0);
  return true;
}

bool parseSG(const char *cmd, double &hoursToUtc) {
  if (strncmp(cmd, ":SG", 3)) return false;
  char *end;
  double v = strtod(cmd + 3, &end);
  if (end == cmd + 3 || *end != '#') return false;
  hoursToUtc = v;
  return true;
}

bool parseSL(const char *cmd, int &h, int &m, int &s) {
  const char *p = cmd;
  if (strncmp(p, ":SL", 3)) return false;
  p += 3;
  return digits(p, 2, h) && expect(p, ':') && digits(p, 2, m) && expect(p, ':') && digits(p, 2, s) &&
         expect(p, '#') && h < 24 && m < 60 && s < 60;
}

bool parseSC(const char *cmd, int &mo, int &d, int &yy) {
  const char *p = cmd;
  if (strncmp(p, ":SC", 3)) return false;
  p += 3;
  return digits(p, 2, mo) && expect(p, '/') && digits(p, 2, d) && expect(p, '/') && digits(p, 2, yy) &&
         expect(p, '#') && mo >= 1 && mo <= 12 && d >= 1 && d <= 31;
}

// ---------------------------------------------------------------- OnStep formats

static bool sexSep(const char *&p) {
  if (*p == ':' || *p == '*' || *p == '\'' || (unsigned char)*p == 0xDF) {
    p++;
    return true;
  }
  return false;
}

// Unsigned decimal number; integer part required
static bool number(const char *&p, double &v, bool &hadFraction) {
  const char *start = p;
  v = 0;
  while (*p >= '0' && *p <= '9') v = v * 10 + (*p++ - '0');
  if (p == start) return false;
  hadFraction = false;
  if (*p == '.') {
    p++;
    double scale = 0.1;
    while (*p >= '0' && *p <= '9') {
      v += (*p++ - '0') * scale;
      scale /= 10;
    }
    hadFraction = true;
  }
  return true;
}

bool parseSexagesimal(const char *p, double &value) {
  int sign = optSign(p);
  double a, b, c = 0;
  bool fa, fb, fc;
  if (!number(p, a, fa) || fa || !sexSep(p) || !number(p, b, fb) || b >= 60) return false;
  if (!fb && sexSep(p)) {
    if (!number(p, c, fc) || c > 60) return false;
  }
  if (*p != '#') return false;
  value = sign * (a + b / 60.0 + c / 3600.0);
  return true;
}

bool parseOnStepRa(const char *p, double &raDeg) {
  double h;
  if (*p == '-' || *p == '+' || !parseSexagesimal(p, h) || h < 0 || h > 24) return false;
  raDeg = wrap360(h * 15.0);
  return true;
}

bool parseOnStepDec(const char *p, double &decDeg) {
  double d;
  if (!parseSexagesimal(p, d) || fabs(d) > 90) return false;
  decDeg = d;
  return true;
}

bool parseOnStepLat(const char *p, double &lat) { return parseOnStepDec(p, lat); }

bool parseOnStepLon(const char *p, double &lonEast) {
  double w;
  if (!parseSexagesimal(p, w) || fabs(w) > 360) return false;
  double east = wrap360(-w);
  if (east > 180) east -= 360;
  lonEast = east;
  return true;
}

bool parseOnStepUtcOffset(const char *p, double &hoursToUtc) {
  double v;
  if (parseSexagesimal(p, v)) {
    hoursToUtc = v;
    return fabs(v) <= 14;
  }
  char *end;
  v = strtod(p, &end);
  if (end == p || *end != '#' || fabs(v) > 14) return false;
  hoursToUtc = v;
  return true;
}

bool parseOnStepTime(const char *p, int &h, int &m, int &s) {
  double v;
  if (*p == '-' || *p == '+' || !parseSexagesimal(p, v) || v >= 24) return false;
  long t = lround(v * 3600) % 86400;
  h = t / 3600;
  m = t / 60 % 60;
  s = t % 60;
  return true;
}

bool parseOnStepDate(const char *p, int &mo, int &d, int &yy) {
  int y;
  if (!digits(p, 2, mo) || !expect(p, '/') || !digits(p, 2, d) || !expect(p, '/')) return false;
  const char *q = p;
  if (digits(q, 4, y) && *q == '#') {
    yy = y % 100;
  } else if (digits(p, 2, y) && *p == '#') {
    yy = y;
  } else {
    return false;
  }
  return mo >= 1 && mo <= 12 && d >= 1 && d <= 31;
}

void formatRaHigh(double deg, char *out, int outLen) {
  long long t = llround(wrap360(deg) / 15.0 * 3600.0 * 10000.0) % (24LL * 3600 * 10000);  // 1e-4 s
  snprintf(out, outLen, "%02lld:%02lld:%02lld.%04lld", t / 36000000, t / 600000 % 60, t / 10000 % 60, t % 10000);
}

void formatDecHigh(double deg, char *out, int outLen) {
  long long t = llround(fabs(deg) * 3600.0 * 1000.0);  // 1e-3 arcsec
  snprintf(out, outLen, "%c%02lld*%02lld:%02lld.%03lld", deg < 0 ? '-' : '+', t / 3600000, t / 60000 % 60,
           t / 1000 % 60, t % 1000);
}

void formatSite(double deg, int degDigits, bool high, char *out, int outLen) {
  char sign = deg < 0 ? '-' : '+';
  if (high) {
    long long t = llround(fabs(deg) * 3600.0 * 1000.0);
    snprintf(out, outLen, "%c%0*lld*%02lld:%02lld.%03lld", sign, degDigits, t / 3600000, t / 60000 % 60,
             t / 1000 % 60, t % 1000);
  } else {
    long t = lround(fabs(deg) * 60.0);
    snprintf(out, outLen, "%c%0*ld*%02ld", sign, degDigits, t / 60, t % 60);
  }
}

// Howard Hinnant's days_from_civil
int64_t unixFromCivil(int year, int month, int day, int h, int m, int s) {
  year -= month <= 2;
  const int64_t era = (year >= 0 ? year : year - 399) / 400;
  const unsigned yoe = (unsigned)(year - era * 400);
  const unsigned doy = (153 * (month + (month > 2 ? -3 : 9)) + 2) / 5 + day - 1;
  const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  int64_t days = era * 146097 + (int64_t)doe - 719468;
  return days * 86400 + h * 3600 + m * 60 + s;
}

}  // namespace astro
