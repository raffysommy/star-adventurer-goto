#include "nmea.h"

#include <stdlib.h>
#include <string.h>

#include "astro.h"

namespace nmea {

static int hex(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

bool checksumOk(const char *line) {
  if (line[0] != '$') return false;
  uint8_t sum = 0;
  const char *p = line + 1;
  while (*p && *p != '*') sum ^= (uint8_t)*p++;
  if (*p != '*' || hex(p[1]) < 0 || hex(p[2]) < 0) return false;
  return sum == (hex(p[1]) << 4 | hex(p[2]));
}

// Splits the sentence body into comma-separated fields (in place, on a copy)
static int split(char *s, char **f, int max) {
  int n = 0;
  f[n++] = s;
  for (char *p = s; *p && n < max; p++) {
    if (*p == ',') {
      *p = 0;
      f[n++] = p + 1;
    } else if (*p == '*') {
      *p = 0;
      break;
    }
  }
  return n;
}

// ddmm.mmmm / dddmm.mmmm + hemisphere -> degrees
static bool coord(const char *v, const char *hemi, double &deg) {
  if (!*v || !*hemi) return false;
  double x = atof(v);
  int d = (int)(x / 100);
  deg = d + (x - d * 100) / 60.0;
  if (*hemi == 'S' || *hemi == 'W') deg = -deg;
  return true;
}

bool parse(const char *line, Fix &fix) {
  if (!checksumOk(line)) return false;
  char buf[96];
  strncpy(buf, line + 1, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = 0;
  char *f[24];
  int n = split(buf, f, 24);
  if (strlen(f[0]) != 5) return false;
  const char *type = f[0] + 2;  // skip the talker (GP, GN, ...)

  if (!strcmp(type, "RMC") && n >= 10) {
    // $GPRMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,speed,course,ddmmyy,...
    if (strlen(f[1]) < 6 || strlen(f[9]) != 6) return true;
    int hh = (f[1][0] - '0') * 10 + f[1][1] - '0', mm = (f[1][2] - '0') * 10 + f[1][3] - '0';
    double ss = atof(f[1] + 4);
    int day = (f[9][0] - '0') * 10 + f[9][1] - '0', mon = (f[9][2] - '0') * 10 + f[9][3] - '0';
    int yy = (f[9][4] - '0') * 10 + f[9][5] - '0';
    bool active = f[2][0] == 'A';
    fix.timeValid = active && mon >= 1 && mon <= 12 && day >= 1;
    if (fix.timeValid) {
      fix.unixUtc = astro::unixFromCivil((yy < 80 ? 2000 : 1900) + yy, mon, day, hh, mm, (int)ss);
      fix.centis = (int)((ss - (int)ss) * 100 + 0.5);
      double lat, lon;
      if (coord(f[3], f[4], lat) && coord(f[5], f[6], lon)) {
        fix.lat = lat;
        fix.lon = lon;
      }
    }
    return true;
  }
  if (!strcmp(type, "GGA") && n >= 10) {
    // $GPGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,q,sats,hdop,alt,M,...
    int quality = atoi(f[6]);
    fix.sats = atoi(f[7]);
    fix.posValid = quality > 0;
    if (fix.posValid) {
      double lat, lon;
      if (coord(f[2], f[3], lat) && coord(f[4], f[5], lon)) {
        fix.lat = lat;
        fix.lon = lon;
      }
      fix.hdop = atof(f[8]);
      fix.altM = atof(f[9]);
    }
    return true;
  }
  return false;
}

}  // namespace nmea
