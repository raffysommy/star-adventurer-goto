#pragma once
// Pure mount math, ported from lx200.py. No Arduino dependencies: unit-tested on
// the host with `pio test -e native` against Python/ephem golden values.
//
// Angles are in degrees. RA/HA follow lx200.py's conventions:
//   HA = (LST - RA) % 360 + offset      RA = (LST - (HA - offset)) % 360
// where offset (offset_star_adventurer, 2 deg) keeps the mount's position register
// away from zero, which the Star Adventurer firmware cannot cross.
//
// Unlike lx200.py, LST here is computed correctly: UTC time and longitude in
// degrees (lx200.py feeds ephem local time and a longitude it reads as radians).
#include <stdint.h>

namespace astro {

// Python-style modulo: result in [0, 360) for any input
double wrap360(double deg);

double julianDate(double unixUtc);
// Greenwich / local mean sidereal time in degrees (Meeus 12.4). lonEast: east positive.
double gmstDeg(double unixUtc);
double lstDeg(double unixUtc, double lonEast);

double hourAngle(double raDeg, double lst, double offset);
double rightAscension(double haDeg, double lst, double offset);

// :Sr target handling (LX200Proxy.set_ra): targets beyond HA 180 are reached
// "through the pole" with the meridian_flipped flag set.
struct RaTarget {
  double ra;     // RA the mount axis is actually driven to
  bool flipped;  // meridian_flipped
};
RaTarget selectTarget(double requestedRa, double lst, double offset);

// RA reported to the client (get_ra / dashboard_ra_string)
double reportedRa(double axisRa, bool flipped, double lst, double offset);

// ---------------------------------------------------------------- formatting

void degToHms(double deg, int &h, int &m, int &s);  // degrees_to_hms (truncating)
double hmsToDeg(int h, int m, int s);               // time_to_degrees (wrapping)
// "HH:MM:SS" without the trailing '#'
void formatRa(double deg, char *out, int outLen);
// "sDD*MM:SS" without the trailing '#'
void formatDec(double deg, char *out, int outLen);

// ---------------------------------------------------------------- parsing
// Each returns false if the command doesn't match. Input includes the leading ':'
// and trailing '#'.

bool parseSr(const char *cmd, double &raDeg);    // :SrHH:MM:SS#
bool parseSd(const char *cmd, double &decDeg);   // :SdsDD*MM:SS#
bool parseSg(const char *cmd, double &lonEast);  // :SgDDD*MM#   (Meade: west positive)
bool parseSt(const char *cmd, double &lat);      // :StsDD*MM#
bool parseSG(const char *cmd, double &hoursToUtc);  // :SGsHH.H#  (local + value = UTC)
bool parseSL(const char *cmd, int &h, int &m, int &s);        // :SLHH:MM:SS#
bool parseSC(const char *cmd, int &mo, int &d, int &yy);      // :SCMM/DD/YY#

// Unix time from a civil UTC date/time (proleptic Gregorian)
int64_t unixFromCivil(int year, int month, int day, int h, int m, int s);

}  // namespace astro
