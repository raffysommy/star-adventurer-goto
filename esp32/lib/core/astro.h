#pragma once
// Pure mount math, ported from lx200.py. No Arduino dependencies: unit-tested on
// the host with `pio test -e native` against Python/ephem golden values.
//
// Angles are in degrees. "HA" here is the mount's register angle:
//   register = (LST - RA + offset - MARGIN) % 360 + MARGIN   RA = (LST - (register - offset)) % 360
// i.e. register = hour angle + offset, wrapped into [MARGIN, 360 + MARGIN).
// offset = MARGIN - east limit: with the east limit at 0 (offset 2, lx200.py's
// offset_star_adventurer) this is exactly lx200.py's formula; a negative east limit
// (e.g. -30: offset 32) moves the register window east of the meridian. MARGIN keeps
// the register away from zero, which the Star Adventurer firmware cannot cross.
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

constexpr double MARGIN = 2.0;       // lowest register angle used
constexpr double WINDOW_MAX = 182.0;  // GoTo window: register [MARGIN, WINDOW_MAX] on both branches

double hourAngle(double raDeg, double lst, double offset);
double rightAscension(double haDeg, double lst, double offset);

// GoTo target (LX200Proxy.set_ra): targets past the window are reached "through the
// pole" (axis RA + 180, DEC 180 - dec) with the meridian_flipped flag set. Unlike
// lx200.py (threshold 180) the threshold is WINDOW_MAX, so no target lands below MARGIN.
struct RaTarget {
  double ra;     // RA the mount axis is actually driven to
  bool flipped;  // meridian_flipped
};
RaTarget selectTarget(double requestedRa, double lst, double offset);

// Sync target: a sync doesn't move the mount, so it must keep the branch the mount
// is physically on. Of the two candidates, those with a register in [MARGIN, trackMax]
// are valid; the one nearest the current register wins (a real sync correction is
// small, the other branch is ~180 deg away). ok = false if neither is valid.
RaTarget selectSyncTarget(double requestedRa, double lst, double offset, double currentRegister, double trackMax,
                          bool &ok);

// RA reported to the client (get_ra / dashboard_ra_string)
double reportedRa(double axisRa, bool flipped, double lst, double offset);

// ---------------------------------------------------------------- DEC steps
// lx200.py maps DEC -180..+180 onto 0..DEC_STEPS_PER_REV steps of the DEC motor
// (step_per_rev_dec = 585600 / 2); 0 deg = 146400.
constexpr long DEC_STEPS_PER_REV = 292800;

long decToSteps(double deg);  // dec_to_steps()
// steps_to_coord() before formatting; reverse = meridian_flipped ^ DEC_AXIS_REVERSED
double stepsToDec(long steps, bool reverse);
// set_dec(): the value handed to dec_to_steps() for a requested DEC
double decForSteps(double requestedDeg, bool reverse);

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
