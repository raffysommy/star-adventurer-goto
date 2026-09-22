#pragma once
#include <Arduino.h>

// RA axis controller: port of lx200.py's tracking(), update_ra_current() (stall
// detector), goto_ra() and the RA guide rates. One FreeRTOS task owns every RA
// motor command; the functions below just queue requests, so they are safe to
// call from any task (LX200 server, web UI, guide timers).
//
// "axis RA" is the RA the motor is physically at (ra_current in lx200.py); the
// meridian-flip bookkeeping that turns it into the RA reported to clients lives
// in the LX200 layer, as in lx200.py.
namespace ra {

constexpr double OFFSET = 2.0;       // offset_star_adventurer: HA of the register's zero margin
constexpr double SIDEREAL = 0.004176;  // sidereal_speed_refracted, deg/s

void begin();

void gotoRa(double axisRa);  // :MS  (slew at max speed, then resume tracking)
void stop();                 // :Q   (abort slew / end guiding / restart tracking)
void sync(double axisRa);    // :CM  (position register := HA(axisRa))
void guide(char dir, int ms);  // 'e' = 1.5x, 'w' = 0.5x sidereal; ms <= 0 until stop()
void setHome();              // register := OFFSET (mount at its home position)
void setRegister(double ha); // debug: redefine the current position (no motion)
void gotoHa(double ha);      // debug: slew to a fixed register angle

// GoTo/sync targets must lie in this register range: HA from the meridian-flip
// logic is [OFFSET, 180 + OFFSET]; below 0 the firmware stalls, above ~241 the
// 24-bit register overflows.
constexpr double HA_MIN = OFFSET, HA_MAX = 183.0;

double lst();                // local sidereal time now, degrees

struct State {
  bool connected;
  bool slewing;
  bool guideEast, guideWest;
  const char *phase;  // disconnected / tracking / guiding / slewing / approach
  long counts;        // raw position register
  double axisHa;      // register in degrees (= hour angle incl. OFFSET)
  double axisRa;      // ra_current
  double slewTarget;
  uint32_t stalls, kicks, keepAlives;
};
State state();

}  // namespace ra
