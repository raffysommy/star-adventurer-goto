#pragma once
#include <Arduino.h>
#include <astro.h>

// RA axis controller: port of lx200.py's tracking(), update_ra_current() (stall
// detector), goto_ra() and the RA guide rates. One FreeRTOS task owns every RA
// motor command; the functions below just queue requests, so they are safe to
// call from any task (LX200 server, web UI, guide timers).
//
// "axis RA" is the RA the motor is physically at (ra_current in lx200.py); the
// meridian-flip bookkeeping that turns it into the RA reported to clients lives
// in the LX200 layer, as in lx200.py.
namespace ra {

// register = HA + offset(), offset = MARGIN - settings.raEastLimit. With the east limit
// at 0 this is lx200.py's offset_star_adventurer (2 deg).
double offset();
constexpr double SIDEREAL = 0.004176;  // sidereal_speed_refracted, deg/s

void begin();

void gotoRa(double axisRa);  // :MS  (slew at max speed, then resume tracking)
void stop();                 // :Q   (abort slew / end guiding / restart tracking)
void sync(double axisRa);    // :CM  (position register := HA(axisRa))
void guide(char dir, int ms);  // 'w' = 1.5x, 'e' = 0.5x sidereal; ms <= 0 until stop()
void setHome();              // register := offset() (mount at its home position, HA 0)
// Move the register window (settings.raEastLimit). The register is shifted by the same
// amount so the mount's physical position keeps its meaning.
void setEastLimit(double deg);
// Tracking off: motor stopped until tracking on, a GoTo or a sync (OnStep :Td / :Te)
void setTracking(bool on);
void setRegister(double ha); // debug: redefine the current position (no motion)
void gotoHa(double ha);      // debug: slew to a fixed register angle

// Register limits (degrees): below 0 the firmware stalls, above ~241.7 the 24-bit
// register overflows. GoTo targets come from the window [MARGIN, WINDOW_MAX] (both
// branches); tracking and sync may go on up to TRACK_MAX, where tracking stops.
constexpr double HA_MIN = astro::MARGIN, HA_MAX = astro::WINDOW_MAX + 1, TRACK_MAX = 235.0;

double lst();                // local sidereal time now, degrees

struct State {
  bool connected;
  bool slewing;
  bool tracking;      // sidereal tracking or guiding (not slewing, not stopped)
  bool guideEast, guideWest;
  const char *phase;  // disconnected / tracking / guiding / slewing / approach / limit / stopped
  long counts;        // raw position register
  double axisHa;      // register in degrees (= hour angle + offset())
  double axisRa;      // ra_current
  double slewTarget;
  uint32_t stalls, kicks, keepAlives;
};
State state();

}  // namespace ra
