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
// True sidereal rate. lx200.py used 0.004176 ("refracted", a fixed 0.05% slow-down);
// refraction is now computed for the actual pointing (settings.refraction).
constexpr double SIDEREAL = 360.9856 / 86400;  // deg/s
constexpr double SIDEREAL_HZ = 60.16427;       // OnStep's rate unit: sidereal = 60.16427 Hz
constexpr double LUNAR_HZ = 57.9, SOLAR_HZ = 60.0, KING_HZ = 60.136;

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
// Tracking rate in OnStep Hz (SIDEREAL_HZ, LUNAR_HZ, ...; not saved, sidereal at boot)
void setTrackRate(double hz);
double trackRateHz();
double refractionFactor();   // current refraction factor applied to the rate (1 if off)
void ratesChanged();         // settings.refraction / guideRate changed
void setRegister(double ha); // debug: redefine the current position (no motion)
void gotoHa(double ha);      // debug: slew to a fixed register angle

// Register limits (degrees): below 0 the firmware stalls, above ~241.7 the 24-bit
// register overflows. GoTo targets come from the window [MARGIN, WINDOW_MAX] (both
// branches); tracking and sync may go on up to TRACK_MAX, where tracking stops.
constexpr double HA_MIN = astro::MARGIN, HA_MAX = astro::WINDOW_MAX + 1, TRACK_MAX = 235.0;
// Where tracking stops: settings.raWestMinutes past the meridian on the flipped branch,
// never beyond TRACK_MAX (register overflow)
double trackMax();

// ---------------------------------------------------------------- PEC
// OnStep-style periodic error correction (lib/core/pec.h). The worm phase comes from the
// register (144-tooth worm); it follows every register rewrite and is saved to NVS every
// 10 s, so it survives power cycles as long as the worm doesn't turn while unpowered.
// Playback never starts while guiding is active (a pulse within the last 60 s): PHD2's
// Predictive PEC would otherwise see the mount change under it.
enum PecState { PEC_IGNORE, PEC_READY_PLAY, PEC_PLAYING, PEC_READY_RECORD, PEC_RECORDING };
struct PecInfo {
  PecState state;
  bool recorded;       // a table exists
  bool indexKnown;     // worm phase known (restored or defined by a recording)
  int segment, segments;
  long wormCounts;
  double countsPerSecond;  // register counts per sidereal second
};
PecInfo pecInfo();
// OnStep $QZ: '+' play, '-' stop, '/' record, 'Z' clear, '!' write to NVS
void pecCommand(char c);
float pecEntry(int seg);                // correction in register counts
void setPecEntry(int seg, float counts);

double lst();                // local sidereal time now, degrees

struct State {
  bool connected;
  bool slewing;
  bool tracking;      // sidereal tracking or guiding (not slewing, not stopped)
  bool guideEast, guideWest;
  const char *phase;  // disconnected / tracking / guiding / slewing / approach / limit / stopped
  long counts;        // raw position register
  uint32_t countsMs;  // millis() when counts was read (rate measurements)
  double axisHa;      // register in degrees (= hour angle + offset())
  double axisRa;      // ra_current
  double slewTarget;
  uint32_t stalls, kicks, keepAlives;
};
State state();

}  // namespace ra
