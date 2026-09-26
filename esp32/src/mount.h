#pragma once
#include <Arduino.h>

// Mount-level state and actions shared by the protocol front-ends (LX200, OnStep):
// the requested target, the meridian-flip branch, guide flags, GoTo/sync/move.
// There is one mount, so there is one flip flag, whichever protocol set it.
// All functions are thread-safe.
namespace mount {

void begin();

struct State {
  bool meridianFlipped;  // on the flipped branch (DEC past the pole)
  double raTarget;       // axis RA of the last GoTo/sync after the branch choice
  bool guideNorth, guideSouth;
};
State state();

void setTargetRa(double raDeg);   // :Sr  (the branch is chosen when used)
void setTargetDec(double decDeg); // :Sd
bool hasTarget();
double targetRaDeg();              // as requested (:Gr#)
double targetDecDeg();             // (:Gd#)
// :MS  GoTo the target, branch from the GoTo window. Returns OnStep's code: 0 started,
// 1 below the horizon limit, 2 above the overhead limit, 6 outside the DEC axis limits,
// 9 no RA target.
int gotoTarget();
double altitudeOf(double raDeg, double decDeg);  // now, at the site
// :CM  Sync to the target, keeping the branch the mount is physically on. false: refused.
bool syncTarget();

void move(char dir);              // :Mn :Ms :Me :Mw  until stopped, at the selected rate
// OnStep :R0..:R9 -> 0.25x 0.5x 1x 2x 4x 8x 20x 48x half-max max (per axis)
void setMoveRate(int index);
int moveRate();
void pulse(char dir, int ms);     // :Mg[nsew]ms
void stop();                      // :Q   all axes
void stopAxis(char dir);          // :Qn :Qs (DEC)  :Qe :Qw (RA)
void setTracking(bool on);        // OnStep :Te / :Td
void applyDecSettings();          // guide rate and axis limits from settings

double reportedRaDeg();
double reportedDecDeg();
String reportedRa();              // "HH:MM:SS"
String reportedDec();             // "sDD*MM:SS"
// Mechanical pier side (ASCOM convention): 'E' = normal branch (looking west),
// 'W' = flipped branch (through the pole).
char pierSide();
bool busy();                      // slewing or guiding (:D)
// Position established by a sync since the last power-on (after a power-on the mount
// starts from an assumed home and needs a sync)
bool positionTrusted();
bool tracking();

}  // namespace mount
