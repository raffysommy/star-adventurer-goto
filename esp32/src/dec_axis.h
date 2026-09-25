#pragma once
#include <Arduino.h>
#include <astro.h>

// DEC stepper (EasyDriver: STEP/DIR/EN, EN active low), port of synscan_esp.ino /
// arduino/dec_axis.ino. Step pulses come from hardware (FastAccelStepper), so
// Wi-Fi and USB load can't disturb guiding. Positions are in lx200.py's DEC steps
// (astro::DEC_STEPS_PER_REV per 360 deg, 146400 = 0 deg). Driven directly by the
// LX200 layer; the old controller's TCP protocol is not needed any more.
//
// Backlash: the gear train has ~250 steps (~0.3 deg) of play, measured with plate
// solves. Positions reported and targeted here are the gear output; the motor runs
// the play out at slew speed on every reversal (guide pulses too), and GoTos always
// end moving forward (+steps), so they land the same way the sync was taken.
namespace dec {

// lx200.py called 6.796 steps/s "sidereal"; it is really 2x sidereal
constexpr float SIDEREAL_STEPS = astro::DEC_STEPS_PER_REV / 360.0 * 360.9856 / 86400;  // 3.398
constexpr float GUIDE_SPEED = SIDEREAL_STEPS * 0.5;  // :Mn / :Ms / pulses, as RA's 0.5x
constexpr float SLEW_SPEED = SIDEREAL_STEPS * 256;   // :MS and backlash take-up (870 steps/s)
constexpr long GOTO_OVERSHOOT = 100;                 // steps past a target reached moving backward

void begin();

void setTarget(long steps);  // :Sd
void slew();                 // :MS  move to the target
void stop();                 // :Q
void guide(int dir);         // :Mn (+1) / :Ms (-1): move at guide speed until stop()
// :Mgn / :Mgs pulse: exactly GUIDE_SPEED * ms steps (fractions carried over to the
// next pulse), instead of a timed stop that would let queued steps overshoot
void guidePulse(int dir, int ms);
void syncToTarget();         // :CM  current position := target
// :CP (pier east side). atBoot: the saved position was taken with this direction already
void setInverted(bool inv, bool atBoot = false);
void setBacklash(long steps);

long position();             // gear output position
long motorPosition();
long target();
bool slewing();              // moving to the target (:D)
bool moving();               // slewing or guiding

}  // namespace dec
