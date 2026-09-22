#pragma once
#include <Arduino.h>

// DEC stepper (EasyDriver: STEP/DIR/EN, EN active low), port of synscan_esp.ino /
// arduino/dec_axis.ino. Step pulses come from hardware (FastAccelStepper), so
// Wi-Fi and USB load can't disturb guiding. Positions are in lx200.py's DEC steps
// (astro::DEC_STEPS_PER_REV per 360 deg, 146400 = 0 deg). Driven directly by the
// LX200 layer; the old controller's TCP protocol is not needed any more.
namespace dec {

constexpr float SIDEREAL_STEPS = 6.796;             // siderealStepsPerSecond
constexpr float GUIDE_SPEED = SIDEREAL_STEPS;       // :Mn / :Ms
constexpr float SLEW_SPEED = SIDEREAL_STEPS * 128;  // :MS

void begin();

void setTarget(long steps);  // :Sd
void slew();                 // :MS  move to the target
void stop();                 // :Q
void guide(int dir);         // :Mn (+1) / :Ms (-1): move at guide speed until stop()
// :Mgn / :Mgs pulse: exactly GUIDE_SPEED * ms steps (fractions carried over to the
// next pulse), instead of a timed stop that would let queued steps overshoot
void guidePulse(int dir, int ms);
void syncToTarget();         // :CM  current position := target
void setInverted(bool inv);  // :CP  (pier east side)

long position();
long target();
bool slewing();              // moving to the target (:D)
bool moving();               // slewing or guiding

}  // namespace dec
