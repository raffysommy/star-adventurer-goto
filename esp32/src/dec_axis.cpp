#include "dec_axis.h"

#include <FastAccelStepper.h>
#include <esp_attr.h>

#include "netlog.h"

namespace dec {

static const int STEP_PIN = 6;
static const int DIR_PIN = 5;
static const int ENABLE_PIN = 38;
static const uint32_t ACCEL = 3000;  // steps/s^2: ~0.3 s to slew speed, negligible at guide speed
static const long HOME_STEPS = 146400;  // lx200.py syncs DEC to this at startup (0 deg)
static const uint32_t POS_MAGIC = 0x44454332;
static const uint32_t POS_MAGIC_V1 = 0x44454331;  // before the backlash model: motor only

static FastAccelStepperEngine engine;
static FastAccelStepper *stepper;
static volatile long targetPos = HOME_STEPS;
static volatile bool slewActive = false, guideActive = false;
static bool inverted = false;
static double pulseRemainder = 0;  // fractional guide steps not yet issued (signed)
static long guideTarget;           // gear position the queued pulses end at
static volatile bool pulseActive = false;

// Backlash model: the gear output moves only when the motor pushes it, so it stays
// within [motor - backlash, motor]; motor - backlash while pushing forward, motor
// while pushing backward.
static volatile long backlash = 250;
static volatile float guideSpeed = SIDEREAL_STEPS * 0.5f;
static volatile long limMin = 0, limMax = astro::DEC_STEPS_PER_REV;
static long gear = HOME_STEPS;
static portMUX_TYPE gearMux = portMUX_INITIALIZER_UNLOCKED;

static volatile bool slewPhase2 = false;  // after an overshoot, finish forward to targetPos
static volatile int pendingRun = 0;       // guide(): start running once the play is taken up

// The stepper is open loop: keep its position across soft resets and OTA updates
__NOINIT_ATTR static int32_t savedPos, savedGear;
__NOINIT_ATTR static uint32_t savedMagic;

static uint32_t milliHz(float stepsPerSec) { return (uint32_t)(stepsPerSec * 1000.0f); }

long motorPosition() { return stepper ? stepper->getCurrentPosition() : 0; }

long position() {
  long m = motorPosition();
  portENTER_CRITICAL(&gearMux);
  if (gear > m) gear = m;
  if (gear < m - backlash) gear = m - backlash;
  long g = gear;
  portEXIT_CRITICAL(&gearMux);
  return g;
}

long target() { return targetPos; }
bool slewing() { return slewActive; }
bool moving() { return slewActive || guideActive; }

void setTarget(long steps) { targetPos = steps; }

void setGuideRate(float x) { guideSpeed = SIDEREAL_STEPS * x; }

void setLimits(long minSteps, long maxSteps) {
  limMin = minSteps;
  limMax = maxSteps;
  logf("dec: axis limits %ld .. %ld steps", minSteps, maxSteps);
}

bool withinLimits(long steps) { return steps >= limMin && steps <= limMax; }

void setBacklash(long steps) {
  backlash = max(0L, steps);
  logf("dec: backlash %ld steps", (long)backlash);
}

// Motor position that leaves the gear at g after moving in direction dir
static long motorFor(long g, int dir) { return dir > 0 ? g + backlash : g; }

// Steps the motor turns without moving the gear before a move in direction dir
static long playBefore(int dir) {
  long m = motorPosition(), g = position();
  return dir > 0 ? g + backlash - m : m - g;
}

void slew() {
  if (!stepper) return;
  if (!withinLimits(targetPos)) {
    logf("dec: slew to %ld REFUSED, outside the axis limits %ld .. %ld", (long)targetPos, (long)limMin, (long)limMax);
    return;
  }
  long g = position();
  guideActive = pulseActive = false;
  pendingRun = 0;
  pulseRemainder = 0;
  stepper->setSpeedInMilliHz(milliHz(SLEW_SPEED));
  if (targetPos < g) {
    // reached moving backward: go past it, then come back forward
    logf("dec: slew %ld -> %ld (via %ld)", g, (long)targetPos, (long)targetPos - GOTO_OVERSHOOT);
    slewPhase2 = true;
    stepper->moveTo(motorFor(targetPos - GOTO_OVERSHOOT, -1));
  } else {
    long mt = motorFor(targetPos, +1);
    if (mt == motorPosition()) return;
    logf("dec: slew %ld -> %ld", g, (long)targetPos);
    slewPhase2 = false;
    stepper->moveTo(mt);
  }
  slewActive = true;
}

void stop() {
  if (!stepper) return;
  slewActive = guideActive = pulseActive = false;
  slewPhase2 = false;
  pendingRun = 0;
  stepper->stopMove();
}

void guide(int dir) {
  if (!stepper) return;
  if ((dir > 0 && position() >= limMax) || (dir < 0 && position() <= limMin)) {
    logf("dec: move REFUSED, at the axis limit");
    return;
  }
  slewActive = false;
  slewPhase2 = false;
  pulseActive = false;
  guideActive = true;
  if (playBefore(dir) > 0) {
    // take the play up fast, the task then starts the guide-speed run
    pendingRun = dir;
    stepper->setSpeedInMilliHz(milliHz(SLEW_SPEED));
    stepper->moveTo(motorFor(position(), dir));
    return;
  }
  pendingRun = 0;
  stepper->setSpeedInMilliHz(milliHz(guideSpeed));
  if (stepper->isRunning()) stepper->applySpeedAcceleration();
  dir > 0 ? stepper->runForward() : stepper->runBackward();
}

void guidePulse(int dir, int ms) {
  if (!stepper || ms <= 0) return;
  slewActive = false;
  slewPhase2 = false;
  pendingRun = 0;
  pulseRemainder += dir * guideSpeed * ms / 1000.0;
  long steps = lround(pulseRemainder);
  pulseRemainder -= steps;
  if (!steps) return;
  int d = steps > 0 ? 1 : -1;
  if (!pulseActive) guideTarget = position();
  pulseActive = true;
  guideTarget += steps;
  if (guideTarget > limMax) guideTarget = limMax;
  if (guideTarget < limMin) guideTarget = limMin;
  long mt = motorFor(guideTarget, d);
  // after a reversal the play is run out at slew speed, together with the (few) pulse
  // steps: the sky then moves by the pulse, not by nothing
  bool takeUp = labs(mt - motorPosition()) > labs(guideTarget - position()) + 1;
  guideActive = true;
  stepper->setSpeedInMilliHz(milliHz(takeUp ? SLEW_SPEED : guideSpeed));
  stepper->moveTo(mt);
  if (takeUp) logf("dec: pulse %+ld steps, backlash take-up %ld", steps, labs(mt - motorPosition()) - labs(steps));
}

void syncToTarget() {
  if (!stepper) return;
  stop();
  while (stepper->isRunning()) delay(1);
  long offset = motorPosition() - position();  // keep the play state
  stepper->setCurrentPosition(targetPos + offset);
  portENTER_CRITICAL(&gearMux);
  gear = targetPos;
  portEXIT_CRITICAL(&gearMux);
  logf("dec: synced to %ld (motor %ld)", (long)targetPos, motorPosition());
}

void setInverted(bool inv, bool atBoot) {
  if (!stepper || inv == inverted) return;
  if (atBoot) {
    inverted = inv;
    stepper->setDirectionPin(DIR_PIN, !inv);
    logf("dec: direction %s", inv ? "inverted (pier east)" : "normal");
    return;
  }
  stop();
  while (stepper->isRunning()) delay(1);
  // the motor turns the other way for the same counts: the side of the play the
  // gear sits on mirrors
  long g = position(), offset = motorPosition() - g;
  inverted = inv;
  stepper->setDirectionPin(DIR_PIN, !inv);
  stepper->setCurrentPosition(g + backlash - offset);
  logf("dec: direction %s", inv ? "inverted (pier east)" : "normal");
}

// ---------------------------------------------------------------- task

static void task(void *) {
  while (true) {
    long pos = position();  // keeps the backlash model current
    // continuous moves (:Mn/:Ms) stop at the axis limits
    if (guideActive && !pulseActive && stepper->isRunning() && (pos > limMax || pos < limMin)) {
      stop();
      logf("dec: move STOPPED at the axis limit (%ld)", pos);
    }
    if (slewActive && !stepper->isRunning()) {
      if (slewPhase2) {
        slewPhase2 = false;
        stepper->moveTo(motorFor(targetPos, +1));
      } else {
        slewActive = false;
        logf("dec: slew complete at %ld (motor %ld)", position(), motorPosition());
      }
    }
    if (guideActive && pendingRun && !stepper->isRunning()) {
      int dir = pendingRun;
      pendingRun = 0;
      stepper->setSpeedInMilliHz(milliHz(guideSpeed));
      dir > 0 ? stepper->runForward() : stepper->runBackward();
    } else if (guideActive && !stepper->isRunning()) {
      guideActive = pulseActive = false;  // pulse done
    }
    savedPos = motorPosition();
    savedGear = position();
    savedMagic = POS_MAGIC;
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void begin() {
  engine.init();
  stepper = engine.stepperConnectToPin(STEP_PIN);
  if (!stepper) {
    logf("dec: cannot attach stepper to GPIO%d", STEP_PIN);
    return;
  }
  stepper->setDirectionPin(DIR_PIN, true);
  stepper->setEnablePin(ENABLE_PIN, true);  // EasyDriver ENABLE is active low
  stepper->setAutoEnable(true);             // driver off while idle, like the old controller
  stepper->setAcceleration(ACCEL);
  bool warm = esp_reset_reason() != ESP_RST_POWERON;
  bool kept = warm && (savedMagic == POS_MAGIC || savedMagic == POS_MAGIC_V1);
  if (kept && savedMagic == POS_MAGIC_V1) savedGear = savedPos;
  stepper->setCurrentPosition(kept ? savedPos : HOME_STEPS);
  gear = kept ? savedGear : HOME_STEPS;
  targetPos = position();
  logf("dec: stepper on STEP %d DIR %d EN %d, position %ld (motor %ld)%s", STEP_PIN, DIR_PIN, ENABLE_PIN,
       position(), motorPosition(), kept ? " (kept across reset)" : " (home, 0 deg)");
  xTaskCreatePinnedToCore(task, "dec_axis", 4096, nullptr, 4, nullptr, 1);
}

}  // namespace dec
