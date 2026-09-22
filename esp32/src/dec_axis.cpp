#include "dec_axis.h"

#include <FastAccelStepper.h>
#include <astro.h>
#include <esp_attr.h>

#include "netlog.h"

namespace dec {

static const int STEP_PIN = 6;
static const int DIR_PIN = 5;
static const int ENABLE_PIN = 38;
static const uint32_t ACCEL = 3000;  // steps/s^2: ~0.3 s to slew speed, negligible at guide speed
static const long HOME_STEPS = 146400;  // lx200.py syncs DEC to this at startup (0 deg)
static const uint32_t POS_MAGIC = 0x44454331;

static FastAccelStepperEngine engine;
static FastAccelStepper *stepper;
static volatile long targetPos = HOME_STEPS;
static volatile bool slewActive = false, guideActive = false;
static bool inverted = false;
static double pulseRemainder = 0;  // fractional guide steps not yet issued (signed)

// The stepper is open loop: keep its position across soft resets and OTA updates
__NOINIT_ATTR static int32_t savedPos;
__NOINIT_ATTR static uint32_t savedMagic;

static uint32_t milliHz(float stepsPerSec) { return (uint32_t)(stepsPerSec * 1000.0f); }

long position() { return stepper ? stepper->getCurrentPosition() : 0; }
long target() { return targetPos; }
bool slewing() { return slewActive; }
bool moving() { return slewActive || guideActive; }

void setTarget(long steps) { targetPos = steps; }

void slew() {
  if (!stepper || targetPos == position()) return;
  guideActive = false;
  logf("dec: slew %ld -> %ld", position(), (long)targetPos);
  stepper->setSpeedInMilliHz(milliHz(SLEW_SPEED));
  stepper->moveTo(targetPos);
  slewActive = true;
}

void stop() {
  if (!stepper) return;
  slewActive = guideActive = false;
  stepper->stopMove();
}

void guide(int dir) {
  if (!stepper) return;
  slewActive = false;
  guideActive = true;
  stepper->setSpeedInMilliHz(milliHz(GUIDE_SPEED));
  if (stepper->isRunning()) stepper->applySpeedAcceleration();
  dir > 0 ? stepper->runForward() : stepper->runBackward();
}

void guidePulse(int dir, int ms) {
  if (!stepper || ms <= 0) return;
  slewActive = false;
  pulseRemainder += dir * GUIDE_SPEED * ms / 1000.0;
  long steps = lround(pulseRemainder);
  pulseRemainder -= steps;
  if (!steps) return;
  guideActive = true;
  stepper->setSpeedInMilliHz(milliHz(GUIDE_SPEED));
  stepper->move(steps);
}

void syncToTarget() {
  if (!stepper) return;
  stop();
  while (stepper->isRunning()) delay(1);
  stepper->setCurrentPosition(targetPos);
  logf("dec: synced to %ld", (long)targetPos);
}

void setInverted(bool inv) {
  if (!stepper || inv == inverted) return;
  inverted = inv;
  stepper->setDirectionPin(DIR_PIN, !inv);
  logf("dec: direction %s", inv ? "inverted (pier east)" : "normal");
}

// ---------------------------------------------------------------- task

static void task(void *) {
  while (true) {
    if (slewActive && !stepper->isRunning()) {
      slewActive = false;
      logf("dec: slew complete at %ld", position());
    }
    if (guideActive && !stepper->isRunning()) guideActive = false;  // pulse done
    savedPos = position();
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
  bool kept = savedMagic == POS_MAGIC && esp_reset_reason() != ESP_RST_POWERON;
  stepper->setCurrentPosition(kept ? savedPos : HOME_STEPS);
  targetPos = stepper->getCurrentPosition();
  logf("dec: stepper on STEP %d DIR %d EN %d, position %ld%s", STEP_PIN, DIR_PIN, ENABLE_PIN, position(),
       kept ? " (kept across reset)" : " (home, 0 deg)");
  xTaskCreatePinnedToCore(task, "dec_axis", 4096, nullptr, 4, nullptr, 1);
}

}  // namespace dec
