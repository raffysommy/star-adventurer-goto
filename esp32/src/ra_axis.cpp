#include "ra_axis.h"

#include <astro.h>
#include <esp_timer.h>

#include "clock.h"
#include "mount_usb.h"
#include "netlog.h"
#include "settings.h"
#include "skywatcher.h"

namespace ra {

static const int AXIS = 1;
static const uint32_t LOOP_MS = 100;
static const uint32_t KEEPALIVE_MS = 400;    // slews: the firmware auto-stops after ~0.5-0.8 s
static const uint32_t RUN_CHECK_MS = 250;    // tracking: poll :f, restart with J if stopped
static const uint32_t STALL_CHECK_MS = 1500; // no progress despite J -> full re-kick
static const uint32_t SLEW_T1 = 5;           // max speed (~0.28 deg/s); lower T1 isn't faster
static const double APPROACH_DEG = 0.1;      // goto_ra(): switch to 8x below this
static const double APPROACH_RATE = 8;       // x sidereal
static const uint32_t SLEW_TIMEOUT_MS = 20 * 60 * 1000;


enum CmdType { GOTO, GOTO_HA, STOP, SYNC, GUIDE, GUIDE_END, HOME, SET_REGISTER };
struct Cmd {
  CmdType type;
  double value;
  char dir;
  int ms;
  uint32_t seq;
};

enum Mode { DISCONNECTED, TRACK, SLEW };

static QueueHandle_t queue;
static SemaphoreHandle_t stateLock;
static State st;
static esp_timer_handle_t guideTimer;
static uint32_t guideSeq;

static sw::Params params;
static Mode mode = DISCONNECTED;
static uint32_t siderealT1, trackT1;  // trackT1: sidereal, or the guide rate while guiding
static long lastCheckCounts;
static uint32_t lastCheckMs;
static uint32_t lastRunCheckMs;

static struct {
  double target, d0;
  bool targetIsHa;  // debug gotoHa(): fixed register angle instead of an RA
  bool ccw, approach;
  uint32_t startMs, lastJMs;
} slew;

double lst() { return astro::lstDeg(clockNow(), settings.lonEast); }

static double hourAngle(double raDeg) { return astro::hourAngle(raDeg, lst(), OFFSET); }

static double axisRaFromCounts(long counts) {
  return astro::rightAscension(sw::countsToDeg(params, counts), lst(), OFFSET);
}

static void setPhase(const char *phase) {
  xSemaphoreTake(stateLock, portMAX_DELAY);
  st.phase = phase;
  st.slewing = mode == SLEW;
  xSemaphoreGive(stateLock);
}

// lx200.py's motor sequence: stop, wait until really stopped (G is refused while
// running), then mode, step period, start.
static bool kick(bool ccw, uint32_t t1, bool hard) {
  hard ? sw::stopHard(AXIS) : sw::stopSoft(AXIS);
  bool ok = sw::waitStopped(AXIS) && sw::setMode(AXIS, false, ccw) && sw::setT1(AXIS, t1) && sw::start(AXIS);
  st.kicks++;
  if (!ok) logf("ra: kick failed (ccw=%d t1=%lu)", ccw, (unsigned long)t1);
  return ok;
}

static void resetStallCheck() {
  lastCheckMs = millis();
  sw::getPos(AXIS, lastCheckCounts);
}

// tracking(): sidereal CW (or the current guide rate)
static void startTracking() {
  mode = TRACK;
  kick(false, trackT1, false);
  resetStallCheck();
  setPhase(st.guideEast || st.guideWest ? "guiding" : "tracking");
}

static void endGuiding() {
  st.guideEast = st.guideWest = false;
  trackT1 = siderealT1;
  if (mode == TRACK) {
    sw::setT1(AXIS, trackT1);
    setPhase("tracking");
  }
}

static void onConnect() {
  if (!sw::readParams(AXIS, params)) {
    logf("ra: cannot read mount parameters, retrying");
    delay(1000);
    return;
  }
  sw::initDone(AXIS);
  siderealT1 = trackT1 = sw::t1ForRate(params, SIDEREAL);
  long counts = 0;
  sw::getPos(AXIS, counts);
  // A freshly powered mount reads ~0 (it starts tracking on its own, so not exactly
  // 0 by the time we look); anything else means only the ESP restarted (e.g. OTA)
  // and the register still holds a valid sync.
  if (labs(counts) < sw::degToCounts(params, 1.0)) {
    sw::setPos(AXIS, sw::degToCounts(params, OFFSET));
    logf("ra: fresh mount, register set to home (HA %.1f deg)", OFFSET);
  } else {
    logf("ra: keeping position register (%.3f deg)", sw::countsToDeg(params, counts));
  }
  logf("ra: connected, CPR %ld, timer %ld Hz, sidereal T1 %lu", params.cpr, params.timerHz,
       (unsigned long)siderealT1);
  xSemaphoreTake(stateLock, portMAX_DELAY);
  st.connected = true;
  xSemaphoreGive(stateLock);
  startTracking();
}

// ---------------------------------------------------------------- slewing

static double targetHa() { return slew.targetIsHa ? slew.target : hourAngle(slew.target); }

// Distance to go, from the raw register (lx200.py wraps HA(current) through RA,
// which misbehaves once the register leaves [OFFSET, 360 + OFFSET))
static double slewDistance(long counts) { return sw::countsToDeg(params, counts) - targetHa(); }

static void finishSlew(const char *why, double d) {
  sw::stopHard(AXIS);
  sw::waitStopped(AXIS);
  logf("ra: slew %s after %.1f s, residual %.4f deg", why, (millis() - slew.startMs) / 1000.0, d);
  startTracking();
}

static void startSlew(double target, bool isHa) {
  if (!isHa && !clockValid()) logf("ra: WARNING clock not set, goto target will be wrong");
  st.guideEast = st.guideWest = false;
  trackT1 = siderealT1;
  long counts;
  if (!sw::getPos(AXIS, counts)) return;
  slew.target = target;
  slew.targetIsHa = isHa;
  double ha = targetHa();
  if (ha < HA_MIN || ha > HA_MAX) {
    logf("ra: goto REFUSED, target HA %.3f outside [%.0f, %.0f]", ha, HA_MIN, HA_MAX);
    return;
  }
  // goto_ra(): CW0 = HA(current) - HA(target); CW when <= 0
  slew.d0 = slewDistance(counts);
  if (fabs(slew.d0) < 1e-4) return;
  slew.ccw = slew.d0 > 0;
  slew.approach = false;
  slew.startMs = slew.lastJMs = millis();
  mode = SLEW;
  xSemaphoreTake(stateLock, portMAX_DELAY);
  st.slewTarget = target;
  xSemaphoreGive(stateLock);
  logf("ra: slew %s %.3f deg to %s %.4f (HA %.4f)", slew.ccw ? "CCW" : "CW", fabs(slew.d0), isHa ? "register" : "RA",
       target, ha);
  bool fast = fabs(slew.d0) >= APPROACH_DEG;
  slew.approach = !fast;
  kick(slew.ccw, fast ? SLEW_T1 : sw::t1ForRate(params, APPROACH_RATE * SIDEREAL), true);
  resetStallCheck();
  setPhase(fast ? "slewing" : "approach");
}

static void slewStep(long counts) {
  double d = slewDistance(counts);
  uint32_t now = millis();
  if (slew.d0 * d <= 0) return finishSlew("done", d);  // sign change: arrived / overshot
  if (now - slew.startMs > SLEW_TIMEOUT_MS) return finishSlew("TIMED OUT", d);

  if (!slew.approach && fabs(d) < APPROACH_DEG) {
    slew.approach = true;
    kick(slew.ccw, sw::t1ForRate(params, APPROACH_RATE * SIDEREAL), true);
    slew.lastJMs = now;
    resetStallCheck();
    setPhase("approach");
    return;
  }
  // The firmware stops a slew on its own after ~0.5-0.8 s (CW always, CCW after a
  // reversal); re-sending J keeps it going without touching mode or speed
  if (now - slew.lastJMs >= KEEPALIVE_MS) {
    sw::start(AXIS);
    slew.lastJMs = now;
    st.keepAlives++;
  }
  if (now - lastCheckMs >= STALL_CHECK_MS) {
    bool stuck = slew.ccw ? counts >= lastCheckCounts : counts <= lastCheckCounts;
    if (stuck) {
      st.stalls++;
      logf("ra: slew stalled at %.3f deg to go, re-kicking", fabs(d));
      kick(slew.ccw, slew.approach ? sw::t1ForRate(params, APPROACH_RATE * SIDEREAL) : SLEW_T1, true);
      slew.lastJMs = millis();
    }
    lastCheckMs = now;
    lastCheckCounts = counts;
  }
}

// update_ra_current(), made non-invasive: a stopped motor gets a bare J, which
// keeps the mode and step period, so a guide pulse in progress is not disturbed.
// Only if the position still doesn't advance is the full stop/G/I/J sequence used.
static void trackStep(long counts) {
  uint32_t now = millis();
  if (now - lastRunCheckMs >= RUN_CHECK_MS) {
    lastRunCheckMs = now;
    sw::Status s;
    if (sw::status(AXIS, s) && !s.running) {
      sw::start(AXIS);
      st.keepAlives++;
    }
  }
  if (now - lastCheckMs < STALL_CHECK_MS) return;
  if (counts <= lastCheckCounts) {
    st.stalls++;
    logf("ra: tracking stalled (%ld -> %ld), full restart", lastCheckCounts, counts);
    kick(false, trackT1, false);
    sw::getPos(AXIS, counts);
  }
  lastCheckMs = millis();
  lastCheckCounts = counts;
}

// ---------------------------------------------------------------- commands

static void guideTimerCb(void *) {
  Cmd c = {GUIDE_END, 0, 0, 0, guideSeq};
  xQueueSend(queue, &c, 0);
}

static void handle(const Cmd &c) {
  switch (c.type) {
    case GOTO:
      startSlew(c.value, false);
      break;
    case GOTO_HA:
      startSlew(c.value, true);
      break;
    case STOP:  // stop_all()
      if (mode == SLEW) {
        finishSlew("aborted", 0);
      } else if (st.guideEast || st.guideWest) {
        esp_timer_stop(guideTimer);
        endGuiding();
      } else {
        startTracking();
      }
      break;
    case SYNC: {
      if (!clockValid()) logf("ra: WARNING clock not set, sync will be wrong");
      double ha = hourAngle(c.value);
      if (ha < HA_MIN || ha > HA_MAX) {
        logf("ra: sync REFUSED, HA %.3f outside [%.0f, %.0f]", ha, HA_MIN, HA_MAX);
        break;
      }
      if (mode == SLEW) finishSlew("aborted by sync", 0);
      sw::stopSoft(AXIS);
      sw::waitStopped(AXIS);
      sw::setPos(AXIS, sw::degToCounts(params, ha));
      logf("ra: synced to RA %.4f (HA register %.4f deg)", c.value, ha);
      startTracking();
      break;
    }
    case SET_REGISTER:
      if (mode == SLEW) finishSlew("aborted", 0);
      sw::stopSoft(AXIS);
      sw::waitStopped(AXIS);
      if (sw::setPos(AXIS, sw::degToCounts(params, c.value))) logf("ra: register set to %.4f deg", c.value);
      else logf("ra: register value %.4f deg REFUSED", c.value);
      startTracking();
      break;
    case HOME:
      sw::stopSoft(AXIS);
      sw::waitStopped(AXIS);
      sw::setPos(AXIS, sw::degToCounts(params, OFFSET));
      logf("ra: register set to home (HA %.1f deg)", OFFSET);
      startTracking();
      break;
    case GUIDE: {
      if (mode != TRACK) break;  // no guiding during a slew
      bool east = c.dir == 'e';
      st.guideEast = east;
      st.guideWest = !east;
      trackT1 = sw::t1ForRate(params, (east ? 1.5 : 0.5) * SIDEREAL);  // slew_ra_east / slew_ra_west
      sw::setT1(AXIS, trackT1);
      setPhase("guiding");
      esp_timer_stop(guideTimer);
      if (c.ms > 0) esp_timer_start_once(guideTimer, (uint64_t)c.ms * 1000);
      break;
    }
    case GUIDE_END:
      if (c.seq == guideSeq && (st.guideEast || st.guideWest)) endGuiding();
      break;
  }
}

static void task(void *) {
  while (true) {
    Cmd c;
    bool got = xQueueReceive(queue, &c, pdMS_TO_TICKS(LOOP_MS));
    if (!mountConnected()) {
      if (mode != DISCONNECTED) {
        mode = DISCONNECTED;
        xSemaphoreTake(stateLock, portMAX_DELAY);
        st.connected = false;
        xSemaphoreGive(stateLock);
        setPhase("disconnected");
        logf("ra: mount disconnected");
      }
      continue;
    }
    if (mode == DISCONNECTED) {
      onConnect();
      continue;
    }
    if (got) handle(c);

    long counts;
    if (!sw::getPos(AXIS, counts)) continue;
    xSemaphoreTake(stateLock, portMAX_DELAY);
    st.counts = counts;
    st.axisHa = sw::countsToDeg(params, counts);
    st.axisRa = axisRaFromCounts(counts);
    xSemaphoreGive(stateLock);
    if (mode == SLEW) slewStep(counts);
    else trackStep(counts);
  }
}

// ---------------------------------------------------------------- public API

static void post(CmdType type, double value = 0, char dir = 0, int ms = 0) {
  Cmd c = {type, value, dir, ms, 0};
  if (type == GUIDE || type == STOP) c.seq = ++guideSeq;  // invalidates pending guide timers
  xQueueSend(queue, &c, pdMS_TO_TICKS(100));
}

void begin() {
  queue = xQueueCreate(16, sizeof(Cmd));
  stateLock = xSemaphoreCreateMutex();
  st.phase = "disconnected";
  esp_timer_create_args_t args = {};
  args.callback = guideTimerCb;
  args.name = "ra_guide";
  esp_timer_create(&args, &guideTimer);
  xTaskCreatePinnedToCore(task, "ra_axis", 6144, nullptr, 5, nullptr, 1);
}

void gotoRa(double axisRa) { post(GOTO, axisRa); }
void stop() { post(STOP); }
void sync(double axisRa) { post(SYNC, axisRa); }
void guide(char dir, int ms) { post(GUIDE, 0, dir, ms); }
void setHome() { post(HOME); }
void setRegister(double ha) { post(SET_REGISTER, ha); }
void gotoHa(double ha) { post(GOTO_HA, ha); }

State state() {
  xSemaphoreTake(stateLock, portMAX_DELAY);
  State s = st;
  xSemaphoreGive(stateLock);
  return s;
}

}  // namespace ra
