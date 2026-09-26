#include "ra_axis.h"

#include <Preferences.h>
#include <astro.h>
#include <pec.h>
#include <esp_timer.h>

#include "clock.h"
#include "mount.h"
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


enum CmdType { GOTO, GOTO_HA, STOP, SYNC, GUIDE, GUIDE_END, HOME, SET_REGISTER, EAST_LIMIT, TRACK_OFF, TRACK_ON, PEC_CMD, RATES, MOVE };
struct Cmd {
  CmdType type;
  double value;
  char dir;
  int ms;
  uint32_t seq;
};

enum Mode { DISCONNECTED, TRACK, SLEW, HALT, MANUAL };  // HALT: stopped (tracking off, or at the limit)
static uint32_t manualLastJMs;
static bool manualRunning;

static QueueHandle_t queue;
static SemaphoreHandle_t stateLock;
static State st;
static esp_timer_handle_t guideTimer;
static uint32_t guideSeq;

static sw::Params params;
static Mode mode = DISCONNECTED;
static uint32_t siderealT1, trackT1;  // trackT1: the base rate, or the guide rate while guiding
static uint32_t baseT1;               // tracking rate (x refraction), or the PEC rate while PEC plays
static volatile double trackHz = SIDEREAL_HZ;

static volatile double refrFactor = 1.0;
static uint32_t lastRefrMs;
static const uint32_t REFRACTION_EVERY_MS = 10000;

// PEC
static const int WORM_TEETH = 144;           // Star Adventurer RA worm wheel
static const uint32_t PEC_GUIDE_QUIET_MS = 60000;
static const uint32_t PHASE_SAVE_MS = 30000;  // +-30 s of phase at most; ~2900 NVS writes/day
static pec::Worm worm;
static double sidCps;                        // register counts per sidereal second
static long wormOrigin;
static bool phaseKnown = false;
static float pecTable[pec::MAX_SEGMENTS];
static bool pecRecorded = false;
static pec::Recorder pecRec;
static pec::Player pecPlayer;
static volatile PecState pecState = PEC_IGNORE;
static volatile int pecSeg = -1;
static long recStartCounts;
static uint32_t lastPulseMs, lastPhaseSaveMs;
static bool guidedSinceGoto = false;  // any guide pulse since boot or the last GoTo
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

double offset() { return astro::MARGIN - settings.raEastLimit; }

static double hourAngle(double raDeg) { return astro::hourAngle(raDeg, lst(), offset()); }

static double axisRaFromCounts(long counts) {
  return astro::rightAscension(sw::countsToDeg(params, counts), lst(), offset());
}

double trackMax() { return fmin(TRACK_MAX, offset() + 180.0 + settings.raWestMinutes / 4.0); }

// ---------------------------------------------------------------- rates

// Tracking rate in register counts per second: sidereal x the chosen rate x refraction
static double baseCps() { return sidCps * (trackHz / SIDEREAL_HZ) * refrFactor; }
// The register runs at exactly timerHz / T1 (test/hil/rate_calibration.py: within 0.04%, T1 280-439)
static uint32_t t1ForCps(double cps) { return (uint32_t)lround(params.timerHz / fmax(cps, 1.0)); }
static void applyBase();

// Base T1 when PEC isn't playing (PEC recomputes it each segment)
static void updateBase() {
  if (!params.timerHz) return;
  if (pecState != PEC_PLAYING) baseT1 = t1ForCps(baseCps());
  applyBase();
}

// Pointing HA/Dec -> refraction factor (single axis, RA only)
static void updateRefraction() {
  double f = 1.0;
  if (settings.refraction) {
    double ha = astro::wrap360(lst() - mount::reportedRaDeg());
    if (ha > 180) ha -= 360;
    f = astro::refractionRateFactor(ha, mount::reportedDecDeg(), settings.lat);
  }
  if (fabs(f - refrFactor) > 2e-6) {
    refrFactor = f;
    updateBase();
  }
}

// ---------------------------------------------------------------- PEC persistence

static void savePhase(long counts) {
  if (!phaseKnown || !worm.counts) return;
  Preferences p;
  p.begin("pec", false);
  p.putInt("phase", (int32_t)pec::phase(worm, counts, wormOrigin));
  p.putInt("cnt", (int32_t)counts);
  p.putBool("ok", true);
  p.end();
}

static void saveTable() {
  Preferences p;
  p.begin("pec", false);
  p.putInt("n", worm.segments);
  p.putBool("rec", pecRecorded);
  p.putBytes("tbl", pecTable, worm.segments * sizeof(float));
  p.end();
  logf("ra: PEC table saved (%d segments, %s)", worm.segments, pecRecorded ? "recorded" : "empty");
}

// fresh: the mount was just powered (register restarted near 0)
static void loadPec(bool fresh, long counts) {
  Preferences p;
  if (!p.begin("pec", true)) return;
  if (p.getInt("n", 0) == worm.segments && p.getBool("rec", false)) {
    pecRecorded = p.getBytes("tbl", pecTable, worm.segments * sizeof(float)) == worm.segments * sizeof(float);
  }
  if (p.getBool("ok", false)) {
    long ph = p.getInt("phase", 0), cnt = p.getInt("cnt", 0);
    // The worm doesn't turn while unpowered: after a power-on the register restarted
    // at 0 where the worm was at the saved phase; after an ESP-only restart the register
    // simply continued.
    wormOrigin = fresh ? -ph : cnt - ph;
    phaseKnown = true;
  }
  p.end();
  logf("ra: PEC %s, worm phase %s (segment %d of %d)", pecRecorded ? "table loaded" : "not recorded",
       phaseKnown ? (fresh ? "restored after power-on" : "restored") : "unknown",
       phaseKnown ? pec::segment(worm, counts, wormOrigin) : -1, worm.segments);
}

// Every register rewrite moves the worm origin with it: the worm itself didn't turn
static bool setRegister(long newCounts) {
  long old;
  bool haveOld = sw::getPos(AXIS, old);
  if (!sw::setPos(AXIS, newCounts)) return false;
  if (haveOld && phaseKnown) {
    wormOrigin += newCounts - old;
    savePhase(newCounts);
  }
  return true;
}

// Apply the base rate unless a guide pulse is running
static void applyBase() {
  if (st.guideEast || st.guideWest || trackT1 == baseT1) return;
  trackT1 = baseT1;
  if (mode == TRACK) sw::setT1(AXIS, trackT1);
}

static void pecStop(const char *why) {
  if (pecState == PEC_PLAYING || pecState == PEC_READY_PLAY || pecState == PEC_RECORDING) logf("ra: PEC %s", why);
  pecState = PEC_IGNORE;
  updateBase();
}

static void pecStep(long counts) {
  if (!phaseKnown || !worm.counts) return;
  int seg = pec::segment(worm, counts, wormOrigin);
  if (seg == pecSeg) return;
  pecSeg = seg;
  switch (pecState) {
    case PEC_RECORDING:
      if (counts - recStartCounts >= worm.counts) {  // one full worm turn
        pec::finish(pecRec, pecTable, worm.segments, 5, pecRecorded);
        pecRecorded = true;
        pecState = PEC_IGNORE;
        saveTable();
        logf("ra: PEC recording done%s", pecRecorded ? " (averaged with the previous table)" : "");
      }
      break;
    case PEC_READY_PLAY:
      // Never switch on under an active guider: PHD2's Predictive PEC would double-correct.
      // Strict: only before guiding starts on this target (a GoTo resets it).
      if (settings.pecStrict ? !guidedSinceGoto : millis() - lastPulseMs > PEC_GUIDE_QUIET_MS) {
        pecState = PEC_PLAYING;
        pecPlayer.reset();
        logf("ra: PEC playing from segment %d", seg);
      }
      break;
    default:
      break;
  }
  if (pecState == PEC_PLAYING) {
    baseT1 = pecPlayer.t1(baseCps(), pecTable[seg], worm.segCounts / sidCps, params.timerHz);
    applyBase();
  }
}

static void pecHandle(char c) {
  long counts = 0;
  sw::getPos(AXIS, counts);
  switch (c) {
    case '+':
      if (!pecRecorded || !phaseKnown) {
        logf("ra: PEC play REFUSED (%s)", !pecRecorded ? "nothing recorded" : "worm phase unknown");
      } else if (pecState != PEC_PLAYING) {
        pecState = PEC_READY_PLAY;
        logf("ra: PEC ready to play (%s)", settings.pecStrict
                                               ? (guidedSinceGoto ? "waits for the next GoTo: guiding already started"
                                                                  : "starts now, before guiding")
                                               : "starts after 60 s without guide pulses");
      }
      break;
    case '-':
      pecStop("stopped");
      break;
    case '/':
      pecStop("stopped for recording");
      if (!phaseKnown) {  // no phase yet: the recording defines it (segment 0 here)
        wormOrigin = counts;
        phaseKnown = true;
        savePhase(counts);
      }
      pecRec.reset(worm.segments);
      recStartCounts = counts;
      pecState = PEC_RECORDING;
      logf("ra: PEC recording one worm turn (%.0f s) from segment %d", worm.counts / sidCps,
           pec::segment(worm, counts, wormOrigin));
      break;
    case 'Z':
      pecStop("cleared");
      memset(pecTable, 0, sizeof(pecTable));
      pecRecorded = false;
      saveTable();
      break;
    case '!':
      saveTable();
      break;
  }
}

static void setPhase(const char *phase) {
  xSemaphoreTake(stateLock, portMAX_DELAY);
  st.phase = phase;
  st.slewing = mode == SLEW;
  st.tracking = mode == TRACK;
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
  trackT1 = baseT1;
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
  sidCps = SIDEREAL * params.cpr / 360.0;
  siderealT1 = trackT1 = baseT1 = t1ForCps(baseCps());
  worm = pec::worm(params.cpr, WORM_TEETH, sidCps);
  long counts = 0;
  sw::getPos(AXIS, counts);
  // A freshly powered mount reads ~0 (it starts tracking on its own, so not exactly
  // 0 by the time we look); anything else means only the ESP restarted (e.g. OTA)
  // and the register still holds a valid sync.
  bool fresh = labs(counts) < sw::degToCounts(params, 1.0);
  loadPec(fresh, counts);
  if (fresh) {
    setRegister(sw::degToCounts(params, offset()));
    logf("ra: fresh mount, register set to home (HA 0, register %.1f deg)", offset());
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
// which misbehaves once the register leaves its wrap window)
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
  trackT1 = baseT1;
  long counts;
  if (!sw::getPos(AXIS, counts)) return;
  if (pecState == PEC_RECORDING) pecStop("recording ABORTED by a slew");
  if (!isHa) guidedSinceGoto = false;  // a new target: PHD2 starts a new guiding run
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
  // Past TRACK_MAX the register would soon overflow: stop instead of tracking on.
  // A GoTo (into the window, possibly flipping) or a sync takes it from here.
  if (sw::countsToDeg(params, counts) > trackMax()) {
    sw::stopSoft(AXIS);
    sw::waitStopped(AXIS);
    st.guideEast = st.guideWest = false;
    trackT1 = baseT1;
    mode = HALT;
    setPhase("limit");
    logf("ra: tracking STOPPED at the RA limit (register %.3f > %.1f deg)", sw::countsToDeg(params, counts), trackMax());
    return;
  }
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
      if (mode == MANUAL) {
        st.guideEast = st.guideWest = false;
        trackT1 = baseT1;
        startTracking();
        logf("ra: manual move stopped, tracking");
      } else if (mode == SLEW) {
        finishSlew("aborted", 0);
      } else if (st.guideEast || st.guideWest) {
        esp_timer_stop(guideTimer);
        endGuiding();
      } else if (mode == TRACK) {
        startTracking();
      }  // HALT: stays stopped (tracking off is not undone by a halt)
      break;
    case TRACK_OFF:
      if (mode == SLEW) finishSlew("aborted", 0);
      esp_timer_stop(guideTimer);
      sw::stopSoft(AXIS);
      sw::waitStopped(AXIS);
      st.guideEast = st.guideWest = false;
      trackT1 = baseT1;
      mode = HALT;
      setPhase("stopped");
      logf("ra: tracking off");
      break;
    case TRACK_ON:
      if (mode == HALT) {
        startTracking();
        logf("ra: tracking on");
      }
      break;
    case SYNC: {
      if (!clockValid()) logf("ra: WARNING clock not set, sync will be wrong");
      double ha = hourAngle(c.value);
      if (ha < HA_MIN || ha > TRACK_MAX) {
        logf("ra: sync REFUSED, HA %.3f outside [%.0f, %.0f]", ha, HA_MIN, TRACK_MAX);
        break;
      }
      if (mode == SLEW) finishSlew("aborted by sync", 0);
      sw::stopSoft(AXIS);
      sw::waitStopped(AXIS);
      setRegister(sw::degToCounts(params, ha));
      logf("ra: synced to RA %.4f (HA register %.4f deg)", c.value, ha);
      startTracking();
      break;
    }
    case SET_REGISTER:
      if (mode == SLEW) finishSlew("aborted", 0);
      sw::stopSoft(AXIS);
      sw::waitStopped(AXIS);
      if (setRegister(sw::degToCounts(params, c.value))) logf("ra: register set to %.4f deg", c.value);
      else logf("ra: register value %.4f deg REFUSED", c.value);
      startTracking();
      break;
    case EAST_LIMIT: {
      if (mode == SLEW) {
        logf("ra: east limit change REFUSED during a slew");
        break;
      }
      long counts;
      if (!sw::getPos(AXIS, counts)) break;
      double delta = c.value - settings.raEastLimit;  // register = HA - eastLimit + MARGIN
      double reg = sw::countsToDeg(params, counts) - delta;
      if (reg < 0.5 || reg > TRACK_MAX) {
        logf("ra: east limit %.1f REFUSED, register would be %.3f deg", c.value, reg);
        break;
      }
      sw::stopSoft(AXIS);
      sw::waitStopped(AXIS);
      setRegister(sw::degToCounts(params, reg));
      settings.raEastLimit = c.value;
      settingsSave();
      logf("ra: east limit %.1f deg (offset %.1f), register shifted to %.3f deg", c.value, offset(), reg);
      startTracking();
      break;
    }
    case PEC_CMD:
      pecHandle(c.dir);
      break;
    case MOVE: {
      if (mode == SLEW || mode == DISCONNECTED) break;
      bool east = c.dir == 'e';
      double r = fmin(fmax(c.value, 0.05), MOVE_MAX_X);
      if (pecState == PEC_RECORDING) pecStop("recording ABORTED by a manual move");
      esp_timer_stop(guideTimer);
      st.guideEast = east;
      st.guideWest = !east;
      if (r < 0.95) {  // a speed change around tracking, like a long guide pulse
        if (mode != TRACK) {
          trackT1 = baseT1;
          startTracking();
        }
        trackT1 = t1ForCps(baseCps() + (east ? -1 : 1) * r * sidCps);
        sw::setT1(AXIS, trackT1);
      } else {
        double motor = (east ? r - 1 : r + 1) * sidCps;  // east from 1x: the motor reverses
        manualRunning = motor > 0.05 * sidCps;
        if (manualRunning) {
          kick(east, max(SLEW_T1, t1ForCps(motor)), true);
        } else {  // exactly 1x east: the sky moves, the motor stands
          sw::stopSoft(AXIS);
          sw::waitStopped(AXIS);
        }
        mode = MANUAL;
        manualLastJMs = millis();
      }
      setPhase("moving");
      logf("ra: manual move %s at %.2fx", east ? "east" : "west", r);
      break;
    }
    case RATES:
      if (c.value > 0) trackHz = c.value;
      lastRefrMs = 0;  // refraction on/off: recompute now
      updateRefraction();
      updateBase();
      logf("ra: tracking rate %.4f Hz, refraction %s (factor %.6f), guide rate %.2fx", (double)trackHz,
           settings.refraction ? "on" : "off", (double)refrFactor, settings.guideRate);
      break;
    case HOME:
      sw::stopSoft(AXIS);
      sw::waitStopped(AXIS);
      setRegister(sw::degToCounts(params, offset()));
      logf("ra: register set to home (HA 0, register %.1f deg)", offset());
      startTracking();
      break;
    case GUIDE: {
      if (mode != TRACK) break;  // no guiding during a slew
      bool east = c.dir == 'e';
      st.guideEast = east;
      st.guideWest = !east;
      if (c.ms > 0) {  // pulses are guiding; continuous moves are not
        lastPulseMs = millis();
        guidedSinceGoto = true;
      }
      // PEC recording: the correction the guider asked for, in register counts
      // (west = 1.5x = the mount had to go faster = +)
      if (pecState == PEC_RECORDING && c.ms > 0 && pecSeg >= 0)
        pecRec.add(pecSeg, (east ? -0.5f : 0.5f) * (float)(sidCps * c.ms / 1000.0));
      // ASCOM/EQMOD convention: west = the scope moves west with the sky = faster.
      // lx200.py had it the other way round (measured on the sky 2026-09-25).
      trackT1 = t1ForCps(baseCps() + (east ? -1 : 1) * settings.guideRate * sidCps);
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
    st.countsMs = millis();
    st.axisHa = sw::countsToDeg(params, counts);
    st.axisRa = axisRaFromCounts(counts);
    xSemaphoreGive(stateLock);
    if (mode == MANUAL) {
      double reg = sw::countsToDeg(params, counts);
      if (reg < 0.5 || reg > trackMax()) {
        logf("ra: manual move STOPPED at the register limit (%.3f deg)", reg);
        st.guideEast = st.guideWest = false;
        trackT1 = baseT1;
        startTracking();
      } else if (manualRunning && millis() - manualLastJMs >= KEEPALIVE_MS) {
        sw::start(AXIS);  // the firmware auto-stops a fast run after ~0.5-0.8 s
        manualLastJMs = millis();
        st.keepAlives++;
      }
    }
    if (mode == SLEW) slewStep(counts);
    else if (mode == TRACK) {
      trackStep(counts);
      pecStep(counts);
      if (millis() - lastRefrMs > REFRACTION_EVERY_MS) {
        lastRefrMs = millis();
        updateRefraction();
      }
    }
    if (phaseKnown && pecRecorded && millis() - lastPhaseSaveMs > PHASE_SAVE_MS) {  // no table: phase unused
      lastPhaseSaveMs = millis();
      savePhase(counts);
    }
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

void setTracking(bool on) { post(on ? TRACK_ON : TRACK_OFF); }

void move(char dir, double rate) { post(MOVE, rate, dir); }

void pecCommand(char c) { post(PEC_CMD, 0, c); }

void setTrackRate(double hz) {
  if (hz > 50 && hz < 70) post(RATES, hz);
}
double trackRateHz() { return trackHz; }
double refractionFactor() { return refrFactor; }
void ratesChanged() { post(RATES, 0); }

PecInfo pecInfo() {
  PecInfo i;
  i.state = pecState;
  i.recorded = pecRecorded;
  i.indexKnown = phaseKnown;
  i.segment = pecSeg;
  i.segments = worm.segments;
  i.wormCounts = worm.counts;
  i.countsPerSecond = sidCps;
  return i;
}

float pecEntry(int seg) { return seg >= 0 && seg < worm.segments ? pecTable[seg] : 0; }

void setPecEntry(int seg, float counts) {
  if (seg < 0 || seg >= worm.segments) return;
  pecTable[seg] = counts;
  pecRecorded = true;  // a table written from outside (e.g. a measured curve) counts as recorded
}

void setEastLimit(double deg) {
  deg = constrain(deg, -90.0, 0.0);
  if (!state().connected) {  // mount off: its register restarts at home anyway
    settings.raEastLimit = deg;
    settingsSave();
    logf("ra: east limit %.1f deg (mount not connected)", deg);
    return;
  }
  post(EAST_LIMIT, deg);
}

State state() {
  xSemaphoreTake(stateLock, portMAX_DELAY);
  State s = st;
  xSemaphoreGive(stateLock);
  return s;
}

}  // namespace ra
