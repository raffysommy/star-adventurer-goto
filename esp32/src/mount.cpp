#include "mount.h"

#include <astro.h>
#include <esp_attr.h>
#include <esp_system.h>
#include <esp_timer.h>

#include "dec_axis.h"
#include "netlog.h"
#include "ra_axis.h"
#include "settings.h"

namespace mount {

static SemaphoreHandle_t lock;
static State st = {false, 0, false, false};
static bool raValid = false, decValid = false;  // no :MS / :CM before a successful :Sr
// :Sr / :Sd only store the request; the branch (meridian flip) is chosen when it's
// used: :MS takes the GoTo window, :CM keeps the branch the mount is physically on.
static double reqRa, reqDec;
static esp_timer_handle_t decGuideTimer;

// The branch must survive OTA/crash resets like the RA register and the DEC position
// do, or the ESP would come back on the wrong side of the pole
__NOINIT_ATTR static uint32_t savedFlipped;
static const uint32_t FLIP_MAGIC = 0x464C5000;  // | flipped bit

struct Guard {
  Guard() { xSemaphoreTake(lock, portMAX_DELAY); }
  ~Guard() { xSemaphoreGive(lock); }
};

static void setFlipped(bool f) {
  st.meridianFlipped = f;
  savedFlipped = FLIP_MAGIC | (f ? 1 : 0);
}

static bool reverseDec() { return st.meridianFlipped ^ settings.decAxisReversed; }

// set_dec(): beyond the pole the axis goes to 180 - dec. Computed when used, with
// the branch just chosen, so the order of :Sr and :Sd doesn't matter.
static void setDecTarget() {
  if (!decValid) return;
  long steps = astro::decToSteps(astro::decForSteps(reqDec, reverseDec()));
  dec::setTarget(steps);
  logf("mount: DEC %.4f -> %ld steps%s", reqDec, steps, reverseDec() ? " (reversed)" : "");
}

// End of a pulse's duration: the step move itself ends on its own
static void decGuideEnd(void *) { st.guideNorth = st.guideSouth = false; }

void begin() {
  lock = xSemaphoreCreateMutex();
  bool kept = esp_reset_reason() != ESP_RST_POWERON && (savedFlipped & ~1u) == FLIP_MAGIC;
  setFlipped(kept && (savedFlipped & 1));
  if (kept && st.meridianFlipped) logf("mount: meridian flipped (kept across reset)");
  esp_timer_create_args_t args = {};
  args.callback = decGuideEnd;
  args.name = "dec_guide";
  esp_timer_create(&args, &decGuideTimer);
}

State state() {
  Guard g;
  return st;
}

void setTargetRa(double raDeg) {
  Guard g;
  reqRa = raDeg;
  raValid = true;
  logf("mount: target RA %.4f", raDeg);
}

void setTargetDec(double decDeg) {
  Guard g;
  reqDec = decDeg;
  decValid = true;
  logf("mount: target DEC %.4f", decDeg);
}

bool hasTarget() {
  Guard g;
  return raValid;
}

double targetRaDeg() {
  Guard g;
  return reqRa;
}

double targetDecDeg() {
  Guard g;
  return reqDec;
}

double altitudeOf(double raDeg, double decDeg) {
  double ha = (ra::lst() - raDeg) * DEG_TO_RAD, dec = decDeg * DEG_TO_RAD, lat = settings.lat * DEG_TO_RAD;
  return asin(sin(dec) * sin(lat) + cos(dec) * cos(lat) * cos(ha)) * RAD_TO_DEG;
}

int gotoTarget() {
  Guard g;
  int code = raValid ? 0 : 9;
  if (raValid && decValid) {
    double alt = altitudeOf(reqRa, reqDec);
    if (alt < settings.horizonLimit) code = 1;
    if (alt > settings.overheadLimit) code = 2;
    if (code) {
      logf("mount: goto REFUSED, target altitude %.1f outside [%d, %d]", alt, (int)settings.horizonLimit,
           (int)settings.overheadLimit);
      return code;
    }
  }
  if (code) {
    logf("mount: goto without a target, ignored");
  } else {
    // set_ra(): targets past the window are reached through the pole
    astro::RaTarget t = astro::selectTarget(reqRa, ra::lst(), ra::offset());
    if (decValid) {
      long steps = astro::decToSteps(astro::decForSteps(reqDec, t.flipped ^ settings.decAxisReversed));
      if (!dec::withinLimits(steps)) {
        logf("mount: goto REFUSED, DEC axis target %ld steps outside the axis limits", steps);
        return 6;
      }
    }
    setFlipped(t.flipped);
    st.raTarget = t.ra;
    logf("mount: goto RA %.4f -> axis RA %.4f%s", reqRa, t.ra, t.flipped ? " (meridian flipped)" : "");
    ra::gotoRa(t.ra);
  }
  setDecTarget();
  dec::slew();
  return code;
}

bool syncTarget() {
  Guard g;
  bool ok = false;
  if (raValid) {
    ra::State rs = ra::state();
    astro::RaTarget t = astro::selectSyncTarget(reqRa, ra::lst(), ra::offset(), rs.axisHa, ra::TRACK_MAX, ok);
    if (ok) {
      if (t.flipped != st.meridianFlipped) logf("mount: sync on the %s branch", t.flipped ? "flipped" : "normal");
      setFlipped(t.flipped);
      st.raTarget = t.ra;
      ra::sync(t.ra);
    } else {
      logf("mount: sync RA %.4f REFUSED, outside the register range on both branches", reqRa);
    }
  } else {
    logf("mount: sync without an RA target, RA not synced");
  }
  setDecTarget();
  dec::syncToTarget();
  return ok;
}

// slow_move(): :Mn :Ms :Me :Mw, or pulse guiding :Mgn1000 (ms > 0)
static void slowMove(char d, int ms) {
  if (d == 'n' || d == 's') {
    st.guideNorth = d == 'n';
    st.guideSouth = d == 's';
    // Beyond the pole (or with DEC_AXIS_REVERSED) north and south swap on the motor
    bool north = (d == 'n') != reverseDec();
    esp_timer_stop(decGuideTimer);
    if (ms > 0) {
      dec::guidePulse(north ? +1 : -1, ms);
      esp_timer_start_once(decGuideTimer, (uint64_t)ms * 1000);  // only clears the :D flags
    } else {
      dec::guide(north ? +1 : -1);  // manual move until stopped
    }
  } else if (d == 'e' || d == 'w') {
    bool swap = st.meridianFlipped && settings.flipRaGuiding;
    char rd = swap ? (d == 'e' ? 'w' : 'e') : d;
    ra::guide(rd, ms);
  }
}

void move(char dir) {
  Guard g;
  slowMove(dir, 0);
}

void pulse(char dir, int ms) {
  Guard g;
  if (ms > 0) slowMove(dir, ms);
}

void stop() {
  Guard g;
  ra::stop();
  esp_timer_stop(decGuideTimer);
  st.guideNorth = st.guideSouth = false;
  dec::stop();
}

void stopAxis(char dir) {
  Guard g;
  if (dir == 'n' || dir == 's') {
    esp_timer_stop(decGuideTimer);
    st.guideNorth = st.guideSouth = false;
    dec::stop();
  } else if (dir == 'e' || dir == 'w') {
    ra::stop();
  }
}

void setTracking(bool on) { ra::setTracking(on); }

void applyDecSettings() {
  dec::setGuideRate(settings.guideRate);
  dec::setLimits(astro::decToSteps(settings.decAxisMin), astro::decToSteps(settings.decAxisMax));
}

double reportedRaDeg() {
  ra::State r = ra::state();
  Guard g;
  return astro::reportedRa(r.axisRa, st.meridianFlipped, ra::lst(), ra::offset());
}

// get_dec() / steps_to_coord()
double reportedDecDeg() {
  Guard g;
  return astro::stepsToDec(dec::position(), reverseDec());
}

String reportedRa() {
  char b[16];
  astro::formatRa(reportedRaDeg(), b, sizeof(b));
  return b;
}

String reportedDec() {
  char b[16];
  astro::formatDec(reportedDecDeg(), b, sizeof(b));
  return b;
}

char pierSide() {
  Guard g;
  return st.meridianFlipped ? 'W' : 'E';
}

bool busy() {
  ra::State rs = ra::state();
  Guard g;
  return rs.slewing || rs.guideEast || rs.guideWest || st.guideNorth || st.guideSouth || dec::slewing();
}

bool tracking() { return ra::state().tracking; }

}  // namespace mount
