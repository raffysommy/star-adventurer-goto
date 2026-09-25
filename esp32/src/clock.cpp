#include "clock.h"

#include <astro.h>
#include <math.h>
#include <esp_sntp.h>
#include <sys/time.h>

#include "netlog.h"
#include "settings.h"

static const char *source = "none";
static int pendingH = -1, pendingM, pendingS;

static void onNtpSync(struct timeval *) {
  source = "ntp";
  logf("clock: NTP sync");
}

void clockBegin() {
  sntp_set_time_sync_notification_cb(onNtpSync);
  configTime(0, 0, "pool.ntp.org", "time.google.com");
}

bool clockValid() { return time(nullptr) > 1700000000; }

double clockNow() {
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  return tv.tv_sec + tv.tv_usec / 1e6;
}

const char *clockSource() { return source; }

// Trust order: NTP (ms, needs internet) > GPS (NMEA, ~0.1 s) > client / browser
static int rank(const char *src) { return !strcmp(src, "ntp") ? 3 : !strcmp(src, "gps") ? 2 : 1; }

bool clockSet(double unixUtc, const char *src) {
  double diff = unixUtc - clockNow();
  if (rank(src) < rank(source)) {
    logf("clock: ignoring %s time (%s in charge, diff %.1f s)", src, source, diff);
    return false;
  }
  if (!strcmp(src, "gps") && !strcmp(source, "gps") && fabs(diff) < 0.5) return true;  // already right
  struct timeval tv = {(time_t)unixUtc, (suseconds_t)((unixUtc - (time_t)unixUtc) * 1e6)};
  settimeofday(&tv, nullptr);
  source = src;
  logf("clock: set from %s (was off by %.1f s)", src, diff);
  return true;
}

// Meade :SG is "hours to add to local time to get UTC"; utcOffset is local - UTC
static void applyLocal(int year, int month, int day, int h, int m, int s) {
  int64_t local = astro::unixFromCivil(year, month, day, h, m, s);
  clockSet(local - settings.utcOffset * 3600.0, "lx200");
}

void clockLx200Time(int h, int m, int s) {
  pendingH = h;
  pendingM = m;
  pendingS = s;
  if (!clockValid()) return;  // wait for :SC to know the date
  time_t localNow = (time_t)(clockNow() + settings.utcOffset * 3600.0);
  struct tm t;
  gmtime_r(&localNow, &t);
  applyLocal(t.tm_year + 1900, t.tm_mon + 1, t.tm_mday, h, m, s);
}

void clockLx200Date(int month, int day, int yy) {
  int h = pendingH, m = pendingM, s = pendingS;
  if (h < 0) {  // no :SL yet: keep the current local time of day
    time_t localNow = (time_t)(clockNow() + settings.utcOffset * 3600.0);
    struct tm t;
    gmtime_r(&localNow, &t);
    h = t.tm_hour;
    m = t.tm_min;
    s = t.tm_sec;
  }
  applyLocal(2000 + yy, month, day, h, m, s);
}
