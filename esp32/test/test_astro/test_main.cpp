#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unity.h>

#include <initializer_list>

#include "astro.h"
#include "nmea.h"
#include "pec.h"
#include "golden.h"

using namespace astro;

#define N(a) (sizeof(a) / sizeof((a)[0]))
static const double OFFSET = 2;

void setUp() {}
void tearDown() {}

static double angDiff(double a, double b) {
  double d = fmod(a - b + 540.0, 360.0) - 180.0;
  return fabs(d);
}

// Meeus mean sidereal time vs ephem's apparent sidereal time: they differ by the
// equation of the equinoxes (< 1.2 s of time = 0.005 deg)
void test_lst_matches_ephem() {
  for (size_t i = 0; i < N(LST_CASES); i++) {
    double lst = lstDeg(LST_CASES[i].t, 14.4377);
    TEST_ASSERT_TRUE_MESSAGE(angDiff(lst, LST_CASES[i].lst) < 0.006, "LST differs from ephem");
  }
}

void test_hour_angle_matches_lx200() {
  for (size_t i = 0; i < N(HA_CASES); i++) {
    const HaCase &c = HA_CASES[i];
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, c.ha, hourAngle(c.ra, c.lst, OFFSET));
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, c.raBack, rightAscension(c.ha, c.lst, OFFSET));
  }
}

void test_meridian_flip_matches_lx200() {
  for (size_t i = 0; i < N(TARGET_CASES); i++) {
    const TargetCase &c = TARGET_CASES[i];
    RaTarget t = selectTarget(c.req, c.lst, OFFSET);
    TEST_ASSERT_EQUAL_INT(c.flipped, t.flipped);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, c.ra, t.ra);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, c.reported, reportedRa(t.ra, t.flipped, c.lst, OFFSET));
  }
}

// East limit -30 deg: register = HA + 32, window HA -30..+150 on the normal branch
void test_east_limit_window() {
  const double off = MARGIN + 30, lst = 100;
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 2, hourAngle(lst + 30, lst, off));    // HA -30 -> window start
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 32, hourAngle(lst, lst, off));        // meridian
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 182, hourAngle(lst - 150, lst, off)); // HA +150 -> window end
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 361, hourAngle(lst + 31, lst, off));  // HA -31 wraps high
  // every requested RA lands in [MARGIN, WINDOW_MAX] on one branch, and round-trips
  for (double req = 0; req < 360; req += 0.25) {
    for (double o : {MARGIN, MARGIN + 30, MARGIN + 60}) {
      RaTarget t = selectTarget(req, lst, o);
      double reg = hourAngle(t.ra, lst, o);
      TEST_ASSERT_TRUE_MESSAGE(reg >= MARGIN && reg <= WINDOW_MAX, "target outside the GoTo window");
      TEST_ASSERT_TRUE(angDiff(reportedRa(t.ra, t.flipped, lst, o), req) < 1e-9);
      TEST_ASSERT_TRUE(angDiff(rightAscension(reg, lst, o), t.ra) < 1e-9);
    }
  }
  // meridian and 2 h east: no flip with -30; flip just beyond -30
  TEST_ASSERT_FALSE(selectTarget(lst, lst, off).flipped);
  TEST_ASSERT_FALSE(selectTarget(lst + 29.9, lst, off).flipped);
  TEST_ASSERT_TRUE(selectTarget(lst + 30.1, lst, off).flipped);
  // the old window (east limit 0) flips east of the meridian
  TEST_ASSERT_TRUE(selectTarget(lst + 1, lst, MARGIN).flipped);
}

// A sync keeps the branch the mount is on, even past the GoTo window or a limit
void test_sync_keeps_branch() {
  const double off = MARGIN + 30, lst = 100;
  // flipped target tracked 1 h past the meridian: axis angle ~ 32 + 180 + 15 = 227
  RaTarget t = selectSyncTarget(lst - 15, lst, off, 227);
  TEST_ASSERT_TRUE(t.flipped);  // selectTarget would say normal branch
  TEST_ASSERT_FALSE(selectTarget(lst - 15, lst, off).flipped);
  TEST_ASSERT_DOUBLE_WITHIN(0.01, 227, hourAngle(t.ra, lst, off));
  // same RA, mount on the normal branch (angle 47): stays normal
  TEST_ASSERT_FALSE(selectSyncTarget(lst - 15, lst, off, 47).flipped);
  // exactly at / past a limit (the 2026-09-27 bug): still the nearest branch
  TEST_ASSERT_FALSE(selectSyncTarget(lst - 30.0001, lst, off, 62.0001).flipped);
  TEST_ASSERT_TRUE(selectSyncTarget(lst - 30.0001, lst, off, 242.0001).flipped);
  // a small correction near the nearest candidate: fine
  TEST_ASSERT_TRUE(selectSyncTarget(lst - 15, lst, off, 227 + 10).flipped);
  // position unknown (assumed home, far from both): the GoTo window decides
  for (double req = 0.5; req < 360; req += 1) {
    double home = off;  // HA 0
    RaTarget s = selectSyncTarget(req, lst, off, home);
    double a0 = hourAngle(req, lst, off);
    if (fabs(a0 - home) > SYNC_NEAR + 1 && fabs(a0 - 180 - home) > SYNC_NEAR + 1 && fabs(a0 + 180 - home) > SYNC_NEAR + 1)
      TEST_ASSERT_EQUAL_INT(selectTarget(req, lst, off).flipped, s.flipped);
  }
}

void test_onstep_parsing() {
  double v;
  TEST_ASSERT_TRUE(parseOnStepRa("17:30:00.00#", v));
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 262.5, v);
  TEST_ASSERT_TRUE(parseOnStepRa("17:23:60.00#", v));  // driver rounding: 60 s carries
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 261.0, v);
  TEST_ASSERT_TRUE(parseOnStepRa("05:35:17#", v));
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 83.8208333333, v);
  TEST_ASSERT_TRUE(parseOnStepRa("05:35.3#", v));  // low precision HH:MM.T
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 83.825, v);
  TEST_ASSERT_FALSE(parseOnStepRa("25:00:00#", v));
  TEST_ASSERT_FALSE(parseOnStepRa("17:30:00", v));  // no '#'
  TEST_ASSERT_TRUE(parseOnStepDec("+10*00:00.0#", v));
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 10, v);
  TEST_ASSERT_TRUE(parseOnStepDec("-05*30:36#", v));
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, -5.51, v);
  TEST_ASSERT_TRUE(parseOnStepDec("+45*30#", v));
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 45.5, v);
  TEST_ASSERT_FALSE(parseOnStepDec("+95*00:00#", v));
  TEST_ASSERT_TRUE(parseOnStepLat("+40:52:22.08#", v));
  TEST_ASSERT_DOUBLE_WITHIN(1e-6, 40.8728, v);
  TEST_ASSERT_TRUE(parseOnStepLon("345:33:44.28#", v));  // Meade west-positive 0..360
  TEST_ASSERT_DOUBLE_WITHIN(1e-6, 14.4377, v);
  TEST_ASSERT_TRUE(parseOnStepLon("-014*26#", v));
  TEST_ASSERT_DOUBLE_WITHIN(1e-6, 14.43333333, v);
  TEST_ASSERT_TRUE(parseOnStepUtcOffset("-02:00#", v));
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, -2, v);
  TEST_ASSERT_TRUE(parseOnStepUtcOffset("+05.5#", v));
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 5.5, v);
  int a, b, c;
  TEST_ASSERT_TRUE(parseOnStepTime("03:00:00#", a, b, c));
  TEST_ASSERT_EQUAL_INT(3, a);
  TEST_ASSERT_TRUE(parseOnStepTime("23:59:59.6#", a, b, c));
  TEST_ASSERT_EQUAL_INT(0, a);  // rounds to midnight
  TEST_ASSERT_TRUE(parseOnStepDate("09/26/26#", a, b, c));
  TEST_ASSERT_EQUAL_INT(26, c);
  TEST_ASSERT_TRUE(parseOnStepDate("09/26/2026#", a, b, c));
  TEST_ASSERT_EQUAL_INT(26, c);
  TEST_ASSERT_FALSE(parseOnStepDate("13/26/26#", a, b, c));
}

void test_onstep_formatting() {
  char b[24];
  formatRaHigh(262.5, b, sizeof(b));
  TEST_ASSERT_EQUAL_STRING("17:30:00.0000", b);
  formatRaHigh(359.9999999999, b, sizeof(b));
  TEST_ASSERT_EQUAL_STRING("00:00:00.0000", b);  // never "24:00:00" or ":60"
  formatDecHigh(-5.51, b, sizeof(b));
  TEST_ASSERT_EQUAL_STRING("-05*30:36.000", b);
  formatDecHigh(10.99999999999, b, sizeof(b));
  TEST_ASSERT_EQUAL_STRING("+11*00:00.000", b);
  formatSite(40.8728, 2, false, b, sizeof(b));
  TEST_ASSERT_EQUAL_STRING("+40*52", b);
  formatSite(-14.4377, 3, true, b, sizeof(b));
  TEST_ASSERT_EQUAL_STRING("-014*26:15.720", b);
  // round trip through the parser
  double v;
  formatRaHigh(123.456789, b, sizeof(b));
  strcat(b, "#");
  TEST_ASSERT_TRUE(parseOnStepRa(b, v));
  TEST_ASSERT_DOUBLE_WITHIN(1e-5, 123.456789, v);
}

void test_nmea() {
  // the classic reference sentences (valid checksums)
  const char *rmc = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A";
  const char *gga = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";
  nmea::Fix f;
  TEST_ASSERT_TRUE(nmea::checksumOk(rmc));
  TEST_ASSERT_TRUE(nmea::parse(rmc, f));
  TEST_ASSERT_TRUE(f.timeValid);
  TEST_ASSERT_EQUAL_INT64(764426119, f.unixUtc);  // 1994-03-23 12:35:19 UTC
  TEST_ASSERT_DOUBLE_WITHIN(1e-6, 48.1173, f.lat);
  TEST_ASSERT_DOUBLE_WITHIN(1e-6, 11.516666667, f.lon);
  TEST_ASSERT_TRUE(nmea::parse(gga, f));
  TEST_ASSERT_TRUE(f.posValid);
  TEST_ASSERT_EQUAL_INT(8, f.sats);
  TEST_ASSERT_DOUBLE_WITHIN(1e-6, 545.4, f.altM);
  // corrupted checksum, void RMC, no-fix GGA, other sentences
  TEST_ASSERT_FALSE(nmea::parse("$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6B", f));
  nmea::Fix g;
  // build a void RMC with a correct checksum
  char v[96] = "$GNRMC,001122.00,V,,,,,,,260926,,,N";
  uint8_t sum = 0;
  for (char *p = v + 1; *p; p++) sum ^= (uint8_t)*p;
  snprintf(v + strlen(v), 8, "*%02X", sum);
  TEST_ASSERT_TRUE(nmea::parse(v, g));
  TEST_ASSERT_FALSE(g.timeValid);
  TEST_ASSERT_FALSE(nmea::parse("$GPGSV,2,1,08,01,40,083,46,02,17,308,41,12,07,344,39,14,22,228,45*75", g));
}

void test_pec() {
  const double sid = 12492146 / 360.0 * 360.9856 / 86400;  // counts per sidereal-rate second
  pec::Worm w = pec::worm(12492146, 144, sid);
  TEST_ASSERT_EQUAL_INT(86751, w.counts);
  TEST_ASSERT_EQUAL_INT(598, w.segments);
  TEST_ASSERT_EQUAL_INT(0, pec::segment(w, 1000, 1000));
  TEST_ASSERT_EQUAL_INT(597, pec::segment(w, 999, 1000));   // just before the origin
  TEST_ASSERT_EQUAL_INT(0, pec::segment(w, 1000 + 86751, 1000));
  // record a sine (+ a drift that must be removed), smoothing keeps its shape
  static pec::Recorder r;
  r.reset(w.segments);
  const double amp = 30.0 / 0.1037;  // +-30" in counts, as a PE curve
  for (int i = 0; i < w.segments; i++) {
    // correction per segment = derivative of -PE: what the guider sends
    double c = amp * 2 * M_PI / w.segments * cos(2 * M_PI * i / w.segments) + 0.7;
    r.add(i, (float)c);
  }
  static float table[pec::MAX_SEGMENTS];
  pec::finish(r, table, w.segments, 5, false);
  double sum = 0, peak = 0;
  for (int i = 0; i < w.segments; i++) {
    sum += table[i];
    if (fabs(table[i]) > peak) peak = fabs(table[i]);
  }
  TEST_ASSERT_DOUBLE_WITHIN(1e-3, 0, sum);                            // drift removed
  TEST_ASSERT_DOUBLE_WITHIN(0.01, amp * 2 * M_PI / w.segments, peak);  // shape kept
  // playback delivers the table exactly over a turn despite coarse T1
  pec::Player pl;
  double delivered = 0, wanted = 0;
  const long timerHz = 62338;
  for (int i = 0; i < w.segments; i++) {
    uint32_t t1 = pl.t1(sid, table[i], w.segCounts / sid, timerHz);
    delivered += (double)timerHz / t1 * (w.segCounts / sid);
    wanted += sid * (w.segCounts / sid) + table[i];
  }
  TEST_ASSERT_DOUBLE_WITHIN(1.0, wanted, delivered);  // within one count over ~10 minutes
}

void test_refraction() {
  TEST_ASSERT_DOUBLE_WITHIN(0.1, 0.0, refractionArcmin(90));
  TEST_ASSERT_DOUBLE_WITHIN(0.1, 1.0, refractionArcmin(45));     // ~1' at 45 deg
  TEST_ASSERT_DOUBLE_WITHIN(0.3, 5.3, refractionArcmin(10));     // ~5.3' at 10 deg
  TEST_ASSERT_DOUBLE_WITHIN(3, 29, refractionArcmin(0));         // ~29' at the horizon
  // refraction lifts: apparent altitude higher; on the meridian HA is unchanged
  double h, d;
  apparentHaDec(0, 0, 40.87, h, d);
  TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0, h);
  TEST_ASSERT_TRUE(d > 0 && d < 0.05);  // pushed north (up) by ~1'
  // rate factor: slightly < 1 even on the meridian (lifted toward the pole: smaller
  // apparent circle, more so near the pole), < 1 low in the west
  TEST_ASSERT_DOUBLE_WITHIN(1e-5, 0.99975, refractionRateFactor(0, 20, 40.87));
  TEST_ASSERT_TRUE(refractionRateFactor(0, 80, 40.87) < refractionRateFactor(0, 20, 40.87));
  double f = refractionRateFactor(75, 5, 40.87);
  TEST_ASSERT_TRUE(f < 0.9995 && f > 0.995);
  // symmetric in the east
  TEST_ASSERT_DOUBLE_WITHIN(1e-6, f, refractionRateFactor(-75, 5, 40.87));
  // below 5 deg: no compensation
  TEST_ASSERT_EQUAL_DOUBLE(1.0, refractionRateFactor(100, 0, 40.87));
}

void test_hms_matches_lx200() {
  for (size_t i = 0; i < N(HMS_CASES); i++) {
    int h, m, s;
    degToHms(HMS_CASES[i].deg, h, m, s);
    TEST_ASSERT_EQUAL_INT(HMS_CASES[i].h, h);
    TEST_ASSERT_EQUAL_INT(HMS_CASES[i].m, m);
    TEST_ASSERT_EQUAL_INT(HMS_CASES[i].s, s);
  }
  for (size_t i = 0; i < N(TIMEDEG_CASES); i++) {
    const TimeDegCase &c = TIMEDEG_CASES[i];
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, c.deg, hmsToDeg(c.h, c.m, c.s));
  }
}

// set_dec() -> steps -> steps_to_coord() round trip, both DEC orientations
void test_dec_steps_match_lx200() {
  for (size_t i = 0; i < N(DEC_CASES); i++) {
    const DecCase &c = DEC_CASES[i];
    long steps = decToSteps(decForSteps(c.deg, c.reverse));
    TEST_ASSERT_EQUAL_INT32(c.steps, steps);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, c.back, stepsToDec(steps, c.reverse));
    TEST_ASSERT_DOUBLE_WITHIN(0.002, c.deg, stepsToDec(steps, c.reverse));  // within one step
  }
  TEST_ASSERT_EQUAL_INT32(146400, decToSteps(0));
}

void test_parse_and_format() {
  double v;
  TEST_ASSERT_TRUE(parseSr(":Sr05:35:17#", v));
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, hmsToDeg(5, 35, 17), v);
  TEST_ASSERT_FALSE(parseSr(":Sr5:35:17#", v));
  TEST_ASSERT_FALSE(parseSr(":Sr05:35.2#", v));

  TEST_ASSERT_TRUE(parseSd(":Sd-05*23:28#", v));
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, -(5 + 23 / 60.0 + 28 / 3600.0), v);
  TEST_ASSERT_TRUE(parseSd(":Sd+89\xdf" "15:00#", v));
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 89.25, v);
  TEST_ASSERT_TRUE(parseSd(":Sd45*00:00#", v));
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 45, v);

  TEST_ASSERT_TRUE(parseSg(":Sg345*34#", v));  // Meade west-positive 345.57 = 14.43 E
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 14 + 26 / 60.0, v);
  TEST_ASSERT_TRUE(parseSg(":Sg-014*26#", v));
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 14 + 26 / 60.0, v);
  TEST_ASSERT_TRUE(parseSg(":Sg074*00#", v));
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, -74, v);
  TEST_ASSERT_TRUE(parseSt(":St+40*52#", v));
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 40 + 52 / 60.0, v);
  TEST_ASSERT_TRUE(parseSt(":St-33*30#", v));
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, -33.5, v);
  TEST_ASSERT_TRUE(parseSG(":SG-02.0#", v));
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, -2, v);

  int a, b, c;
  TEST_ASSERT_TRUE(parseSL(":SL21:05:09#", a, b, c));
  TEST_ASSERT_EQUAL_INT(21, a);
  TEST_ASSERT_TRUE(parseSC(":SC09/22/26#", a, b, c));
  TEST_ASSERT_EQUAL_INT(9, a);
  TEST_ASSERT_EQUAL_INT(26, c);

  char buf[16];
  formatDec(-5.391111, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_STRING("-05*23:28", buf);
  formatDec(44.99999, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_STRING("+45*00:00", buf);
  formatRa(hmsToDeg(5, 35, 17) + 1e-7, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_STRING("05:35:17", buf);
}

void test_unix_from_civil() {
  TEST_ASSERT_EQUAL_INT64(0, unixFromCivil(1970, 1, 1, 0, 0, 0));
  TEST_ASSERT_EQUAL_INT64(1758570000, unixFromCivil(2025, 9, 22, 19, 40, 0));
  TEST_ASSERT_EQUAL_INT64(951782400, unixFromCivil(2000, 2, 29, 0, 0, 0));
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_lst_matches_ephem);
  RUN_TEST(test_hour_angle_matches_lx200);
  RUN_TEST(test_meridian_flip_matches_lx200);
  RUN_TEST(test_east_limit_window);
  RUN_TEST(test_sync_keeps_branch);
  RUN_TEST(test_onstep_parsing);
  RUN_TEST(test_onstep_formatting);
  RUN_TEST(test_nmea);
  RUN_TEST(test_pec);
  RUN_TEST(test_refraction);
  RUN_TEST(test_hms_matches_lx200);
  RUN_TEST(test_dec_steps_match_lx200);
  RUN_TEST(test_parse_and_format);
  RUN_TEST(test_unix_from_civil);
  return UNITY_END();
}
