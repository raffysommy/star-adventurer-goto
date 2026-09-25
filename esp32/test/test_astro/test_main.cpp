#include <math.h>
#include <string.h>
#include <unity.h>

#include <initializer_list>

#include "astro.h"
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

// A sync keeps the branch the mount is on, even past the GoTo window
void test_sync_keeps_branch() {
  const double off = MARGIN + 30, lst = 100, trackMax = 235;
  bool ok;
  // flipped target tracked 1 h past the meridian: register ~ 32 + 180 + 15 = 227
  RaTarget t = selectSyncTarget(lst - 15, lst, off, 227, trackMax, ok);
  TEST_ASSERT_TRUE(ok);
  TEST_ASSERT_TRUE(t.flipped);                 // selectTarget would say normal branch
  TEST_ASSERT_FALSE(selectTarget(lst - 15, lst, off).flipped);
  TEST_ASSERT_DOUBLE_WITHIN(0.01, 227, hourAngle(t.ra, lst, off));
  // same RA, mount on the normal branch (register 47): stays normal
  t = selectSyncTarget(lst - 15, lst, off, 47, trackMax, ok);
  TEST_ASSERT_TRUE(ok);
  TEST_ASSERT_FALSE(t.flipped);
  // fresh mount (register at home) agrees with the GoTo choice (off the exact
  // window edge, where either branch is fine)
  for (double req = 0.5; req < 360; req += 1) {
    t = selectSyncTarget(req, lst, off, off, trackMax, ok);
    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_INT(selectTarget(req, lst, off).flipped, t.flipped);
  }
  // past the tracking limit on both branches: refused
  selectSyncTarget(lst - 239, lst, MARGIN, 100, trackMax, ok);  // registers 241 and 61 -> 61 valid
  TEST_ASSERT_TRUE(ok);
}

// What the INDI OnStep driver actually sends (tools/onstep/capture-indi-2.2.0.log)
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
  RUN_TEST(test_hms_matches_lx200);
  RUN_TEST(test_dec_steps_match_lx200);
  RUN_TEST(test_parse_and_format);
  RUN_TEST(test_unix_from_civil);
  return UNITY_END();
}
