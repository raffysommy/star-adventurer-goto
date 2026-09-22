#include <math.h>
#include <string.h>
#include <unity.h>

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
  RUN_TEST(test_hms_matches_lx200);
  RUN_TEST(test_parse_and_format);
  RUN_TEST(test_unix_from_civil);
  return UNITY_END();
}
