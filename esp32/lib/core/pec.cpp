#include "pec.h"

#include <math.h>
#include <string.h>

namespace pec {

Worm worm(long cpr, int teeth, double siderealCountsPerSec) {
  Worm w;
  w.counts = cpr / teeth;
  w.segments = (int)lround(w.counts / siderealCountsPerSec);
  if (w.segments > MAX_SEGMENTS) w.segments = MAX_SEGMENTS;
  if (w.segments < 1) w.segments = 1;
  w.segCounts = (double)w.counts / w.segments;
  return w;
}

long phase(const Worm &w, long counts, long origin) {
  long p = (counts - origin) % w.counts;
  return p < 0 ? p + w.counts : p;
}

int segment(const Worm &w, long counts, long origin) {
  int s = (int)(phase(w, counts, origin) / w.segCounts);
  return s >= w.segments ? w.segments - 1 : s;
}

void Recorder::reset(int segments) {
  n = segments;
  memset(acc, 0, sizeof(acc));
}

void Recorder::add(int seg, float counts) {
  if (seg >= 0 && seg < n) acc[seg] += counts;
}

void finish(const Recorder &r, float *table, int n, int smoothing, bool merge) {
  static float tmp[MAX_SEGMENTS];
  int half = smoothing / 2;
  double mean = 0;
  for (int i = 0; i < n; i++) {
    double s = 0;
    for (int k = -half; k <= half; k++) s += r.acc[((i + k) % n + n) % n];
    tmp[i] = (float)(s / (2 * half + 1));
    mean += tmp[i];
  }
  mean /= n;
  for (int i = 0; i < n; i++) {
    float v = tmp[i] - (float)mean;
    table[i] = merge ? (table[i] + v) / 2 : v;
  }
}

uint32_t Player::t1(double siderealCountsPerSec, float correctionCounts, double segSeconds, long timerHz) {
  double want = siderealCountsPerSec + (correctionCounts + carry) / segSeconds;
  if (want < siderealCountsPerSec * 0.5) want = siderealCountsPerSec * 0.5;  // never more than a guide pulse
  if (want > siderealCountsPerSec * 1.5) want = siderealCountsPerSec * 1.5;
  uint32_t t1 = (uint32_t)lround(timerHz / want);
  double got = (double)timerHz / t1;
  carry = (want - got) * segSeconds;
  return t1;
}

}  // namespace pec
