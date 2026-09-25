#pragma once
// Periodic error correction, OnStep-style: one correction per worm segment of about one
// sidereal second, in RA register counts. Pure logic (no Arduino), host-tested; the RA
// task owns the state machine and the motor.
//
// The segment is taken from the worm position (register counts relative to a worm
// origin, modulo the counts of one worm turn), not from time, so slews and guiding don't
// shift the phase.
#include <stdint.h>

namespace pec {

constexpr int MAX_SEGMENTS = 1024;

// Worm geometry from the mount's counts per revolution and sidereal counts per second
struct Worm {
  long counts;         // register counts per worm turn (CPR / teeth)
  int segments;        // table size: worm period in sidereal seconds, rounded
  double segCounts;    // counts per segment (~ counts per sidereal second)
};
Worm worm(long cpr, int teeth, double siderealCountsPerSec);

// Worm position of a register value, 0 .. counts-1
long phase(const Worm &w, long counts, long origin);
int segment(const Worm &w, long counts, long origin);

// Recording: RA guide corrections (in counts, + = the mount had to go faster) are added
// to the segment they happened in over one worm turn; finish() turns them into a table.
struct Recorder {
  float acc[MAX_SEGMENTS];
  int n;
  void reset(int segments);
  void add(int seg, float counts);
};

// Recorded corrections -> table: circular moving average (smoothing width in segments,
// odd), mean removed (a drift is not periodic error), and averaged with the previous
// table when there is one (merge).
void finish(const Recorder &r, float *table, int n, int smoothing, bool merge);

// Playback: tracking rate for a segment, as a T1 value (timerHz / counts per second).
// T1 is coarse (~0.23% per unit at sidereal), so the rate error of each segment is
// carried into the next one: over a worm turn the delivered correction is exact.
struct Player {
  double carry = 0;  // counts owed from earlier segments
  uint32_t t1(double siderealCountsPerSec, float correctionCounts, double segSeconds, long timerHz);
  void reset() { carry = 0; }
};

}  // namespace pec
