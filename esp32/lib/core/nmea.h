#pragma once
// NMEA 0183 parsing for the GPS (GY-GPS6MV2 / u-blox NEO-6M, and anything that sends
// RMC + GGA). Pure logic, host-tested.
#include <stdint.h>

namespace nmea {

struct Fix {
  bool timeValid = false;  // RMC with status A and a date
  int64_t unixUtc = 0;     // whole seconds (RMC time of the fix)
  int centis = 0;          // hundredths of a second
  bool posValid = false;   // GGA with fix quality > 0
  double lat = 0, lon = 0; // degrees, north/east positive
  double altM = 0;
  int sats = 0;
  double hdop = 99;
};

// Checksum "*hh" must match. Accepts any talker (GP, GN, GL, ...).
bool checksumOk(const char *line);
// Parses one sentence into fix (RMC: time/date/position; GGA: position/alt/sats).
// Returns true if the sentence was an RMC or GGA with a valid checksum.
bool parse(const char *line, Fix &fix);

}  // namespace nmea
