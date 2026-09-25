#pragma once
#include <Arduino.h>

// NMEA GPS (GY-GPS6MV2 / NEO-6M) on UART1: RX 17 <- GPS TX, TX 18 -> GPS RX, 9600 baud.
// - time: sets the clock (trusted below NTP, above clients)
// - site: a good 3D fix updates settings.lat/lon (settings.gpsSetsSite)
// - relay: raw sentences on TCP 10110 (NMEA over TCP) for INDI's "GPS NMEA" driver,
//   so KStars/Ekos get GPS time and location like from a USB GPS
// - OnStep: :GU# reports 'S' (synced) while the GPS keeps the clock
// Without the module nothing happens (checksums reject line noise).
namespace gps {

void begin();

struct State {
  bool enabled;
  bool receiving;    // valid sentences within the last 5 s
  bool timeValid;    // RMC with status A
  bool posValid;     // GGA with a fix
  double lat, lon, altM, hdop;
  int sats;
  uint32_t sentences, badChecksums;
  int32_t lastTimeSyncAgeS;  // -1 if never
};
State state();
bool timeSynced();          // the clock was set from the GPS within the last 10 minutes
void inject(const char *line);  // debug: handle a sentence as if received (/api/gps_nmea)

}  // namespace gps
