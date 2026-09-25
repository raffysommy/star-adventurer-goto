#pragma once
#include <Arduino.h>

// UTC wall clock. Sources, in order of trust: NTP (when the Wi-Fi has internet), GPS,
// then the client (:SG/:SL/:SC) or the dashboard's browser time.
void clockBegin();
bool clockValid();
double clockNow();  // unix seconds, UTC, with sub-second resolution
const char *clockSource();

// Sets the clock unless a more trusted source is in charge. Returns true if applied.
bool clockSet(double unixUtc, const char *source);

// LX200 time commands: :SL sets local time of day, :SC the local date (applied
// together with the last :SL), using settings.utcOffset from :SG
void clockLx200Time(int h, int m, int s);
void clockLx200Date(int month, int day, int yy);
