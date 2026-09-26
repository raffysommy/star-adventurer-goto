#pragma once
#include <Arduino.h>

// State that must survive ESP-only resets (OTA, reboot, crash, watchdog) while the mount
// keeps running, and that changes too often for flash: one versioned struct in RTC memory
// (no flash wear, lost on power-off, which resets the mount too anyway).
//
// Being one struct in its own memory section, its address doesn't move when unrelated
// code changes, so it survives OTAs of new firmware (separate __NOINIT variables didn't).
// A new layout gets a new VERSION and is treated as lost once.
namespace persist {

struct State {
  uint32_t magic;          // MAGIC | VERSION: valid content
  int32_t decMotor;        // DEC motor position (steps)
  int32_t decGear;         // DEC gear-output position (steps, backlash model)
  uint8_t flipped;         // meridian-flip branch
  uint8_t trusted;         // position established by a sync since the last power-on
  double raShiftDeg;       // RA register = axis angle + raShiftDeg (see ra_axis)
};

extern State &s;
// true if the content was kept from before this boot (checked once at boot)
bool kept();
void begin();              // call first: validates, or resets to defaults
void touch();              // mark the content valid (after writing fields)

}  // namespace persist
