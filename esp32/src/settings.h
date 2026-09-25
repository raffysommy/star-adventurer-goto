#pragma once
#include <Arduino.h>

// Runtime settings persisted in NVS. Defaults match lx200.py's CONFIGURATION block.
struct Settings {
  double lat = 40.8728;
  double lonEast = 14.4377;
  double utcOffset = 0;          // hours, local = UTC + utcOffset (from :SG or the browser)
  bool decAxisReversed = false;  // DEC_AXIS_REVERSED
  bool flipRaGuiding = false;    // FLIP_RA_GUIDING_ON_MERIDIAN
  bool pierEast = false;         // PIER_EAST_SIDE: inverts the DEC motor direction (:CP)
  uint16_t lx200Port = 5001;     // SERVER_PORT
  double raEastLimit = 0;        // HA where the RA register window starts (deg, <= 0); see ra::offset()
  int32_t decBacklash = 250;     // DEC gear play in steps (plate-solve measured ~250-270)
};

extern Settings settings;

void settingsLoad();
void settingsSave();
