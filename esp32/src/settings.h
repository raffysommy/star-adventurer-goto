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
  uint16_t lx200Port = 5001;     // SERVER_PORT (OnStep/LX200 commands; 9999 is served too)
  double raEastLimit = 0;        // HA where the RA register window starts (deg, <= 0); see ra::offset()
  int32_t decBacklash = 250;     // DEC gear play in steps (plate-solve measured ~250-270)
  // Flipped branch: tracking goes on this many minutes past the meridian (OnStep :SXEA),
  // capped by the register overflow (ra::trackMax())
  int32_t raWestMinutes = 120;
  int32_t horizonLimit = -30;    // GoTo refused below this altitude (deg, OnStep :Sh, -30..30)
  int32_t overheadLimit = 90;    // GoTo refused above this altitude (deg, OnStep :So, 60..90)
  // PEC may start playing only before guiding begins for the target (no guide pulse since
  // boot or the last GoTo); false: after 60 s without pulses (a guiding pause is enough)
  bool pecStrict = true;
  bool refraction = true;        // refraction-compensated RA tracking (OnStep :Tr / :Tn)
  double guideRate = 0.5;        // pulse-guide rate, x sidereal, both axes (0.1..0.9)
  double decAxisMin = -180;      // DEC mechanical limits, deg of the DEC axis model (-180..180 = none)
  double decAxisMax = 180;
  bool gpsEnabled = true;        // NMEA GPS on UART1 (RX 17, TX 18, 9600 baud)
  bool gpsSetsSite = true;       // a 3D fix updates lat/lon
};

extern Settings settings;

void settingsLoad();
void settingsSave();
