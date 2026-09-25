#include "settings.h"

#include <Preferences.h>

Settings settings;

void settingsLoad() {
  Preferences p;
  if (!p.begin("mount", true)) return;  // nothing saved yet: keep defaults
  Settings d;
  settings.lat = p.getDouble("lat", d.lat);
  settings.lonEast = p.getDouble("lon", d.lonEast);
  settings.utcOffset = p.getDouble("utc_off", d.utcOffset);
  settings.decAxisReversed = p.getBool("dec_rev", d.decAxisReversed);
  settings.flipRaGuiding = p.getBool("flip_ra", d.flipRaGuiding);
  settings.pierEast = p.getBool("pier_east", d.pierEast);
  settings.lx200Port = p.getUShort("lx_port", d.lx200Port);
  settings.decBacklash = p.getInt("dec_bl", d.decBacklash);
  settings.raEastLimit = p.getDouble("ra_east", d.raEastLimit);
  settings.raWestMinutes = p.getInt("ra_west_min", d.raWestMinutes);
  settings.horizonLimit = p.getInt("hor_lim", d.horizonLimit);
  settings.overheadLimit = p.getInt("ovh_lim", d.overheadLimit);
  settings.gpsEnabled = p.getBool("gps_en", d.gpsEnabled);
  settings.refraction = p.getBool("refr", d.refraction);
  settings.pecStrict = p.getBool("pec_strict", d.pecStrict);
  settings.guideRate = p.getDouble("guide_rate", d.guideRate);
  settings.decAxisMin = p.getDouble("dec_min", d.decAxisMin);
  settings.decAxisMax = p.getDouble("dec_max", d.decAxisMax);
  settings.gpsSetsSite = p.getBool("gps_site", d.gpsSetsSite);
  p.end();
}

void settingsSave() {
  Preferences p;
  p.begin("mount", false);
  p.putDouble("lat", settings.lat);
  p.putDouble("lon", settings.lonEast);
  p.putDouble("utc_off", settings.utcOffset);
  p.putBool("dec_rev", settings.decAxisReversed);
  p.putBool("flip_ra", settings.flipRaGuiding);
  p.putBool("pier_east", settings.pierEast);
  p.putUShort("lx_port", settings.lx200Port);
  p.putInt("dec_bl", settings.decBacklash);
  p.putDouble("ra_east", settings.raEastLimit);
  p.putInt("ra_west_min", settings.raWestMinutes);
  p.putInt("hor_lim", settings.horizonLimit);
  p.putInt("ovh_lim", settings.overheadLimit);
  p.putBool("gps_en", settings.gpsEnabled);
  p.putBool("refr", settings.refraction);
  p.putBool("pec_strict", settings.pecStrict);
  p.putDouble("guide_rate", settings.guideRate);
  p.putDouble("dec_min", settings.decAxisMin);
  p.putDouble("dec_max", settings.decAxisMax);
  p.putBool("gps_site", settings.gpsSetsSite);
  p.end();
}
