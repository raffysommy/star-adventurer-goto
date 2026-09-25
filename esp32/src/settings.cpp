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
  p.end();
}
