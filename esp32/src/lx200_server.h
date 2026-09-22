#pragma once
#include <Arduino.h>

// LX200 TCP server: port of lx200.py's LX200Proxy.process_command(), with the
// same replies so existing INDI/Stellarium setups behave identically.
// DEC is simulated for now (no DEC motor on the ESP yet): :GD reports the last
// :Sd target once a slew or sync "reaches" it.
namespace lx200 {

void begin();
String process(const String &cmd);  // one command, ":...#" (also used by tests/web)

struct State {
  bool meridianFlipped;
  double raTarget;  // axis RA target after the meridian-flip adjustment
  double decCurrent, decTarget;
  bool guideNorth, guideSouth;
  int clients;
};
State state();
String reportedRa();   // "HH:MM:SS" as :GR would answer
String reportedDec();  // "sDD*MM:SS" as :GD would answer

}  // namespace lx200
