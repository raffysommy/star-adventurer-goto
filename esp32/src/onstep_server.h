#pragma once
#include <Arduino.h>

// OnStepX-compatible command server (a superset of the Meade LX200 set), so INDI/Ekos
// and ASIAIR (OnStep drivers), NINA (ASCOM OnStep) and plain LX200 clients (Stellarium,
// SkySafari, INDI "LX200 Basic/GPS") all work. What the mount doesn't have (focuser,
// rotator, weather, park position, other track rates) answers OnStepX's "not supported".
// The command subset and reply formats: docs/onstep-protocol.md.
namespace onstep {

void begin();                       // TCP servers on settings.lx200Port (5001) and 9999
String process(const String &cmd); // one command ":...#" (also used by the web UI)
int clients();

}  // namespace onstep
