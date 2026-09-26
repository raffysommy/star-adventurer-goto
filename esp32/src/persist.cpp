#include "persist.h"

#include <esp_attr.h>
#include <esp_system.h>

#include "netlog.h"

namespace persist {

static const uint32_t MAGIC = 0x53544D00, VERSION = 1;  // "STM" + version

RTC_NOINIT_ATTR static State state;
State &s = state;
static bool wasKept = false;

bool kept() { return wasKept; }

void touch() { state.magic = MAGIC | VERSION; }

void begin() {
  wasKept = esp_reset_reason() != ESP_RST_POWERON && state.magic == (MAGIC | VERSION);
  if (!wasKept) {
    state = State{};
    state.decMotor = state.decGear = 146400;  // DEC home, 0 deg
    touch();
  }
  logf("persist: %s (DEC %ld, %s, RA shift %.3f deg, position %s)", wasKept ? "kept across reset" : "fresh",
       (long)state.decGear, state.flipped ? "flipped" : "normal", state.raShiftDeg,
       state.trusted ? "trusted" : "not synced");
}

}  // namespace persist
