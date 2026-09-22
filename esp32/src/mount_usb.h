#pragma once
#include <Arduino.h>

// Star Adventurer link over the ESP32-S3 USB OTG port (host mode).
// The mount exposes a Prolific PL2303 HXD USB-serial bridge (067b:2303),
// talking the SkyWatcher motor-controller protocol at 115200 8N1.

// Installs the USB host stack. From here on the native USB PHY belongs to the
// OTG controller: the USB-Serial-JTAG console disappears, use the network log.
void mountUsbBegin();

bool mountConnected();
String mountInfo();

// Sends a raw command (e.g. ":e1\r") and copies the reply (e.g. "=038207\r"),
// with any command echo stripped, into resp (NUL-terminated).
// Returns the reply length, or -1 on timeout / no mount. Thread-safe: commands
// from any task are serialized, so request/response pairs never interleave.
int mountCmd(const char *cmd, size_t cmdLen, char *resp, size_t respMax, uint32_t timeoutMs = 500);

struct MountStats {
  uint32_t commands;
  uint32_t timeouts;
  uint32_t connects;
};
MountStats mountStats();
