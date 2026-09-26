#pragma once
#include <Arduino.h>

// Boot and power history in NVS, to diagnose power problems after the fact (a power-on
// wipes the RAM log): the last 10 boots with their reset reason and how long the previous
// boot ran, and the last 10 mount USB connects/disconnects.
//   brownout     -> the 5 V rail sagged (power bank, current peak)
//   power-on     -> the supply cut out completely
//   usb event without a boot -> only the mount lost power
// Flash wear: one record per boot or USB event, plus the uptime every 5 minutes.
namespace bootlog {

void begin();                  // record this boot
void tick();                   // from loop(): uptime save, boot time once the clock is set
void beforeRestart();          // planned restart (OTA, reboot): save the exact uptime
void usbEvent(bool connected); // mount USB connect / disconnect (RAM only; tick() saves)
String json();                 // /api/boots

}  // namespace bootlog
