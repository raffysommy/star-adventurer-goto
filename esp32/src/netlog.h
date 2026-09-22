#pragma once
#include <Arduino.h>

// Log sink that works without the USB console: everything goes to a RAM
// history buffer that survives soft resets (served at /log), to telnet clients
// on port 23 and to Serial (UART0).
void netlogBegin();
void netlogLoop();                       // call from loop() after Wi-Fi is up: serves telnet clients
void logf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
String netlogHistory();                  // last NETLOG_HISTORY bytes
