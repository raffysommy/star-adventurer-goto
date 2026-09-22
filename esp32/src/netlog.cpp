#include "netlog.h"
#include <WiFi.h>
#include <esp_attr.h>
#include <esp_log.h>
#include <stdarg.h>

#define NETLOG_HISTORY 16384
#define NETLOG_PORT 23
#define NETLOG_MAX_CLIENTS 3

#define NETLOG_MAGIC 0x5374724c

// Circular buffer indexed by a monotonically increasing write position, so every
// telnet client can keep its own read position and catch up independently.
// Kept in .noinit RAM: after a panic/watchdog reset the previous boot's log is
// still there, which is the only post-mortem we get with the USB port busy.
__NOINIT_ATTR static char ring[NETLOG_HISTORY];
__NOINIT_ATTR static uint32_t head;
__NOINIT_ATTR static uint32_t magic;
static SemaphoreHandle_t ringLock;

static WiFiServer server(NETLOG_PORT);
static WiFiClient clients[NETLOG_MAX_CLIENTS];
static uint32_t clientPos[NETLOG_MAX_CLIENTS];

static void ringWrite(const char *s, size_t n) {
  xSemaphoreTake(ringLock, portMAX_DELAY);
  for (size_t i = 0; i < n; i++) ring[(head + i) % NETLOG_HISTORY] = s[i];
  head += n;
  xSemaphoreGive(ringLock);
}

static void vlogf(const char *fmt, va_list ap) {
  char buf[512];
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  if (n <= 0) return;
  if (n >= (int)sizeof(buf)) n = sizeof(buf) - 1;
  ringWrite(buf, n);
  Serial.write((const uint8_t *)buf, n);
}

void logf(const char *fmt, ...) {
  char line[480];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(line, sizeof(line), fmt, ap);
  va_end(ap);
  char stamped[512];
  int n = snprintf(stamped, sizeof(stamped), "[%9.3f] %s\r\n", millis() / 1000.0, line);
  if (n <= 0) return;
  if (n >= (int)sizeof(stamped)) n = sizeof(stamped) - 1;
  ringWrite(stamped, n);
  Serial.write((const uint8_t *)stamped, n);
}

// ESP-IDF component logs (USB host errors etc.) land here too.
static int idfVprintf(const char *fmt, va_list ap) {
  vlogf(fmt, ap);
  return 0;
}

void netlogBegin() {
  ringLock = xSemaphoreCreateMutex();
  bool kept = magic == NETLOG_MAGIC && esp_reset_reason() != ESP_RST_POWERON;
  if (!kept) {
    head = 0;
    magic = NETLOG_MAGIC;
  }
  logf("======== boot, reset reason %d%s", esp_reset_reason(), kept ? " (previous log above)" : "");
  esp_log_set_vprintf(idfVprintf);
}

String netlogHistory() {
  xSemaphoreTake(ringLock, portMAX_DELAY);
  uint32_t start = head > NETLOG_HISTORY ? head - NETLOG_HISTORY : 0;
  String out;
  out.reserve(head - start);
  for (uint32_t p = start; p < head; p++) out += ring[p % NETLOG_HISTORY];
  xSemaphoreGive(ringLock);
  return out;
}

void netlogLoop() {
  static bool started = false;
  if (!started) {  // lwIP only exists once Wi-Fi has been brought up
    server.begin();
    server.setNoDelay(true);
    started = true;
  }
  if (server.hasClient()) {
    WiFiClient c = server.accept();
    int slot = -1;
    for (int i = 0; i < NETLOG_MAX_CLIENTS; i++)
      if (!clients[i] || !clients[i].connected()) { slot = i; break; }
    if (slot < 0) {
      c.println("too many log clients");
      c.stop();
    } else {
      clients[slot] = c;
      // New clients get the recent history first
      clientPos[slot] = head > 4096 ? head - 4096 : 0;
    }
  }

  for (int i = 0; i < NETLOG_MAX_CLIENTS; i++) {
    if (!clients[i] || !clients[i].connected()) continue;
    while (clients[i].available()) clients[i].read();  // ignore input
    char chunk[512];
    size_t n = 0;
    xSemaphoreTake(ringLock, portMAX_DELAY);
    if (head - clientPos[i] > NETLOG_HISTORY) clientPos[i] = head - NETLOG_HISTORY;  // fell behind
    while (clientPos[i] + n < head && n < sizeof(chunk)) {
      chunk[n] = ring[(clientPos[i] + n) % NETLOG_HISTORY];
      n++;
    }
    xSemaphoreGive(ringLock);
    if (n) {
      size_t w = clients[i].write((const uint8_t *)chunk, n);
      clientPos[i] += w;
    }
  }
}
