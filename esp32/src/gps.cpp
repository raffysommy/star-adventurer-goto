#include "gps.h"

#include <WiFi.h>
#include <nmea.h>

#include "clock.h"
#include "netlog.h"
#include "settings.h"

namespace gps {

static const int RX_PIN = 17, TX_PIN = 18;
static const uint32_t BAUD = 9600;
static const uint16_t NMEA_PORT = 10110;
static const int MAX_CLIENTS = 2;
static const uint32_t TIME_SYNC_EVERY_MS = 60000;
static const uint32_t SITE_SAVE_EVERY_MS = 600000;

static SemaphoreHandle_t lock;
static nmea::Fix fix;
static uint32_t sentences, badChecksums, lastValidMs, lastTimeSyncMs, lastSiteSaveMs;
static bool everSynced = false;
static WiFiServer *server;
static WiFiClient clients[MAX_CLIENTS];

static void relay(const char *line) {
  for (auto &c : clients)
    if (c.connected()) {
      c.print(line);
      c.print("\r\n");
    }
}

static void handle(const char *line) {
  if (line[0] != '$') return;
  xSemaphoreTake(lock, portMAX_DELAY);
  bool known = nmea::parse(line, fix);
  bool valid = nmea::checksumOk(line);
  if (valid) {
    sentences++;
    lastValidMs = millis();
  } else {
    badChecksums++;
  }
  nmea::Fix f = fix;
  xSemaphoreGive(lock);
  if (!valid) return;
  relay(line);
  if (!known) return;

  uint32_t now = millis();
  // Time: RMC time is the start of the second in which the sentence is sent (~0.1-0.5 s
  // late at 9600 baud); a sync is applied only if the clock is off by more than 0.5 s
  if (f.timeValid && (!everSynced || now - lastTimeSyncMs > TIME_SYNC_EVERY_MS)) {
    if (clockSet(f.unixUtc + f.centis / 100.0 + 0.2, "gps") || !strcmp(clockSource(), "gps")) {
      if (!everSynced) logf("gps: clock set from GPS");
      everSynced = true;
      lastTimeSyncMs = now;
    }
  }
  // Site: only a good 3D fix
  if (settings.gpsSetsSite && f.posValid && f.sats >= 4 && f.hdop < 5) {
    bool moved = fabs(f.lat - settings.lat) > 0.001 || fabs(f.lon - settings.lonEast) > 0.001;
    if (moved) {
      bool first = fabs(f.lat - settings.lat) > 0.01 || fabs(f.lon - settings.lonEast) > 0.01;
      settings.lat = f.lat;
      settings.lonEast = f.lon;
      if (first || now - lastSiteSaveMs > SITE_SAVE_EVERY_MS) {
        settingsSave();
        lastSiteSaveMs = now;
        logf("gps: site %.5f %.5f (%d sats, hdop %.1f)", f.lat, f.lon, f.sats, f.hdop);
      }
    }
  }
}

void inject(const char *line) { handle(line); }

static void task(void *) {
  char line[100];
  int len = 0;
  server = new WiFiServer(NMEA_PORT);
  server->begin();
  logf("gps: %s, NMEA relay on TCP %u", settings.gpsEnabled ? "UART1 RX 17 / TX 18 at 9600" : "disabled",
       NMEA_PORT);
  if (settings.gpsEnabled) Serial1.begin(BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
  while (true) {
    if (server->hasClient()) {
      WiFiClient c = server->accept();
      bool placed = false;
      for (auto &slot : clients)
        if (!slot.connected()) {
          slot = c;
          placed = true;
          logf("gps: NMEA client %s connected", c.remoteIP().toString().c_str());
          break;
        }
      if (!placed) c.stop();
    }
    while (settings.gpsEnabled && Serial1.available()) {
      char ch = Serial1.read();
      if (ch == '\n' || ch == '\r') {
        if (len) {
          line[len] = 0;
          handle(line);
          len = 0;
        }
      } else if (len < (int)sizeof(line) - 1) {
        if (ch == '$') len = 0;  // resync on a sentence start
        line[len++] = ch;
      } else {
        len = 0;  // overlong: garbage
      }
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void begin() {
  lock = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(task, "gps", 4096, nullptr, 2, nullptr, 0);
}

State state() {
  State s;
  xSemaphoreTake(lock, portMAX_DELAY);
  s.enabled = settings.gpsEnabled;
  s.receiving = lastValidMs && millis() - lastValidMs < 5000;
  s.timeValid = fix.timeValid;
  s.posValid = fix.posValid;
  s.lat = fix.lat;
  s.lon = fix.lon;
  s.altM = fix.altM;
  s.hdop = fix.hdop;
  s.sats = fix.sats;
  s.sentences = sentences;
  s.badChecksums = badChecksums;
  s.lastTimeSyncAgeS = everSynced ? (int32_t)((millis() - lastTimeSyncMs) / 1000) : -1;
  xSemaphoreGive(lock);
  return s;
}

bool timeSynced() {
  return everSynced && !strcmp(clockSource(), "gps") && millis() - lastTimeSyncMs < 600000;
}

}  // namespace gps
