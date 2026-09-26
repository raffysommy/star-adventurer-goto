#include "bootlog.h"

#include <Preferences.h>
#include <esp_system.h>

#include "clock.h"
#include "netlog.h"

namespace bootlog {

static const int N = 10;
static const uint32_t UPTIME_SAVE_MS = 5 * 60 * 1000;

struct Boot {
  uint32_t no;        // boot counter
  uint32_t unix;      // clock at boot (0 until the clock is set)
  uint32_t prevUpS;   // how long the previous boot ran (last saved uptime, <= 5 min short)
  uint8_t reason;     // esp_reset_reason_t
};
struct UsbEv {
  uint32_t boot;
  uint32_t upS;       // uptime of that boot
  uint32_t unix;      // clock (0 if not set)
  uint8_t connected;
};

static Boot boots[N];
static UsbEv usb[N];
static uint32_t bootNo;
static uint8_t bootIdx, usbIdx;
static uint32_t lastSaveMs;
static bool unixDone = false;
static volatile bool usbPending = false;  // written from loop(): the USB task has little stack
static SemaphoreHandle_t lock;

static const char *reasonName(uint8_t r) {
  switch (r) {
    case ESP_RST_POWERON: return "power-on";
    case ESP_RST_EXT: return "external pin";
    case ESP_RST_SW: return "software (OTA/reboot)";
    case ESP_RST_PANIC: return "panic";
    case ESP_RST_INT_WDT: return "interrupt watchdog";
    case ESP_RST_TASK_WDT: return "task watchdog";
    case ESP_RST_WDT: return "watchdog";
    case ESP_RST_DEEPSLEEP: return "deep sleep";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_SDIO: return "sdio";
    case ESP_RST_USB: return "usb";
    case ESP_RST_JTAG: return "jtag";
    default: return "unknown";
  }
}

static void save(const char *what) {
  Preferences p;
  p.begin("bootlog", false);
  if (!strcmp(what, "boots")) {
    p.putUInt("n", bootNo);
    p.putUChar("bi", bootIdx);
    p.putBytes("boots", boots, sizeof(boots));
  } else if (!strcmp(what, "usb")) {
    p.putUChar("ui", usbIdx);
    p.putBytes("usb", usb, sizeof(usb));
  } else {
    p.putUInt("up", millis() / 1000);
  }
  p.end();
}

void begin() {
  lock = xSemaphoreCreateMutex();
  Preferences p;
  uint32_t prevUp = 0;
  if (p.begin("bootlog", true)) {
    bootNo = p.getUInt("n", 0);
    bootIdx = p.getUChar("bi", 0);
    usbIdx = p.getUChar("ui", 0);
    prevUp = p.getUInt("up", 0);
    p.getBytes("boots", boots, sizeof(boots));
    p.getBytes("usb", usb, sizeof(usb));
    p.end();
  }
  bootNo++;
  bootIdx = (bootIdx + 1) % N;
  boots[bootIdx] = {bootNo, 0, prevUp, (uint8_t)esp_reset_reason()};
  save("boots");
  save("up");  // uptime 0: a crash before the first save reads as "ran < 5 min"
  logf("bootlog: boot #%lu, reset reason %s, previous boot ran >= %lu s", (unsigned long)bootNo,
       reasonName(boots[bootIdx].reason), (unsigned long)prevUp);
}

void tick() {
  uint32_t now = millis();
  if (usbPending) {
    usbPending = false;
    save("usb");
  }
  if (now - lastSaveMs >= UPTIME_SAVE_MS) {
    lastSaveMs = now;
    save("up");
  }
  if (!unixDone && clockValid()) {
    unixDone = true;
    xSemaphoreTake(lock, portMAX_DELAY);
    boots[bootIdx].unix = (uint32_t)(clockNow() - now / 1000.0);
    for (auto &e : usb)
      if (e.boot == bootNo && !e.unix) e.unix = boots[bootIdx].unix + e.upS;
    xSemaphoreGive(lock);
    save("boots");
    save("usb");
  }
}

void beforeRestart() {
  if (usbPending) save("usb");
  save("up");
}

void usbEvent(bool connected) {
  xSemaphoreTake(lock, portMAX_DELAY);
  usbIdx = (usbIdx + 1) % N;
  usb[usbIdx] = {bootNo, millis() / 1000, clockValid() ? (uint32_t)clockNow() : 0, (uint8_t)connected};
  xSemaphoreGive(lock);
  usbPending = true;
}

static String iso(uint32_t unix) {
  if (!unix) return "null";
  time_t t = unix;
  struct tm tm;
  gmtime_r(&t, &tm);
  char b[32];
  strftime(b, sizeof(b), "\"%Y-%m-%dT%H:%M:%SZ\"", &tm);
  return b;
}

String json() {
  xSemaphoreTake(lock, portMAX_DELAY);
  String s = "{\"boots\":[";
  bool first = true;
  for (int k = 0; k < N; k++) {  // newest first
    const Boot &b = boots[(bootIdx - k + N) % N];
    if (!b.no) continue;
    s += String(first ? "" : ",") + "{\"no\":" + b.no + ",\"reason\":\"" + reasonName(b.reason) +
         "\",\"at\":" + iso(b.unix) + ",\"previous_ran_s\":" + b.prevUpS + "}";
    first = false;
  }
  s += "],\"mount_usb\":[";
  first = true;
  for (int k = 0; k < N; k++) {
    const UsbEv &e = usb[(usbIdx - k + N) % N];
    if (!e.boot) continue;
    s += String(first ? "" : ",") + "{\"boot\":" + e.boot + ",\"event\":\"" + (e.connected ? "connected" : "disconnected") +
         "\",\"uptime_s\":" + e.upS + ",\"at\":" + iso(e.unix) + "}";
    first = false;
  }
  xSemaphoreGive(lock);
  return s + "]}";
}

}  // namespace bootlog
