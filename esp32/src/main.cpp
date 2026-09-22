// Star Adventurer ESP32-S3 controller - phase 1: USB-host bridge + OTA + network log
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <Update.h>
#include <WebServer.h>
#include <WiFi.h>
#include <esp_ota_ops.h>
#include <esp_task_wdt.h>

#include "clock.h"
#include "dashboard.h"
#include "lx200_server.h"
#include "mount_usb.h"
#include "netlog.h"
#include "ra_axis.h"
#include "settings.h"
#include "synscan_udp.h"

#define HOSTNAME "starmount"
#define AP_SSID "StarMount"
#define AP_PASS "starmount"
#define BOOT_BUTTON 0
#define RECOVERY_WINDOW_MS 10000

#ifndef WIFI_SSID
#define WIFI_SSID ""
#endif
#ifndef WIFI_PASS
#define WIFI_PASS ""
#endif

static Preferences prefs;
static WebServer web(80);
static bool usbHostActive = false;

// Keep the new image in "pending verify" until Wi-Fi + OTA are up; if it
// crashes before that, the bootloader rolls back to the previous image.
extern "C" bool verifyRollbackLater() { return true; }

// ---------------------------------------------------------------- Wi-Fi

static void wifiBegin() {
  prefs.begin("wifi", true);
  String ssid = prefs.getString("ssid", WIFI_SSID);
  String pass = prefs.getString("pass", WIFI_PASS);
  prefs.end();

  WiFi.setHostname(HOSTNAME);
  WiFi.setAutoReconnect(true);
  if (ssid.length()) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid.c_str(), pass.c_str());
    logf("wifi: connecting to %s", ssid.c_str());
    for (int i = 0; i < 40 && WiFi.status() != WL_CONNECTED; i++) delay(500);
  }
  if (WiFi.status() == WL_CONNECTED) {
    WiFi.setSleep(false);  // keeps UDP/LX200 latency low
    logf("wifi: connected, IP %s", WiFi.localIP().toString().c_str());
  } else {
    // AP stays up alongside STA, which keeps retrying in the background
    WiFi.mode(ssid.length() ? WIFI_AP_STA : WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    logf("wifi: access point '%s' (pass '%s'), IP %s", AP_SSID, AP_PASS, WiFi.softAPIP().toString().c_str());
  }
}

// ---------------------------------------------------------------- web UI

static const char PAGE_HEAD[] PROGMEM =
    "<!doctype html><html><head><meta charset=utf-8><meta name=viewport content='width=device-width,initial-scale=1'>"
    "<title>StarMount</title><style>body{font-family:sans-serif;max-width:760px;margin:20px auto;padding:0 16px;"
    "background:#111;color:#eee}a{color:#8cf}pre{background:#000;padding:8px;overflow:auto;max-height:60vh}"
    "input,button{padding:6px;margin:4px 0}td{padding:2px 12px 2px 0}</style></head><body><h1>StarMount</h1>"
    "<p><a href=/>mount</a> | <a href=/sys>system</a> | <a href=/log>log</a> | <a href=/cmd>command</a> | <a href=/wifi>wifi</a> | "
    "<a href=/update>firmware</a></p>";

static String jsonStatus() {
  MountStats s = mountStats();
  const esp_partition_t *part = esp_ota_get_running_partition();
  char buf[512];
  snprintf(buf, sizeof(buf),
           "{\"uptime_s\":%lu,\"heap\":%lu,\"psram\":%lu,\"rssi\":%d,\"ip\":\"%s\",\"partition\":\"%s\","
           "\"usb_host\":%s,\"mount_connected\":%s,\"mount\":\"%s\",\"commands\":%lu,\"timeouts\":%lu,"
           "\"connects\":%lu}",
           millis() / 1000, (unsigned long)ESP.getFreeHeap(), (unsigned long)ESP.getFreePsram(), WiFi.RSSI(),
           WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString().c_str() : WiFi.softAPIP().toString().c_str(),
           part ? part->label : "?", usbHostActive ? "true" : "false", mountConnected() ? "true" : "false",
           mountInfo().c_str(), (unsigned long)s.commands, (unsigned long)s.timeouts, (unsigned long)s.connects);
  return buf;
}

static void handleRoot() {
  String html = FPSTR(PAGE_HEAD);
  html += F("<table id=t></table><p>");
  html += usbHostActive ? F("<a href='/usbhost?enable=0'>Disable USB host on next boot</a>")
                        : F("<a href='/usbhost?enable=1'>Enable USB host on next boot</a>");
  html += F(" | <a href=/reboot>reboot</a></p><script>async function r(){const s=await(await fetch('/api/status'))"
            ".json();t.innerHTML=Object.entries(s).map(([k,v])=>`<tr><td>${k}</td><td>${v}</td></tr>`).join('')}"
            "r();setInterval(r,1000)</script></body></html>");
  web.send(200, "text/html", html);
}

static void handleCmd() {
  if (!web.hasArg("c")) {
    String html = FPSTR(PAGE_HEAD);
    html += F("<p>Raw SkyWatcher motor command, e.g. <code>:e1</code> (version), <code>:j1</code> (RA position), "
              "<code>:f1</code> (status). The trailing \\r is added for you.</p><form><input name=c value=':e1'> "
              "<button>send</button></form><pre id=o></pre><script>document.forms[0].onsubmit=async e=>{e.preventDefault();"
              "o.textContent=await(await fetch('/cmd?c='+encodeURIComponent(c.value))).text()}</script></body></html>");
    web.send(200, "text/html", html);
    return;
  }
  String c = web.arg("c");
  if (!c.endsWith("\r")) c += "\r";
  char resp[64];
  uint32_t t0 = micros();
  int n = mountCmd(c.c_str(), c.length(), resp, sizeof(resp));
  uint32_t dt = micros() - t0;
  if (n < 0) {
    web.send(504, "text/plain", "no reply from mount\n");
    return;
  }
  resp[strcspn(resp, "\r")] = 0;
  web.send(200, "text/plain", String(resp) + "   (" + String(dt / 1000.0, 1) + " ms)\n");
}

static void handleWifi() {
  if (web.method() == HTTP_POST) {
    prefs.begin("wifi", false);
    prefs.putString("ssid", web.arg("ssid"));
    prefs.putString("pass", web.arg("pass"));
    prefs.end();
    web.send(200, "text/plain", "saved, rebooting\n");
    delay(500);
    ESP.restart();
  }
  String html = FPSTR(PAGE_HEAD);
  html += F("<form method=post><p>SSID<br><input name=ssid></p><p>Password<br><input name=pass type=password></p>"
            "<button>save &amp; reboot</button></form></body></html>");
  web.send(200, "text/html", html);
}

static void handleUpdatePage() {
  String html = FPSTR(PAGE_HEAD);
  html += F("<p>Upload <code>.pio/build/usb/firmware.bin</code></p><form method=post enctype=multipart/form-data>"
            "<input type=file name=fw accept=.bin> <button>flash</button></form></body></html>");
  web.send(200, "text/html", html);
}

static void handleUpdateUpload() {
  HTTPUpload &up = web.upload();
  if (up.status == UPLOAD_FILE_START) {
    logf("ota(http): receiving %s", up.filename.c_str());
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) logf("ota(http): %s", Update.errorString());
  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (Update.write(up.buf, up.currentSize) != up.currentSize) logf("ota(http): %s", Update.errorString());
  } else if (up.status == UPLOAD_FILE_END) {
    if (Update.end(true)) logf("ota(http): %u bytes written", up.totalSize);
    else logf("ota(http): %s", Update.errorString());
  }
}

static void handleUpdateDone() {
  bool ok = !Update.hasError();
  web.send(ok ? 200 : 500, "text/plain", ok ? "OK, rebooting\n" : String("failed: ") + Update.errorString() + "\n");
  if (ok) {
    delay(500);
    ESP.restart();
  }
}

static void webBegin() {
  web.on("/sys", handleRoot);
  dashboardBegin(web);
  web.on("/api/status", [] { web.send(200, "application/json", jsonStatus()); });
  web.on("/log", [] { web.send(200, "text/plain; charset=utf-8", netlogHistory()); });
  web.on("/cmd", handleCmd);
  web.on("/wifi", handleWifi);
  web.on("/update", HTTP_GET, handleUpdatePage);
  web.on("/update", HTTP_POST, handleUpdateDone, handleUpdateUpload);
  web.on("/usbhost", [] {
    bool enable = web.arg("enable") == "1";
    prefs.begin("sys", false);
    prefs.putBool("usbhost", enable);
    prefs.end();
    web.send(200, "text/plain", String("USB host ") + (enable ? "enabled" : "disabled") + " from next boot\n");
  });
  web.on("/reboot", [] {
    web.send(200, "text/plain", "rebooting\n");
    delay(500);
    ESP.restart();
  });
  web.begin();
  MDNS.addService("http", "tcp", 80);
}

// ---------------------------------------------------------------- OTA

static void otaBegin() {
  ArduinoOTA.setHostname(HOSTNAME);
#ifdef OTA_PASS
  ArduinoOTA.setPassword(OTA_PASS);
#endif
  ArduinoOTA.onStart([] { logf("ota: start"); });
  ArduinoOTA.onEnd([] { logf("ota: done, rebooting"); });
  ArduinoOTA.onError([](ota_error_t e) { logf("ota: error %u", e); });
  ArduinoOTA.begin();  // also starts mDNS as starmount.local
}

// ---------------------------------------------------------------- main

void setup() {
  Serial.begin(115200);
  pinMode(BOOT_BUTTON, INPUT_PULLUP);
  netlogBegin();
  const esp_partition_t *part = esp_ota_get_running_partition();
  logf("StarMount booting from %s, built " __DATE__ " " __TIME__, part ? part->label : "?");

  settingsLoad();
  wifiBegin();
  clockBegin();
  otaBegin();
  webBegin();
  synscanUdpBegin();  // raw motor access for debugging; don't use while the RA task is driving
  ra::begin();
  lx200::begin();

  // A hang (e.g. starved idle tasks) reboots instead of silently killing Wi-Fi
  esp_task_wdt_config_t wdt = {.timeout_ms = 15000, .idle_core_mask = 0b11, .trigger_panic = true};
  if (esp_task_wdt_reconfigure(&wdt) != ESP_OK) esp_task_wdt_init(&wdt);

  // Recovery window: OTA always gets a chance before the USB host starts, and
  // holding BOOT skips the USB host for this boot.
  prefs.begin("sys", false);
  bool usbHostEnabled = prefs.getBool("usbhost", true);
  prefs.end();
  bool bootHeld = false;
  for (uint32_t t0 = millis(); millis() - t0 < RECOVERY_WINDOW_MS;) {
    if (digitalRead(BOOT_BUTTON) == LOW) bootHeld = true;
    ArduinoOTA.handle();
    web.handleClient();
    netlogLoop();
    delay(10);
  }
  if (!usbHostEnabled || bootHeld) {
    logf("usb: host NOT started (%s)", bootHeld ? "BOOT held" : "disabled in settings");
  } else {
    logf("usb: starting host on the OTG port");
    mountUsbBegin();
    usbHostActive = true;
  }

  esp_ota_mark_app_valid_cancel_rollback();
  logf("ready: http://%s.local  lx200 :%u  telnet log :23  synscan udp :11880", HOSTNAME, settings.lx200Port);
}

void loop() {
  ArduinoOTA.handle();
  web.handleClient();
  netlogLoop();
  delay(2);
}
