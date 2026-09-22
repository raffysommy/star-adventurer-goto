#include "synscan_udp.h"
#include "mount_usb.h"
#include "netlog.h"
#include <WiFi.h>
#include <WiFiUdp.h>

#define SYNSCAN_PORT 11880

static WiFiUDP udp;

static void udpTask(void *) {
  udp.begin(SYNSCAN_PORT);
  logf("synscan: UDP bridge on port %d", SYNSCAN_PORT);
  char req[64], resp[64];
  while (true) {
    int len = udp.parsePacket();
    if (len <= 0) {
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }
    len = udp.read((uint8_t *)req, sizeof(req) - 1);
    if (len <= 0) continue;
    req[len] = 0;
    // No reply when the mount is missing: clients see a timeout, like a real dongle
    int n = mountCmd(req, len, resp, sizeof(resp));
    if (n > 0) {
      udp.beginPacket(udp.remoteIP(), udp.remotePort());
      udp.write((const uint8_t *)resp, n);
      udp.endPacket();
    } else {
      req[strcspn(req, "\r")] = 0;
      logf("synscan: no reply to %s", req);
    }
  }
}

void synscanUdpBegin() { xTaskCreatePinnedToCore(udpTask, "synscan_udp", 4096, nullptr, 4, nullptr, 1); }
