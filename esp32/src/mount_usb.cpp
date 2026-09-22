#include "mount_usb.h"
#include "netlog.h"
#include "pl2303.h"
#include <usb/usb_host.h>
#include <freertos/stream_buffer.h>

#define PL2303_VID 0x067b
#define PL2303_PID 0x2303
#define MOUNT_BAUD 115200
#define USB_CORE 0

static usb_host_client_handle_t client;
static usb_device_handle_t dev;
static uint8_t epIn, epOut;
static uint16_t epInMps = 64;
static volatile bool connected = false;
static volatile bool inFlight = false;
static uint16_t devVid, devPid;

static TaskHandle_t clientTaskHandle, workerTaskHandle;
static volatile uint8_t newDevAddr = 0;
static volatile bool devGone = false;

static usb_transfer_t *ctrlXfer, *inXfer, *outXfer;
static SemaphoreHandle_t ctrlDone, outDone, cmdLock;
static StreamBufferHandle_t rx;
static MountStats stats;

// ---------------------------------------------------------------- USB plumbing

static void clientEventCb(const usb_host_client_event_msg_t *msg, void *) {
  if (msg->event == USB_HOST_CLIENT_EVENT_NEW_DEV) {
    newDevAddr = msg->new_dev.address;
  } else if (msg->event == USB_HOST_CLIENT_EVENT_DEV_GONE) {
    devGone = true;
  }
  xTaskNotifyGive(workerTaskHandle);
}

// The prebuilt IDF 5.5 has CONFIG_USB_HOST_ENABLE_ENUM_FILTER_CALLBACK set, and
// with a NULL callback enumeration silently stalls: always install one.
static bool enumFilterCb(const usb_device_desc_t *desc, uint8_t *bConfigurationValue) {
  logf("usb: enumerating %04x:%04x (class %02x, %d config(s))", desc->idVendor, desc->idProduct,
       desc->bDeviceClass, desc->bNumConfigurations);
  return true;
}

static void usbLibTask(void *) {
  usb_host_config_t cfg = {};
  cfg.skip_phy_setup = false;
  cfg.intr_flags = ESP_INTR_FLAG_LEVEL1;
  cfg.enum_filter_cb = enumFilterCb;
  logf("usb: installing host driver");
  esp_err_t err = usb_host_install(&cfg);
  if (err != ESP_OK) {
    logf("usb: host install failed: %s", esp_err_to_name(err));
    vTaskDelete(nullptr);
  }
  xTaskNotifyGive(clientTaskHandle);
  while (true) {
    uint32_t flags;
    // NO_CLIENTS also fires right after install, before our client registers;
    // freeing devices there would orphan a mount that is already plugged in.
    usb_host_lib_handle_events(portMAX_DELAY, &flags);
  }
}

// Transfer callbacks run in the client task (inside usb_host_client_handle_events)
static void clientTask(void *) {
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // wait for usb_host_install
  usb_host_client_config_t cc = {};
  cc.is_synchronous = false;
  cc.max_num_event_msg = 5;
  cc.async.client_event_callback = clientEventCb;
  ESP_ERROR_CHECK(usb_host_client_register(&cc, &client));
  logf("usb: host ready, waiting for the mount");
  uint32_t lastScan = 0;
  while (true) {
    usb_host_client_handle_events(client, pdMS_TO_TICKS(1000));
    // Safety net: a device enumerated before we registered (or whose NEW_DEV
    // event was missed) is picked up from the device list instead.
    if (!dev && !newDevAddr && millis() - lastScan > 5000) {
      lastScan = millis();
      uint8_t addrs[8];
      int n = 0;
      if (usb_host_device_addr_list_fill(sizeof(addrs), addrs, &n) == ESP_OK && n > 0) {
        logf("usb: found %d device(s) without NEW_DEV event, opening address %d", n, addrs[0]);
        newDevAddr = addrs[0];
        xTaskNotifyGive(workerTaskHandle);
      }
    }
  }
}

static void semDoneCb(usb_transfer_t *t) { xSemaphoreGive((SemaphoreHandle_t)t->context); }

static void inDoneCb(usb_transfer_t *t) {
  if (t->status == USB_TRANSFER_STATUS_COMPLETED && t->actual_num_bytes > 0) {
    xStreamBufferSend(rx, t->data_buffer, t->actual_num_bytes, 0);
  }
  if (connected && !devGone &&
      (t->status == USB_TRANSFER_STATUS_COMPLETED || t->status == USB_TRANSFER_STATUS_TIMED_OUT)) {
    if (usb_host_transfer_submit(t) == ESP_OK) return;
  }
  inFlight = false;
}

static esp_err_t ctrlTransfer(uint8_t bmRequestType, uint8_t bRequest, uint16_t wValue, uint16_t wIndex,
                              uint8_t *data, uint16_t wLength) {
  usb_setup_packet_t *setup = (usb_setup_packet_t *)ctrlXfer->data_buffer;
  setup->bmRequestType = bmRequestType;
  setup->bRequest = bRequest;
  setup->wValue = wValue;
  setup->wIndex = wIndex;
  setup->wLength = wLength;
  bool in = bmRequestType & 0x80;
  if (!in && wLength) memcpy(ctrlXfer->data_buffer + sizeof(usb_setup_packet_t), data, wLength);
  ctrlXfer->num_bytes = sizeof(usb_setup_packet_t) + wLength;
  ctrlXfer->device_handle = dev;
  ctrlXfer->bEndpointAddress = 0;
  ctrlXfer->callback = semDoneCb;
  ctrlXfer->context = ctrlDone;
  ctrlXfer->timeout_ms = 1000;
  esp_err_t err = usb_host_transfer_submit_control(client, ctrlXfer);
  if (err != ESP_OK) return err;
  if (!xSemaphoreTake(ctrlDone, pdMS_TO_TICKS(1500))) return ESP_ERR_TIMEOUT;
  if (ctrlXfer->status != USB_TRANSFER_STATUS_COMPLETED) return ESP_FAIL;
  if (in && wLength) memcpy(data, ctrlXfer->data_buffer + sizeof(usb_setup_packet_t), wLength);
  return ESP_OK;
}

static bool findBulkEndpoints() {
  const usb_config_desc_t *cfg;
  if (usb_host_get_active_config_descriptor(dev, &cfg) != ESP_OK) return false;
  int offset = 0;
  const usb_intf_desc_t *intf = usb_parse_interface_descriptor(cfg, 0, 0, &offset);
  if (!intf) return false;
  epIn = epOut = 0;
  for (int i = 0; i < intf->bNumEndpoints; i++) {
    int epOffset = offset;
    const usb_ep_desc_t *ep = usb_parse_endpoint_descriptor_by_index(intf, i, cfg->wTotalLength, &epOffset);
    if (!ep || (ep->bmAttributes & USB_BM_ATTRIBUTES_XFERTYPE_MASK) != USB_BM_ATTRIBUTES_XFER_BULK) continue;
    if (ep->bEndpointAddress & 0x80) {
      epIn = ep->bEndpointAddress;
      epInMps = ep->wMaxPacketSize;
    } else {
      epOut = ep->bEndpointAddress;
    }
  }
  return epIn && epOut;
}

static void closeDevice() {
  connected = false;
  // Pending IN transfer completes with NO_DEVICE / CANCELED once the device is gone
  if (inFlight) {
    usb_host_endpoint_halt(dev, epIn);
    usb_host_endpoint_flush(dev, epIn);
  }
  for (int i = 0; i < 100 && inFlight; i++) vTaskDelay(pdMS_TO_TICKS(10));
  usb_host_interface_release(client, dev, 0);
  usb_host_device_close(client, dev);
  dev = nullptr;
}

static void openDevice(uint8_t addr) {
  if (dev) return;  // already open (NEW_DEV and the periodic scan can both report it)
  if (usb_host_device_open(client, addr, &dev) != ESP_OK) {
    logf("usb: cannot open device %d", addr);
    dev = nullptr;
    return;
  }
  const usb_device_desc_t *desc;
  usb_host_get_device_descriptor(dev, &desc);
  devVid = desc->idVendor;
  devPid = desc->idProduct;
  logf("usb: device %04x:%04x bcdDevice %04x connected", devVid, devPid, desc->bcdDevice);
  if (devVid != PL2303_VID || devPid != PL2303_PID) {
    logf("usb: not a PL2303, ignoring");
    usb_host_device_close(client, dev);
    dev = nullptr;
    return;
  }
  if (!findBulkEndpoints()) {
    logf("usb: PL2303 bulk endpoints not found");
    usb_host_device_close(client, dev);
    dev = nullptr;
    return;
  }
  esp_err_t err = usb_host_interface_claim(client, dev, 0, 0);
  if (err != ESP_OK) {
    logf("usb: claim failed: %s", esp_err_to_name(err));
    usb_host_device_close(client, dev);
    dev = nullptr;
    return;
  }
  err = pl2303Init(ctrlTransfer, MOUNT_BAUD);
  if (err != ESP_OK) {
    usb_host_interface_release(client, dev, 0);
    usb_host_device_close(client, dev);
    dev = nullptr;
    return;
  }

  xStreamBufferReset(rx);
  inXfer->device_handle = dev;
  inXfer->bEndpointAddress = epIn;
  inXfer->num_bytes = epInMps;
  inXfer->callback = inDoneCb;
  inXfer->timeout_ms = 0;
  connected = true;
  inFlight = true;
  if (usb_host_transfer_submit(inXfer) != ESP_OK) {
    logf("usb: cannot start bulk IN");
    inFlight = false;
    closeDevice();
    return;
  }
  stats.connects++;
  logf("usb: PL2303 ready (IN 0x%02x, OUT 0x%02x, %lu baud)", epIn, epOut, (unsigned long)MOUNT_BAUD);
}

// Opens/closes devices outside the client task, since control transfers
// block waiting for callbacks that the client task has to deliver.
static void workerTask(void *) {
  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (devGone) {
      devGone = false;
      if (dev) {
        xSemaphoreTake(cmdLock, portMAX_DELAY);
        closeDevice();
        xSemaphoreGive(cmdLock);
        logf("usb: mount disconnected");
      }
    }
    if (newDevAddr) {
      uint8_t addr = newDevAddr;
      newDevAddr = 0;
      logf("usb: opening device at address %d", addr);
      xSemaphoreTake(cmdLock, portMAX_DELAY);
      openDevice(addr);
      xSemaphoreGive(cmdLock);
    }
  }
}

// ---------------------------------------------------------------- public API

void mountUsbBegin() {
  ctrlDone = xSemaphoreCreateBinary();
  outDone = xSemaphoreCreateBinary();
  cmdLock = xSemaphoreCreateMutex();
  rx = xStreamBufferCreate(1024, 1);
  usb_host_transfer_alloc(64 + sizeof(usb_setup_packet_t), 0, &ctrlXfer);
  usb_host_transfer_alloc(512, 0, &inXfer);
  usb_host_transfer_alloc(64, 0, &outXfer);

  xTaskCreatePinnedToCore(workerTask, "usb_worker", 4096, nullptr, 5, &workerTaskHandle, USB_CORE);
  xTaskCreatePinnedToCore(clientTask, "usb_client", 4096, nullptr, 6, &clientTaskHandle, USB_CORE);
  xTaskCreatePinnedToCore(usbLibTask, "usb_lib", 4096, nullptr, 7, nullptr, USB_CORE);
}

bool mountConnected() { return connected; }

String mountInfo() {
  if (!connected) return "not connected";
  char buf[64];
  snprintf(buf, sizeof(buf), "PL2303 %04x:%04x @ %d baud", devVid, devPid, MOUNT_BAUD);
  return buf;
}

MountStats mountStats() { return stats; }

static bool writeAll(const char *data, size_t len) {
  while (len) {
    size_t n = len > 64 ? 64 : len;
    memcpy(outXfer->data_buffer, data, n);
    outXfer->num_bytes = n;
    outXfer->device_handle = dev;
    outXfer->bEndpointAddress = epOut;
    outXfer->callback = semDoneCb;
    outXfer->context = outDone;
    outXfer->timeout_ms = 500;
    if (usb_host_transfer_submit(outXfer) != ESP_OK) return false;
    if (!xSemaphoreTake(outDone, pdMS_TO_TICKS(600))) return false;
    if (outXfer->status != USB_TRANSFER_STATUS_COMPLETED) return false;
    data += n;
    len -= n;
  }
  return true;
}

int mountCmd(const char *cmd, size_t cmdLen, char *resp, size_t respMax, uint32_t timeoutMs) {
  xSemaphoreTake(cmdLock, portMAX_DELAY);
  int result = -1;
  if (connected && respMax > 1) {
    stats.commands++;
    uint8_t junk[64];
    while (xStreamBufferReceive(rx, junk, sizeof(junk), 0) > 0) {
    }
    if (writeAll(cmd, cmdLen)) {
      // Read until a '\r'-terminated line that contains '=' or '!'. A line
      // without them is a command echo and gets discarded (as pysynscan does).
      size_t n = 0;
      TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeoutMs);
      while (true) {
        TickType_t now = xTaskGetTickCount();
        if ((int32_t)(deadline - now) <= 0) break;
        char c;
        if (xStreamBufferReceive(rx, &c, 1, deadline - now) != 1) break;
        if (n < respMax - 1) resp[n++] = c;
        if (c != '\r') continue;
        size_t start = 0;
        while (start < n && resp[start] != '=' && resp[start] != '!') start++;
        if (start < n) {
          memmove(resp, resp + start, n - start);
          result = n - start;
          resp[result] = 0;
          break;
        }
        n = 0;
      }
    }
    if (result < 0) stats.timeouts++;
  }
  xSemaphoreGive(cmdLock);
  if (result < 0) resp[0] = 0;
  return result;
}
