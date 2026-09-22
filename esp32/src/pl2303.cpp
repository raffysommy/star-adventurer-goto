#include "pl2303.h"
#include "netlog.h"

#define VENDOR_WRITE_TYPE 0x40
#define VENDOR_READ_TYPE 0xC0
#define VENDOR_REQUEST 0x01
#define CLASS_OUT_TYPE 0x21
#define SET_LINE_REQUEST 0x20
#define SET_CONTROL_REQUEST 0x22
#define CONTROL_DTR 0x01
#define CONTROL_RTS 0x02

static pl2303_ctrl_fn ctrl;

static esp_err_t vendorRead(uint16_t value) {
  uint8_t buf[1];
  return ctrl(VENDOR_READ_TYPE, VENDOR_REQUEST, value, 0, buf, 1);
}

static esp_err_t vendorWrite(uint16_t value, uint16_t index) {
  return ctrl(VENDOR_WRITE_TYPE, VENDOR_REQUEST, value, index, nullptr, 0);
}

esp_err_t pl2303Init(pl2303_ctrl_fn fn, uint32_t baud) {
  ctrl = fn;
  esp_err_t err = ESP_OK;
#define TRY(x)                                          \
  if ((err = (x)) != ESP_OK) {                          \
    logf("pl2303: %s failed: %s", #x, esp_err_to_name(err)); \
    return err;                                         \
  }

  // pl2303_startup(): the magic "vendor dance" for non-HXN chips
  TRY(vendorRead(0x8484));
  TRY(vendorWrite(0x0404, 0));
  TRY(vendorRead(0x8484));
  TRY(vendorRead(0x8383));
  TRY(vendorRead(0x8484));
  TRY(vendorWrite(0x0404, 1));
  TRY(vendorRead(0x8484));
  TRY(vendorRead(0x8383));
  TRY(vendorWrite(0, 1));
  TRY(vendorWrite(1, 0));
  TRY(vendorWrite(2, 0x44));  // HX type

  // pl2303_open(): reset upstream/downstream data pipes
  TRY(vendorWrite(8, 0));
  TRY(vendorWrite(9, 0));

  // SET_LINE_CODING: baud (LE32), 1 stop bit, no parity, 8 data bits
  uint8_t line[7] = {(uint8_t)baud, (uint8_t)(baud >> 8), (uint8_t)(baud >> 16), (uint8_t)(baud >> 24), 0, 0, 8};
  TRY(ctrl(CLASS_OUT_TYPE, SET_LINE_REQUEST, 0, 0, line, sizeof(line)));

  // Raise DTR/RTS like a normal tty open (flow control stays off from startup)
  TRY(ctrl(CLASS_OUT_TYPE, SET_CONTROL_REQUEST, CONTROL_DTR | CONTROL_RTS, 0, nullptr, 0));
#undef TRY
  return ESP_OK;
}
