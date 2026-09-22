#pragma once
#include <stdint.h>
#include <esp_err.h>

// Minimal PL2303 (HX / HXD, not HXN) setup, following the Linux pl2303.c driver.
// ctrl() must perform a control transfer on the device's default pipe; for IN
// requests (bmRequestType & 0x80) it fills data with wLength bytes.
typedef esp_err_t (*pl2303_ctrl_fn)(uint8_t bmRequestType, uint8_t bRequest, uint16_t wValue,
                                    uint16_t wIndex, uint8_t *data, uint16_t wLength);

esp_err_t pl2303Init(pl2303_ctrl_fn ctrl, uint32_t baud);
