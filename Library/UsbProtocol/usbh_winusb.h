/*
 * Copyright (c) 2025, Your Name
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef USBH_WINUSB_H
#define USBH_WINUSB_H

#include "usbh_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief WinUSB instance structure
 */
struct usbh_winusb {
    struct usbh_hubport *hport;
    uint8_t intf;               /* Control interface number */
    uint8_t data_intf;          /* Data interface number (for endpoints) */
    uint8_t minor;              /* Device index (e.g., /dev/winusbX) */

    struct usb_endpoint_descriptor *bulkin;   /* Bulk IN endpoint */
    struct usb_endpoint_descriptor *bulkout;  /* Bulk OUT endpoint */

    struct usbh_urb bulkout_urb;

    uint8_t *iobuffer;          /* I/O buffer for transfers */
    uint8_t ref_count;
    uint32_t open_flags;

    void *user_data;
};

/**
 * @brief Open a WinUSB device by name (e.g., "/dev/winusb0")
 *
 * @param devname Device name
 * @param flags   Open flags (reserved)
 * @return struct usbh_winusb* Handle or NULL on failure
 */
struct usbh_winusb *usbh_winusb_open(const char *devname, uint32_t flags);

/**
 * @brief Close a WinUSB device
 *
 * @param winusb WinUSB instance
 * @return 0 on success, negative error code on failure
 */
int usbh_winusb_close(struct usbh_winusb *winusb);

/**
 * @brief Write data to the device (Bulk OUT)
 *
 * @param winusb WinUSB instance
 * @param buffer Data buffer
 * @param len    Data length in bytes
 * @return Number of bytes written, or negative error code
 */
int usbh_winusb_write(struct usbh_winusb *winusb, const uint8_t *buffer, uint32_t len);

/**
 * @brief Send a vendor-specific control request
 *
 * @param winusb    WinUSB instance
 * @param bmRequestType Request type (USB_DIR_IN/OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE/INTERFACE)
 * @param bRequest  Request code
 * @param wValue    Value field
 * @param wIndex    Index field
 * @param buffer    Data buffer (can be NULL for no data stage)
 * @param wLength   Data length
 * @return Number of bytes transferred, or negative error code
 */
int usbh_winusb_control_transfer(struct usbh_winusb *winusb,
                                 uint8_t bmRequestType, uint8_t bRequest,
                                 uint16_t wValue, uint16_t wIndex,
                                 uint8_t *buffer, uint16_t wLength);

#ifdef __cplusplus
}
#endif

#endif /* USBH_WINUSB_H */
