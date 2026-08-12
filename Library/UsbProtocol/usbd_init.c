/*
 * Copyright (c) 2024, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "usbd_core.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

#define WINUSB_VENDOR_CODE 0x17

#define WINUSB_NUM 1

// note that if device is composite device, you should use USB_MSOSV2_COMP_ID_FUNCTION_WINUSB_MULTI_DESCRIPTOR_INIT
const uint8_t WINUSB_WCIDDescriptor[] = {
    USB_MSOSV2_COMP_ID_SET_HEADER_DESCRIPTOR_INIT(10 + USB_MSOSV2_COMP_ID_FUNCTION_WINUSB_SINGLE_DESCRIPTOR_LEN),
    USB_MSOSV2_COMP_ID_FUNCTION_WINUSB_SINGLE_DESCRIPTOR_INIT(),
};

const uint8_t USBD_BinaryObjectStoreDescriptor[] = {
    USB_BOS_HEADER_DESCRIPTOR_INIT(5 + USB_BOS_CAP_PLATFORM_WINUSB_DESCRIPTOR_LEN, 1),
    USB_BOS_CAP_PLATFORM_WINUSB_DESCRIPTOR_INIT(WINUSB_VENDOR_CODE, sizeof(WINUSB_WCIDDescriptor)),
};

const struct usb_msosv2_descriptor msosv2_desc = {
    .vendor_code = WINUSB_VENDOR_CODE,
    .compat_id = WINUSB_WCIDDescriptor,
    .compat_id_len = sizeof(WINUSB_WCIDDescriptor),
};

const struct usb_bos_descriptor bos_desc = {
    .string = USBD_BinaryObjectStoreDescriptor,
    .string_len = sizeof(USBD_BinaryObjectStoreDescriptor),
};

#define WINUSB_IN_EP  0x81
#define WINUSB_OUT_EP 0x02

#define USBD_VID           0xFFFE
#define USBD_PID           0xffff
#define USBD_MAX_POWER     100
#define USBD_LANGID_STRING 1033

#define USB_CONFIG_SIZE (9 + 9 + 7 + 7)
#define INTF_NUM        1


#ifdef CONFIG_USB_HS
#define WINUSB_EP_MPS 512
#else
#define WINUSB_EP_MPS 64
#endif

static const uint8_t device_descriptor[] = {
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_1, 0x00, 0x00, 0x00, USBD_VID, USBD_PID, 0x0001, 0x01)
};

static const uint8_t config_descriptor[] = {
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, INTF_NUM, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    USB_INTERFACE_DESCRIPTOR_INIT(0x00, 0x00, 0x02, 0xff, 0xff, 0x00, 0x04),
    USB_ENDPOINT_DESCRIPTOR_INIT(WINUSB_IN_EP, 0x02, WINUSB_EP_MPS, 0x00),
    USB_ENDPOINT_DESCRIPTOR_INIT(WINUSB_OUT_EP, 0x02, WINUSB_EP_MPS, 0x00),
};

static const uint8_t device_quality_descriptor[] = {
    ///////////////////////////////////////
    /// device qualifier descriptor
    ///////////////////////////////////////
    0x0a,
    USB_DESCRIPTOR_TYPE_DEVICE_QUALIFIER,
    0x00,
    0x02,
    0x00,
    0x00,
    0x00,
    0x40,
    0x00,
    0x00,
};

static const char *string_descriptors[] = {
    (const char[]){ 0x09, 0x04 }, /* Langid */
    "CherryUSB",                  /* Manufacturer */
    "CherryUSB WINUSB DEMO",      /* Product */
    "2022123456",                 /* Serial Number */
    "CherryUSB WINUSB DEMO 1",    /* STRING4 */
    "CherryUSB WINUSB DEMO 2",    /* STRING5 */
};

static const uint8_t *device_descriptor_callback(uint8_t speed)
{
    return device_descriptor;
}

static const uint8_t *config_descriptor_callback(uint8_t speed)
{
    return config_descriptor;
}

static const uint8_t *device_quality_descriptor_callback(uint8_t speed)
{
    return device_quality_descriptor;
}

static const char *string_descriptor_callback(uint8_t speed, uint8_t index)
{
    if (index >= (sizeof(string_descriptors) / sizeof(char *))) {
        return NULL;
    }
    return string_descriptors[index];
}

const struct usb_descriptor winusbv2_descriptor = {
    .device_descriptor_callback = device_descriptor_callback,
    .config_descriptor_callback = config_descriptor_callback,
    .device_quality_descriptor_callback = device_quality_descriptor_callback,
    .string_descriptor_callback = string_descriptor_callback,
    .msosv2_descriptor = &msosv2_desc,
    .bos_descriptor = &bos_desc,
};

USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t read_buffer[2048];
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t write_buffer[2048];

volatile bool ep_tx_busy_flag = false;
static volatile bool usbd_configured_flag = false;
static uint32_t usbd_send_count = 0;

#define USBD_PERIODIC_TASK_STACK_SIZE 512
#define USBD_PERIODIC_SEND_INTERVAL_MS 1500

static void usbd_event_handler(uint8_t busid, uint8_t event)
{
    
    switch (event) {
        case USBD_EVENT_RESET:
            USB_LOG_RAW("dev[%d]:Reset\r\n", busid);
            usbd_configured_flag = false;
            break;
        case USBD_EVENT_DISCONNECTED:
            usbd_configured_flag = false;
            break;
        case USBD_EVENT_SUSPEND:        // 应该是设备接入或插拔
            ep_tx_busy_flag = false;
            break;
        case USBD_EVENT_CONFIGURED:
            USB_LOG_RAW("dev[%d]:Connected\r\n", busid);    // 枚举成功
            usbd_configured_flag = true;
            ep_tx_busy_flag = false;
            /* setup first out ep read transfer */
            usbd_ep_start_read(busid, WINUSB_OUT_EP, read_buffer, 2048);
            break;

        default:
            USB_LOG_RAW("dev[%d]:event %d\r\n",busid, event);
            break;
    }
}

void usbd_winusb_out(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    USB_LOG_RAW("dev[%d]: rev len %d\r\n", busid, nbytes);

    if (!ep_tx_busy_flag) {
        ep_tx_busy_flag = true;
        if (usbd_ep_start_write(busid, WINUSB_IN_EP, read_buffer, nbytes) < 0) {
            ep_tx_busy_flag = false;
        }
    }
    
    /* setup next out ep read transfer */
    usbd_ep_start_read(busid, WINUSB_OUT_EP, read_buffer, 2048);
}

void usbd_winusb_in(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    USB_LOG_RAW("dev[%d]: send done len %d\r\n", busid, nbytes);

    if ((nbytes % WINUSB_EP_MPS) == 0 && nbytes) {
        /* send zlp */
        usbd_ep_start_write(busid, WINUSB_IN_EP, NULL, 0);
    } else {
        ep_tx_busy_flag = false;
    }
}

struct usbd_endpoint winusb_out_ep1 = {
    .ep_addr = WINUSB_OUT_EP,
    .ep_cb = usbd_winusb_out
};

struct usbd_endpoint winusb_in_ep1 = {
    .ep_addr = WINUSB_IN_EP,
    .ep_cb = usbd_winusb_in
};


//
#include "main.h"

#define BUSID 0

static void usbd_periodic_send_task(void *pvParameters)
{
    int len;
    int ret;

    (void)pvParameters;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(USBD_PERIODIC_SEND_INTERVAL_MS));
        if (!usbd_configured_flag || ep_tx_busy_flag) {
            continue;
        }

        len = snprintf((char *)write_buffer, sizeof(write_buffer), "cnt %lu\r",
                       (unsigned long)usbd_send_count);
        if (len < 0 || len >= (int)sizeof(write_buffer)) {
            continue;
        }

        ep_tx_busy_flag = true;
        ret = usbd_ep_start_write(BUSID, WINUSB_IN_EP, write_buffer, (uint32_t)len);
        if (ret == 0) {
            usbd_send_count++;
        } else {
            ep_tx_busy_flag = false;
        }
    }
}

struct usbd_interface intf0;

void usbd_init(void )
{
    
    uint8_t busid = BUSID;
    uintptr_t reg_base = USB_OTG_FS_PERIPH_BASE;
    
    usbd_desc_register(busid, &winusbv2_descriptor);

    usbd_add_interface(busid, &intf0);
    usbd_add_endpoint(busid, &winusb_out_ep1);
    usbd_add_endpoint(busid, &winusb_in_ep1);

    usbd_initialize(busid, reg_base, usbd_event_handler);

    if (xTaskCreate(usbd_periodic_send_task,
                    "UsbdPeriodic",
                    USBD_PERIODIC_TASK_STACK_SIZE,
                    NULL,
                    tskIDLE_PRIORITY + 2,
                    NULL) != pdPASS) {
        Error_Handler();
    }
}

