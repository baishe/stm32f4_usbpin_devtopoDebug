#include "main.h"
#include "usbh_core.h"
#include "usbh_winusb.h"
#include "usbh_poll.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usb_log.h"
#include <string.h>
#include <stdio.h>

#define WINUSB_PRODUCT_STRING_KEYWORD  "WINUSB DEMO"

#if USBH_POLL_FULL_HEXDUMP
static void poll_hexdump(const uint8_t *data, uint16_t len)
{
    char line[49];
    uint16_t off;
    for (off = 0; off < len; off += 16) {
        uint16_t n = len - off > 16 ? 16 : len - off;
        uint16_t i;
        for (i = 0; i < n; i++) {
            snprintf(&line[i * 3], sizeof(line) - i * 3, "%02x ", data[off + i]);
        }
        line[n * 3] = 0;
        USB_LOG_INFO("[poll] dump+%u %s\r\n", off, line);
    }
}
#endif



/* -------------------------------------------------------------------------- */
/* 覆盖驱动弱函数：通过产品字符串描述符进行设备身份验证                      */
/* -------------------------------------------------------------------------- */
int usbh_winusb_check(struct usbh_hubport *hport)
{
    struct usb_setup_packet setup;
    uint8_t buf[128];
    int ret;

    /* 请求产品字符串描述符，索引为 2 */
    setup.bmRequestType = 0x80;            /* 设备到主机，标准请求 */
    setup.bRequest      = USB_REQUEST_GET_DESCRIPTOR;
    setup.wValue        = (USB_DESCRIPTOR_TYPE_STRING << 8) | 2;  /* 字符串索引 2 */
    setup.wIndex        = 0x0409;          /* 语言 ID (US English) */
    setup.wLength       = sizeof(buf);

    ret = usbh_control_transfer(hport, &setup, buf);
    if (ret < 0) {
        USB_LOG_ERR("WinUSB check: Failed to get product string descriptor\r\n");
        return -1;
    }

    /* 字符串描述符的第 0 字节是长度，第 1 字节是类型，后续是 UTF-16LE 编码 */
    if (ret < 2 || buf[1] != USB_DESCRIPTOR_TYPE_STRING) {
        USB_LOG_ERR("WinUSB check: Invalid string descriptor\r\n");
        return -1;
    }

    /* 将 UTF-16LE 转为 ASCII 以便比较（简化：取低字节） */
    uint16_t str_len = buf[0];
    if (str_len > ret) {
        str_len = ret;
    }
    uint8_t ascii_str[128] = {0};
    for (int i = 2; i < str_len; i += 2) {
        ascii_str[(i - 2) / 2] = buf[i];
    }

    USB_LOG_INFO("WinUSB check: Product string = \"%s\"\r\n", ascii_str);

    /* 检查是否包含关键词 */
    if (strstr((const char *)ascii_str, WINUSB_PRODUCT_STRING_KEYWORD) == NULL) {
        USB_LOG_WRN("WinUSB check: Product string mismatch (expected \"%s\")\r\n",
                     WINUSB_PRODUCT_STRING_KEYWORD);
        return -1;
    }

    return 0;
}

/* -------------------------------------------------------------------------- */
/* 覆盖驱动弱函数：接收连接通知，释放信号量给 HostDemoTask                 */
/* -------------------------------------------------------------------------- */
void dummy_event_handler(uint8_t busid, uint8_t hub_index, uint8_t hub_port, uint8_t intf, uint8_t event)
{
    switch (event) {
    case USBH_EVENT_INTERFACE_UNSUPPORTED:
    case USBH_EVENT_INTERFACE_START:
        CONFIG_USB_PRINTF("hub_index: %d, hub_port: %d, intf: %d, event type: %d\n",
                          hub_index, hub_port, intf, event);
        break;
    case USBH_EVENT_INTERFACE_STOP:
        break;
    default:
        break;
    }
}

static void UsbEventTask(void *arg)
{
    struct usbh_poll_event ev;
    char hex[24];
    uint8_t i;
    (void)arg;
    for (;;) {
        if (usbh_poll_event_recv(&ev, portMAX_DELAY) == 0) {
            usbh_poll_on_event(&ev);
            if (ev.type == USBH_POLL_EVENT_DATA) {
                hex[0] = 0;
                for (i = 0; i < ev.len && i < 8; i++) {
                    snprintf(&hex[i * 3], sizeof(hex) - i * 3, "%02x ", ev.data[i]);
                }
                USB_LOG_INFO("[poll] slot=%u path=%u-%u-%u len=%u data=%s\r\n",
                             ev.slot, ev.path[0], ev.path[1], ev.path[2], ev.len, hex);
#if USBH_POLL_FULL_HEXDUMP
                poll_hexdump(ev.data, ev.len);
#endif
                usbh_poll_buf_release(ev.data);
            } else if (ev.type == USBH_POLL_EVENT_ERROR) {
                USB_LOG_ERR("[poll] slot=%u error=%d\r\n", ev.slot, ev.errorcode);
            } else {
                USB_LOG_INFO("[poll] slot=%u event=%u path=%u-%u-%u\r\n", ev.slot, ev.type, ev.path[0], ev.path[1], ev.path[2]);
            }
        }
    }
}

static void UsbStatsTask(void *arg)
{
    (void)arg;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        usbh_poll_stats_dump();
    }
}

void usbh_init(void)
{
    usbh_poll_init();
    usbh_initialize(0, USB_OTG_HS_PERIPH_BASE, dummy_event_handler);
    if (xTaskCreate(UsbEventTask, "UsbEvent", 320, NULL, tskIDLE_PRIORITY + 3, NULL) != pdPASS ||
        xTaskCreate(UsbStatsTask, "UsbStats", 256, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
        Error_Handler();
    }
}
