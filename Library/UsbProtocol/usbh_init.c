#include "main.h"
#include "usbh_core.h"
#include "usbh_winusb.h"
#include "usbh_poll.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usb_log.h"
#include <string.h>

#define WINUSB_PRODUCT_STRING_KEYWORD  "WINUSB DEMO"



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
    (void)busid; (void)hub_index; (void)hub_port; (void)intf; (void)event;
}

static void UsbEventTask(void *arg)
{
    struct usbh_poll_event ev;
    (void)arg;
    for (;;) {
        if (usbh_poll_event_recv(&ev, portMAX_DELAY) == 0) {
            usbh_poll_on_event(&ev);
            if (ev.type == USBH_POLL_EVENT_DATA) {
                USB_LOG_INFO("[poll] slot=%u path=%u-%u-%u len=%u data=%02x %02x %02x %02x %02x %02x %02x %02x\r\n",
                    ev.slot, ev.path[0], ev.path[1], ev.path[2], ev.len,
                    ev.len > 0 ? ev.data[0] : 0, ev.len > 1 ? ev.data[1] : 0,
                    ev.len > 2 ? ev.data[2] : 0, ev.len > 3 ? ev.data[3] : 0,
                    ev.len > 4 ? ev.data[4] : 0, ev.len > 5 ? ev.data[5] : 0,
                    ev.len > 6 ? ev.data[6] : 0, ev.len > 7 ? ev.data[7] : 0);
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
    usbh_initialize(0, USB_OTG_HS_PERIPH_BASE, dummy_event_handler);
    usbh_poll_init();
    if (xTaskCreate(UsbEventTask, "UsbEvent", 512, NULL, tskIDLE_PRIORITY + 3, NULL) != pdPASS ||
        xTaskCreate(UsbStatsTask, "UsbStats", 384, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
        Error_Handler();
    }
}
