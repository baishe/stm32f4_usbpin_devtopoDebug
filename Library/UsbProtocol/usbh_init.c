#include "main.h"
#include "usbh_core.h"
#include "usbh_winusb.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "usb_log.h"
#include <string.h>

#define HOST_DEMO_TASK_STACK_SIZE  512

/* -------------------------------------------------------------------------- */
/* 信号量与设备名缓冲区，用于连接通知                                       */
/* -------------------------------------------------------------------------- */
static SemaphoreHandle_t g_winusb_sem = NULL;
static char g_winusb_devname[CONFIG_USBHOST_DEV_NAMELEN];
static volatile uint32_t g_winusb_disconnect_seq = 0;

static void HostDemoTask(void *pvParameters);

/* -------------------------------------------------------------------------- */
/* 产品字符串验证关键词（请与实际 Device 端保持一致）                        */
/* -------------------------------------------------------------------------- */
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
void usbh_winusb_notify_connect(const char *devname)
{
    if (g_winusb_sem != NULL) {
        strncpy(g_winusb_devname, devname, CONFIG_USBHOST_DEV_NAMELEN - 1);
        g_winusb_devname[CONFIG_USBHOST_DEV_NAMELEN - 1] = '\0';
        xSemaphoreGive(g_winusb_sem);
    }
}

void usbh_winusb_notify_disconnect(const char *devname)
{
    (void)devname;
    g_winusb_disconnect_seq++;
}

/* -------------------------------------------------------------------------- */
/* USB 主机初始化                                                             */
/* -------------------------------------------------------------------------- */
void dummy_event_handler(uint8_t busid, uint8_t hub_index, uint8_t hub_port, uint8_t intf, uint8_t event);
void usbh_init(void)
{
    g_winusb_sem = xSemaphoreCreateBinary();
    if (g_winusb_sem == NULL) {
        Error_Handler();
    }

    usbh_initialize(0, USB_OTG_HS_PERIPH_BASE, dummy_event_handler);

    if (xTaskCreate(HostDemoTask,
                    "HostDemo",
                    HOST_DEMO_TASK_STACK_SIZE,
                    NULL,
                    tskIDLE_PRIORITY + 2,
                    NULL) != pdPASS) {
        Error_Handler();
    }
}

void dummy_event_handler(uint8_t busid, uint8_t hub_index, uint8_t hub_port, uint8_t intf, uint8_t event)
{
    switch(event) {
        case USBH_EVENT_INTERFACE_UNSUPPORTED:
            CONFIG_USB_PRINTF("hub_index: %d, hub_port: %d, intf: %d, event type: %d\n", hub_index, hub_port, intf, event);
            break;
        case USBH_EVENT_INTERFACE_START:
            CONFIG_USB_PRINTF("hub_index: %d, hub_port: %d, intf: %d, event type: %d\n", hub_index, hub_port, intf, event);
            break;
        case USBH_EVENT_INTERFACE_STOP:
            break;
    }
}

/* -------------------------------------------------------------------------- */
/* HostDemoTask: 等待连接信号量，打开对应设备，阻塞等待 Device 发送数据     */
/* -------------------------------------------------------------------------- */
struct usbh_winusb *winusb = NULL;
static void HostDemoTask(void *pvParameters)
{
    const char *devname;
    uint8_t rx_buf[64];
    uint32_t disconnect_seq;
    int ret;

    for (;;) {
        /* 阻塞等待连接通知 */
        if (xSemaphoreTake(g_winusb_sem, portMAX_DELAY) == pdTRUE) {
            devname = g_winusb_devname;

            disconnect_seq = g_winusb_disconnect_seq;
            winusb = usbh_winusb_open(devname, 0);
            if (!winusb) {
                USB_LOG_ERR("Failed to open %s\r\n", devname);
                continue;
            }
            USB_LOG_INFO("WinUSB device %s opened\r\n", devname);

            /* 阻塞读循环：等待 Device 端通过串口触发写入 */
            while (g_winusb_disconnect_seq == disconnect_seq) {
                ret = usbh_winusb_read(winusb, rx_buf, sizeof(rx_buf));
                if (ret > 0) {
                    USB_LOG_INFO("Received %d bytes\r\n", ret);
                    usb_hexdump(rx_buf, ret);
                } else {
                    /* 检查设备是否拔出 */
                    if (g_winusb_disconnect_seq != disconnect_seq) {
                        USB_LOG_WRN("Device %s disconnected\r\n", devname);
                        winusb = NULL;
                        break;
                    }
                    /* 其他错误延时后重试读 */
                    vTaskDelay(200);
                }
            }
        }
    }
}
