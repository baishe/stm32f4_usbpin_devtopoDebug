#include "usbh_core.h"
#include "usbh_winusb.h"
#include "usbh_poll.h"

#undef USB_DBG_TAG
#define USB_DBG_TAG "usbh_winusb"
#include "usb_log.h"

#define DEV_FORMAT_WINUSB  "/dev/winusb%d"

#ifndef CONFIG_USBHOST_MAX_WINUSB_CLASS
#define CONFIG_USBHOST_MAX_WINUSB_CLASS 2
#endif
#ifndef CONFIG_USBHOST_WINUSB_BUFFER_SIZE
#define CONFIG_USBHOST_WINUSB_BUFFER_SIZE 512
#endif

static struct usbh_winusb g_winusb_class[CONFIG_USBHOST_MAX_WINUSB_CLASS];
static uint32_t g_devinuse = 0;

USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t g_winusb_iobuffer[CONFIG_USBHOST_MAX_WINUSB_CLASS][USB_ALIGN_UP(CONFIG_USBHOST_WINUSB_BUFFER_SIZE, CONFIG_USB_ALIGN_SIZE)];

static int usbh_winusb_connect(struct usbh_hubport *hport, uint8_t intf);
static int usbh_winusb_disconnect(struct usbh_hubport *hport, uint8_t intf);

/*
 * 弱函数：应用层覆盖，用于检查设备是否符合 WinUSB 定制要求
 * 返回 0 表示检查通过，非 0 表示拒绝该设备
 */
__attribute__((weak)) int usbh_winusb_check(struct usbh_hubport *hport)
{
    (void)hport;
    return 0;   /* 默认允许所有 */
}

/*
 * 弱函数：应用层覆盖，用于在设备连接并检查通过后通知应用任务
 * @param devname 设备在系统中注册的名称
 */
__attribute__((weak)) void usbh_winusb_notify_connect(const char *devname)
{
    (void)devname;
    /* 默认空实现 */
}

__attribute__((weak)) void usbh_winusb_notify_disconnect(const char *devname)
{
    (void)devname;
}

static struct usbh_winusb *usbh_winusb_alloc(void)
{
    uint8_t devno;
    for (devno = 0; devno < CONFIG_USBHOST_MAX_WINUSB_CLASS; devno++) {
        if ((g_devinuse & (1U << devno)) == 0) {
            g_devinuse |= (1U << devno);
            memset(&g_winusb_class[devno], 0, sizeof(struct usbh_winusb));
            g_winusb_class[devno].minor = devno;
            g_winusb_class[devno].iobuffer = g_winusb_iobuffer[devno];
            return &g_winusb_class[devno];
        }
    }
    return NULL;
}

static void usbh_winusb_free(struct usbh_winusb *winusb)
{
    uint8_t devno = winusb->minor;
    if (devno < 32) {
        g_devinuse &= ~(1U << devno);
    }
}

static int usbh_winusb_attach(struct usbh_hubport *hport, uint8_t intf,
                              struct usb_endpoint_descriptor **bulkin,
                              struct usb_endpoint_descriptor **bulkout,
                              uint8_t *data_intf)
{
    struct usb_endpoint_descriptor *ep_desc;
    uint8_t bNumEndpoints;
    uint8_t i;

    *data_intf = intf;

    bNumEndpoints = hport->config.intf[intf].altsetting[0].intf_desc.bNumEndpoints;
    if (bNumEndpoints == 0) {
        if ((intf + 1) < hport->config.config_desc.bNumInterfaces) {
            *data_intf = intf + 1;
            bNumEndpoints = hport->config.intf[*data_intf].altsetting[0].intf_desc.bNumEndpoints;
        }
    }

    for (i = 0; i < bNumEndpoints; i++) {
        ep_desc = &hport->config.intf[*data_intf].altsetting[0].ep[i].ep_desc;
        if (USB_GET_ENDPOINT_TYPE(ep_desc->bmAttributes) == USB_ENDPOINT_TYPE_BULK) {
            if (ep_desc->bEndpointAddress & 0x80) {
                *bulkin = ep_desc;
            } else {
                *bulkout = ep_desc;
            }
        }
    }

    if (!*bulkin || !*bulkout) {
        USB_LOG_ERR("WinUSB: Bulk endpoints not found\r\n");
        return -USB_ERR_NODEV;
    }
    return 0;
}

static int usbh_winusb_connect(struct usbh_hubport *hport, uint8_t intf)
{
    struct usbh_winusb *winusb;
    struct usb_endpoint_descriptor *bulkin = NULL;
    struct usb_endpoint_descriptor *bulkout = NULL;
    uint8_t data_intf;
    int ret;

    winusb = usbh_winusb_alloc();
    if (!winusb) {
        USB_LOG_ERR("No memory for WinUSB class\r\n");
        return -USB_ERR_NOMEM;
    }

    ret = usbh_winusb_attach(hport, intf, &bulkin, &bulkout, &data_intf);
    if (ret < 0) {
        usbh_winusb_free(winusb);
        return ret;
    }

    /* 自定义检查：若不通过则释放并拒绝此设备 */
    if (usbh_winusb_check(hport) != 0) {
        USB_LOG_WRN("WinUSB device check failed, rejecting\r\n");
        usbh_winusb_free(winusb);
        return -USB_ERR_NODEV;
    }

    winusb->hport = hport;
    winusb->intf = intf;
    winusb->data_intf = data_intf;
    winusb->bulkin = bulkin;
    winusb->bulkout = bulkout;

    snprintf(hport->config.intf[intf].devname, CONFIG_USBHOST_DEV_NAMELEN,
             DEV_FORMAT_WINUSB, winusb->minor);

    hport->config.intf[intf].priv = winusb;

    USB_LOG_INFO("Register WinUSB Class: %s\r\n", hport->config.intf[intf].devname);

    /* 检查通过且注册成功后通知应用层 */
    usbh_poll_register(winusb);
    usbh_winusb_notify_connect(hport->config.intf[intf].devname);

    return 0;
}

static int usbh_winusb_disconnect(struct usbh_hubport *hport, uint8_t intf)
{
    struct usbh_winusb *winusb = (struct usbh_winusb *)hport->config.intf[intf].priv;
    if (winusb) {
        usbh_poll_unregister(winusb);
        usbh_winusb_notify_disconnect(hport->config.intf[intf].devname);
        usbh_winusb_close(winusb);
        usbh_winusb_free(winusb);
        hport->config.intf[intf].priv = NULL;
    }
    return 0;
}

struct usbh_winusb *usbh_winusb_open(const char *devname, uint32_t flags)
{
    struct usbh_winusb *winusb;
    winusb = (struct usbh_winusb *)usbh_find_class_instance(devname);
    if (!winusb) {
        USB_LOG_ERR("Device %s not found\r\n", devname);
        return NULL;
    }
    if (winusb->ref_count != 0) {
        USB_LOG_ERR("Device %s is busy\r\n", devname);
        return NULL;
    }
    winusb->ref_count++;
    winusb->open_flags = flags;
    return winusb;
}

int usbh_winusb_close(struct usbh_winusb *winusb)
{
    if (!winusb || !winusb->hport) {
        return -USB_ERR_INVAL;
    }
    if (winusb->ref_count == 0) {
        return 0;
    }
    if (winusb->bulkout) {
        usbh_kill_urb(&winusb->bulkout_urb);
    }
    winusb->ref_count--;
    return 0;
}

int usbh_winusb_write(struct usbh_winusb *winusb, const uint8_t *buffer, uint32_t len)
{
    struct usbh_urb *urb;
    int ret;
    if (!winusb || !winusb->hport || !winusb->hport->connected || !winusb->bulkout) {
        return -USB_ERR_INVAL;
    }
    if (winusb->ref_count == 0) {
        return -USB_ERR_NODEV;
    }
    urb = &winusb->bulkout_urb;
    usbh_bulk_urb_fill(urb, winusb->hport, winusb->bulkout,
                       (uint8_t *)buffer, len, 0xffffffff, NULL, NULL);
    ret = usbh_submit_urb(urb);
    if (ret == 0) {
        ret = urb->actual_length;
    }
    return ret;
}

int usbh_winusb_control_transfer(struct usbh_winusb *winusb,
                                 uint8_t bmRequestType, uint8_t bRequest,
                                 uint16_t wValue, uint16_t wIndex,
                                 uint8_t *buffer, uint16_t wLength)
{
    struct usb_setup_packet *setup;
    if (!winusb || !winusb->hport) {
        return -USB_ERR_INVAL;
    }
    setup = winusb->hport->setup;
    setup->bmRequestType = bmRequestType;
    setup->bRequest = bRequest;
    setup->wValue = wValue;
    setup->wIndex = wIndex;
    setup->wLength = wLength;
    return usbh_control_transfer(winusb->hport, setup, buffer);
}

static const struct usbh_class_driver winusb_class_driver = {
    .driver_name = "winusb",
    .connect = usbh_winusb_connect,
    .disconnect = usbh_winusb_disconnect
};

CLASS_INFO_DEFINE const struct usbh_class_info winusb_class_info = {
    .match_flags = USB_CLASS_MATCH_INTF_CLASS,
    .bInterfaceClass = USB_DEVICE_CLASS_VEND_SPECIFIC,
    .bInterfaceSubClass = 0x00,
    .bInterfaceProtocol = 0x00,
    .id_table = NULL,
    .class_driver = &winusb_class_driver
};