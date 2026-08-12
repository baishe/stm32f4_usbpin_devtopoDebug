#ifndef USBH_POLL_H
#define USBH_POLL_H

#include "usbh_winusb.h"
#include "FreeRTOS.h"
#include "queue.h"

#define USBH_POLL_MAX_SLOTS 20
#define USBH_POLL_CONCURRENCY 3
#define USBH_POLL_BUF_NUM 8
#define USBH_POLL_BUF_SIZE 64
#define USBH_POLL_PERIOD_MS 1
#define USBH_POLL_SELFTEST_COUNT 200

enum usbh_poll_event_type {
    USBH_POLL_EVENT_ATTACH,
    USBH_POLL_EVENT_DETACH,
    USBH_POLL_EVENT_DATA,
    USBH_POLL_EVENT_ERROR
};

struct usbh_poll_event {
    uint8_t type;
    uint8_t slot;
    uint8_t hub_addr;
    uint8_t hub_port;
    uint8_t depth;
    uint8_t path[4];
    uint8_t dev_addr;
    int16_t errorcode;
    uint16_t len;
    uint8_t *data;
};

struct usbh_poll_devinfo {
    uint8_t slot;
    uint8_t dev_addr;
    uint8_t hub_addr;
    uint8_t hub_port;
    uint8_t depth;
    uint8_t path[4];
    uint16_t vid;
    uint16_t pid;
    uint8_t enabled;
};

void usbh_poll_init(void);
int usbh_poll_register(struct usbh_winusb *winusb);
int usbh_poll_first_slot(void);
struct usbh_winusb *usbh_poll_get_device(uint8_t slot);
void usbh_poll_unregister(struct usbh_winusb *winusb);
int usbh_poll_enable(uint8_t slot);
int usbh_poll_disable(uint8_t slot);
bool usbh_poll_is_enabled(uint8_t slot);
int usbh_poll_get_info(uint8_t slot, struct usbh_poll_devinfo *info);
int usbh_poll_event_recv(struct usbh_poll_event *ev, uint32_t timeout_ms);
void usbh_poll_buf_release(uint8_t *buf);
void usbh_poll_stats_dump(void);

void usbh_poll_on_event(const struct usbh_poll_event *ev);

#endif
