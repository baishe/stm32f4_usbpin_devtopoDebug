#include "main.h"
#include "usbh_poll.h"
#include "usbh_core.h"
#include "usb_log.h"
#include "task.h"
#include <string.h>

struct poll_slot {
    volatile uint8_t state;
    bool in_queue;
    struct usbh_winusb *winusb;
    struct usbh_hubport *hport;
    struct usb_endpoint_descriptor *bulkin;
    struct usbh_urb urb;
    uint8_t *buf;
    uint8_t dev_addr;
    uint8_t hub_addr;
    uint8_t hub_port;
    uint8_t depth;
    uint8_t path[4];
    uint16_t vid;
    uint16_t pid;
    uint32_t rx_packets;
    uint32_t nak_count;
    uint32_t err_count;
    uint32_t last_us;
    uint32_t max_us;
    uint32_t selftest_left;
    bool selftest;
    uint32_t selftest_min;
    uint32_t selftest_max;
    uint64_t selftest_sum;
};

#define POLL_FREE 0
#define POLL_IDLE 1
#define POLL_INFLIGHT 2

static struct poll_slot g_slots[USBH_POLL_MAX_SLOTS];
static uint8_t g_buffers[USBH_POLL_BUF_NUM][USBH_POLL_BUF_SIZE] USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX;
static bool g_buf_used[USBH_POLL_BUF_NUM];
static QueueHandle_t g_event_queue;
static TaskHandle_t g_poll_task;
static volatile uint32_t g_round;
static volatile uint32_t g_overrun;
static volatile uint32_t g_scan;
static volatile uint32_t g_busy_min = 0xffffffffU;
static volatile uint32_t g_busy_max;
static volatile uint64_t g_busy_sum;
static volatile uint32_t g_busy_count;
static volatile uint32_t g_pool_low;
static volatile uint32_t g_evtq_high;
static volatile bool g_pool_blocked;
static uint8_t g_next_slot;

static uint32_t poll_now_us(void)
{
    return (uint32_t)(DWT->CYCCNT / (SystemCoreClock / 1000000U));
}

static int poll_buf_alloc(uint8_t **buf)
{
    UBaseType_t key = taskENTER_CRITICAL_FROM_ISR();
    int i;
    for (i = 0; i < USBH_POLL_BUF_NUM; i++) {
        if (!g_buf_used[i]) {
            g_buf_used[i] = true;
            *buf = g_buffers[i];
            taskEXIT_CRITICAL_FROM_ISR(key);
            return 0;
        }
    }
    taskEXIT_CRITICAL_FROM_ISR(key);
    g_pool_low++;
    g_pool_blocked = true;
    return -1;
}

void usbh_poll_buf_release(uint8_t *buf)
{
    int i;
    if (!buf) {
        return;
    }
    for (i = 0; i < USBH_POLL_BUF_NUM; i++) {
        if (buf == g_buffers[i]) {
            taskENTER_CRITICAL();
            g_buf_used[i] = false;
            taskEXIT_CRITICAL();
            return;
        }
    }
}

static void poll_path(struct usbh_hubport *hport, uint8_t *path)
{
    uint8_t rev[4] = {0};
    uint8_t n = 0;
    struct usbh_hubport *p = hport;
    while (p && n < 4) {
        rev[n++] = p->port;
        p = p->parent ? p->parent->parent : NULL;
    }
    while (n) {
        *path++ = rev[--n];
    }
}

static void poll_event(struct poll_slot *s, uint8_t type, uint8_t *data,
                       uint16_t len, int errorcode)
{
    struct usbh_poll_event ev;
    BaseType_t woken = pdFALSE;
    memset(&ev, 0, sizeof(ev));
    ev.type = type;
    ev.slot = (uint8_t)(s - g_slots);
    ev.hub_addr = s->hub_addr;
    ev.hub_port = s->hub_port;
    ev.depth = s->depth;
    memcpy(ev.path, s->path, sizeof(ev.path));
    ev.dev_addr = s->dev_addr;
    ev.data = data;
    ev.len = len;
    ev.errorcode = (int16_t)errorcode;
    if (xPortIsInsideInterrupt()) {
        xQueueSendFromISR(g_event_queue, &ev, &woken);
        portYIELD_FROM_ISR(woken);
    } else {
        xQueueSend(g_event_queue, &ev, 0);
    }
    {
        UBaseType_t high = uxQueueMessagesWaitingFromISR(g_event_queue);
        if (high > g_evtq_high) {
            g_evtq_high = high;
        }
    }
}

static void poll_submit(struct poll_slot *s)
{
    uint8_t *buf;
    int ret;
    if (!s->in_queue || s->state != POLL_IDLE || !s->bulkin || g_pool_blocked) {
        return;
    }
    if (poll_buf_alloc(&buf) != 0) {
        return;
    }
    s->buf = buf;
    usbh_bulk_urb_fill(&s->urb, s->hport, s->bulkin, buf,
                       USBH_POLL_BUF_SIZE, 0, NULL, s);
    s->urb.transfer_flags = USBH_URB_FLAG_POLL_ONESHOT;
    s->urb.arg = s;
    s->state = POLL_INFLIGHT;
    s->urb.start_frame = poll_now_us();
    HAL_GPIO_WritePin(DEBUG_IO_GPIO_Port, DEBUG_IO_Pin, GPIO_PIN_SET);
    ret = usbh_submit_urb(&s->urb);
    if (ret < 0) {
        s->state = POLL_IDLE;
        usbh_poll_buf_release(buf);
        s->buf = NULL;
    }
}

static void poll_complete(void *arg, int nbytes)
{
    struct poll_slot *s = (struct poll_slot *)arg;
    struct usbh_urb *urb = &s->urb;
    uint32_t elapsed = poll_now_us() - urb->start_frame;
    BaseType_t woken = pdFALSE;
    (void)nbytes;
    HAL_GPIO_WritePin(DEBUG_IO_GPIO_Port, DEBUG_IO_Pin, GPIO_PIN_RESET);
    s->last_us = elapsed;
    if (elapsed > s->max_us) s->max_us = elapsed;
    if (elapsed < g_busy_min) g_busy_min = elapsed;
    if (elapsed > g_busy_max) g_busy_max = elapsed;
    g_busy_sum += elapsed;
    g_busy_count++;
    s->state = POLL_IDLE;
    if (!s->in_queue) {
        usbh_poll_buf_release(s->buf);
        s->buf = NULL;
        poll_event(s, USBH_POLL_EVENT_DETACH, NULL, 0, 0);
        s->state = POLL_FREE;
        return;
    }
    if (s->selftest) {
        uint8_t *completed_buf = s->buf;
        uint32_t n = USBH_POLL_SELFTEST_COUNT - s->selftest_left;
        if (elapsed < s->selftest_min) s->selftest_min = elapsed;
        if (elapsed > s->selftest_max) s->selftest_max = elapsed;
        s->selftest_sum += elapsed;
        USB_LOG_INFO("[selftest] n=%lu rc=%d us=%lu\r\n",
                     (unsigned long)n, urb->errorcode, (unsigned long)elapsed);
        if (s->selftest_left) {
            s->selftest_left--;
            poll_submit(s);
        } else {
            USB_LOG_INFO("[selftest] done min/avg/max=%lu/%lu/%lu us\r\n",
                         (unsigned long)s->selftest_min,
                         (unsigned long)(s->selftest_sum / USBH_POLL_SELFTEST_COUNT),
                         (unsigned long)s->selftest_max);
            s->selftest = false;
        }
        usbh_poll_buf_release(completed_buf);
        if (!s->selftest) s->buf = NULL;
        return;
    }
    if (urb->errorcode == -USB_ERR_NAK) {
        s->nak_count++;
        usbh_poll_buf_release(s->buf);
    } else if (urb->errorcode == 0 && urb->actual_length > 0) {
        s->rx_packets++;
        poll_event(s, USBH_POLL_EVENT_DATA, s->buf, (uint16_t)urb->actual_length, 0);
        s->buf = NULL;
    } else if (urb->errorcode == 0) {
        usbh_poll_buf_release(s->buf);
    } else {
        s->err_count++;
        usbh_poll_buf_release(s->buf);
        poll_event(s, USBH_POLL_EVENT_ERROR, NULL, 0, urb->errorcode);
    }
    s->buf = NULL;
    if (g_next_slot < USBH_POLL_MAX_SLOTS) {
        uint8_t next = g_next_slot++;
        if (g_slots[next].in_queue && g_slots[next].state == POLL_IDLE) {
            poll_submit(&g_slots[next]);
        }
    }
    (void)woken;
}

static void poll_start(struct poll_slot *s)
{
    s->urb.complete = poll_complete;
    poll_submit(s);
}

static void poll_task(void *arg)
{
    TickType_t wake = xTaskGetTickCount();
    (void)arg;
    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(USBH_POLL_PERIOD_MS));
        if (g_next_slot != USBH_POLL_MAX_SLOTS) {
            g_overrun++;
            continue;
        }
        g_round++;
        g_pool_blocked = false;
        g_next_slot = 0;
        g_scan = 0;
        while (g_next_slot < USBH_POLL_MAX_SLOTS &&
               g_scan < USBH_POLL_CONCURRENCY) {
            struct poll_slot *s = &g_slots[g_next_slot++];
            if (s->in_queue && s->state == POLL_IDLE) {
                g_scan++;
                poll_start(s);
            }
        }
        if (g_scan < USBH_POLL_CONCURRENCY) g_next_slot = USBH_POLL_MAX_SLOTS;
    }
}

void usbh_poll_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    g_event_queue = xQueueCreate(8, sizeof(struct usbh_poll_event));
    xTaskCreate(poll_task, "UsbPoll", 512, NULL, tskIDLE_PRIORITY + 1, &g_poll_task);
}

int usbh_poll_register(struct usbh_winusb *winusb)
{
    int i;
    struct poll_slot *s = NULL;
    for (i = 0; i < USBH_POLL_MAX_SLOTS; i++) {
        if (g_slots[i].state == POLL_FREE) { s = &g_slots[i]; break; }
        if (g_slots[i].hport == winusb->hport) { s = &g_slots[i]; break; }
    }
    if (!s) return -USB_ERR_NOMEM;
    memset(s, 0, sizeof(*s));
    s->state = POLL_IDLE;
    s->in_queue = true;
    s->winusb = winusb;
    s->hport = winusb->hport;
    s->bulkin = winusb->bulkin;
    s->dev_addr = s->hport->dev_addr;
    s->hub_port = s->hport->port;
    s->depth = s->hport->depth;
    s->hub_addr = s->hport->parent ? s->hport->parent->hub_addr : 0;
    s->vid = s->hport->device_desc.idVendor;
    s->pid = s->hport->device_desc.idProduct;
    poll_path(s->hport, s->path);
    poll_event(s, USBH_POLL_EVENT_ATTACH, NULL, 0, 0);
    if (g_round == 0) {
        s->selftest = true;
        s->selftest_left = USBH_POLL_SELFTEST_COUNT - 1;
        s->selftest_min = 0xffffffffU;
        USB_LOG_INFO("[selftest] start slot=%u count=%u\r\n",
                     (unsigned)(s - g_slots), USBH_POLL_SELFTEST_COUNT);
    }
    return (int)(s - g_slots);
}

void usbh_poll_unregister(struct usbh_winusb *winusb)
{
    int i;
    for (i = 0; i < USBH_POLL_MAX_SLOTS; i++) {
        if (g_slots[i].winusb == winusb) {
            g_slots[i].in_queue = false;
            if (g_slots[i].state == POLL_IDLE) {
                poll_event(&g_slots[i], USBH_POLL_EVENT_DETACH, NULL, 0, 0);
                g_slots[i].state = POLL_FREE;
            }
            return;
        }
    }
}

int usbh_poll_enable(uint8_t slot)
{
    if (slot >= USBH_POLL_MAX_SLOTS) return -USB_ERR_INVAL;
    taskENTER_CRITICAL();
    g_slots[slot].in_queue = true;
    taskEXIT_CRITICAL();
    return 0;
}

int usbh_poll_disable(uint8_t slot)
{
    if (slot >= USBH_POLL_MAX_SLOTS) return -USB_ERR_INVAL;
    taskENTER_CRITICAL();
    g_slots[slot].in_queue = false;
    taskEXIT_CRITICAL();
    return 0;
}
bool usbh_poll_is_enabled(uint8_t slot) { return slot < USBH_POLL_MAX_SLOTS && g_slots[slot].in_queue; }

int usbh_poll_get_info(uint8_t slot, struct usbh_poll_devinfo *info)
{
    struct poll_slot *s;
    if (slot >= USBH_POLL_MAX_SLOTS || !info || g_slots[slot].state == POLL_FREE) return -USB_ERR_INVAL;
    s = &g_slots[slot];
    memset(info, 0, sizeof(*info));
    info->slot = slot; info->dev_addr = s->dev_addr; info->hub_addr = s->hub_addr;
    info->hub_port = s->hub_port; info->depth = s->depth; info->vid = s->vid;
    info->pid = s->pid; info->enabled = s->in_queue; memcpy(info->path, s->path, 4);
    return 0;
}

int usbh_poll_event_recv(struct usbh_poll_event *ev, uint32_t timeout_ms)
{
    return (xQueueReceive(g_event_queue, ev, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) ? 0 : -USB_ERR_TIMEOUT;
}

void usbh_poll_stats_dump(void)
{
    uint32_t avg = g_busy_count ? (uint32_t)(g_busy_sum / g_busy_count) : 0;
    USB_LOG_INFO("[poll] round=%lu overrun=%lu scan=%lu busy_us(min/avg/max)=%lu/%lu/%lu pool_low=%lu evtq_high=%lu\r\n",
                 (unsigned long)g_round, (unsigned long)g_overrun, (unsigned long)g_scan,
                 (unsigned long)(g_busy_min == 0xffffffffU ? 0 : g_busy_min),
                 (unsigned long)avg, (unsigned long)g_busy_max,
                 (unsigned long)g_pool_low, (unsigned long)g_evtq_high);
    for (uint8_t i = 0; i < USBH_POLL_MAX_SLOTS; i++) {
        if (g_slots[i].state != POLL_FREE) {
            USB_LOG_INFO("[slot%u] path=%u-%u-%u addr=%u vid=%04x pid=%04x rx=%lu nak=%lu err=%lu max_us=%lu\r\n",
                         i, g_slots[i].path[0], g_slots[i].path[1], g_slots[i].path[2],
                         g_slots[i].dev_addr, g_slots[i].vid, g_slots[i].pid,
                         (unsigned long)g_slots[i].rx_packets,
                         (unsigned long)g_slots[i].nak_count,
                         (unsigned long)g_slots[i].err_count,
                         (unsigned long)g_slots[i].max_us);
        }
    }
}

__attribute__((weak)) void usbh_poll_on_event(const struct usbh_poll_event *ev) { (void)ev; }
