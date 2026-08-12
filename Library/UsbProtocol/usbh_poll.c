#include "main.h"
#include "usbh_poll.h"
#include "usbh_core.h"
#include "usb_log.h"
#include "usb_osal.h"
#include "task.h"
#include <string.h>

#define POLL_FREE 0
#define POLL_IDLE 1
#define POLL_INFLIGHT 2
#define USBH_POLL_URB_TIMEOUT_MS 5
#define USBH_POLL_SELFTEST_IDLE 0
#define USBH_POLL_SELFTEST_RUNNING 1
#define USBH_POLL_SELFTEST_DONE 2

struct poll_selftest_sample {
    uint16_t us_x10;
    int8_t rc;
};

struct poll_slot {
    volatile uint8_t state;
    bool in_queue;
    bool selftest;
    bool timeout_pending;
    bool unregister_pending;
    bool timeout_report;
    bool detach_report;
    bool debug_active;
    struct usbh_winusb *winusb;
    struct usbh_hubport *hport;
    struct usb_endpoint_descriptor *bulkin;
    struct usbh_urb urb;
    uint8_t *buf;
    uint32_t start_cyc;
    TickType_t start_tick;
    uint8_t dev_addr;
    uint8_t hub_addr;
    uint8_t hub_port;
    uint8_t depth;
    uint8_t path[4];
    uint16_t vid;
    uint16_t pid;
    uint32_t rx_packets;
    uint32_t nak_count;
    uint32_t zlp_count;
    uint32_t err_count;
    uint32_t err_streak;
    uint32_t next_poll_round;
    int16_t err_last_code;
    uint32_t timeout_count;
    uint32_t evt_drop;
    uint32_t last_us;
    uint32_t max_us;
    uint16_t selftest_count;
};

static struct poll_slot g_slots[USBH_POLL_MAX_SLOTS];
static uint8_t g_buffers[USBH_POLL_BUF_NUM][USBH_POLL_BUF_SIZE] USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX;
static bool g_buf_used[USBH_POLL_BUF_NUM];
static QueueHandle_t g_event_queue;
static TaskHandle_t g_poll_task;
static volatile uint32_t g_round;
static volatile uint32_t g_overrun;
static volatile uint32_t g_round_scan;
static volatile uint32_t g_scan_snapshot;
static volatile uint8_t g_cursor;
static volatile uint8_t g_inflight;
static volatile bool g_round_active;
static volatile bool g_pool_blocked;
static volatile bool g_pumping;
static volatile uint32_t g_busy_min = 0xffffffffU;
static volatile uint32_t g_busy_max;
static volatile uint64_t g_busy_sum;
static volatile uint32_t g_busy_count;
static volatile uint32_t g_busy_max_all;
static volatile uint32_t g_pool_empty;
static volatile uint32_t g_evtq_high;
static volatile uint32_t g_evt_drop;
static volatile uint32_t g_timeout_count;
static volatile bool g_dwt_ok;
static volatile uint8_t g_selftest_state = USBH_POLL_SELFTEST_IDLE;
static volatile uint8_t g_selftest_slot = 0xff;
static volatile bool g_selftest_reported;
static volatile uint16_t g_selftest_n;
static volatile uint32_t g_selftest_start_cyc;
static volatile TickType_t g_selftest_start_tick;
static struct poll_selftest_sample g_selftest_samples[USBH_POLL_SELFTEST_COUNT];

static uint32_t poll_cycles(void)
{
    return DWT->CYCCNT;
}

static bool poll_dwt_enable(void)
{
    uint32_t before;
    volatile uint32_t delay;
    uint8_t attempt;

    for (attempt = 0; attempt < 3; attempt++) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
        before = DWT->CYCCNT;
        for (delay = 0; delay < 512U; delay++) {
        }
        if (DWT->CYCCNT != before) return true;
    }
    return false;
}

static uint32_t poll_cycles_to_us(uint32_t cycles)
{
    return cycles / (SystemCoreClock / 1000000U);
}

static uint32_t poll_cycles_to_us_x10(uint32_t cycles)
{
    return (uint32_t)(((uint64_t)cycles * 10U) /
                      (SystemCoreClock / 1000000U));
}

static bool poll_debug_pin_enabled(const struct poll_slot *s)
{
#if USBH_POLL_DEBUG_PIN_SELFTEST_ONLY
    return s->selftest;
#else
    (void)s;
    return true;
#endif
}

static void poll_debug_pin(const struct poll_slot *s, uint32_t state)
{
    struct poll_slot *slot = (struct poll_slot *)s;
    if (state == GPIO_PIN_SET) {
        if (poll_debug_pin_enabled(s)) {
            HAL_GPIO_WritePin(DEBUG_IO_GPIO_Port, DEBUG_IO_Pin, state);
            slot->debug_active = true;
        }
    } else if (slot->debug_active) {
        HAL_GPIO_WritePin(DEBUG_IO_GPIO_Port, DEBUG_IO_Pin, state);
        slot->debug_active = false;
    }
}

static void poll_critical_enter(size_t *flags)
{
    *flags = usb_osal_enter_critical_section();
}

static void poll_critical_leave(size_t flags)
{
    usb_osal_leave_critical_section(flags);
}

static int poll_buf_alloc(uint8_t **buf)
{
    size_t flags;
    int i;
    poll_critical_enter(&flags);
    for (i = 0; i < USBH_POLL_BUF_NUM; i++) {
        if (!g_buf_used[i]) {
            g_buf_used[i] = true;
            *buf = g_buffers[i];
            poll_critical_leave(flags);
            return 0;
        }
    }
    poll_critical_leave(flags);
    g_pool_empty++;
    g_pool_blocked = true;
    return -1;
}

void usbh_poll_buf_release(uint8_t *buf)
{
    size_t flags;
    int i;
    if (!buf) return;
    poll_critical_enter(&flags);
    for (i = 0; i < USBH_POLL_BUF_NUM; i++) {
        if (buf == g_buffers[i]) {
            g_buf_used[i] = false;
            break;
        }
    }
    poll_critical_leave(flags);
}

static void poll_path(struct usbh_hubport *hport, uint8_t *path)
{
    uint8_t reverse[4] = { 0 };
    uint8_t n = 0;
    struct usbh_hubport *p = hport;
    while (p && n < 4) {
        reverse[n++] = p->port;
        if (!p->parent) break;
        p = p->parent->parent;
    }
    while (n) *path++ = reverse[--n];
}

static bool poll_same_path(const struct poll_slot *s, const uint8_t *path, uint8_t depth)
{
    return s->depth == depth && memcmp(s->path, path, 4) == 0;
}

static bool poll_event(struct poll_slot *s, uint8_t type, uint8_t *data,
                       uint16_t len, int errorcode, uint16_t count)
{
    struct usbh_poll_event ev;
    BaseType_t woken = pdFALSE;
    UBaseType_t high;
    bool in_isr = xPortIsInsideInterrupt() != pdFALSE;
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
    ev.count = count;
    if (in_isr) {
        if (xQueueSendFromISR(g_event_queue, &ev, &woken) != pdTRUE) {
            g_evt_drop++;
            s->evt_drop++;
            return false;
        }
        high = uxQueueMessagesWaitingFromISR(g_event_queue);
        portYIELD_FROM_ISR(woken);
    } else {
        if (xQueueSend(g_event_queue, &ev, 0) != pdTRUE) {
            g_evt_drop++;
            s->evt_drop++;
            return false;
        }
        high = uxQueueMessagesWaiting(g_event_queue);
    }
    if (high > g_evtq_high) g_evtq_high = high;
    return true;
}

static uint16_t poll_error_count(const struct poll_slot *s)
{
    return s->err_streak > 0xffffU ? 0xffffU : (uint16_t)s->err_streak;
}

static void poll_error_record(struct poll_slot *s, int errorcode,
                              bool report_first)
{
    size_t flags;
    bool first = false;
    poll_critical_enter(&flags);
    s->err_count++;
    s->err_last_code = (int16_t)errorcode;
    if (s->err_streak != 0xffffffffU) s->err_streak++;
    if (s->err_streak >= 4U) {
        s->next_poll_round = g_round + 32U;
    }
    first = report_first && s->err_streak == 1U;
    poll_critical_leave(flags);
    if (first) {
        poll_event(s, USBH_POLL_EVENT_ERROR, NULL, 0, errorcode, 1);
    }
}

static void poll_error_recover(struct poll_slot *s)
{
    size_t flags;
    uint16_t count = 0;
    int16_t errorcode = 0;
    /* Fast path: the common NAK completion must not pay for a critical section. */
    if (s->err_streak == 0 && s->next_poll_round == 0) return;
    poll_critical_enter(&flags);
    if (s->err_streak > 1U) {
        count = poll_error_count(s);
        errorcode = s->err_last_code;
    }
    s->err_streak = 0;
    s->next_poll_round = 0;
    s->err_last_code = 0;
    poll_critical_leave(flags);
    if (count != 0) {
        poll_event(s, USBH_POLL_EVENT_ERROR, NULL, 0,
                   errorcode, count);
    }
}

static void poll_complete(void *arg, int nbytes);
static void poll_pump(void);

static bool poll_submit_claimed(struct poll_slot *s)
{
    uint8_t *buf;
    int ret;
    bool selftest = s->selftest;
    if (poll_buf_alloc(&buf) != 0) return false;
    s->buf = buf;
    usbh_bulk_urb_fill(&s->urb, s->hport, s->bulkin, buf,
                       USBH_POLL_BUF_SIZE, 0, poll_complete, s);
    s->urb.transfer_flags = USBH_URB_FLAG_POLL_ONESHOT;
    s->urb.arg = s;
    s->start_cyc = poll_cycles();
    s->start_tick = xTaskGetTickCount();
    poll_debug_pin(s, GPIO_PIN_SET);
    ret = usbh_submit_urb(&s->urb);
    if (ret < 0) {
        poll_debug_pin(s, GPIO_PIN_RESET);
        usbh_poll_buf_release(buf);
        s->buf = NULL;
        return false;
    }
    return true;
}

static bool poll_claim_slot(struct poll_slot *s, bool allow_pool_blocked)
{
    size_t flags;
    bool claimed = false;
    poll_critical_enter(&flags);
    if (s->in_queue && s->state == POLL_IDLE && s->bulkin &&
        (allow_pool_blocked || !g_pool_blocked)) {
        s->state = POLL_INFLIGHT;
        g_inflight++;
        claimed = true;
    }
    poll_critical_leave(flags);
    return claimed;
}

static bool poll_submit(struct poll_slot *s)
{
    if (!poll_claim_slot(s, s->selftest)) return false;
    if (!poll_submit_claimed(s)) {
        size_t flags;
        poll_critical_enter(&flags);
        s->state = POLL_IDLE;
        if (g_inflight) g_inflight--;
        poll_critical_leave(flags);
        return false;
    }
    return true;
}

static void poll_pump(void)
{
    size_t flags;
    poll_critical_enter(&flags);
    if (g_pumping) {
        poll_critical_leave(flags);
        return;
    }
    g_pumping = true;
    poll_critical_leave(flags);

    for (;;) {
        struct poll_slot *s;
        bool claimed = false;
        poll_critical_enter(&flags);
        if (!g_round_active || g_inflight >= USBH_POLL_CONCURRENCY ||
            g_cursor >= USBH_POLL_MAX_SLOTS) {
            poll_critical_leave(flags);
            break;
        }
        s = &g_slots[g_cursor++];
        if (s->in_queue && s->state == POLL_IDLE && s->bulkin &&
            !g_pool_blocked &&
            (s->next_poll_round == 0 || g_round >= s->next_poll_round)) {
            s->state = POLL_INFLIGHT;
            g_inflight++;
            g_round_scan++;
            claimed = true;
        }
        poll_critical_leave(flags);
        if (claimed) {
            if (!poll_submit_claimed(s)) {
                poll_critical_enter(&flags);
                s->state = POLL_IDLE;
                if (g_inflight) g_inflight--;
                if (g_round_scan) g_round_scan--;
                poll_critical_leave(flags);
            }
            if (g_pool_blocked) {
                poll_critical_enter(&flags);
                g_cursor = USBH_POLL_MAX_SLOTS;
                poll_critical_leave(flags);
                break;
            }
        }
        if (g_pool_blocked) {
            poll_critical_enter(&flags);
            g_cursor = USBH_POLL_MAX_SLOTS;
            poll_critical_leave(flags);
            break;
        }
    }
    poll_critical_enter(&flags);
    if (g_round_active && g_cursor >= USBH_POLL_MAX_SLOTS && g_inflight == 0) {
        g_scan_snapshot = g_round_scan;
        g_round_active = false;
    }
    g_pumping = false;
    poll_critical_leave(flags);
}

static void poll_selftest_report(void)
{
    uint32_t min = 0xffffffffU, max = 0, sum = 0;
    uint32_t nak = 0, zero = 0, other = 0;
    uint32_t bins[5] = { 0 };
    uint16_t i;
    if (g_selftest_state != USBH_POLL_SELFTEST_DONE || g_selftest_reported) return;
    if (g_selftest_n == 0) {
        USB_LOG_INFO("[selftest] n=0\r\n");
        g_selftest_reported = true;
        return;
    }
    if (!g_dwt_ok) {
        for (i = 0; i < g_selftest_n; i++) {
            int rc = g_selftest_samples[i].rc;
            if (rc == -USB_ERR_NAK) nak++;
            else if (rc == 0) zero++;
            else other++;
        }
        USB_LOG_INFO("[selftest] n=%u rc(nak/zero/other)=%lu/%lu/%lu us_x10(min/avg/max)=na/na/na\r\n",
                     g_selftest_n, (unsigned long)nak, (unsigned long)zero,
                     (unsigned long)other);
        USB_LOG_INFO("[selftest] hist(<10/10-20/20-50/50-100/>100us)=na/na/na/na/na\r\n");
        for (i = 0; i < 20 && i < g_selftest_n; i++) {
            USB_LOG_INFO("[selftest] sample[%u] us_x10=na rc=%d\r\n", i,
                         g_selftest_samples[i].rc);
        }
        g_selftest_reported = true;
        return;
    }
    for (i = 0; i < g_selftest_n; i++) {
        uint32_t us_x10 = g_selftest_samples[i].us_x10;
        int rc = g_selftest_samples[i].rc;
        if (us_x10 < min) min = us_x10;
        if (us_x10 > max) max = us_x10;
        sum += us_x10;
        if (rc == -USB_ERR_NAK) nak++;
        else if (rc == 0) zero++;
        else other++;
        if (us_x10 < 100) bins[0]++;
        else if (us_x10 < 200) bins[1]++;
        else if (us_x10 < 500) bins[2]++;
        else if (us_x10 < 1000) bins[3]++;
        else bins[4]++;
    }
    USB_LOG_INFO("[selftest] n=%u rc(nak/zero/other)=%lu/%lu/%lu us_x10(min/avg/max)=%lu/%lu/%lu\r\n",
                 g_selftest_n, (unsigned long)nak, (unsigned long)zero,
                 (unsigned long)other, (unsigned long)min,
                 (unsigned long)(sum / g_selftest_n), (unsigned long)max);
    USB_LOG_INFO("[selftest] hist(<10/10-20/20-50/50-100/>100us)=%lu/%lu/%lu/%lu/%lu\r\n",
                 (unsigned long)bins[0], (unsigned long)bins[1], (unsigned long)bins[2],
                 (unsigned long)bins[3], (unsigned long)bins[4]);
    for (i = 0; i < 20 && i < g_selftest_n; i++) {
        USB_LOG_INFO("[selftest] sample[%u] us_x10=%u rc=%d\r\n", i,
                     g_selftest_samples[i].us_x10, g_selftest_samples[i].rc);
    }
    g_selftest_reported = true;
}

static void poll_complete(void *arg, int nbytes)
{
    struct poll_slot *s = (struct poll_slot *)arg;
    struct usbh_urb *urb = &s->urb;
    uint8_t *buf = s->buf;
    uint32_t elapsed_cyc = (uint32_t)(poll_cycles() - s->start_cyc);
    uint32_t elapsed = g_dwt_ok ? poll_cycles_to_us(elapsed_cyc) : 0;
    bool timeout = s->timeout_pending;
    bool unregister = s->unregister_pending;
    size_t flags;
    (void)nbytes;
    poll_debug_pin(s, GPIO_PIN_RESET);
    poll_critical_enter(&flags);
    if (g_inflight) g_inflight--;
    s->state = POLL_IDLE;
    poll_critical_leave(flags);
    if (g_dwt_ok) {
        s->last_us = elapsed;
        if (elapsed > s->max_us) s->max_us = elapsed;
        if (elapsed < g_busy_min) g_busy_min = elapsed;
        if (elapsed > g_busy_max) g_busy_max = elapsed;
        if (elapsed > g_busy_max_all) g_busy_max_all = elapsed;
        g_busy_sum += elapsed;
        g_busy_count++;
    }
    s->timeout_pending = false;

    if (timeout || unregister) {
        usbh_poll_buf_release(buf);
        s->buf = NULL;
        if (timeout) {
            s->timeout_count++;
            g_timeout_count++;
            poll_error_record(s, -USB_ERR_TIMEOUT, false);
            s->timeout_report = true;
        }
        if (unregister) {
            s->detach_report = true;
        }
        s->unregister_pending = false;
        return;
    }
    if (!s->in_queue) {
        usbh_poll_buf_release(buf);
        s->buf = NULL;
        poll_error_recover(s);
        poll_event(s, USBH_POLL_EVENT_DETACH, NULL, 0, 0, 0);
        s->winusb = NULL;
        s->hport = NULL;
        s->bulkin = NULL;
        s->state = POLL_FREE;
        if (g_selftest_state == USBH_POLL_SELFTEST_RUNNING &&
            g_selftest_slot == (uint8_t)(s - g_slots)) {
            g_selftest_state = USBH_POLL_SELFTEST_DONE;
            g_selftest_slot = 0xff;
        }
        return;
    }
    if (s->selftest) {
        uint16_t n = s->selftest_count;
        if (n < USBH_POLL_SELFTEST_COUNT) {
            uint32_t us_x10 = g_dwt_ok ? poll_cycles_to_us_x10(elapsed_cyc) : 0;
            g_selftest_samples[n].us_x10 =
                us_x10 > 0xffffU ? 0xffffU : (uint16_t)us_x10;
            g_selftest_samples[n].rc = (int8_t)urb->errorcode;
            s->selftest_count++;
            g_selftest_n = s->selftest_count;
        }
        usbh_poll_buf_release(buf);
        s->buf = NULL;
        if (s->selftest_count < USBH_POLL_SELFTEST_COUNT) {
            poll_submit(s);
        } else {
            s->selftest = false;
            g_selftest_state = USBH_POLL_SELFTEST_DONE;
        }
        return;
    }
    if (timeout) {
        s->timeout_count++;
        g_timeout_count++;
        usbh_poll_buf_release(buf);
        s->buf = NULL;
        poll_event(s, USBH_POLL_EVENT_ERROR, NULL, 0, -USB_ERR_TIMEOUT, 1);
    } else if (urb->errorcode == -USB_ERR_NAK) {
        s->nak_count++;
        poll_error_recover(s);
        usbh_poll_buf_release(buf);
        s->buf = NULL;
    } else if (urb->errorcode == 0 && urb->actual_length > 0) {
        s->rx_packets++;
        poll_error_recover(s);
        if (poll_event(s, USBH_POLL_EVENT_DATA, buf,
                       (uint16_t)urb->actual_length, 0, 0)) {
            s->buf = NULL;
        } else {
            usbh_poll_buf_release(buf);
            s->buf = NULL;
        }
    } else if (urb->errorcode == 0) {
        s->zlp_count++;
        poll_error_recover(s);
        usbh_poll_buf_release(buf);
        s->buf = NULL;
    } else {
        poll_error_record(s, urb->errorcode, true);
        usbh_poll_buf_release(buf);
        s->buf = NULL;
    }
    poll_pump();
}

static void poll_watchdog(void)
{
    uint32_t now = poll_cycles();
    TickType_t now_tick = xTaskGetTickCount();
    uint8_t i;
    for (i = 0; i < USBH_POLL_MAX_SLOTS; i++) {
        struct poll_slot *s = &g_slots[i];
        size_t flags;
        poll_critical_enter(&flags);
        if (s->state == POLL_INFLIGHT &&
            ((g_dwt_ok &&
              poll_cycles_to_us((uint32_t)(now - s->start_cyc)) >=
                  USBH_POLL_URB_TIMEOUT_MS * 1000U) ||
             (!g_dwt_ok &&
              (TickType_t)(now_tick - s->start_tick) >=
                  pdMS_TO_TICKS(USBH_POLL_URB_TIMEOUT_MS)))) {
            s->timeout_pending = true;
            usbh_kill_urb(&s->urb);
        }
        poll_critical_leave(flags);
    }
}

static void poll_drain_reports(void)
{
    uint8_t i;
    for (i = 0; i < USBH_POLL_MAX_SLOTS; i++) {
        struct poll_slot *s = &g_slots[i];
        if (s->timeout_report) {
            size_t flags;
            bool first;
            poll_critical_enter(&flags);
            s->timeout_report = false;
            first = s->err_streak == 1U;
            poll_critical_leave(flags);
            if (first) {
                poll_event(s, USBH_POLL_EVENT_ERROR, NULL, 0,
                           -USB_ERR_TIMEOUT, 1);
            }
        }
        if (s->detach_report) {
            s->detach_report = false;
            poll_error_recover(s);
            poll_event(s, USBH_POLL_EVENT_DETACH, NULL, 0, 0, 0);
            s->winusb = NULL;
            s->hport = NULL;
            s->bulkin = NULL;
            s->state = POLL_FREE;
            if (g_selftest_state == USBH_POLL_SELFTEST_RUNNING &&
                g_selftest_slot == i) {
                g_selftest_state = USBH_POLL_SELFTEST_DONE;
                g_selftest_slot = 0xff;
            }
        }
    }
}

static void poll_task(void *arg)
{
    TickType_t wake = xTaskGetTickCount();
    (void)arg;
    for (;;) {
        /* DelayUntil is disabled in CubeMX FreeRTOSConfig; phase drift is acceptable here. */
        vTaskDelay(pdMS_TO_TICKS(USBH_POLL_PERIOD_MS));
        poll_watchdog();
        poll_drain_reports();
        if (g_selftest_state == USBH_POLL_SELFTEST_RUNNING) {
            TickType_t now_tick = xTaskGetTickCount();
            if ((g_dwt_ok &&
                 poll_cycles_to_us((uint32_t)(poll_cycles() -
                                               g_selftest_start_cyc)) >= 2000000U) ||
                (!g_dwt_ok &&
                 (TickType_t)(now_tick - g_selftest_start_tick) >=
                     pdMS_TO_TICKS(2000U))) {
                if (g_selftest_slot < USBH_POLL_MAX_SLOTS) {
                    struct poll_slot *selftest_slot = &g_slots[g_selftest_slot];
                    size_t flags;
                    poll_critical_enter(&flags);
                    selftest_slot->selftest = false;
                    if (selftest_slot->state == POLL_INFLIGHT) {
                        selftest_slot->timeout_pending = true;
                        usbh_kill_urb(&selftest_slot->urb);
                    }
                    poll_critical_leave(flags);
                }
                g_selftest_state = USBH_POLL_SELFTEST_DONE;
                USB_LOG_INFO("[selftest] aborted n=%u\r\n", g_selftest_n);
                continue;
            }
            if (g_round_active) {
                poll_pump();
                continue;
            }
            if (g_inflight != 0) continue;
            if (g_selftest_slot < USBH_POLL_MAX_SLOTS && g_slots[g_selftest_slot].state == POLL_IDLE)
                poll_submit(&g_slots[g_selftest_slot]);
            continue;
        }
        if (g_round_active) {
            g_overrun++;
            poll_pump();
            continue;
        }
        g_round++;
        g_cursor = 0;
        g_round_scan = 0;
        g_pool_blocked = false;
        g_round_active = true;
        poll_pump();
    }
}

void usbh_poll_init(void)
{
    g_dwt_ok = poll_dwt_enable();
    if (!g_dwt_ok) {
        USB_LOG_INFO("[poll] DWT unavailable, timing disabled\r\n");
    }
    g_event_queue = xQueueCreate(8, sizeof(struct usbh_poll_event));
    xTaskCreate(poll_task, "UsbPoll", 256, NULL, tskIDLE_PRIORITY + 1, &g_poll_task);
}

int usbh_poll_register(struct usbh_winusb *winusb)
{
    uint8_t path[4] = { 0 };
    int i;
    struct poll_slot *s = NULL;
    if (!winusb || !winusb->hport) return -USB_ERR_INVAL;
    poll_path(winusb->hport, path);
    for (i = 0; i < USBH_POLL_MAX_SLOTS; i++) {
        if (g_slots[i].state == POLL_FREE && poll_same_path(&g_slots[i], path, winusb->hport->depth)) {
            s = &g_slots[i];
            break;
        }
    }
    if (!s) {
        for (i = 0; i < USBH_POLL_MAX_SLOTS; i++) {
            if (g_slots[i].state == POLL_FREE) { s = &g_slots[i]; break; }
        }
    }
    if (!s) return -USB_ERR_NOMEM;
    memset(s, 0, sizeof(*s));
    memcpy(s->path, path, sizeof(path));
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
    poll_event(s, USBH_POLL_EVENT_ATTACH, NULL, 0, 0, 0);
    if (g_selftest_state == USBH_POLL_SELFTEST_IDLE) {
        g_selftest_state = USBH_POLL_SELFTEST_RUNNING;
        g_selftest_slot = (uint8_t)(s - g_slots);
        g_selftest_reported = false;
        g_selftest_n = 0;
        g_selftest_start_cyc = poll_cycles();
        g_selftest_start_tick = xTaskGetTickCount();
        s->selftest = true;
        s->selftest_count = 0;
        USB_LOG_INFO("[selftest] start slot=%u count=%u\r\n",
                     (unsigned)(s - g_slots), USBH_POLL_SELFTEST_COUNT);
    }
    return (int)(s - g_slots);
}

int usbh_poll_first_slot(void)
{
    size_t flags;
    uint8_t i;
    int slot = -1;
    poll_critical_enter(&flags);
    for (i = 0; i < USBH_POLL_MAX_SLOTS; i++) {
        if (g_slots[i].state != POLL_FREE && g_slots[i].winusb != NULL) {
            slot = i;
            break;
        }
    }
    poll_critical_leave(flags);
    return slot;
}

struct usbh_winusb *usbh_poll_get_device(uint8_t slot)
{
    size_t flags;
    struct usbh_winusb *winusb = NULL;
    if (slot >= USBH_POLL_MAX_SLOTS) return NULL;
    poll_critical_enter(&flags);
    if (g_slots[slot].state != POLL_FREE && g_slots[slot].winusb != NULL) {
        winusb = g_slots[slot].winusb;
    }
    poll_critical_leave(flags);
    return winusb;
}

void usbh_poll_unregister(struct usbh_winusb *winusb)
{
    uint8_t i;
    for (i = 0; i < USBH_POLL_MAX_SLOTS; i++) {
        struct poll_slot *s = &g_slots[i];
        size_t flags;
        if (s->winusb == winusb) {
            poll_critical_enter(&flags);
            s->in_queue = false;
            if (s->state == POLL_INFLIGHT) {
                s->unregister_pending = true;
                usbh_kill_urb(&s->urb);
            }
            poll_critical_leave(flags);
            poll_drain_reports();
            if (s->state == POLL_IDLE) {
                poll_error_recover(s);
                poll_event(s, USBH_POLL_EVENT_DETACH, NULL, 0, 0, 0);
                s->winusb = NULL;
                s->hport = NULL;
                s->bulkin = NULL;
                s->state = POLL_FREE;
                if (g_selftest_state == USBH_POLL_SELFTEST_RUNNING &&
                    g_selftest_slot == i) {
                    g_selftest_state = USBH_POLL_SELFTEST_DONE;
                    g_selftest_slot = 0xff;
                }
            }
            return;
        }
    }
}

int usbh_poll_enable(uint8_t slot)
{
    size_t flags;
    if (slot >= USBH_POLL_MAX_SLOTS) return -USB_ERR_INVAL;
    poll_critical_enter(&flags);
    g_slots[slot].in_queue = true;
    poll_critical_leave(flags);
    return 0;
}

int usbh_poll_disable(uint8_t slot)
{
    size_t flags;
    if (slot >= USBH_POLL_MAX_SLOTS) return -USB_ERR_INVAL;
    poll_critical_enter(&flags);
    g_slots[slot].in_queue = false;
    poll_critical_leave(flags);
    return 0;
}

bool usbh_poll_is_enabled(uint8_t slot)
{
    return slot < USBH_POLL_MAX_SLOTS && g_slots[slot].in_queue;
}

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
    TickType_t timeout = timeout_ms == 0xffffffffU ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xQueueReceive(g_event_queue, ev, timeout) == pdTRUE ? 0 : -USB_ERR_TIMEOUT;
}

void usbh_poll_stats_dump(void)
{
    size_t flags;
    uint32_t busy_min;
    uint32_t busy_max;
    uint32_t busy_avg;
    uint32_t busy_max_all;
    uint32_t busy_count;
    uint64_t busy_sum;
    uint32_t evt_drop;
    poll_critical_enter(&flags);
    busy_min = g_busy_min == 0xffffffffU ? 0 : g_busy_min;
    busy_max = g_busy_max;
    busy_count = g_busy_count;
    busy_sum = g_busy_sum;
    busy_max_all = g_busy_max_all;
    evt_drop = g_evt_drop;
    g_busy_min = 0xffffffffU;
    g_busy_max = 0;
    g_busy_sum = 0;
    g_busy_count = 0;
    poll_critical_leave(flags);
    busy_avg = busy_count ? (uint32_t)(busy_sum / busy_count) : 0;
    poll_selftest_report();
    if (g_dwt_ok) {
        USB_LOG_INFO("[poll] round=%lu overrun=%lu scan=%lu busy_us(min/avg/max)=%lu/%lu/%lu max_all=%lu pool_empty=%lu evtq_high=%lu urbto=%lu evt_drop=%lu\r\n",
                     (unsigned long)g_round, (unsigned long)g_overrun, (unsigned long)g_scan_snapshot,
                     (unsigned long)busy_min, (unsigned long)busy_avg,
                     (unsigned long)busy_max, (unsigned long)busy_max_all,
                     (unsigned long)g_pool_empty, (unsigned long)g_evtq_high,
                     (unsigned long)g_timeout_count, (unsigned long)evt_drop);
    } else {
        USB_LOG_INFO("[poll] round=%lu overrun=%lu scan=%lu busy_us(min/avg/max)=na/na/na max_all=na pool_empty=%lu evtq_high=%lu urbto=%lu evt_drop=%lu\r\n",
                     (unsigned long)g_round, (unsigned long)g_overrun, (unsigned long)g_scan_snapshot,
                     (unsigned long)g_pool_empty, (unsigned long)g_evtq_high,
                     (unsigned long)g_timeout_count, (unsigned long)evt_drop);
    }
    for (uint8_t i = 0; i < USBH_POLL_MAX_SLOTS; i++) {
        if (g_slots[i].state != POLL_FREE) {
            if (g_dwt_ok) {
                USB_LOG_INFO("[slot%u] path=%u-%u-%u addr=%u vid=%04x pid=%04x rx=%lu nak=%lu zlp=%lu err=%lu err_streak=%lu max_us=%lu urbto=%lu evt_drop=%lu\r\n",
                             i, g_slots[i].path[0], g_slots[i].path[1], g_slots[i].path[2],
                             g_slots[i].dev_addr, g_slots[i].vid, g_slots[i].pid,
                             (unsigned long)g_slots[i].rx_packets,
                             (unsigned long)g_slots[i].nak_count,
                             (unsigned long)g_slots[i].zlp_count,
                             (unsigned long)g_slots[i].err_count,
                             (unsigned long)g_slots[i].err_streak,
                             (unsigned long)g_slots[i].max_us,
                             (unsigned long)g_slots[i].timeout_count,
                             (unsigned long)g_slots[i].evt_drop);
            } else {
                USB_LOG_INFO("[slot%u] path=%u-%u-%u addr=%u vid=%04x pid=%04x rx=%lu nak=%lu zlp=%lu err=%lu err_streak=%lu max_us=na urbto=%lu evt_drop=%lu\r\n",
                             i, g_slots[i].path[0], g_slots[i].path[1], g_slots[i].path[2],
                             g_slots[i].dev_addr, g_slots[i].vid, g_slots[i].pid,
                             (unsigned long)g_slots[i].rx_packets,
                             (unsigned long)g_slots[i].nak_count,
                             (unsigned long)g_slots[i].zlp_count,
                             (unsigned long)g_slots[i].err_count,
                             (unsigned long)g_slots[i].err_streak,
                             (unsigned long)g_slots[i].timeout_count,
                             (unsigned long)g_slots[i].evt_drop);
            }
        }
    }
}

__attribute__((weak)) void usbh_poll_on_event(const struct usbh_poll_event *ev)
{
    (void)ev;
}
