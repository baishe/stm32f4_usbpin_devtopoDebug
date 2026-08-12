#include "stm32f4xx_hal.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>

extern UART_HandleTypeDef huart1;
#define USB_LOG_BUF_SIZE 4096
static uint8_t s_log_buf[USB_LOG_BUF_SIZE];
static volatile uint16_t s_head;
static volatile uint16_t s_tail;
static volatile uint16_t s_tx_len;
static volatile uint8_t s_tx_busy;

#define ENTER_CRITICAL() __disable_irq()
#define EXIT_CRITICAL() __enable_irq()

static void usb_log_kick(void)
{
    uint16_t len;
    uint16_t contiguous;
    if (s_tx_busy || s_tail == s_head) return;
    contiguous = (s_head > s_tail) ? (s_head - s_tail) : (USB_LOG_BUF_SIZE - s_tail);
    len = contiguous;
    s_tx_len = len;
    s_tx_busy = 1;
    (void)HAL_UART_Transmit_IT(&huart1, &s_log_buf[s_tail], len);
}

void usb_log_write(const char *fmt, ...)
{
    char tmp[256];
    va_list args;
    int len, i;
    va_start(args, fmt);
    len = vsnprintf(tmp, sizeof(tmp), fmt, args);
    va_end(args);
    if (len >= (int)sizeof(tmp)) len = sizeof(tmp) - 1;
    ENTER_CRITICAL();
    for (i = 0; i < len; i++) {
        uint16_t next = (uint16_t)((s_head + 1U) % USB_LOG_BUF_SIZE);
        if (next == s_tail) break;
        s_log_buf[s_head] = (uint8_t)tmp[i];
        s_head = next;
    }
    usb_log_kick();
    EXIT_CRITICAL();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) return;
    ENTER_CRITICAL();
    s_tail = (uint16_t)((s_tail + s_tx_len) % USB_LOG_BUF_SIZE);
    s_tx_len = 0;
    s_tx_busy = 0;
    usb_log_kick();
    EXIT_CRITICAL();
}
