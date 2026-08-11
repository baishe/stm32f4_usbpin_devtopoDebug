#include "stm32f4xx_hal.h"
#include <stdarg.h>
#include <stdio.h>

extern UART_HandleTypeDef huart1;

#define USB_LOG_BUF_SIZE  4096

static uint8_t           s_log_buf[USB_LOG_BUF_SIZE];
static volatile uint16_t s_head = 0;
static volatile uint16_t s_tail = 0;
static volatile uint8_t  s_tx_busy = 0;

// 修复问题1：使用静态变量作为发送数据缓冲，确保其生命周期贯穿整个异步传输
static uint8_t           s_tx_byte = 0;

/* 临界区保护宏（可根据中断优先级调整） */
#define ENTER_CRITICAL()   __disable_irq()
#define EXIT_CRITICAL()    __enable_irq()

void usb_log_write(const char *fmt, ...)
{
    char tmp[256];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(tmp, sizeof(tmp), fmt, args);
    va_end(args);
    
//    HAL_UART_Transmit(&huart1, &tmp, len, 500);
//    return;

    // 避免 vsnprintf 返回长度大于缓冲区（截断情况）
    if (len >= (int)sizeof(tmp)) {
        len = sizeof(tmp) - 1;
    }

    // 写入环形缓冲区（临界区保护）
    ENTER_CRITICAL();
    for (int i = 0; i < len; i++) {
        uint16_t next = (s_head + 1) % USB_LOG_BUF_SIZE;
        if (next != s_tail) {               // 缓冲区未满
            s_log_buf[s_head] = (uint8_t)tmp[i];
            s_head = next;
        } else {
            // 缓冲区满，丢弃剩余数据（可添加错误计数）
            break;
        }
    }
    // 如果当前没有发送任务且缓冲区非空，则启动发送
    if (!s_tx_busy && (s_tail != s_head)) {
        s_tx_busy = 1;
        s_tx_byte = s_log_buf[s_tail];
        s_tail = (s_tail + 1) % USB_LOG_BUF_SIZE;
        // 注意：此处仍处于临界区，HAL 中断传输启动后即可退出
        HAL_UART_Transmit_IT(&huart1, &s_tx_byte, 1);
    }
    EXIT_CRITICAL();
}

/* UART 发送完成回调 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        // 进入临界区保护共享变量
        ENTER_CRITICAL();

        if (s_tail != s_head) {
            // 尚有数据待发送
            s_tx_byte = s_log_buf[s_tail];
            s_tail = (s_tail + 1) % USB_LOG_BUF_SIZE;
            // 启动下一字节发送（中断传输会在完成后再次回调）
            HAL_UART_Transmit_IT(&huart1, &s_tx_byte, 1);
        } else {
            // 缓冲区已空，标记发送空闲
            s_tx_busy = 0;
            // 问题6修复：在置空闲后，需防止新数据在此期间到来而未启动发送。
            // 由于我们处于临界区内，此时不可能有 usb_log_write 打断，
            // 但退出临界区后可能有中断抢占，因此这里无需额外操作。
            // 更稳健的做法：在 usb_log_write 中每次都会检查 !s_tx_busy 并启动，
            // 若新数据在退出临界区后到达，会由写函数自行触发。
            // 但为了保险，可在此再次检查 s_tail != s_head（但理论上不可能，因为已空）。
            // 若担心由于其他中断导致写操作在此期间插入，由于我们关中断，不会发生。
        }

        EXIT_CRITICAL();
    }
}