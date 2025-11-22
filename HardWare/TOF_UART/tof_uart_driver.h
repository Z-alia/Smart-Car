#ifndef __TOF_UART_DRIVER_H__
#define __TOF_UART_DRIVER_H__

#include "tof_type.h"

// Initialize the TOF UART driver
void TOF_UART_Driver_Init(void);

// Main loop task for TOF UART driver
void TOF_UART_Driver_Task(void);

// Call this from HAL_UART_RxCpltCallback in main.c or usart.c
void TOF_UART_RxCpltCallback(void *huart);

// Function to get the latest distance reading
uint16_t TOF_UART_GetDistance(void);

// Function to set the device address using broadcast (0x00)
// new_addr: The new address to assign (e.g., 0x01)
void TOF_UART_SetAddress(uint8_t new_addr);

typedef struct {
    volatile uint32_t tx_cnt;           // 发送次数
    volatile uint32_t rx_isr_cnt;       // 接收中断次数 (收到字节数)
    volatile uint8_t  last_byte;        // 最后收到的字节
    volatile uint32_t frame_process_cnt;// 尝试解析帧的次数 (超时触发)
    volatile uint32_t crc_err_cnt;      // CRC 校验失败次数
    volatile uint32_t valid_frame_cnt;  // 成功解析帧次数
    volatile uint8_t  uart_init_ok;     // UART接收中断是否成功启动
    volatile uint32_t tx_hal_err;       // HAL发送错误次数
    volatile uint8_t  raw_data[10];     // 最后一帧原始数据
    volatile uint8_t  raw_len;          // 最后一帧长度
    volatile uint32_t rx_timeout_cnt;   // 接收超时次数（发送后3ms内无回复）
    volatile uint8_t  sent_data[10];    // 最后发送的数据（含CRC）
    volatile uint8_t  sent_len;         // 最后发送的数据长度
} TOF_Debug_t;

extern TOF_Debug_t g_tof_debug;

#endif // __TOF_UART_DRIVER_H__
