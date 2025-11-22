#ifndef __TOF_CFG_H_
#define __TOF_CFG_H_

#include "usart.h"

// =============================================================================
// TOF 测距模块 UART 配置
// =============================================================================

// 定义 TOF 模块使用的 UART 句柄
// 如果需要更改 UART 端口，请修改此处，并确保在 CubeMX 中配置了相应的 UART
#define TOF_UART_HANDLE     huart4
#define TOF_UART_INSTANCE   UART4

// 注意：
// 1. 更改上述宏定义后，请确保 usart.h 中有对应的句柄声明（例如 extern UART_HandleTypeDef huart4;）。
// 2. 具体的 GPIO 引脚配置（TX/RX）在 Core/Src/usart.c 的 HAL_UART_MspInit 函数中。
//    如果更改了 UART 号，请务必检查并修改对应的 GPIO 初始化代码。
//    例如 UART4 通常对应 PA0/PA1 或 PC10/PC11，具体取决于硬件连接。

// =============================================================================
// 其他配置
// =============================================================================

// 自动读取周期 (ms)
#define TOF_READ_PERIOD_MS  100

// Modbus 协议相关
#define TOF_DEV_ADDR        0x00    // 设备地址 (0x00 为广播，或根据实际设置修改)
#define TOF_REG_ADDR        0x0010  // 距离寄存器地址

#endif // __TOF_CFG_H_
