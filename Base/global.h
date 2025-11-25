/**
 * @file global.h
 * @brief 全局头文件 - 包含所有常用依赖
 * @note 在需要使用多个模块的文件中包含此头文件即可
 */

#ifndef __GLOBAL_H
#define __GLOBAL_H

#ifdef __cplusplus
extern "C" {
#endif

// ==================== STM32 HAL库 ====================
#include "main.h"
#include "dcmi.h"
#include "dma.h"
#include "i2c.h"
#include "quadspi.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

// ==================== 标准库 ====================
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

// ==================== 硬件驱动 ====================
// 显示屏
#include "lcd_spi_200.h"
#include "lcd_fonts.h"
#include "lcd_image.h"

// 摄像头
#include "dcmi_ov2640.h"
#include "sccb.h"

// 陀螺仪
#include "ICM-42688P.h"
#include "ICM42688P_Config.h"

// 电机
#include "motor.h"

// ==================== 图像处理 ====================
#include "Binarization.h"
#include "scan_line.h"
#include "Element_recognition.h"

// ==================== 控制系统 ====================
#include "integral.h"
#include "open_loop_pid.h"
#include "system_config.h"

// ==================== 元素识别模块 ====================
#include "red_obstacle.h"
#include "red_obstacle_navigation.h"
#include "circle.h"
#include "cross.h"
#include "slope.h"
#include "zebra.h"
#include "patch_line.h"
// 可以继续添加其他元素模块

// ==================== 全局类型定义 ====================
// 可以在这里添加通用的类型定义

// ==================== 全局宏定义 ====================
// 可以在这里添加通用的宏定义

#ifdef __cplusplus
}
#endif

#endif /* __GLOBAL_H */
