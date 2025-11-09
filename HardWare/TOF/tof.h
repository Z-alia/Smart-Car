/**
  ******************************************************************************
  * @file    tof.h
  * @author  kevin_guo
  * @version V2.0.0
  * @date    2025-11-09
  * @brief   TOF sensor I2C driver for STM32H750
  ******************************************************************************
  * @attention
  ******************************************************************************  
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TOF_H
#define __TOF_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* User Configuration --------------------------------------------------------*/
// 配置TOF传感器使用的GPIO端口和引脚
#define TOF_GPIO_PORT           GPIOB
#define TOF_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOB_CLK_ENABLE()
#define TOF_SCL_PIN             GPIO_PIN_10
#define TOF_SDA_PIN             GPIO_PIN_11

// TOF传感器I2C设备地址
#define TOF_ADDR            0xA4

// 内部辅助宏：根据GPIO_PIN_x获取引脚编号
#define TOF_PIN_NUM_SCL         10   // 如果SCL引脚改变，这里需要同步修改
#define TOF_PIN_NUM_SDA         11   // 如果SDA引脚改变，这里需要同步修改

/* Exported macros -----------------------------------------------------------*/
//IO方向设置 - STM32H7使用MODER寄存器配置GPIO模式
#define TOF_SDA_IN()  do { \
    TOF_GPIO_PORT->MODER &= ~(3 << (TOF_PIN_NUM_SDA*2)); \
    TOF_GPIO_PORT->MODER |= (0 << (TOF_PIN_NUM_SDA*2)); \
} while(0)

#define TOF_SDA_OUT() do { \
    TOF_GPIO_PORT->MODER &= ~(3 << (TOF_PIN_NUM_SDA*2)); \
    TOF_GPIO_PORT->MODER |= (1 << (TOF_PIN_NUM_SDA*2)); \
} while(0)

//IO操作函数
#define TOF_SCL_HIGH()          HAL_GPIO_WritePin(TOF_GPIO_PORT, TOF_SCL_PIN, GPIO_PIN_SET)
#define TOF_SCL_LOW()           HAL_GPIO_WritePin(TOF_GPIO_PORT, TOF_SCL_PIN, GPIO_PIN_RESET)
#define TOF_SDA_HIGH()          HAL_GPIO_WritePin(TOF_GPIO_PORT, TOF_SDA_PIN, GPIO_PIN_SET)
#define TOF_SDA_LOW()           HAL_GPIO_WritePin(TOF_GPIO_PORT, TOF_SDA_PIN, GPIO_PIN_RESET)
#define TOF_READ_SDA()          HAL_GPIO_ReadPin(TOF_GPIO_PORT, TOF_SDA_PIN)
/* TOF Register Addresses ----------------------------------------------------*/
#define TOF_REG_DISTANCE_REAL       0x00    // 实时距离 (2字节, 只读)
#define TOF_REG_DISTANCE_FILTER     0x02    // 滤波距离 (4字节, 只读)
#define TOF_REG_OFFSET              0x06    // offset偏移 (2字节, 读写, -99~99mm)
#define TOF_REG_XTALK               0x0A    // xtalk串扰校准 (1字节, 读写, 0-200)
#define TOF_REG_OUTPUT_MODE         0x0B    // 输出模式 (1字节, 读写, 0-滤波 1-实时)
#define TOF_REG_TRIGGER_MODE        0x0C    // 触发方式 (1字节, 读写, 0-连续 1-主机读取)
#define TOF_REG_INTERVAL_TIME       0x0D    // 间隔判断时间 (2字节, 读写, 10-9999ms)
#define TOF_REG_DISTANCE_LIMIT      0x0E    // 距离上下限 (2字节, 读写, 10-1800mm)
#define TOF_REG_I2C_SLAVE_ID        0x10    // I2C从机ID (2字节, 读写, 0x0001-0x00FE)

/* Exported functions ------------------------------------------------------- */
// 初始化
void TOF_Init(void);

// 距离读取
unsigned short TOF_ReadDistance(void);           // 读取实时距离 (10-1800mm)
unsigned short TOF_ReadDistanceFiltered(void);   // 读取滤波距离 (更稳定)

// 配置功能
unsigned char TOF_SetOutputMode(unsigned char mode);        // 0=滤波 1=实时
unsigned char TOF_SetTriggerMode(unsigned char mode);       // 0=连续 1=主机触发
unsigned char TOF_SetOffset(short offset);                  // 偏移校准 (-99~99mm)
unsigned char TOF_SetXtalk(unsigned char xtalk);            // 串扰校准 (0-200)
unsigned char TOF_SetIntervalTime(unsigned short time_ms);  // 间隔时间 (10-9999ms)
unsigned char TOF_SetI2CAddress(unsigned short new_addr);   // 修改I2C地址

// 底层寄存器访问
unsigned char TOF_WriteNByte(unsigned char *txbuff, unsigned char regaddr, unsigned char size);
unsigned char TOF_ReadNByte(unsigned char *rxbuff, unsigned char regaddr, unsigned char size);
#endif /* __TOF_H */

/************************END OF FILE*************************/

