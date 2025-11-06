/**
 * @file ICM42688P_Simple.h
 * @brief ICM-42688-P 简化驱动头文件
 * @date 2025-11-06
 */

#ifndef ICM42688P_SIMPLE_H
#define ICM42688P_SIMPLE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "ICM-42688P.h"  // 使用已有的IMU_Data结构体定义

/**
 * @brief ICM-42688-P 简化初始化
 * @return 0=成功, 1=WHOAMI错误, 3=传感器未启动
 * 
 * 该函数执行最简化的初始化流程：
 * - 验证WHOAMI
 * - 软件复位
 * - 配置±2000dps陀螺仪，±16g加速度计
 * - ODR=1kHz
 * - 启动传感器
 */
uint8_t ICM42688P_Simple_Init(void);

/**
 * @brief 读取六轴数据
 * @param data 指向IMU_Data结构体
 * @return 0=成功, 非0=失败
 * 
 * 读取温度、加速度和陀螺仪数据
 */
uint8_t ICM42688P_Simple_ReadData(IMU_Data *data);

/**
 * @brief 读取WHOAMI寄存器
 * @return WHOAMI值（应为0x47）
 */
uint8_t ICM42688P_Simple_ReadWHOAMI(void);

/**
 * @brief 快速检查传感器状态
 * @return 0=正常, 1=WHOAMI错误, 2=传感器未启动
 */
uint8_t ICM42688P_Simple_QuickCheck(void);

/**
 * @brief 打印传感器配置信息（需要printf支持）
 */
void ICM42688P_Simple_PrintConfig(void);

#ifdef __cplusplus
}
#endif

#endif /* ICM42688P_SIMPLE_H */
