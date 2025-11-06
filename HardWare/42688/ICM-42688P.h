/**
 ******************************************************************************
 * @file    ICM-42688P.h
 * @brief   ICM-42688P IMU传感器驱动头文件
 * @note    该文件定义了ICM-42688P IMU传感器的驱动接口，包括寄存器操作、
 *          数据读取、配置设置等功能。
 ******************************************************************************
 */

#ifndef ICM42688P_H
#define ICM42688P_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"  // 需要HAL_GPIO_WritePin等HAL库函数
#include "math.h"  // 需要fabs等数学函数
#include "float.h" // 需要FLT_EPSILON等浮点常量

/* Exported constants --------------------------------------------------------*/

/**
 * @brief 片选信号置高 (SPI4硬件NSS: PE11)
 * @note  使用硬件NSS时，HAL库会自动控制片选信号
 *        这些宏保留用于需要手动控制的场景
 */
#define cs_high() HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET)

/**
 * @brief 片选信号置低 (SPI4硬件NSS: PE11)
 */
#define cs_low() HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET)

/**
 * @brief 陀螺仪全量程，设置为2000dps
 */
#define GYRO_FULL_SCALE 2000.0

/**
 * @brief 陀螺仪灵敏度，16位分辨率
 */
#define GYRO_SENSITIVITY (GYRO_FULL_SCALE / 32768.0)

/**
 * @brief 加速度计全量程，设置为16g
 */
#define ACCEL_FULL_SCALE 16.0

/**
 * @brief 加速度计灵敏度，16位分辨率
 */
#define ACCEL_SENSITIVITY (ACCEL_FULL_SCALE / 32768.0)

/**
 * @brief 零偏校准参数 - IMU2
 */
#define axzeroffset 0.0f
#define ayzeroffset 0.0f
#define azzeroffset 0.0f
#define gxzeroffset 0.0f
#define gyzeroffset 0.0f
#define gzzeroffset 0.0f

/**
 * @brief SPI读取寄存器标志位
 */
#define ICM42688P_READ 0x80

/**
 * @brief WHOAMI寄存器地址
 */
#define ICM42688P_WHOAMI 0x75

/* Exported types ------------------------------------------------------------*/

/**
 * @brief IMU数据结构体
 *
 * 包含加速度计、陀螺仪、四元数和时间戳数据
 */
typedef struct
{
    /** 加速度计数据，单位为g（重力加速度） */
    float accel_x; // X轴加速度
    float accel_y; // Y轴加速度
    float accel_z; // Z轴加速度

    /** 陀螺仪数据，单位为度每秒（dps） */
    float gyro_x; // X轴角速度
    float gyro_y; // Y轴角速度
    float gyro_z; // Z轴角速度

    /** 四元数，用于表示设备的旋转状态 */
    float q0; // q0（四元数实部）
    float q1; // q1
    float q2; // q2
    float q3; // q3
    
    /** 温度数据，单位为摄氏度 */
    float temperature;

    /** 时间戳，用于记录系统时间或某个标准时间的数据 */
    uint16_t timestamp;

} IMU_Data;

extern IMU_Data imu_data;//数据接口
/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 初始化ICM42688P传感器（优化版）
 * @return 0=成功, 1=WHOAMI读取失败, 2=配置后通信失败, 3=传感器未正确启动
 * 
 * 优化的初始化流程包含13个步骤和完整的错误检测：
 * 1. SPI使能检查
 * 2. 上电稳定延时(150ms)
 * 3. Bank 0选择
 * 4. WHOAMI读取验证
 * 5. 如需要则软件复位
 * 6. 复位后重试(最多5次)
 * 7. 传感器配置开始
 * 8. 时钟配置
 * 9. 中断配置
 * 10. 输出数据速率配置
 * 11. 启动传感器
 * 12. 最终WHOAMI验证
 * 13. PWR_MGMT0状态验证
 * 
 * 所有延时使用软件循环实现,避免HAL_Delay在特殊环境下的问题
 */
uint8_t ICM42688P_Init(void);

/**
 * @brief 最小化测试函数 - 只测试 SPI 通信
 * @return 读取到的 WHOAMI 值
 * 
 * 这个函数只做最基础的 SPI 读取操作，用于测试 SPI 是否工作
 */
uint8_t ICM42688P_Test_MinimalRead(void);

/**
 * @brief 停止ICM42688P传感器
 */
void ICM42688P_Stop(void);

/**
 * @brief 启动ICM42688P传感器
 */
void ICM42688P_Start(void);

/**
 * @brief 配置输出数据速率(ODR)
 */
void ICM42688P_ODR_Config(void);

/**
 * @brief 配置中断
 */
void ICM42688P_Interrupt_Config(void);

/**
 * @brief 配置时钟
 */
void ICM42688P_Clock_Config(void);

/**
 * @brief 软件复位
 */
void ICM42688P_Software_Reset(void);

/**
 * @brief 读取IMU数据（包含温度）
 * @param data 指向IMU_Data结构体的指针，用于存储读取的数据
 * 
 * 从0x1D开始一次性读取温度和IMU数据，提高效率
 */
void ICM42688P_ReadIMUData(IMU_Data *data);

/**
 * @brief 选择寄存器组
 * @param bank 寄存器组编号
 */
void ICM42688P_Bank_Select(uint8_t bank);

/**
 * @brief 读取寄存器
 * @param reg_address 寄存器地址
 * @param rxdata 接收数据缓冲区
 * @param length 数据长度
 */
void ICM42688P_ReadRegister(uint8_t reg_address, uint8_t *rxdata, uint8_t length);

/**
 * @brief SPI 安全读取函数
 * @param reg_address 寄存器地址
 * @param rxdata 接收数据缓冲区
 * @param length 数据长度
 * @return 0=成功, 1=失败
 */
uint8_t ICM42688P_SafeRead(uint8_t reg_address, uint8_t *rxdata, uint8_t length);

/**
 * @brief 手动 SPI 读取寄存器（不依赖 HAL 库超时）
 * @param reg_address 寄存器地址
 * @param rxdata 接收数据缓冲区
 * @param length 数据长度
 * @return 0=成功, 1=失败
 */
uint8_t ICM42688P_ManualRead(uint8_t reg_address, uint8_t *rxdata, uint8_t length);

/**
 * @brief 写入寄存器
 * @param reg_address 寄存器地址
 * @param txdata 发送数据缓冲区
 * @param length 数据长度
 * @return 0表示成功，1表示失败
 */
uint8_t ICM42688P_WriteRegister(uint8_t reg_address, uint8_t *txdata, uint8_t length);

// 测试函数
uint8_t ICM42688P_Test_MinimalRead(void);
void ICM42688P_SPI_Diagnostic(void);  // SPI 通信诊断
uint8_t ICM42688P_Test_DirectRead(void);  // 直接读取测试

/**
 * @brief ICM42688P 传感器检测和显示函数
 * @note 该函数执行完整的ICM检测流程并在LCD上显示结果和实时数据
 * 
 * 功能包括：
 * 1. 硬件连接检测（CS引脚、SPI状态）
 * 2. 传感器初始化
 * 3. WHOAMI验证
 * 4. 实时显示IMU数据（加速度、角速度、温度）
 * 5. 数据刷新率显示
 */
void ICM42688P_DetectAndDisplay(void);

/**
 * @brief ICM42688P 快速状态检测（用于调试）
 * @return 0=正常, 1=WHOAMI错误, 2=传感器未启动
 * 
 * 该函数快速检测ICM传感器状态，不初始化，只读取关键寄存器
 */
uint8_t ICM42688P_QuickCheck(void);

/**
 * @brief 在LCD上显示简化的IMU数据（适合集成到主循环）
 * @param x 显示起始X坐标
 * @param y 显示起始Y坐标
 * 
 * 该函数显示紧凑的IMU数据，适合在主循环中周期性调用
 */
void ICM42688P_DisplayCompact(uint16_t x, uint16_t y);

#ifdef __cplusplus
}
#endif

#endif /* ICM42688P_H */

#ifdef __cplusplus
}
#endif

//#endif /* ICM42688P_H */