/**
 * @file ICM42688P_Simple.c
 * @brief ICM-42688-P 简化驱动 - 基于官方寄存器映射
 * 
 * 参考资料：
 * - ICM-42688-P Datasheet (DS-000347 v1.8)
 * - TDK InvenSense 官方文档
 * 
 * 该驱动专注于最基本的六轴数据读取功能
 * 
 * @date 2025-11-06
 */

#include "ICM-42688P.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// 外部SPI句柄
extern SPI_HandleTypeDef hspi4;

/* ============================================================
 * ICM-42688-P 寄存器地址定义（来自官方数据手册）
 * ============================================================ */

// Bank 0 寄存器
#define REG_DEVICE_CONFIG         0x11   // 设备配置
#define REG_DRIVE_CONFIG          0x13   // I2C/SPI驱动配置
#define REG_INT_CONFIG            0x14   // 中断配置
#define REG_FIFO_CONFIG           0x16   // FIFO配置
#define REG_TEMP_DATA1            0x1D   // 温度数据高字节
#define REG_TEMP_DATA0            0x1E   // 温度数据低字节
#define REG_ACCEL_DATA_X1         0x1F   // 加速度X高字节
#define REG_ACCEL_DATA_X0         0x20   // 加速度X低字节
#define REG_ACCEL_DATA_Y1         0x21   // 加速度Y高字节
#define REG_ACCEL_DATA_Y0         0x22   // 加速度Y低字节
#define REG_ACCEL_DATA_Z1         0x23   // 加速度Z高字节
#define REG_ACCEL_DATA_Z0         0x24   // 加速度Z低字节
#define REG_GYRO_DATA_X1          0x25   // 陀螺仪X高字节
#define REG_GYRO_DATA_X0          0x26   // 陀螺仪X低字节
#define REG_GYRO_DATA_Y1          0x27   // 陀螺仪Y高字节
#define REG_GYRO_DATA_Y0          0x28   // 陀螺仪Y低字节
#define REG_GYRO_DATA_Z1          0x29   // 陀螺仪Z高字节
#define REG_GYRO_DATA_Z0          0x2A   // 陀螺仪Z低字节
#define REG_INT_STATUS            0x2D   // 中断状态
#define REG_PWR_MGMT0             0x4E   // 电源管理0
#define REG_GYRO_CONFIG0          0x4F   // 陀螺仪配置0（ODR和FS）
#define REG_ACCEL_CONFIG0         0x50   // 加速度计配置0（ODR和FS）
#define REG_GYRO_CONFIG1          0x51   // 陀螺仪配置1（滤波器）
#define REG_GYRO_ACCEL_CONFIG0    0x52   // 陀螺仪加速度计配置0
#define REG_ACCEL_CONFIG1         0x53   // 加速度计配置1（滤波器）
#define REG_INT_CONFIG0           0x63   // 中断配置0
#define REG_INT_CONFIG1           0x64   // 中断配置1
#define REG_INT_SOURCE0           0x65   // 中断源0
#define REG_WHOAMI                0x75   // WHO AM I
#define REG_BANK_SEL              0x76   // Bank选择

// Bank 1 寄存器
#define REG_INTF_CONFIG4          0x7A   // 接口配置4
#define REG_INTF_CONFIG5          0x7B   // 接口配置5

/* ============================================================
 * 配置值定义
 * ============================================================ */

// PWR_MGMT0 配置（0x4E）
#define PWR_TEMP_ENABLE           (1 << 5)  // 使能温度传感器
#define PWR_GYRO_MODE_LN          (3 << 2)  // 陀螺仪低噪声模式
#define PWR_ACCEL_MODE_LN         (3 << 0)  // 加速度计低噪声模式
#define PWR_MGMT0_FULL_ON         (PWR_TEMP_ENABLE | PWR_GYRO_MODE_LN | PWR_ACCEL_MODE_LN)  // 0x2F

// GYRO_CONFIG0 配置（0x4F） - 陀螺仪ODR和量程
#define GYRO_ODR_1KHZ             0x06      // ODR = 1kHz
#define GYRO_FS_2000DPS           0x00      // ±2000 dps
#define GYRO_FS_1000DPS           0x01      // ±1000 dps
#define GYRO_FS_500DPS            0x02      // ±500 dps
#define GYRO_FS_250DPS            0x03      // ±250 dps
#define GYRO_CONFIG0_1K_2000DPS   ((GYRO_FS_2000DPS << 5) | GYRO_ODR_1KHZ)

// ACCEL_CONFIG0 配置（0x50） - 加速度计ODR和量程
#define ACCEL_ODR_1KHZ            0x06      // ODR = 1kHz
#define ACCEL_FS_16G              0x00      // ±16g
#define ACCEL_FS_8G               0x01      // ±8g
#define ACCEL_FS_4G               0x02      // ±4g
#define ACCEL_FS_2G               0x03      // ±2g
#define ACCEL_CONFIG0_1K_16G      ((ACCEL_FS_16G << 5) | ACCEL_ODR_1KHZ)

// 滤波器配置
#define GYRO_UI_FILT_BW_LOW_LATENCY   0x0E  // 低延迟滤波器
#define ACCEL_UI_FILT_BW_LOW_LATENCY  0x0E  // 低延迟滤波器

// DEVICE_CONFIG 配置（0x11）
#define SOFT_RESET_CONFIG         0x01      // 软件复位位

// 灵敏度定义（根据量程）
#define GYRO_SENSITIVITY_2000DPS  16.4f     // LSB/(dps) for ±2000dps
#define ACCEL_SENSITIVITY_16G     2048.0f   // LSB/g for ±16g

/* ============================================================
 * 简化的SPI读写函数
 * ============================================================ */

/**
 * @brief 简化SPI读取 - 直接操作寄存器
 */
static uint8_t Simple_SPI_ReadReg(uint8_t reg_addr, uint8_t *data, uint8_t len)
{
    cs_low();
    HAL_Delay(1);
    
    // 发送寄存器地址（带读标志）
    uint8_t tx_addr = reg_addr | 0x80;
    HAL_SPI_Transmit(&hspi4, &tx_addr, 1, 50);
    
    // 读取数据
    HAL_SPI_Receive(&hspi4, data, len, 50);
    
    cs_high();
    HAL_Delay(1);
    
    return 0;
}

/**
 * @brief 简化SPI写入 - 直接操作寄存器
 */
static uint8_t Simple_SPI_WriteReg(uint8_t reg_addr, uint8_t data)
{
    cs_low();
    HAL_Delay(1);
    
    uint8_t tx_buf[2] = {reg_addr, data};
    HAL_SPI_Transmit(&hspi4, tx_buf, 2, 50);
    
    cs_high();
    HAL_Delay(2);  // 写入后需要更长延时
    
    return 0;
}

/**
 * @brief Bank选择
 */
static void Simple_Bank_Select(uint8_t bank)
{
    Simple_SPI_WriteReg(REG_BANK_SEL, bank);
    HAL_Delay(1);  // Bank切换延时
}

/* ============================================================
 * 简化的初始化函数
 * ============================================================ */

/**
 * @brief ICM-42688-P 简化初始化
 * @return 0=成功, 非0=失败
 * 
 * 基于官方推荐的初始化序列：
 * 1. 读取WHOAMI验证
 * 2. 软件复位
 * 3. 等待启动完成
 * 4. 配置电源管理
 * 5. 配置陀螺仪和加速度计
 * 6. 启用传感器
 */
uint8_t ICM42688P_Simple_Init(void)
{
    uint8_t whoami = 0;
    
    // 1. 确保在Bank 0
    Simple_Bank_Select(0);
    HAL_Delay(10);
    
    // 2. 读取WHOAMI验证通信
    Simple_SPI_ReadReg(REG_WHOAMI, &whoami, 1);
    
    if(whoami != 0x47) {
        // WHOAMI错误，尝试复位后重试
        Simple_SPI_WriteReg(REG_DEVICE_CONFIG, SOFT_RESET_CONFIG);
        HAL_Delay(100);  // 复位后等待
        
        // 重新读取
        Simple_Bank_Select(0);
        HAL_Delay(10);
        Simple_SPI_ReadReg(REG_WHOAMI, &whoami, 1);
        
        if(whoami != 0x47) {
            return 1;  // WHOAMI验证失败
        }
    }
    
    // 3. 软件复位（确保干净状态）
    Simple_SPI_WriteReg(REG_DEVICE_CONFIG, SOFT_RESET_CONFIG);
    HAL_Delay(100);  // 等待复位完成（数据手册建议1ms，这里保守100ms）
    
    // 4. 确保在Bank 0
    Simple_Bank_Select(0);
    HAL_Delay(10);
    
    // 5. 配置陀螺仪：±2000dps, 1kHz ODR
    Simple_SPI_WriteReg(REG_GYRO_CONFIG0, GYRO_CONFIG0_1K_2000DPS);
    HAL_Delay(1);
    
    // 6. 配置加速度计：±16g, 1kHz ODR
    Simple_SPI_WriteReg(REG_ACCEL_CONFIG0, ACCEL_CONFIG0_1K_16G);
    HAL_Delay(1);
    
    // 7. 配置滤波器
    Simple_SPI_WriteReg(REG_GYRO_CONFIG1, GYRO_UI_FILT_BW_LOW_LATENCY);
    HAL_Delay(1);
    Simple_SPI_WriteReg(REG_ACCEL_CONFIG1, ACCEL_UI_FILT_BW_LOW_LATENCY);
    HAL_Delay(1);
    
    // 8. 启动传感器：温度+陀螺仪+加速度计都进入低噪声模式
    Simple_SPI_WriteReg(REG_PWR_MGMT0, PWR_MGMT0_FULL_ON);
    HAL_Delay(50);  // 等待传感器启动（数据手册建议30ms）
    
    // 9. 验证传感器已启动
    uint8_t pwr_status = 0;
    Simple_SPI_ReadReg(REG_PWR_MGMT0, &pwr_status, 1);
    
    // 检查是否正确配置（0x2F = 0b00101111）
    if((pwr_status & 0x0F) != 0x0F) {
        return 3;  // 传感器未正确启动
    }
    
    // 10. 等待数据稳定
    HAL_Delay(50);
    
    return 0;  // 初始化成功
}

/* ============================================================
 * 简化的数据读取函数
 * ============================================================ */

/**
 * @brief 读取原始传感器数据
 * @param data 指向IMU_Data结构体
 * @return 0=成功
 * 
 * 从0x1D开始连续读取14字节：
 * - 0x1D-0x1E: 温度 (2字节)
 * - 0x1F-0x24: 加速度 XYZ (6字节)
 * - 0x25-0x2A: 陀螺仪 XYZ (6字节)
 */
uint8_t ICM42688P_Simple_ReadData(IMU_Data *data)
{
    uint8_t raw_data[14] = {0};
    
    // 1. 确保在Bank 0
    static uint8_t bank_selected = 0;
    if(!bank_selected) {
        Simple_Bank_Select(0);
        HAL_Delay(1);
        bank_selected = 1;
    }
    
    // 2. 从0x1D开始读取14字节
    if(Simple_SPI_ReadReg(REG_TEMP_DATA1, raw_data, 14) != 0) {
        return 1;  // 读取失败
    }
    
    // 3. 解析温度数据 (16位有符号，高字节在前)
    int16_t temp_raw = (int16_t)((raw_data[0] << 8) | raw_data[1]);
    
    // 温度公式：Temperature = (TEMP_DATA / 132.48) + 25°C
    if(temp_raw == 0 || temp_raw == (int16_t)0x8000) {
        data->temperature = 25.0f;  // 无效值，使用默认
    } else {
        data->temperature = ((float)temp_raw / 132.48f) + 25.0f;
    }
    
    // 4. 解析加速度数据 (16位有符号，高字节在前)
    int16_t accel_raw[3];
    accel_raw[0] = (int16_t)((raw_data[2] << 8) | raw_data[3]);   // X
    accel_raw[1] = (int16_t)((raw_data[4] << 8) | raw_data[5]);   // Y
    accel_raw[2] = (int16_t)((raw_data[6] << 8) | raw_data[7]);   // Z
    
    // 转换为g（±16g量程）
    data->accel_x = (float)accel_raw[0] / ACCEL_SENSITIVITY_16G;
    data->accel_y = (float)accel_raw[1] / ACCEL_SENSITIVITY_16G;
    data->accel_z = (float)accel_raw[2] / ACCEL_SENSITIVITY_16G;
    
    // 5. 解析陀螺仪数据 (16位有符号，高字节在前)
    int16_t gyro_raw[3];
    gyro_raw[0] = (int16_t)((raw_data[8] << 8) | raw_data[9]);    // X
    gyro_raw[1] = (int16_t)((raw_data[10] << 8) | raw_data[11]);  // Y
    gyro_raw[2] = (int16_t)((raw_data[12] << 8) | raw_data[13]);  // Z
    
    // 转换为dps（±2000dps量程）
    data->gyro_x = (float)gyro_raw[0] / GYRO_SENSITIVITY_2000DPS;
    data->gyro_y = (float)gyro_raw[1] / GYRO_SENSITIVITY_2000DPS;
    data->gyro_z = (float)gyro_raw[2] / GYRO_SENSITIVITY_2000DPS;
    
    return 0;
}

/**
 * @brief 读取WHOAMI寄存器
 * @return WHOAMI值（应为0x47）
 */
uint8_t ICM42688P_Simple_ReadWHOAMI(void)
{
    uint8_t whoami = 0;
    Simple_Bank_Select(0);
    HAL_Delay(1);
    Simple_SPI_ReadReg(REG_WHOAMI, &whoami, 1);
    return whoami;
}

/**
 * @brief 快速检查传感器状态
 * @return 0=正常, 非0=异常
 */
uint8_t ICM42688P_Simple_QuickCheck(void)
{
    // 检查WHOAMI
    if(ICM42688P_Simple_ReadWHOAMI() != 0x47) {
        return 1;
    }
    
    // 检查电源管理状态
    uint8_t pwr = 0;
    Simple_SPI_ReadReg(REG_PWR_MGMT0, &pwr, 1);
    if((pwr & 0x0F) != 0x0F) {
        return 2;
    }
    
    return 0;
}

/**
 * @brief 打印传感器配置信息
 */
void ICM42688P_Simple_PrintConfig(void)
{
    uint8_t whoami, pwr, gyro_cfg, accel_cfg;
    
    Simple_Bank_Select(0);
    HAL_Delay(1);
    
    Simple_SPI_ReadReg(REG_WHOAMI, &whoami, 1);
    Simple_SPI_ReadReg(REG_PWR_MGMT0, &pwr, 1);
    Simple_SPI_ReadReg(REG_GYRO_CONFIG0, &gyro_cfg, 1);
    Simple_SPI_ReadReg(REG_ACCEL_CONFIG0, &accel_cfg, 1);
    
    printf("=== ICM-42688-P Config ===\n");
    printf("WHOAMI: 0x%02X %s\n", whoami, whoami == 0x47 ? "(OK)" : "(ERROR!)");
    printf("PWR_MGMT0: 0x%02X\n", pwr);
    printf("GYRO_CONFIG0: 0x%02X\n", gyro_cfg);
    printf("ACCEL_CONFIG0: 0x%02X\n", accel_cfg);
    printf("==========================\n");
}
