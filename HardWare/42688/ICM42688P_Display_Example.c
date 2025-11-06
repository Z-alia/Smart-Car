/**
 * @file ICM42688P_Display_Example.c
 * @brief ICM42688P传感器LCD显示使用示例
 * @date 2025-11-06
 * 
 * 该文件演示如何使用ICM42688P的检测和显示函数
 */

#include "ICM-42688P.h"
#include "lcd_spi_200.h"
#include "main.h"

/**
 * @brief 示例1: 完整的传感器检测和数据显示
 * 
 * 在main函数中调用此函数，可以看到：
 * - 硬件检测结果
 * - 初始化过程和结果
 * - 实时IMU数据显示（持续100帧）
 * 
 * 使用方法:
 * 在main.c中添加:
 *   LCD_Init();
 *   ICM42688P_DetectAndDisplay();
 */
void Example1_FullDetectionAndDisplay(void)
{
    // 1. 初始化LCD
    LCD_Init();
    
    // 2. 运行完整的检测和显示流程
    ICM42688P_DetectAndDisplay();
    
    // 此函数会自动执行所有检测步骤并显示结果
    // 包含约3秒的初始化检测 + 约2秒的数据显示（100帧@50Hz）
}

/**
 * @brief 示例2: 快速状态检查
 * 
 * 适用于已经初始化过ICM，只需要快速检查状态的场景
 * 
 * 使用方法:
 *   uint8_t status = ICM42688P_QuickCheck();
 *   if(status == 0) {
 *       // 传感器工作正常
 *   }
 */
void Example2_QuickStatusCheck(void)
{
    LCD_Init();
    LCD_Clear();
    
    char str[64];
    
    LCD_DisplayString(10, 10, "Quick Check:");
    
    uint8_t status = ICM42688P_QuickCheck();
    
    switch(status) {
        case 0:
            LCD_SetColor(LCD_GREEN);
            LCD_DisplayString(10, 30, "ICM Status: OK");
            LCD_SetColor(LCD_WHITE);
            break;
            
        case 1:
            LCD_SetColor(LCD_RED);
            LCD_DisplayString(10, 30, "WHOAMI Error!");
            LCD_DisplayString(10, 50, "Check connection");
            LCD_SetColor(LCD_WHITE);
            break;
            
        case 2:
            LCD_SetColor(LCD_YELLOW);
            LCD_DisplayString(10, 30, "Sensor Off");
            LCD_DisplayString(10, 50, "Need Init");
            LCD_SetColor(LCD_WHITE);
            break;
            
        default:
            LCD_SetColor(LCD_RED);
            LCD_DisplayString(10, 30, "Unknown Error");
            LCD_SetColor(LCD_WHITE);
            break;
    }
    
    HAL_Delay(2000);
}

/**
 * @brief 示例3: 在主循环中持续显示紧凑数据
 * 
 * 适用于需要在屏幕上同时显示其他信息的场景
 * ICM数据只占用3行（36像素高度）
 * 
 * 使用方法:
 *   while(1) {
 *       // 其他任务...
 *       ICM42688P_DisplayCompact(10, 100);  // 在(10,100)位置显示
 *       HAL_Delay(50);  // 20Hz刷新
 *   }
 */
void Example3_CompactDisplayInLoop(void)
{
    LCD_Init();
    
    // 初始化ICM（只需一次）
    uint8_t init_result = ICM42688P_Init();
    
    if(init_result != 0) {
        LCD_DisplayString(10, 10, "ICM Init Failed!");
        while(1) {
            HAL_Delay(1000);
        }
    }
    
    LCD_Clear();
    LCD_DisplayString(10, 10, "=== System Status ===");
    LCD_DisplayString(10, 30, "Motor: OK");
    LCD_DisplayString(10, 50, "Camera: OK");
    LCD_DisplayString(10, 70, "IMU Data:");
    
    // 主循环
    while(1) {
        // 在(10, 86)位置显示紧凑的IMU数据
        ICM42688P_DisplayCompact(10, 86);
        
        // 其他任务...
        
        HAL_Delay(50);  // 20Hz刷新
    }
}

/**
 * @brief 示例4: 自定义显示布局
 * 
 * 演示如何自定义显示IMU数据的格式和位置
 */
void Example4_CustomDisplay(void)
{
    LCD_Init();
    
    // 初始化ICM
    ICM42688P_Init();
    
    LCD_Clear();
    LCD_SetBackColor(LCD_BLACK);
    
    // 设置标题
    LCD_SetColor(LCD_CYAN);
    LCD_DisplayString(60, 10, "IMU Monitor");
    LCD_SetColor(LCD_WHITE);
    
    // 绘制分隔线
    LCD_DrawLine_H(10, 30, 220);
    
    char str[64];
    
    while(1) {
        // 读取数据
        ICM42688P_ReadIMUData(&imu_data);
        
        // 加速度计 - 左侧
        LCD_SetColor(LCD_YELLOW);
        LCD_DisplayString(10, 40, "Accelerometer");
        LCD_SetColor(LCD_WHITE);
        
        sprintf(str, "X: %6.2f g ", imu_data.accel_x);
        LCD_DisplayString(10, 60, str);
        
        sprintf(str, "Y: %6.2f g ", imu_data.accel_y);
        LCD_DisplayString(10, 76, str);
        
        sprintf(str, "Z: %6.2f g ", imu_data.accel_z);
        LCD_DisplayString(10, 92, str);
        
        // 陀螺仪 - 右侧
        LCD_SetColor(LCD_GREEN);
        LCD_DisplayString(10, 120, "Gyroscope");
        LCD_SetColor(LCD_WHITE);
        
        sprintf(str, "X: %6.0f dps", imu_data.gyro_x);
        LCD_DisplayString(10, 140, str);
        
        sprintf(str, "Y: %6.0f dps", imu_data.gyro_y);
        LCD_DisplayString(10, 156, str);
        
        sprintf(str, "Z: %6.0f dps", imu_data.gyro_z);
        LCD_DisplayString(10, 172, str);
        
        // 温度 - 底部
        LCD_SetColor(LCD_MAGENTA);
        sprintf(str, "Temp: %.1f C", imu_data.temperature);
        LCD_DisplayString(10, 200, str);
        LCD_SetColor(LCD_WHITE);
        
        HAL_Delay(50);
    }
}

/**
 * @brief 示例5: 在main.c中的典型用法
 * 
 * 将以下代码添加到main.c的main()函数中：
 */
/*
int main(void)
{
    // HAL库初始化...
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI1_Init();  // LCD的SPI
    MX_SPI4_Init();  // ICM的SPI
    
    // 初始化LCD
    LCD_Init();
    
    // ===== 方式1: 完整检测流程（推荐用于调试） =====
    ICM42688P_DetectAndDisplay();
    
    // ===== 方式2: 简单初始化 + 主循环显示 =====
    // uint8_t icm_status = ICM42688P_Init();
    // if(icm_status == 0) {
    //     LCD_DisplayString(10, 10, "ICM Ready!");
    // } else {
    //     LCD_DisplayString(10, 10, "ICM Error!");
    // }
    // 
    // while(1) {
    //     ICM42688P_DisplayCompact(10, 50);
    //     HAL_Delay(50);
    // }
    
    // ===== 方式3: 只初始化，数据用于控制（不显示） =====
    // ICM42688P_Init();
    // 
    // while(1) {
    //     ICM42688P_ReadIMUData(&imu_data);
    //     
    //     // 使用 imu_data.gyro_z 进行车体姿态控制
    //     float yaw_rate = imu_data.gyro_z;
    //     // ... 控制逻辑
    //     
    //     HAL_Delay(10);
    // }
}
*/
