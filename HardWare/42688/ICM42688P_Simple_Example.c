/**
 * @file    ICM42688P_Simple_Example.c
 * @brief   ICM-42688-P 简化驱动使用示例
 * @author  AI Assistant
 * @date    2025-11-06
 * @version 1.0
 * 
 * @note    基于官方数据手册 DS-000347 v1.8
 */

#include "ICM42688P_Simple.h"
#include "lcd_spi_200.h"
#include "usart.h"
#include <stdio.h>

/* 全局变量 */
extern IMU_Data imu_data;

/**
 * @brief  示例1: 最简单的使用
 * @note   只初始化和读取数据，通过串口打印
 */
void Example1_BasicUsage(void)
{
    printf("\n=== Example 1: Basic Usage ===\n");
    
    // 1. 初始化
    uint8_t init_result = ICM42688P_Simple_Init();
    
    if(init_result == 0) {
        printf("ICM-42688-P initialized successfully!\n");
    } else {
        printf("ICM-42688-P initialization failed! Error code: %d\n", init_result);
        return;
    }
    
    // 2. 读取数据（10次）
    for(int i = 0; i < 10; i++) {
        ICM42688P_Simple_ReadData(&imu_data);
        
        printf("\n--- Sample %d ---\n", i+1);
        printf("Accel: X=%6.2f  Y=%6.2f  Z=%6.2f g\n", 
               imu_data.accel_x, imu_data.accel_y, imu_data.accel_z);
        printf("Gyro:  X=%6.1f  Y=%6.1f  Z=%6.1f dps\n", 
               imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z);
        printf("Temp: %5.1f C\n", imu_data.temperature);
        
        HAL_Delay(100);
    }
}

/**
 * @brief  示例2: 与LCD显示结合
 * @note   在LCD上实时显示六轴数据
 */
void Example2_WithLCD(void)
{
    printf("\n=== Example 2: With LCD Display ===\n");
    
    // 1. 初始化LCD
    LCD_Init();
    LCD_Clear();
    
    // 2. 显示标题
    LCD_SetColor(LCD_CYAN);
    LCD_DisplayString(60, 5, "ICM-42688-P");
    LCD_SetColor(LCD_WHITE);
    LCD_DisplayString(50, 25, "Simple Driver");
    
    // 3. 初始化ICM
    uint8_t init_result = ICM42688P_Simple_Init();
    
    if(init_result == 0) {
        LCD_SetColor(LCD_GREEN);
        LCD_DisplayString(70, 50, "Init: OK");
        LCD_SetColor(LCD_WHITE);
    } else {
        LCD_SetColor(LCD_RED);
        LCD_DisplayString(60, 50, "Init: FAIL");
        LCD_SetColor(LCD_WHITE);
        return;
    }
    
    HAL_Delay(1000);
    LCD_Clear();
    
    // 4. 显示数据标签
    LCD_SetColor(LCD_YELLOW);
    LCD_DisplayString(10, 5, "Accelerometer (g):");
    LCD_SetColor(LCD_WHITE);
    
    LCD_DisplayString(10, 30, "X:");
    LCD_DisplayString(10, 46, "Y:");
    LCD_DisplayString(10, 62, "Z:");
    
    LCD_SetColor(LCD_YELLOW);
    LCD_DisplayString(10, 90, "Gyroscope (dps):");
    LCD_SetColor(LCD_WHITE);
    
    LCD_DisplayString(10, 115, "X:");
    LCD_DisplayString(10, 131, "Y:");
    LCD_DisplayString(10, 147, "Z:");
    
    LCD_SetColor(LCD_YELLOW);
    LCD_DisplayString(10, 175, "Temperature:");
    LCD_SetColor(LCD_WHITE);
    
    // 5. 循环显示数据
    char str[32];
    while(1) {
        ICM42688P_Simple_ReadData(&imu_data);
        
        // 加速度
        sprintf(str, "%7.3f", imu_data.accel_x);
        LCD_DisplayString(40, 30, str);
        sprintf(str, "%7.3f", imu_data.accel_y);
        LCD_DisplayString(40, 46, str);
        sprintf(str, "%7.3f", imu_data.accel_z);
        LCD_DisplayString(40, 62, str);
        
        // 陀螺仪
        sprintf(str, "%8.2f", imu_data.gyro_x);
        LCD_DisplayString(40, 115, str);
        sprintf(str, "%8.2f", imu_data.gyro_y);
        LCD_DisplayString(40, 131, str);
        sprintf(str, "%8.2f", imu_data.gyro_z);
        LCD_DisplayString(40, 147, str);
        
        // 温度
        sprintf(str, "%6.2f C", imu_data.temperature);
        LCD_DisplayString(40, 190, str);
        
        HAL_Delay(50);  // 20Hz刷新
    }
}

/**
 * @brief  示例3: 带状态检查的使用
 * @note   定期检查传感器状态，确保通信正常
 */
void Example3_WithStatusCheck(void)
{
    printf("\n=== Example 3: With Status Check ===\n");
    
    // 1. 初始化
    if(ICM42688P_Simple_Init() != 0) {
        printf("Initialization failed!\n");
        return;
    }
    
    printf("ICM-42688-P initialized successfully!\n");
    
    // 2. 打印配置信息
    ICM42688P_Simple_PrintConfig();
    
    // 3. 循环读取数据，每10次检查一次状态
    uint32_t sample_count = 0;
    
    while(1) {
        // 每10次采样检查一次状态
        if(sample_count % 10 == 0) {
            uint8_t status = ICM42688P_Simple_QuickCheck();
            
            if(status == 0) {
                printf("[%lu] Status: OK\n", sample_count);
            } else {
                printf("[%lu] Status: ERROR (code %d)\n", sample_count, status);
                
                // 尝试重新初始化
                printf("Attempting re-initialization...\n");
                if(ICM42688P_Simple_Init() == 0) {
                    printf("Re-initialization successful!\n");
                } else {
                    printf("Re-initialization failed!\n");
                    break;
                }
            }
        }
        
        // 读取数据
        ICM42688P_Simple_ReadData(&imu_data);
        
        printf("[%lu] AX=%6.2f AY=%6.2f AZ=%6.2f | ", 
               sample_count, imu_data.accel_x, imu_data.accel_y, imu_data.accel_z);
        printf("GX=%6.1f GY=%6.1f GZ=%6.1f | T=%5.1f\n",
               imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z, imu_data.temperature);
        
        sample_count++;
        HAL_Delay(100);
    }
}

/**
 * @brief  示例4: 数据采集和简单滤波
 * @note   演示如何进行数据缓冲和简单的移动平均滤波
 */
void Example4_DataFiltering(void)
{
    printf("\n=== Example 4: Data Filtering ===\n");
    
    #define FILTER_SIZE 5
    
    float accel_x_buffer[FILTER_SIZE] = {0};
    float accel_y_buffer[FILTER_SIZE] = {0};
    float accel_z_buffer[FILTER_SIZE] = {0};
    uint8_t buffer_index = 0;
    
    // 初始化
    if(ICM42688P_Simple_Init() != 0) {
        printf("Initialization failed!\n");
        return;
    }
    
    printf("Collecting data with moving average filter (N=%d)...\n", FILTER_SIZE);
    
    // 填充缓冲区
    printf("Filling buffer...\n");
    for(int i = 0; i < FILTER_SIZE; i++) {
        ICM42688P_Simple_ReadData(&imu_data);
        accel_x_buffer[i] = imu_data.accel_x;
        accel_y_buffer[i] = imu_data.accel_y;
        accel_z_buffer[i] = imu_data.accel_z;
        HAL_Delay(10);
    }
    
    // 循环采集和滤波
    while(1) {
        // 读取新数据
        ICM42688P_Simple_ReadData(&imu_data);
        
        // 更新缓冲区
        accel_x_buffer[buffer_index] = imu_data.accel_x;
        accel_y_buffer[buffer_index] = imu_data.accel_y;
        accel_z_buffer[buffer_index] = imu_data.accel_z;
        
        buffer_index = (buffer_index + 1) % FILTER_SIZE;
        
        // 计算移动平均
        float avg_x = 0, avg_y = 0, avg_z = 0;
        for(int i = 0; i < FILTER_SIZE; i++) {
            avg_x += accel_x_buffer[i];
            avg_y += accel_y_buffer[i];
            avg_z += accel_z_buffer[i];
        }
        avg_x /= FILTER_SIZE;
        avg_y /= FILTER_SIZE;
        avg_z /= FILTER_SIZE;
        
        // 打印原始值和滤波值
        printf("Raw:      AX=%6.3f AY=%6.3f AZ=%6.3f\n",
               imu_data.accel_x, imu_data.accel_y, imu_data.accel_z);
        printf("Filtered: AX=%6.3f AY=%6.3f AZ=%6.3f\n\n",
               avg_x, avg_y, avg_z);
        
        HAL_Delay(100);
    }
}

/**
 * @brief  示例5: 运动检测
 * @note   检测加速度变化，判断是否有运动
 */
void Example5_MotionDetection(void)
{
    printf("\n=== Example 5: Motion Detection ===\n");
    
    #define MOTION_THRESHOLD 0.5f  // 加速度变化阈值（g）
    
    float prev_accel_x = 0, prev_accel_y = 0, prev_accel_z = 0;
    uint32_t motion_count = 0;
    uint32_t still_count = 0;
    
    // 初始化
    if(ICM42688P_Simple_Init() != 0) {
        printf("Initialization failed!\n");
        return;
    }
    
    // 读取初始值
    ICM42688P_Simple_ReadData(&imu_data);
    prev_accel_x = imu_data.accel_x;
    prev_accel_y = imu_data.accel_y;
    prev_accel_z = imu_data.accel_z;
    
    printf("Motion detection started (threshold = %.2f g)...\n", MOTION_THRESHOLD);
    
    while(1) {
        // 读取数据
        ICM42688P_Simple_ReadData(&imu_data);
        
        // 计算加速度变化
        float delta_x = imu_data.accel_x - prev_accel_x;
        float delta_y = imu_data.accel_y - prev_accel_y;
        float delta_z = imu_data.accel_z - prev_accel_z;
        
        // 计算总变化量
        float total_delta = sqrtf(delta_x*delta_x + delta_y*delta_y + delta_z*delta_z);
        
        // 判断运动
        if(total_delta > MOTION_THRESHOLD) {
            motion_count++;
            still_count = 0;
            printf("[MOTION] Delta = %.3f g | AX=%6.3f AY=%6.3f AZ=%6.3f\n",
                   total_delta, imu_data.accel_x, imu_data.accel_y, imu_data.accel_z);
        } else {
            still_count++;
            if(still_count == 1) {
                printf("[STILL]  Delta = %.3f g | Motion events: %lu\n",
                       total_delta, motion_count);
            }
        }
        
        // 更新前一次的值
        prev_accel_x = imu_data.accel_x;
        prev_accel_y = imu_data.accel_y;
        prev_accel_z = imu_data.accel_z;
        
        HAL_Delay(50);  // 20Hz采样
    }
}

/**
 * @brief  主测试函数
 * @note   在main.c中调用此函数运行示例
 */
void ICM42688P_Simple_RunExamples(void)
{
    printf("\n");
    printf("========================================\n");
    printf("  ICM-42688-P Simple Driver Examples   \n");
    printf("========================================\n");
    
    // 取消下面的注释来运行不同的示例
    
    // Example1_BasicUsage();           // 基本使用
    // Example2_WithLCD();              // LCD显示（会阻塞）
    // Example3_WithStatusCheck();      // 状态检查（会阻塞）
    // Example4_DataFiltering();        // 数据滤波（会阻塞）
    // Example5_MotionDetection();      // 运动检测（会阻塞）
    
    printf("\nNo example selected. Please uncomment one example in the code.\n");
}
