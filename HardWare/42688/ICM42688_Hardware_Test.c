/**
 * @file ICM42688_Hardware_Test.c
 * @brief ICM-42688P 硬件层诊断代码
 * 
 * 将这些测试函数添加到项目中以诊断 WHOAMI 返回 0x00 的问题
 */

#include "main.h"
#include "ICM-42688P.h"
#include "lcd_spi_200.h"
#include <stdio.h>

extern SPI_HandleTypeDef hspi4;

/**
 * @brief 测试 MISO 引脚是否能接收数据
 * 
 * 这个测试在没有 ICM 连接的情况下也能工作
 * 如果 MISO 悬空，应该读到不确定的值或 0xFF
 * 如果 MISO 接地，会一直读到 0x00
 */
void Test_MISO_Line(void)
{
    char buf[32];
    LCD_Clear();
    LCD_DisplayString(10, 10, "=== MISO Test ===");
    
    // 1. 读取 MISO 引脚状态（空闲时）
    uint8_t miso_idle = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13);
    sprintf(buf, "MISO idle: %d", miso_idle);
    LCD_DisplayString(10, 30, buf);
    
    // 2. 发送数据时监测 MISO
    cs_low();
    HAL_Delay(2);
    
    uint8_t tx_data[8] = {0xAA, 0x55, 0xFF, 0x00, 0xF0, 0x0F, 0xCC, 0x33};
    uint8_t rx_data[8] = {0};
    
    HAL_SPI_TransmitReceive(&hspi4, tx_data, rx_data, 8, 100);
    
    cs_high();
    HAL_Delay(2);
    
    // 显示接收到的数据
    LCD_DisplayString(10, 50, "Received:");
    for(int i = 0; i < 8; i++) {
        sprintf(buf, "0x%02X", rx_data[i]);
        LCD_DisplayString(10 + i*30, 70, buf);
    }
    
    // 分析结果
    uint8_t all_zero = 1;
    uint8_t all_ff = 1;
    for(int i = 0; i < 8; i++) {
        if(rx_data[i] != 0x00) all_zero = 0;
        if(rx_data[i] != 0xFF) all_ff = 0;
    }
    
    LCD_SetColor(LCD_YELLOW);
    if(all_zero) {
        LCD_DisplayString(10, 100, "WARN: MISO stuck LOW");
        LCD_DisplayString(10, 120, "Check PE13 connection");
    } else if(all_ff) {
        LCD_DisplayString(10, 100, "WARN: MISO stuck HIGH");
        LCD_DisplayString(10, 120, "Check PE13 pull-up");
    } else {
        LCD_SetColor(LCD_GREEN);
        LCD_DisplayString(10, 100, "MISO seems OK");
    }
    LCD_SetColor(LCD_WHITE);
    
    HAL_Delay(3000);
}

/**
 * @brief 测试 ICM-42688P 供电和连接
 * 
 * 通过读取多个寄存器来判断芯片是否正常
 */
void Test_ICM_Power_And_Connection(void)
{
    char buf[32];
    LCD_Clear();
    LCD_DisplayString(10, 10, "=== Power Test ===");
    
    // 确保片选初始为高
    cs_high();
    HAL_Delay(100);
    
    // 读取多个寄存器
    uint8_t whoami = 0;
    uint8_t bank_sel = 0;
    uint8_t pwr_mgmt = 0;
    
    // 读取 WHOAMI (0x75)
    cs_low();
    HAL_Delay(2);
    uint8_t cmd = 0x75 | 0x80;
    HAL_SPI_Transmit(&hspi4, &cmd, 1, 100);
    HAL_Delay(1);
    HAL_SPI_Receive(&hspi4, &whoami, 1, 100);
    cs_high();
    HAL_Delay(5);
    
    // 读取 BANK_SEL (0x76)
    cs_low();
    HAL_Delay(2);
    cmd = 0x76 | 0x80;
    HAL_SPI_Transmit(&hspi4, &cmd, 1, 100);
    HAL_Delay(1);
    HAL_SPI_Receive(&hspi4, &bank_sel, 1, 100);
    cs_high();
    HAL_Delay(5);
    
    // 读取 PWR_MGMT0 (0x4E)
    cs_low();
    HAL_Delay(2);
    cmd = 0x4E | 0x80;
    HAL_SPI_Transmit(&hspi4, &cmd, 1, 100);
    HAL_Delay(1);
    HAL_SPI_Receive(&hspi4, &pwr_mgmt, 1, 100);
    cs_high();
    HAL_Delay(5);
    
    // 显示结果
    sprintf(buf, "WHOAMI: 0x%02X", whoami);
    LCD_DisplayString(10, 30, buf);
    if(whoami == 0x47) {
        LCD_SetColor(LCD_GREEN);
        LCD_DisplayString(150, 30, "OK!");
    } else {
        LCD_SetColor(LCD_RED);
        LCD_DisplayString(150, 30, "ERR");
    }
    LCD_SetColor(LCD_WHITE);
    
    sprintf(buf, "BANK: 0x%02X", bank_sel);
    LCD_DisplayString(10, 50, buf);
    
    sprintf(buf, "PWR: 0x%02X", pwr_mgmt);
    LCD_DisplayString(10, 70, buf);
    
    // 诊断
    LCD_SetColor(LCD_YELLOW);
    if(whoami == 0x00 && bank_sel == 0x00 && pwr_mgmt == 0x00) {
        LCD_DisplayString(10, 100, "All zeros!");
        LCD_DisplayString(10, 120, "Possible causes:");
        LCD_DisplayString(10, 140, "1.MISO not connected");
        LCD_DisplayString(10, 160, "2.ICM not powered");
        LCD_DisplayString(10, 180, "3.ICM damaged");
    } else if(whoami == 0xFF && bank_sel == 0xFF && pwr_mgmt == 0xFF) {
        LCD_DisplayString(10, 100, "All 0xFF!");
        LCD_DisplayString(10, 120, "ICM not responding");
        LCD_DisplayString(10, 140, "Check CS/SCK/MOSI");
    } else if(whoami != 0x47) {
        LCD_DisplayString(10, 100, "Wrong ID!");
        LCD_DisplayString(10, 120, "May need reset");
    }
    LCD_SetColor(LCD_WHITE);
    
    HAL_Delay(5000);
}

/**
 * @brief 尝试不同的 SPI 时钟极性和相位
 * 
 * 有时候 SPI 配置不对会导致读取错误
 */
void Test_SPI_Modes(void)
{
    char buf[32];
    LCD_Clear();
    LCD_DisplayString(10, 10, "=== SPI Mode Test ===");
    
    // 保存原始配置
    uint32_t original_cfg1 = hspi4.Instance->CFG1;
    uint32_t original_cfg2 = hspi4.Instance->CFG2;
    
    // 测试 4 种 SPI 模式
    const char* mode_names[4] = {"Mode0", "Mode1", "Mode2", "Mode3"};
    uint8_t whoami_results[4] = {0};
    
    for(int mode = 0; mode < 4; mode++) {
        // 禁用 SPI
        hspi4.Instance->CR1 &= ~SPI_CR1_SPE;
        HAL_Delay(10);
        
        // 配置时钟极性和相位
        if(mode == 0) {
            // Mode 0: CPOL=0, CPHA=0
            hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
            hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
        } else if(mode == 1) {
            // Mode 1: CPOL=0, CPHA=1
            hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
            hspi4.Init.CLKPhase = SPI_PHASE_2EDGE;
        } else if(mode == 2) {
            // Mode 2: CPOL=1, CPHA=0
            hspi4.Init.CLKPolarity = SPI_POLARITY_HIGH;
            hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
        } else {
            // Mode 3: CPOL=1, CPHA=1
            hspi4.Init.CLKPolarity = SPI_POLARITY_HIGH;
            hspi4.Init.CLKPhase = SPI_PHASE_2EDGE;
        }
        
        HAL_SPI_Init(&hspi4);
        HAL_Delay(10);
        
        // 尝试读取 WHOAMI
        cs_low();
        HAL_Delay(5);
        
        uint8_t cmd = 0xF5;
        HAL_SPI_Transmit(&hspi4, &cmd, 1, 100);
        HAL_Delay(1);
        HAL_SPI_Receive(&hspi4, &whoami_results[mode], 1, 100);
        
        cs_high();
        HAL_Delay(10);
        
        // 显示结果
        sprintf(buf, "%s: 0x%02X", mode_names[mode], whoami_results[mode]);
        LCD_DisplayString(10, 30 + mode*20, buf);
        
        if(whoami_results[mode] == 0x47) {
            LCD_SetColor(LCD_GREEN);
            LCD_DisplayString(120, 30 + mode*20, "OK!");
            LCD_SetColor(LCD_WHITE);
        }
    }
    
    // 恢复原始配置
    hspi4.Instance->CR1 &= ~SPI_CR1_SPE;
    hspi4.Instance->CFG1 = original_cfg1;
    hspi4.Instance->CFG2 = original_cfg2;
    hspi4.Instance->CR1 |= SPI_CR1_SPE;
    
    HAL_Delay(5000);
}

/**
 * @brief 主诊断函数 - 按顺序运行所有测试
 * 
 * 在 main.c 中调用此函数来进行完整的硬件诊断
 */
void ICM42688_Full_Hardware_Diagnostic(void)
{
    // 测试 1: MISO 线测试
    Test_MISO_Line();
    
    // 测试 2: ICM 供电和连接测试
    Test_ICM_Power_And_Connection();
    
    // 测试 3: SPI 模式测试
    Test_SPI_Modes();
    
    // 完成
    LCD_Clear();
    LCD_SetColor(LCD_GREEN);
    LCD_DisplayString(10, 100, "Diagnostic Complete!");
    LCD_SetColor(LCD_WHITE);
    LCD_DisplayString(10, 120, "Check results above");
    HAL_Delay(2000);
}

/* 
 * 使用方法：
 * 
 * 1. 将此文件添加到项目中
 * 2. 在 main.c 中添加函数声明：
 *    void ICM42688_Full_Hardware_Diagnostic(void);
 * 
 * 3. 在 LCD_Init() 之后调用：
 *    ICM42688_Full_Hardware_Diagnostic();
 * 
 * 4. 观察 LCD 显示的每个测试结果
 */
