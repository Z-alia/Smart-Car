/**
 * @file ICM-426688P.c
 * @brief ICM-42688P IMU传感器驱动实现
 *
 * 该文件实现了ICM-42688P IMU传感器的驱动功能，包括SPI通信、
 * 寄存器操作、数据解析和传感器配置等功能。
 * 
 * @note SPI4配置说明:
 *       - 硬件NSS: PE11 (SPI4_NSS)
 *       - 虽然配置为硬件NSS，但为了精确控制时序，代码中手动控制片选
 *       - SCK: PE12, MISO: PE13, MOSI: PE14
 */

#include "ICM-42688P.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "ICM42688P_Config.h"
#include "lcd_spi_200.h"  // 添加 LCD 头文件
#include "spi.h"          // 添加 SPI 头文件

// 外部SPI句柄声明
extern SPI_HandleTypeDef hspi4;
IMU_Data imu_data;

/**
 * @brief 简化的 SPI 通信诊断函数（使用 LCD 显示）
 * @note 测试 SPI4 关键状态
 */
void ICM42688P_SPI_Diagnostic(void)
{
    char buf[32];
    uint16_t y = 50;  // 起始行
    uint16_t line_h = 16;  // 行高
    
    LCD_SetBackColor(LCD_BLACK);
    LCD_SetColor(LCD_WHITE);
    
    // 清空诊断区域
    LCD_ClearRect(0, y, 240, 200);
    
    // 1. 检查 PE11 模式
    uint32_t moder_bits = (GPIOE->MODER >> 22) & 0x3;
    snprintf(buf, sizeof(buf), "PE11 MODE:%d", (int)moder_bits);
    LCD_DisplayString(5, y, buf);
    LCD_SetColor(moder_bits == 1 ? LCD_GREEN : LCD_RED);
    LCD_DisplayString(140, y, moder_bits == 1 ? "OK" : "ERR!");
    LCD_SetColor(LCD_WHITE);
    y += line_h;
    
    // 2. CS 控制测试
    cs_high();
    HAL_Delay(2);
    uint8_t cs_h = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11);
    
    cs_low();
    HAL_Delay(2);
    uint8_t cs_l = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11);
    
    cs_high();
    HAL_Delay(2);
    
    snprintf(buf, sizeof(buf), "CS H:%d L:%d", cs_h, cs_l);
    LCD_DisplayString(5, y, buf);
    LCD_SetColor((cs_h == 1 && cs_l == 0) ? LCD_GREEN : LCD_RED);
    LCD_DisplayString(140, y, (cs_h == 1 && cs_l == 0) ? "OK" : "ERR!");
    LCD_SetColor(LCD_WHITE);
    y += line_h;
    
    // 3. SPI 传输测试
    SPI4->CR1 &= ~0x00000001;  // 禁用 SPI
    HAL_Delay(1);
    
    // 清空 FIFO
    while(SPI4->SR & SPI_SR_RXP) {
        volatile uint8_t dummy = *((__IO uint8_t *)&SPI4->RXDR);
        (void)dummy;
    }
    
    SPI4->CR1 |= 0x00000001;  // 使能 SPI
    HAL_Delay(5);
    
    cs_low();
    HAL_Delay(2);
    
    uint32_t timeout = 100000;
    while(!(SPI4->SR & SPI_SR_TXP) && timeout--);
    
    uint8_t spi_ok = 0;
    uint8_t rx_data = 0xFF;
    
    if(timeout > 0) {
        *((__IO uint8_t *)&SPI4->TXDR) = 0xAA;
        SPI4->CR1 |= 0x00000200;  // CSTART
        
        timeout = 100000;
        while(!(SPI4->SR & SPI_SR_RXP) && timeout--);
        
        if(timeout > 0) {
            rx_data = *((__IO uint8_t *)&SPI4->RXDR);
            spi_ok = 1;
        }
        
        timeout = 100000;
        while(!(SPI4->SR & 0x00000008) && timeout--);
        SPI4->IFCR |= 0x00000008;
    }
    
    cs_high();
    
    snprintf(buf, sizeof(buf), "SPI RX:0x%02X", rx_data);
    LCD_DisplayString(5, y, buf);
    LCD_SetColor(spi_ok ? LCD_GREEN : LCD_RED);
    LCD_DisplayString(140, y, spi_ok ? "OK" : "ERR!");
    LCD_SetColor(LCD_WHITE);
    y += line_h;
    
    // 4. 完成提示
    y += 5;
    LCD_SetColor(LCD_YELLOW);
    LCD_DisplayString(5, y, "Press to continue...");
    LCD_SetColor(LCD_WHITE);
    
    HAL_Delay(3000);  // 等待3秒查看结果
}

/**
 * @brief 手动 SPI 读写一个字节（不依赖 HAL 库超时）
 * @param data 要发送的数据
 * @return 接收到的数据
 * 
 * @note STM32H7 SPI 标志位：
 *       - SPI_SR_TXP (Tx-Packet space available) - 发送缓冲区有空间
 *       - SPI_SR_RXP (Rx-Packet available) - 接收缓冲区有数据
 */
uint8_t SPI_ReadWriteByte_Manual(uint8_t data)
{
    uint32_t timeout = 100000;  // 软件超时计数器
    
    // 等待 TXP (发送缓冲区空) - STM32H7 使用 TXP 而不是 TXE
    while(!(hspi4.Instance->SR & SPI_SR_TXP) && timeout--);
    if(timeout == 0) return 0xFF;  // 超时返回
    
    // 写入发送数据寄存器
    *((__IO uint8_t *)&hspi4.Instance->TXDR) = data;
    
    // 启动传输（对于每个字节都需要启动）
    hspi4.Instance->CR1 |= 0x00000200;  // CSTART bit
    
    // 等待 RXP (接收缓冲区非空) - STM32H7 使用 RXP 而不是 RXNE
    timeout = 100000;
    while(!(hspi4.Instance->SR & SPI_SR_RXP) && timeout--);
    if(timeout == 0) return 0xFF;  // 超时返回
    
    // 读取接收到的数据
    uint8_t rx = *((__IO uint8_t *)&hspi4.Instance->RXDR);
    
    // 移除 EOT 等待 - RXP 已经表示数据接收完成
    // EOT 等待可能导致不必要的延时累积
    
    return rx;
}

/**
 * @brief 手动 SPI 读取寄存器（完全不依赖 HAL 库）
 * @param reg_address 寄存器地址
 * @param rxdata 接收数据缓冲区
 * @param length 数据长度
 * @return 0=成功, 1=失败
 */
uint8_t ICM42688P_ManualRead(uint8_t reg_address, uint8_t *rxdata, uint8_t length)
{
    // 确保 SPI 已使能 (CR1 bit 0 = SPE)
    if(!(hspi4.Instance->CR1 & 0x00000001)) {
        hspi4.Instance->CR1 |= 0x00000001;  // SPE bit
        // 短暂延时，改用 for 循环避免 HAL_Delay 重载
        for(volatile uint32_t i = 0; i < 4800; i++);  // 约1ms @ 480MHz
    }
    
    cs_low();
    // 片选延时，改用 for 循环
    for(volatile uint32_t i = 0; i < 2400; i++);  // 约0.5ms @ 480MHz
    
    // 发送地址（带读标志）
    uint8_t addr_byte = reg_address | ICM42688P_READ;
    uint8_t dummy = SPI_ReadWriteByte_Manual(addr_byte);
    (void)dummy;  // 忽略第一个字节的返回值
    
    // 读取数据
    for(uint8_t i = 0; i < length; i++) {
        rxdata[i] = SPI_ReadWriteByte_Manual(0xFF);
    }
    
    // 等待最后一个字节传输完成 (TXC 标志)
    // 修正：检查 TXC (bit 1) 而不是 EOT
    uint32_t timeout = 100000;
    while(!(hspi4.Instance->SR & (1 << 1)) && timeout--);  // 等待 TXC=1
    
    // 释放片选前短暂延时
    for(volatile uint32_t i = 0; i < 480; i++);  // 约0.1ms
    
    cs_high();
    
    // 片选释放延时
    for(volatile uint32_t i = 0; i < 2400; i++);  // 约0.5ms @ 480MHz
    
    // 清除传输完成标志
    hspi4.Instance->IFCR = 0xFFFFFFFF;  // 清除所有标志位
    
    return 0;
}

/**
 * @brief SPI 安全读取函数 - 使用 TransmitReceive 避免阻塞
 * @param reg_address 寄存器地址
 * @param rxdata 接收数据缓冲区
 * @param length 数据长度
 * @return 0=成功, 1=失败
 */
uint8_t ICM42688P_SafeRead(uint8_t reg_address, uint8_t *rxdata, uint8_t length)
{
    // 准备发送缓冲区：地址 + dummy bytes
    uint8_t tx_buffer[32];  // 最大支持读取31字节数据
    uint8_t rx_buffer[32];
    
    if(length > 31) return 1;  // 超过最大长度
    
    tx_buffer[0] = reg_address | ICM42688P_READ;
    for(uint8_t i = 1; i <= length; i++) {
        tx_buffer[i] = 0xFF;  // dummy bytes
    }
    
    cs_low();
    HAL_Delay(2);
    
    // 使用 TransmitReceive 同时发送和接收
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi4, tx_buffer, rx_buffer, length + 1, 50);
    
    cs_high();
    HAL_Delay(2);
    
    if(status == HAL_OK) {
        // 复制接收到的数据（跳过第一个字节，那是地址的回复）
        for(uint8_t i = 0; i < length; i++) {
            rxdata[i] = rx_buffer[i + 1];
        }
        return 0;
    } else {
        // 失败时填充 0xFF
        for(uint8_t i = 0; i < length; i++) {
            rxdata[i] = 0xFF;
        }
        return 1;
    }
}

/**
 * @brief 通过SPI发送数据（使用HAL库）
 * @param bytes 要发送的数据缓冲区
 * @param length 数据长度
 */
void spi_send_bytes(uint8_t *bytes, uint8_t length)
{
    HAL_SPI_Transmit(&hspi4, bytes, length, HAL_MAX_DELAY);
}

/**
 * @brief 延时函数
 * @param ms 延时时间（毫秒）
 */
void delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}

/**
 * @brief 通过SPI接收数据（使用HAL库）
 * @param bytes 接收数据缓冲区
 * @param length 数据长度
 */
void spi_read_bytes(uint8_t *bytes, uint8_t length)
{
    HAL_SPI_Receive(&hspi4, bytes, length, HAL_MAX_DELAY);
}

/**
 * @brief 突发读取寄存器（底层硬件实现 - HAL库版本）
 * @param reg_address 寄存器地址
 * @param rxdata 接收数据缓冲区
 * @param length 数据长度
 *
 * @note 使用HAL库进行SPI通信，提供稳定可靠的寄存器读取功能
 *
 * @warning 此函数为内部实现，不处理Bank选择等上层逻辑
 */
void ICM42688P_BurstRead_Internal(uint8_t reg_address, uint8_t *rxdata, uint8_t length)
{
    uint8_t tx_addr = reg_address | ICM42688P_READ;
    HAL_StatusTypeDef status;
    
    cs_low();
    HAL_Delay(2);  // 片选延时
    
    // 发送寄存器地址（带读标志），使用较短超时 50ms
    status = HAL_SPI_Transmit(&hspi4, &tx_addr, 1, 50);
    if(status != HAL_OK)
    {
        cs_high();
        // 如果发送失败，清空接收缓冲区
        for(uint8_t i = 0; i < length; i++) rxdata[i] = 0xFF;
        return;
    }
    
    HAL_Delay(1);  // 地址和数据之间加延时
    
    // 读取数据
    if (length > 0)
    {
        status = HAL_SPI_Receive(&hspi4, rxdata, length, 50);
        if(status != HAL_OK)
        {
            // 如果接收失败，清空接收缓冲区
            for(uint8_t i = 0; i < length; i++) rxdata[i] = 0xFF;
        }
    }
    
    cs_high();
    HAL_Delay(2);  // 片选释放后延时
}

/**
 * @brief 最小化测试函数 - 只测试 SPI 通信
 * @return 读取到的 WHOAMI 值
 * 
 * 这个函数只做最基础的 SPI 读取操作，用于测试 SPI 是否工作
 * 不做任何复位、配置等复杂操作
 */
uint8_t ICM42688P_Test_MinimalRead(void)
{
    uint8_t whoami = 0;
    uint8_t tx_addr = ICM42688P_WHOAMI | ICM42688P_READ;  // 0x75 | 0x80 = 0xF5
    
    // 片选拉低
    cs_low();
    HAL_Delay(2);  // 等待稳定
    
    // 发送读取命令（地址 + 读标志）
    HAL_StatusTypeDef status1 = HAL_SPI_Transmit(&hspi4, &tx_addr, 1, 100);
    
    // 读取 1 字节数据
    HAL_StatusTypeDef status2 = HAL_SPI_Receive(&hspi4, &whoami, 1, 100);
    
    // 片选拉高
    cs_high();
    HAL_Delay(2);
    
    // 如果任何一步失败，返回错误标记 0xFF
    if(status1 != HAL_OK || status2 != HAL_OK)
    {
        return 0xFF;
    }
    
    return whoami;
}

/**
 * @brief 直接读取测试（使用手动字节传输）
 * @return 读取到的 WHOAMI 值
 * 
 * 使用完全手动的方式进行 SPI 读取，绕过所有 HAL 抽象层
 */
uint8_t ICM42688P_Test_DirectRead(void)
{
    // 确保 SPI 已使能
    if(!(hspi4.Instance->CR1 & 0x00000001)) {
        hspi4.Instance->CR1 |= 0x00000001;  // SPE bit
        HAL_Delay(10);
    }
    
    // 片选拉低
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
    HAL_Delay(5);
    
    // 发送地址字节 (0xF5 = 0x75 | 0x80)
    uint8_t addr_byte = ICM42688P_WHOAMI | ICM42688P_READ;
    uint8_t dummy = SPI_ReadWriteByte_Manual(addr_byte);
    (void)dummy;
    
    HAL_Delay(1);
    
    // 读取数据字节
    uint8_t whoami = SPI_ReadWriteByte_Manual(0xFF);
    
    // 片选拉高
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
    HAL_Delay(5);
    
    return whoami;
}

/**
 * @brief 初始化ICM42688P传感器
 *
 * 执行传感器初始化流程：
 * 1. 片选置高
 * 2. 软件复位
 * 3. 时钟配置
 * 4. 输出数据速率配置
 * 5. 启动传感器
 */
uint8_t ICM42688P_Init(void)
{
    uint8_t whoami = 0;
    uint8_t retry_count = 0;
    
    // ===== 步骤1: 确保 SPI4 已使能 =====
    if(!(hspi4.Instance->CR1 & 0x00000001)) {
        hspi4.Instance->CR1 |= 0x00000001;  // SPE bit
        // 使用软件延时避免HAL_Delay可能的问题
        for(volatile uint32_t i = 0; i < 48000; i++);  // 约10ms @ 480MHz
    }
    
    // ===== 步骤2: 初始片选拉高，等待上电稳定 =====
    cs_high();
    // 150ms稳定时间
    for(volatile uint32_t i = 0; i < 720000; i++);
    
    // ===== 步骤3: 确保选择 Bank 0 =====
    cs_low();
    for(volatile uint32_t i = 0; i < 48000; i++);  // 10ms
    
    SPI_ReadWriteByte_Manual(0x76);  // Bank Select 寄存器
    for(volatile uint32_t i = 0; i < 9600; i++);  // 2ms
    SPI_ReadWriteByte_Manual(0x00);  // Bank 0
    
    // 等待传输完成
    uint32_t timeout = 100000;
    while(!(hspi4.Instance->SR & (1 << 1)) && timeout--);  // TXC标志
    
    cs_high();
    for(volatile uint32_t i = 0; i < 96000; i++);  // 20ms Bank切换延时
    
    // ===== 步骤4: 读取 WHOAMI验证通信 =====
    whoami = ICM42688P_Test_DirectRead();
    
    if (whoami == 0x47)
    {
        // 芯片已正常工作，跳过复位直接配置
        goto config_sensor;
    }
    
    // ===== 步骤5: WHOAMI失败，尝试软件复位 =====
    for(volatile uint32_t i = 0; i < 240000; i++);  // 50ms
    ICM42688P_Software_Reset();
    for(volatile uint32_t i = 0; i < 960000; i++);  // 200ms 复位后等待
    
    // ===== 步骤6: 复位后重试读取WHOAMI =====
    for(retry_count = 0; retry_count < 5; retry_count++)
    {
        whoami = ICM42688P_Test_DirectRead();
        
        if (whoami == 0x47)
        {
            goto config_sensor;
        }
        
        // 重试间隔150ms
        for(volatile uint32_t i = 0; i < 720000; i++);
    }
    
    // 所有尝试都失败，返回错误码1（WHOAMI读取失败）
    return 1;

config_sensor:
    // ===== 步骤7: WHOAMI正确，开始配置传感器 =====
    for(volatile uint32_t i = 0; i < 48000; i++);  // 10ms
    
    // ===== 步骤8: 时钟配置 =====
    ICM42688P_Clock_Config();
    for(volatile uint32_t i = 0; i < 96000; i++);  // 20ms - 增加延时，确保Bank切换和配置生效
    
    // ===== 步骤9: 中断配置 =====
    ICM42688P_Interrupt_Config();
    for(volatile uint32_t i = 0; i < 48000; i++);  // 10ms
    
    // ===== 步骤10: 输出数据速率配置 =====
    ICM42688P_ODR_Config();
    for(volatile uint32_t i = 0; i < 48000; i++);  // 10ms
    
    // ===== 步骤11: 启动传感器 =====
    ICM42688P_Start();
    // 关键修复：传感器启动需要更长时间，增加到50ms
    for(volatile uint32_t i = 0; i < 240000; i++);  // 50ms - 等待传感器完全启动
    
    // ===== 步骤12: 确保回到Bank 0（某些配置可能改变了Bank） =====
    ICM42688P_Bank_Select(0);
    for(volatile uint32_t i = 0; i < 48000; i++);  // 10ms - 确保Bank切换完成
    
    // ===== 步骤13: 最终验证 - 再次读取WHOAMI确认配置未破坏通信 =====
    // 增加重试机制，因为传感器启动后可能需要稳定时间
    for(retry_count = 0; retry_count < 3; retry_count++) {
        whoami = ICM42688P_Test_DirectRead();
        if (whoami == 0x47) {
            break;  // 读取成功
        }
        // 重试延时
        for(volatile uint32_t i = 0; i < 48000; i++);  // 10ms
    }
    
    if (whoami != 0x47)
    {
        // 配置后通信失败，返回错误码2
        return 2;
    }
    
    // ===== 步骤14: 验证传感器已启动 - 读取PWR_MGMT0寄存器 =====
    uint8_t pwr_status = 0;
    ICM42688P_Bank_Select(0);
    for(volatile uint32_t i = 0; i < 48000; i++);  // 10ms - 确保Bank切换完成
    ICM42688P_ReadRegister(0x4E, &pwr_status, 1);
    
    // 期望值：0x0F (加速度计和陀螺仪都在LN模式)
    if (pwr_status != 0x0F)
    {
        // 传感器未正确启动，返回错误码3
        return 3;
    }
    
    // ===== 步骤15: 等待传感器数据稳定 =====
    // 传感器启动后，前几次读取的数据可能不准确
    for(volatile uint32_t i = 0; i < 96000; i++);  // 20ms
    
    return 0;  // 初始化成功
}

/**
 * @brief 选择寄存器组
 * @param bank 寄存器组编号
 */
void ICM42688P_Bank_Select(uint8_t bank)
{
    uint8_t config = bank;
    ICM42688P_WriteRegister(0x76, &config, 1);
}

/**
 * @brief 软件复位（使用手动 SPI 方式）
 *
 * 向设备发送软件复位命令，使设备恢复到默认状态
 */
void ICM42688P_Software_Reset(void)
{
    // 确保 SPI 已使能
    if(!(hspi4.Instance->CR1 & 0x00000001)) {
        hspi4.Instance->CR1 |= 0x00000001;  // SPE bit
        HAL_Delay(10);
    }
    
    // 先切换到 Bank 0
    cs_high();
    HAL_Delay(20);  // 增加延时，确保芯片准备好
    cs_low();
    HAL_Delay(10);  // 增加片选稳定时间
    
    SPI_ReadWriteByte_Manual(0x76);  // Bank Select 寄存器
    HAL_Delay(2);
    SPI_ReadWriteByte_Manual(0x00);  // Bank 0
    
    // 等待传输完成
    uint32_t timeout = 100000;
    while(!(hspi4.Instance->SR & 0x00000008) && timeout--);
    hspi4.Instance->IFCR |= 0x00000008;  // 清除 EOT 标志
    
    cs_high();
    HAL_Delay(20);  // Bank切换后延时（增加）
    
    // 发送复位命令
    cs_low();
    HAL_Delay(10);
    
    SPI_ReadWriteByte_Manual(0x11);  // Device Config 寄存器
    HAL_Delay(2);
    SPI_ReadWriteByte_Manual(0x01);  // Soft Reset bit
    
    // 等待传输完成
    timeout = 100000;
    while(!(hspi4.Instance->SR & 0x00000008) && timeout--);
    hspi4.Instance->IFCR |= 0x00000008;
    
    cs_high();
    HAL_Delay(50);  // 复位命令后需要足够的延时
    
    // ===== 关键修复：复位后等待芯片重新启动 =====
    // 根据 ICM-42688P 数据手册，软件复位后需要等待设备重新初始化
    // 典型启动时间：10ms，最大启动时间：25ms
    // 这里等待 50ms 确保完全启动
    HAL_Delay(50);
    
    // 复位后自动切换到 Bank 0，但为了保险起见，再次确认
    cs_low();
    HAL_Delay(10);
    
    SPI_ReadWriteByte_Manual(0x76);  // Bank Select 寄存器
    HAL_Delay(2);
    SPI_ReadWriteByte_Manual(0x00);  // Bank 0
    
    timeout = 100000;
    while(!(hspi4.Instance->SR & 0x00000008) && timeout--);
    hspi4.Instance->IFCR |= 0x00000008;
    
    cs_high();
    HAL_Delay(20);
}

/**
 * @brief 启动传感器
 *
 * 配置传感器进入正常工作模式
 */
void ICM42688P_Start(void)
{
    ICM42688P_Bank_Select(0);
    // 增加延时，确保Bank切换完成
    for(volatile uint32_t i = 0; i < 24000; i++);  // 5ms @ 480MHz
    
    uint8_t address = 0x4e;  // PWR_MGMT0 寄存器
    // 0b00000011: Bit[1:0]=11 加速度计低功耗模式, Bit[3:2]=00 陀螺仪关闭
    // 0b00001111: Bit[1:0]=11 加速度计低功耗模式, Bit[3:2]=11 陀螺仪低功耗模式  
    // 正确配置应该是高性能模式，但根据数据手册：
    // 需要配置为 LN (Low Noise) 模式以获得最佳性能
    // 加速度计和陀螺仪都使用 LN 模式：
    // Bit[1:0]=11 (Accel LN mode), Bit[3:2]=11 (Gyro LN mode)
    uint8_t config = 0b00001111;  // 保持原配置，这实际上是 LN 模式
    ICM42688P_WriteRegister(address, &config, 1);
}

/**
 * @brief 停止传感器
 *
 * 配置传感器进入低功耗模式
 */
void ICM42688P_Stop(void)
{
    ICM42688P_Bank_Select(0);
    // 增加延时，确保Bank切换完成
    for(volatile uint32_t i = 0; i < 24000; i++);  // 5ms @ 480MHz
    
    uint8_t address = 0x4e;
    uint8_t config = 0;
    ICM42688P_WriteRegister(address, &config, 1);
}

/**
 * @brief 配置输出数据速率(ODR)
 *
 * 配置加速度计和陀螺仪的输出数据速率
 */
void ICM42688P_ODR_Config(void)
{
    ICM42688P_Bank_Select(0);
    // 增加延时，确保Bank切换完成
    for(volatile uint32_t i = 0; i < 24000; i++);  // 5ms @ 480MHz
    
    uint8_t config = 1;
    ICM42688P_WriteRegister(0x4f, &config, 1);
    ICM42688P_WriteRegister(0x50, &config, 1);
}

/**
 * @brief 配置时钟
 *
 * 配置传感器的时钟源和时钟设置
 */
void ICM42688P_Clock_Config(void)
{
    // 切换到Bank 1
    ICM42688P_Bank_Select(1);
    // 增加延时，确保Bank切换完成
    for(volatile uint32_t i = 0; i < 48000; i++);  // 10ms @ 480MHz
    
    uint8_t config = 0x04;
    ICM42688P_WriteRegister(0x7b, &config, 1);
    
    // 切换回Bank 0
    ICM42688P_Bank_Select(0);
    // 增加延时，确保Bank切换完成
    for(volatile uint32_t i = 0; i < 48000; i++);  // 10ms @ 480MHz
    
    config = 0x95;
    ICM42688P_WriteRegister(0x4d, &config, 1);
}

/**
 * @brief 配置中断
 *
 * 配置传感器的中断设置
 */
void ICM42688P_Interrupt_Config(void)
{
    ICM42688P_Bank_Select(0);
    // 增加延时，确保Bank切换完成
    for(volatile uint32_t i = 0; i < 24000; i++);  // 5ms @ 480MHz
    
    uint8_t config = 0x2;
    ICM42688P_WriteRegister(0x14, &config, 1);
    config = 0x8;
    ICM42688P_WriteRegister(0x65, &config, 1);
}

/**
 * @brief 解析12字节数据为6个int16数据
 * @param data 输入的12字节数据
 * @param output 输出的6个int16数据数组
 */
void parse_12bytes_to_6int16(uint8_t *data, int16_t *output)
{
    for (int i = 0; i < 6; i++)
    {
        // 每个int16占两个字节
        // 先获取高位字节然后左移8位确定高位
        output[i] = (int16_t)(data[i * 2] << 8);
        // 然后把低位字节放在低位
        output[i] |= data[i * 2 + 1];
    }
}

/**
 * @brief 将IMU原始数据转换为物理量
 * @param input 输入的6个int16原始数据
 * @param output 输出的6个double物理量数据
 */
void parse_imu_data_to_physical(int16_t *input, double *output)
{
    for (int i = 0; i < 6; i++)
    {
        if (i < 3)
        {                                                     // 前三个是加速度计数据
            output[i] = (double)input[i] * ACCEL_SENSITIVITY; // 加速度计
        }
        else
        {                                                    // 后三个是陀螺仪数据
            output[i] = (double)input[i] * GYRO_SENSITIVITY; // 陀螺仪
        }
    }
}

/**
 * @brief 读取IMU数据（包含温度）- 优化版
 * @param data 指向IMU_Data结构体的指针，用于存储读取的数据
 *
 * 从0x1D开始一次性读取14个字节：
 * - 0x1D: 温度高字节
 * - 0x1E: 温度低字节
 * - 0x1F-0x2A: 加速度计和陀螺仪数据（12字节）
 * 
 * 优化: 添加延时,确保Bank选择生效，增加数据验证
 */
void ICM42688P_ReadIMUData(IMU_Data *data)
{
    uint8_t raw_data[14]; // 2字节温度 + 12字节IMU数据
    int16_t int16_data[6];
    double physical_data[6];
    
    // 初始化为0,防止读取失败时使用未初始化的数据
    for(int i = 0; i < 14; i++) {
        raw_data[i] = 0;
    }

    // 确保在 Bank 0（温度和IMU数据寄存器在Bank 0）
    // 注意：减少不必要的Bank切换，如果已经在Bank 0就不切换
    static uint8_t current_bank = 0xFF;  // 0xFF表示未知
    
    if(current_bank != 0) {
        ICM42688P_Bank_Select(0);
        // Bank选择后延时,确保切换完成
        for(volatile uint32_t i = 0; i < 14400; i++);  // 约3ms @ 480MHz（增加延时）
        current_bank = 0;
    } else {
        // 即使在正确的Bank，也添加小延时确保稳定
        for(volatile uint32_t i = 0; i < 4800; i++);  // 约1ms @ 480MHz
    }
    
    // 从0x1D开始一次性读取14个字节
    ICM42688P_ReadRegister(0x1D, raw_data, 14);
    
    // 添加读取后延时
    for(volatile uint32_t i = 0; i < 2400; i++);  // 约0.5ms @ 480MHz

    // ===== 数据验证：检查是否读取到全0或全FF =====
    uint8_t all_zero = 1;
    uint8_t all_ff = 1;
    for(int i = 2; i < 14; i++) {  // 跳过温度数据，只检查IMU数据
        if(raw_data[i] != 0x00) all_zero = 0;
        if(raw_data[i] != 0xFF) all_ff = 0;
    }
    
    // 如果数据异常，尝试重新读取一次
    if(all_zero || all_ff) {
        for(volatile uint32_t i = 0; i < 9600; i++);  // 2ms延时
        ICM42688P_ReadRegister(0x1D, raw_data, 14);
        for(volatile uint32_t i = 0; i < 2400; i++);  // 0.5ms延时
    }

    // 解析温度数据（前2个字节）
    // 温度寄存器：0x1D(高字节) 0x1E(低字节)
    uint16_t temp_raw = (raw_data[0] << 8) | raw_data[1];
    
    // 安全的温度计算,避免除零或异常值
    if(temp_raw == 0 || temp_raw == 0xFFFF) {
        data->temperature = 25.0f;  // 默认室温
    } else {
        data->temperature = ((float)temp_raw / 132.48f) + 25.0f;
    }

    // 解析IMU数据（后12个字节）
    // 注意：ICM42688P数据寄存器顺序是 AccelX, AccelY, AccelZ, GyroX, GyroY, GyroZ
    parse_12bytes_to_6int16(raw_data + 2, int16_data);
    parse_imu_data_to_physical(int16_data, physical_data);

    data->accel_x = physical_data[0];
    data->accel_y = physical_data[1];
    data->accel_z = physical_data[2];
    data->gyro_x = physical_data[3];
    data->gyro_y = physical_data[4];
    data->gyro_z = physical_data[5];

    // 陀螺仪阈值滤波
    // float epsilon = FLT_EPSILON;
    // const float threshold = 0.2;

    // if (fabs(data->gyro_x) < threshold + epsilon)
    // {
    //     data->gyro_x = 0.0;
    // }

    // if (fabs(data->gyro_y) < threshold + epsilon)
    // {
    //     data->gyro_y = 0.0;
    // }

    // if (fabs(data->gyro_z) < threshold + epsilon)
    // {
    //     data->gyro_z = 0.0;
    // }
}

/**
 * @brief 读取寄存器（对外API接口）
 * @param reg_address 寄存器地址
 * @param rxdata 接收数据缓冲区
 * @param length 数据长度
 *
 * @note 这是对外的API接口函数，适用于以下场景：
 *       - 单字节寄存器读取（如读取WHO_AM_I）
 *       - 连续多字节寄存器读取（如读取12字节IMU数据）
 *       - 不需要频繁切换Bank的批量读取
 *
 * @example 读取单个寄存器：
 *          uint8_t whoami;
 *          ICM42688P_ReadRegister(ICM42688P_WHOAMI, &whoami, 1);
 *
 * @example 批量读取IMU数据（加速度+陀螺仪）：
 *          uint8_t imu_data[12];
 *          ICM42688P_ReadRegister(0x1F, imu_data, 12);
 */
void ICM42688P_ReadRegister(uint8_t reg_address, uint8_t *rxdata, uint8_t length)
{
    // 使用完全手动的寄存器读取方法，不依赖 HAL 库超时
    ICM42688P_ManualRead(reg_address, rxdata, length);
}

/**
 * @brief 写入寄存器（简化版，不验证）
 * @param reg_address 寄存器地址
 * @param txdata 发送数据缓冲区
 * @param length 数据长度
 * @return 0表示成功，1表示失败
 */
uint8_t ICM42688P_WriteRegister(uint8_t reg_address, uint8_t *txdata, uint8_t length)
{
    // 确保 SPI 已使能
    if(!(hspi4.Instance->CR1 & 0x00000001)) {
        hspi4.Instance->CR1 |= 0x00000001;
        for(volatile uint32_t i = 0; i < 4800; i++);  // 约1ms @ 480MHz
    }
    
    cs_low();
    // 片选延时，改用 for 循环避免 HAL_Delay
    for(volatile uint32_t i = 0; i < 2400; i++);  // 约0.5ms @ 480MHz
    
    // 发送地址（写操作，地址不需要 OR 0x80）
    SPI_ReadWriteByte_Manual(reg_address);
    
    // 发送数据
    for(uint8_t i = 0; i < length; i++) {
        SPI_ReadWriteByte_Manual(txdata[i]);
    }
    
    // 等待最后一个字节传输完成
    uint32_t timeout = 100000;
    while(!(hspi4.Instance->SR & (1 << 1)) && timeout--);  // 等待 TXC=1
    
    // 释放片选前短暂延时
    for(volatile uint32_t i = 0; i < 480; i++);  // 约0.1ms
    
    cs_high();
    
    // 片选释放延时，给ICM处理时间
    for(volatile uint32_t i = 0; i < 9600; i++);  // 约2ms @ 480MHz
    
    // 清除传输完成标志
    hspi4.Instance->IFCR = 0xFFFFFFFF;

    return 0;
}

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
void ICM42688P_DetectAndDisplay(void)
{
    char display_str[64];
    uint16_t y_pos = 10;
    const uint16_t line_height = 16;
    
    // ===== 步骤1: LCD初始化检查 =====
    LCD_SetBackColor(LCD_BLACK);
    LCD_SetColor(LCD_WHITE);
    LCD_Clear();
    
    LCD_DisplayString(10, y_pos, "=== ICM42688P Test ===");
    y_pos += line_height * 2;
    
    // ===== 步骤2: 硬件检测 =====
    LCD_DisplayString(10, y_pos, "Hardware Check:");
    y_pos += line_height;
    
    // 检查CS引脚控制
    cs_high();
    HAL_Delay(2);
    uint8_t cs_high_state = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11);
    
    cs_low();
    HAL_Delay(2);
    uint8_t cs_low_state = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11);
    
    cs_high();
    HAL_Delay(2);
    
    sprintf(display_str, "CS H:%d L:%d", cs_high_state, cs_low_state);
    LCD_DisplayString(10, y_pos, display_str);
    
    if(cs_high_state == 1 && cs_low_state == 0) {
        LCD_SetColor(LCD_GREEN);
        LCD_DisplayString(140, y_pos, "OK");
        LCD_SetColor(LCD_WHITE);
    } else {
        LCD_SetColor(LCD_RED);
        LCD_DisplayString(140, y_pos, "FAIL");
        LCD_SetColor(LCD_WHITE);
        HAL_Delay(3000);
        return;
    }
    y_pos += line_height;
    
    // ===== 步骤3: 传感器初始化 =====
    LCD_DisplayString(10, y_pos, "Initializing...");
    y_pos += line_height;
    
    uint32_t start_tick = HAL_GetTick();
    uint8_t init_result = ICM42688P_Init();
    uint32_t init_time = HAL_GetTick() - start_tick;
    
    sprintf(display_str, "Init Time: %lu ms", init_time);
    LCD_DisplayString(10, y_pos, display_str);
    y_pos += line_height;
    
    if(init_result == 0) {
        LCD_SetColor(LCD_GREEN);
        LCD_DisplayString(10, y_pos, "Init SUCCESS!");
        LCD_SetColor(LCD_WHITE);
    } else {
        LCD_SetColor(LCD_RED);
        sprintf(display_str, "Init FAIL! Code:%d", init_result);
        LCD_DisplayString(10, y_pos, display_str);
        LCD_SetColor(LCD_WHITE);
        
        y_pos += line_height;
        LCD_SetColor(LCD_YELLOW);
        if(init_result == 1) {
            LCD_DisplayString(10, y_pos, "WHOAMI Error");
        } else if(init_result == 2) {
            LCD_DisplayString(10, y_pos, "Config Error");
        } else if(init_result == 3) {
            LCD_DisplayString(10, y_pos, "Sensor Not Start");
        }
        LCD_SetColor(LCD_WHITE);
        
        HAL_Delay(3000);
        return;
    }
    y_pos += line_height;
    
    // ===== 步骤4: 验证WHOAMI =====
    uint8_t whoami = ICM42688P_Test_DirectRead();
    sprintf(display_str, "WHOAMI: 0x%02X", whoami);
    LCD_DisplayString(10, y_pos, display_str);
    
    if(whoami == 0x47) {
        LCD_SetColor(LCD_GREEN);
        LCD_DisplayString(140, y_pos, "OK");
        LCD_SetColor(LCD_WHITE);
    } else {
        LCD_SetColor(LCD_RED);
        LCD_DisplayString(140, y_pos, "ERR");
        LCD_SetColor(LCD_WHITE);
    }
    y_pos += line_height;
    
    // ===== 步骤5: 读取电源管理状态 =====
    uint8_t pwr_status = 0;
    ICM42688P_Bank_Select(0);
    ICM42688P_ReadRegister(0x4E, &pwr_status, 1);
    
    sprintf(display_str, "PWR_MGMT: 0x%02X", pwr_status);
    LCD_DisplayString(10, y_pos, display_str);
    
    if(pwr_status == 0x0F) {
        LCD_SetColor(LCD_GREEN);
        LCD_DisplayString(140, y_pos, "ON");
        LCD_SetColor(LCD_WHITE);
    } else {
        LCD_SetColor(LCD_YELLOW);
        LCD_DisplayString(140, y_pos, "?");
        LCD_SetColor(LCD_WHITE);
    }
    y_pos += line_height * 2;
    
    LCD_SetColor(LCD_CYAN);
    LCD_DisplayString(10, y_pos, "Press to continue...");
    LCD_SetColor(LCD_WHITE);
    
    HAL_Delay(3000);
    
    // ===== 步骤6: 实时数据显示 =====
    LCD_Clear();
    LCD_DisplayString(10, 10, "=== IMU Data ===");
    
    // 显示标签
    LCD_DisplayString(10, 30, "Accel(g):");
    LCD_DisplayString(10, 70, "Gyro(dps):");
    LCD_DisplayString(10, 110, "Temp(C):");
    LCD_DisplayString(10, 130, "Rate(Hz):");
    
    uint32_t frame_count = 0;
    uint32_t last_time = HAL_GetTick();
    float fps = 0.0f;
    
    // 持续显示数据（可以通过按键退出，这里演示连续显示100次）
    for(int loop = 0; loop < 100; loop++) {
        // 读取IMU数据
        ICM42688P_ReadIMUData(&imu_data);
        
        // 显示加速度数据
        sprintf(display_str, "X:%6.2f", imu_data.accel_x);
        LCD_DisplayString(10, 46, display_str);
        
        sprintf(display_str, "Y:%6.2f", imu_data.accel_y);
        LCD_DisplayString(10, 54, display_str);
        
        sprintf(display_str, "Z:%6.2f", imu_data.accel_z);
        LCD_DisplayString(10, 62, display_str);
        
        // 显示陀螺仪数据
        sprintf(display_str, "X:%7.1f", imu_data.gyro_x);
        LCD_DisplayString(10, 86, display_str);
        
        sprintf(display_str, "Y:%7.1f", imu_data.gyro_y);
        LCD_DisplayString(10, 94, display_str);
        
        sprintf(display_str, "Z:%7.1f", imu_data.gyro_z);
        LCD_DisplayString(10, 102, display_str);
        
        // 显示温度
        sprintf(display_str, "%5.1f     ", imu_data.temperature);
        LCD_DisplayString(80, 110, display_str);
        
        // 计算并显示刷新率
        frame_count++;
        if(frame_count >= 10) {
            uint32_t current_time = HAL_GetTick();
            uint32_t elapsed = current_time - last_time;
            if(elapsed > 0) {
                fps = (float)frame_count * 1000.0f / (float)elapsed;
            }
            frame_count = 0;
            last_time = current_time;
        }
        
        sprintf(display_str, "%5.1f     ", fps);
        LCD_DisplayString(80, 130, display_str);
        
        // 状态指示
        LCD_SetColor(LCD_GREEN);
        LCD_DisplayString(10, 150, "Running...");
        LCD_SetColor(LCD_WHITE);
        
        // 控制刷新率，避免过快
        HAL_Delay(20);  // 50Hz刷新
    }
    
    // 测试完成
    LCD_SetColor(LCD_YELLOW);
    LCD_DisplayString(10, 170, "Test Complete!");
    LCD_SetColor(LCD_WHITE);
    
    HAL_Delay(2000);
}

/**
 * @brief ICM42688P 快速状态检测（用于调试）
 * @return 0=正常, 1=WHOAMI错误, 2=传感器未启动
 * 
 * 该函数快速检测ICM传感器状态，不初始化，只读取关键寄存器
 */
uint8_t ICM42688P_QuickCheck(void)
{
    // 读取WHOAMI
    uint8_t whoami = ICM42688P_Test_DirectRead();
    if(whoami != 0x47) {
        return 1;  // WHOAMI错误
    }
    
    // 读取PWR_MGMT0
    uint8_t pwr_status = 0;
    ICM42688P_Bank_Select(0);
    ICM42688P_ReadRegister(0x4E, &pwr_status, 1);
    
    if(pwr_status != 0x0F) {
        return 2;  // 传感器未启动
    }
    
    return 0;  // 正常
}

/**
 * @brief 在LCD上显示简化的IMU数据（适合集成到主循环）
 * @param x 显示起始X坐标
 * @param y 显示起始Y坐标
 * 
 * 该函数显示紧凑的IMU数据，适合在主循环中周期性调用
 */
void ICM42688P_DisplayCompact(uint16_t x, uint16_t y)
{
    char str[32];
    
    // 读取数据
    ICM42688P_ReadIMUData(&imu_data);
    
    // 显示加速度（一行）
    sprintf(str, "A:%4.1f,%4.1f,%4.1f", 
            imu_data.accel_x, imu_data.accel_y, imu_data.accel_z);
    LCD_DisplayString(x, y, str);
    
    // 显示角速度（一行）
    sprintf(str, "G:%4.0f,%4.0f,%4.0f", 
            imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z);
    LCD_DisplayString(x, y + 12, str);
    
    // 显示温度
    sprintf(str, "T:%4.1fC", imu_data.temperature);
    LCD_DisplayString(x, y + 24, str);
}
