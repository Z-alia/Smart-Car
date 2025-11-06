/**
 * @file ICM42688_QuickTest.c
 * @brief ICM-42688P 快速测试代码片段
 * 
 * 将这些代码添加到 main.c 中的适当位置进行测试
 */

// ==================== 测试 1: 基础硬件检查 ====================
// 在 main() 的 USER CODE BEGIN 2 部分，LCD_Init() 之后添加：

void ICM_HardwareCheck(void)
{
    char buf[32];
    LCD_SetBackColor(LCD_BLACK);
    LCD_SetColor(LCD_WHITE);
    LCD_Clear();
    
    LCD_DisplayString(10, 10, "=== HW Check ===");
    
    // 1. GPIO 配置检查
    uint32_t pe11_mode = (GPIOE->MODER >> 22) & 0x3;
    sprintf(buf, "PE11 MODE: %d", (int)pe11_mode);
    LCD_DisplayString(10, 30, buf);
    LCD_SetColor(pe11_mode == 1 ? LCD_GREEN : LCD_RED);
    LCD_DisplayString(150, 30, pe11_mode == 1 ? "OK" : "ERR");
    LCD_SetColor(LCD_WHITE);
    
    // 2. CS 引脚测试
    cs_high();
    HAL_Delay(5);
    uint8_t cs_h = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11);
    
    cs_low();
    HAL_Delay(5);
    uint8_t cs_l = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11);
    
    cs_high();
    
    sprintf(buf, "CS H:%d L:%d", cs_h, cs_l);
    LCD_DisplayString(10, 50, buf);
    LCD_SetColor((cs_h && !cs_l) ? LCD_GREEN : LCD_RED);
    LCD_DisplayString(150, 50, (cs_h && !cs_l) ? "OK" : "ERR");
    LCD_SetColor(LCD_WHITE);
    
    // 3. SPI 状态检查
    sprintf(buf, "SPI CR1: 0x%04X", (uint16_t)hspi4.Instance->CR1);
    LCD_DisplayString(10, 70, buf);
    
    sprintf(buf, "SPI SR: 0x%04X", (uint16_t)hspi4.Instance->SR);
    LCD_DisplayString(10, 90, buf);
    
    // 4. SPI 使能检查
    uint8_t spi_en = (hspi4.Instance->CR1 & 0x1) ? 1 : 0;
    sprintf(buf, "SPI EN: %d", spi_en);
    LCD_DisplayString(10, 110, buf);
    LCD_SetColor(spi_en ? LCD_GREEN : LCD_RED);
    LCD_DisplayString(150, 110, spi_en ? "OK" : "ERR");
    LCD_SetColor(LCD_WHITE);
    
    HAL_Delay(3000);
}


// ==================== 测试 2: 简单 SPI 通信测试 ====================
void ICM_SimpleReadTest(void)
{
    char buf[32];
    LCD_Clear();
    LCD_DisplayString(10, 10, "=== Read Test ===");
    
    // 测试 5 次读取
    for(int i = 0; i < 5; i++)
    {
        uint8_t whoami = 0;
        
        cs_low();
        HAL_Delay(5);
        
        // 发送地址
        uint8_t addr = 0xF5;  // 0x75 | 0x80
        HAL_SPI_Transmit(&hspi4, &addr, 1, 100);
        HAL_Delay(1);
        
        // 读取数据
        HAL_SPI_Receive(&hspi4, &whoami, 1, 100);
        
        cs_high();
        HAL_Delay(5);
        
        sprintf(buf, "Try %d: 0x%02X", i+1, whoami);
        LCD_DisplayString(10, 30 + i*20, buf);
        
        if(whoami == 0x47) {
            LCD_SetColor(LCD_GREEN);
            LCD_DisplayString(150, 30 + i*20, "OK!");
            LCD_SetColor(LCD_WHITE);
        }
        
        HAL_Delay(100);
    }
    
    HAL_Delay(3000);
}


// ==================== 测试 3: 手动字节级 SPI 测试 ====================
void ICM_ManualSPITest(void)
{
    char buf[32];
    LCD_Clear();
    LCD_DisplayString(10, 10, "=== Manual Test ===");
    
    // 确保 SPI 使能
    if(!(hspi4.Instance->CR1 & 0x1)) {
        hspi4.Instance->CR1 |= 0x1;
        HAL_Delay(10);
    }
    
    cs_low();
    HAL_Delay(5);
    
    // 发送地址字节
    while(!(hspi4.Instance->SR & SPI_SR_TXP));
    *((__IO uint8_t *)&hspi4.Instance->TXDR) = 0xF5;
    hspi4.Instance->CR1 |= 0x200;  // CSTART
    
    while(!(hspi4.Instance->SR & SPI_SR_RXP));
    uint8_t dummy = *((__IO uint8_t *)&hspi4.Instance->RXDR);
    
    sprintf(buf, "Addr Echo: 0x%02X", dummy);
    LCD_DisplayString(10, 30, buf);
    
    HAL_Delay(2);
    
    // 读取数据字节
    while(!(hspi4.Instance->SR & SPI_SR_TXP));
    *((__IO uint8_t *)&hspi4.Instance->TXDR) = 0xFF;
    hspi4.Instance->CR1 |= 0x200;
    
    while(!(hspi4.Instance->SR & SPI_SR_RXP));
    uint8_t whoami = *((__IO uint8_t *)&hspi4.Instance->RXDR);
    
    // 等待完成
    while(!(hspi4.Instance->SR & 0x8));
    hspi4.Instance->IFCR |= 0x8;
    
    cs_high();
    
    sprintf(buf, "WHOAMI: 0x%02X", whoami);
    LCD_DisplayString(10, 50, buf);
    
    if(whoami == 0x47) {
        LCD_SetColor(LCD_GREEN);
        LCD_DisplayString(10, 70, "SUCCESS!");
    } else {
        LCD_SetColor(LCD_RED);
        LCD_DisplayString(10, 70, "FAILED!");
    }
    LCD_SetColor(LCD_WHITE);
    
    HAL_Delay(3000);
}


// ==================== 测试 4: 完整初始化测试 ====================
void ICM_FullInitTest(void)
{
    char buf[32];
    LCD_Clear();
    LCD_DisplayString(10, 10, "=== Init Test ===");
    
    // 1. 复位
    LCD_DisplayString(10, 30, "Reset...");
    ICM42688P_Software_Reset();
    HAL_Delay(100);
    LCD_DisplayString(10, 30, "Reset... OK");
    
    // 2. Bank 0
    LCD_DisplayString(10, 50, "Bank 0...");
    ICM42688P_Bank_Select(0);
    HAL_Delay(10);
    LCD_DisplayString(10, 50, "Bank 0... OK");
    
    // 3. 读取 WHOAMI
    LCD_DisplayString(10, 70, "Read WHOAMI...");
    uint8_t whoami = 0;
    ICM42688P_ReadRegister(ICM42688P_WHOAMI, &whoami, 1);
    
    sprintf(buf, "WHOAMI: 0x%02X", whoami);
    LCD_DisplayString(10, 90, buf);
    
    if(whoami == 0x47) {
        LCD_SetColor(LCD_GREEN);
        LCD_DisplayString(10, 110, "SUCCESS!");
        LCD_SetColor(LCD_WHITE);
        
        // 4. 完成配置
        LCD_DisplayString(10, 130, "Config...");
        ICM42688P_Clock_Config();
        ICM42688P_ODR_Config();
        ICM42688P_Start();
        LCD_DisplayString(10, 130, "Config... OK");
        
    } else {
        LCD_SetColor(LCD_RED);
        LCD_DisplayString(10, 110, "FAILED!");
        LCD_SetColor(LCD_WHITE);
    }
    
    HAL_Delay(3000);
}


// ==================== 使用方法 ====================
/*
在 main.c 的 main() 函数中，在 USER CODE BEGIN 2 部分添加：

  LCD_Init();
  HAL_Delay(100);
  
  // 运行测试（按顺序）
  ICM_HardwareCheck();      // 测试 1
  ICM_SimpleReadTest();     // 测试 2
  ICM_ManualSPITest();      // 测试 3
  ICM_FullInitTest();       // 测试 4
  
  // 然后继续正常的初始化
  uint8_t icm_result = ICM42688P_Init();
  ...

按下复位按钮，观察 LCD 显示的每一步结果。
找到第一个失败的测试，然后针对性地解决。
*/
