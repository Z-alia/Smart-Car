#include "stm32h7xx_it.h"
#include "stm32h7xx_hal.h"
#include "ec11.h"
volatile int32_t encoderCount = 0;
volatile uint8_t lastState = 0;
volatile uint8_t buttonPressed = 0;
void EC11_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // 使能GPIO时钟
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    // 配置A相和B相为输入模式，带上拉电阻
    GPIO_InitStruct.Pin = EC11_B_PIN | EC11_C_PIN | EC11_SW_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(EC11_B_PORT, &GPIO_InitStruct);
    
    // 获取初始状态
    lastState = (HAL_GPIO_ReadPin(EC11_B_PORT, EC11_B_PIN) << 1) | 
                HAL_GPIO_ReadPin(EC11_C_PORT, EC11_C_PIN);
}
// 获取编码器计数值
int32_t EC11_GetCount(void) {
    return encoderCount;
}

// 重置编码器计数
void EC11_ResetCount(void) {
    encoderCount = 0;
}

// 检查按键是否按下
uint8_t EC11_IsButtonPressed(void) {
    if(buttonPressed) {
        buttonPressed = 0;
        return 1;
    }
    return 0;
}