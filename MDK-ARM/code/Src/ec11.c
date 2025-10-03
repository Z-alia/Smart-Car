#include "stm32h7xx_it.h"
#include "stm32h7xx_hal.h"
#include "ec11.h"
volatile int32_t encoderCount = 0;
volatile int32_t last_encoder_state = 0;
volatile uint8_t buttonPressed = 0;

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