#define __EC11_H
#include "stm32h7xx_hal.h"
#include "stm32h7xx_it.h"

#define EC11_B_PIN GPIO_PIN_0  // A相
#define EC11_B_PORT GPIOB
#define EC11_C_PIN GPIO_PIN_5// B相
#define EC11_C_PORT GPIOC
#define EC11_SW_PIN GPIO_PIN_4// 按键
#define EC11_SW_PORT GPIOC
extern volatile int32_t encoderCount;
extern volatile uint8_t buttonPressed;
extern volatile uint8_t lastState;
// 函数声明
void EC11_Init(void);
int32_t EC11_GetCount(void);
void EC11_ResetCount(void);
uint8_t EC11_IsButtonPressed(void);