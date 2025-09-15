#include "motor.h"
#include "main.h"
#include "tim.h"
// 初始化电机驱动
void motor_init(void)
{
    // 初始化电机控制引脚
    HAL_GPIO_WritePin(GPIOB, MOTOR_RIGHT_DIRE_Pin|MOTOR_LEFT_DIRE_Pin, GPIO_PIN_RESET);

}

// 控制电机运行 (speed: -100 to 100)
void motor_run(int speed)
{

}

//左转
void motor_turnleft(void)
{

}

// 右转
void motor_turnright(void)
{

}

//滑行
void motor_coast(void)
{
    HAL_GPIO_WritePin(GPIOB, MOTOR_RIGHT_DIRE_Pin|MOTOR_LEFT_DIRE_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1|TIM_CHANNEL_3, 0);
}

// 刹车
void motor_stop(void)
{
    
}