#include "motor.h"
#include "main.h"
#include "tim.h"
#include <math.h>   // fabsf
#include "Element_recognition.h"
#include "Binarization.h"
//本工程的PWM分辨率为1000
#define tgtspd 100
#define TL_tgtspd 100
#define TR_tgtspd 100
Motor leftmotor={0, 1, tgtspd, 0};
Motor rightmotor={0, 1, tgtspd, 1};

// 初始化电机驱动
void motor_init(void)
{
    // 初始化电机控制引脚
    HAL_GPIO_WritePin(PH_LMOTOR_GPIO_Port, PH_LMOTOR_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PH_RMOTOR_GPIO_Port, PH_RMOTOR_Pin, GPIO_PIN_SET);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    motor_stop();
}

// 控制单个电机运行 (speed: -1000 to 1000)
void motor_run(Motor *motor_ptr, int16_t speed)
{
    if ((speed<-1000)||(speed>1000))
        return;
    motor_ptr->speed = speed;
    if(motor_ptr->lor == 0) // 左电机
    {
        if(speed >= 0)
        {
            motor_ptr->dir = 1; // 正转
            HAL_GPIO_WritePin(PH_LMOTOR_GPIO_Port, PH_LMOTOR_Pin, GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
        }
        else if(speed < 0)
        {
            motor_ptr->dir = 0; // 反转
            speed = -speed;
            HAL_GPIO_WritePin(PH_LMOTOR_GPIO_Port, PH_LMOTOR_Pin, GPIO_PIN_SET);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
        }
    }
    else // 右电机
    {
        if(speed >= 0)
        {
            motor_ptr->dir = 1; // 正转
            HAL_GPIO_WritePin(PH_RMOTOR_GPIO_Port, PH_RMOTOR_Pin, GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);
        }
        else if(speed < 0)
        {
            motor_ptr->dir = 0; // 反转
            speed = -speed;
            HAL_GPIO_WritePin(PH_RMOTOR_GPIO_Port, PH_RMOTOR_Pin, GPIO_PIN_SET);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);
        }
    }
}

//左转
void motor_turnleft(void)
{
    motor_run(&leftmotor, tgtspd);
    motor_run(&rightmotor, TR_tgtspd);
}

// 右转
void motor_turnright(void)
{
    motor_run(&leftmotor, TL_tgtspd);
    motor_run(&rightmotor, tgtspd);
}

//滑行
void motor_coast(void)
{
    HAL_GPIO_WritePin(PH_LMOTOR_GPIO_Port, PH_LMOTOR_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PH_RMOTOR_GPIO_Port, PH_RMOTOR_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
}

// 刹车
void motor_stop(void)
{
    motor_run(&leftmotor, 0);
    motor_run(&rightmotor, 0);
}
/*
void straight_error_get(PIDController *PID)
{
    uint8_t edge_store[10]; // 存储边沿位置的数组，大小根据实际情况调整
    uint8_t left=0, right=0;
    for(uint8_t i=0;i<10;i++)
    {
       if(edge_store[i]>left&&edge_store[i]<94)
           left=edge_store[i];
        else if(edge_store[i]<right&&edge_store[i]>94)
            right=edge_store[i];
    }
    get_orign_edges(imo[5],edge_store);
    PID->error = (left + right) / 2 - 94;
}
*/