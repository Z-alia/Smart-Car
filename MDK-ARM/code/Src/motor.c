#include "motor.h"
#include "main.h"
#include "tim.h"

//pid控制器初始化
void pid_init(PIDController* pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->error = 0.0;
    pid->last_error = 0.0;
    pid->integral = 0.0;
    pid->derivative = 0.0;
    pid->output = 0.0;
}

//pid计算
float pid_calculate(PIDController* pid, float setpoint, float feedback) {
    // 计算误差
    pid->error = setpoint - feedback;
    
    // 计算积分项
    pid->integral += pid->error;
    
    // 计算微分项
    pid->derivative = pid->error - pid->last_error;
    
    // 计算输出
    pid->output = pid->kp * pid->error + 
                  pid->ki * pid->integral + 
                  pid->kd * pid->derivative;
    
    // 保存上次误差
    pid->last_error = pid->error;
    
    return pid->output;
}

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