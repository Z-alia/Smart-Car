#include "motor.h"
#include "main.h"
#include "tim.h"
//本工程的PWM分辨率为1000
#define tgtspd 700
#define TL_tgtspd 900
#define TR_tgtspd 900
Motor leftmotor={0, 1, tgtspd, 0};
Motor rightmotor={0, 1, tgtspd, 1};
PIDController pid;

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
    pid->integral_limit = 500.0;
    pid->output_limit = 500.0;
}

//pid计算 setpoint希望是0 feedback时摄像头输出的偏差值
float pid_calculate(PIDController* pid, float setpoint, float feedback) {
    // 计算误差
    pid->error = setpoint - feedback;
    
    // 计算积分项
    pid->integral += pid->error;
    pid->integral = (pid->integral > pid->integral_limit) ? pid->integral_limit : ((pid->integral < -pid->integral_limit) ? -pid->integral_limit : pid->integral);
    // 计算微分项
    pid->derivative = pid->error - pid->last_error;
    
    // 计算输出
    float output = pid->kp * pid->error + 
                  pid->ki * pid->integral + 
                  pid->kd * pid->derivative;
    pid->output = (output > pid->output_limit) ? pid->output_limit : ((output < -pid->output_limit) ? -pid->output_limit : output);
    // 保存上次误差
    pid->last_error = pid->error;
    
    return pid->output;
}

// 初始化电机驱动
void motor_init(void)
{
    // 初始化电机控制引脚
    HAL_GPIO_WritePin(GPIOB, MOTOR_RIGHT_DIRE_Pin|MOTOR_LEFT_DIRE_Pin, GPIO_PIN_RESET);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    //这里记得加个停车/滑行

}

// 控制单个电机运行 (speed: -1000 to 1000)
void motor_run(Motor *motor_ptr, int16_t speed)
{
    motor_ptr->speed = speed;
    if(motor_ptr->lor == 0) // 左电机
    {
        if(speed >= 0)
        {
            motor_ptr->dir = 1; // 正转
            HAL_GPIO_WritePin(GPIOB, MOTOR_LEFT_DIRE_Pin, GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, speed);
        }
        else if(speed < 0)
        {
            motor_ptr->dir = 0; // 反转
            speed = -speed;
            HAL_GPIO_WritePin(GPIOB, MOTOR_LEFT_DIRE_Pin, GPIO_PIN_SET);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, speed);
        }
    }
    else // 右电机
    {
        if(speed >= 0)
        {
            motor_ptr->dir = 1; // 正转
            HAL_GPIO_WritePin(GPIOB, MOTOR_RIGHT_DIRE_Pin, GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
        }
        else if(speed < 0)
        {
            motor_ptr->dir = 0; // 反转
            speed = -speed;
            HAL_GPIO_WritePin(GPIOB, MOTOR_RIGHT_DIRE_Pin, GPIO_PIN_SET);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
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
    HAL_GPIO_WritePin(GPIOB, MOTOR_RIGHT_DIRE_Pin|MOTOR_LEFT_DIRE_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1|TIM_CHANNEL_3, 0);
}

// 刹车
void motor_stop(void)
{
    motor_run(&leftmotor, 0);
    motor_run(&rightmotor, 0);
}