#ifndef __MOTOR_H
#define __MOTOR_H
#include "stdint.h"
// PID控制结构体
typedef struct {
    volatile float kp;           // 比例系数
    volatile float ki;           // 积分系数
    volatile float kd;           // 微分系数
    volatile float error;        // 当前误差
    volatile float last_error;   // 上次误差
    volatile float integral;     // 积分项
    volatile float derivative;   // 微分项
    volatile float output;       // 输出值
    float integral_limit;        // 积分限幅，初始化控制器时赋值
    float output_limit;          // 输出限幅
} PIDController;

// 单电机控制结构体
typedef struct {
    volatile int16_t speed;     // 速度  这里的速度储存编码器测速值 其实暂时没什么用 后面可以试试与target_speed做差进行分段速度变化
    volatile uint8_t dir;      // 方向（1正,0反）
    volatile int16_t target_speed;   // 目标速度  目标速度用于接收pid系统返值并传参给电机控制函数
    uint8_t lor;                // 左或右电机标识（0左,1右）
} Motor;

// PID控制器初始化
void pid_init(PIDController* pid, float kp, float ki, float kd);

//pid计算
float pid_calculate(PIDController* pid, float setpoint, float feedback);

// 初始化电机驱动
void motor_init(void);

// 控制电机运行 (speed: -100 to 100)
void motor_run(Motor *motor_ptr, int16_t speed);

//左转
void motor_turnleft(void);

// 右转
void motor_turnright(void);

//滑行
void motor_coast(void);

// 刹车
void motor_stop(void);

extern Motor leftmotor;
extern Motor rightmotor;

#endif /* __MOTOR_H */