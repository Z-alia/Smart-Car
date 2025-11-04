#ifndef __MOTOR_H
#define __MOTOR_H
#include "stdint.h"
// 单电机控制结构体
typedef struct {
    volatile int16_t speed;     // 速度  这里的速度储存编码器测速值 其实暂时没什么用 后面可以试试与target_speed做差进行分段速度变化
    volatile uint8_t dir;      // 方向（1正,0反）
    volatile int16_t target_speed;   // 目标速度  目标速度用于接收pid系统返值并传参给电机控制函数
    uint8_t lor;                // 左或右电机标识（0左,1右）
} Motor;

// 初始化电机驱动
void motor_init(void);

// 控制电机运行 (speed: -1000 to 1000)
void motor_run(Motor *motor_ptr, int16_t speed);

//左转
void motor_turnleft(void);

// 右转
void motor_turnright(void);

//滑行
void motor_coast(void);

// 刹车
void motor_stop(void);

//循迹
void motor_follow_line(void);

extern Motor leftmotor;
extern Motor rightmotor;

#endif 