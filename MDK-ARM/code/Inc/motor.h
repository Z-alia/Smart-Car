#ifndef __MOTOR_H
#define __MOTOR_H

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
} PIDController;

// 单电机控制结构体
typedef struct {
    volatile int8_t speed;     // 速度
    volatile uint8_t dir;      // 方向（1正转）
    volatile int8_t target_speed;   // 目标速度
    uint8_t lor;                // 左或右电机标识（0左，1右）
} Motor;

// PID控制器初始化
void pid_init(PIDController* pid, float kp, float ki, float kd);

//pid计算
float pid_calculate(PIDController* pid, float setpoint, float feedback);

// 初始化电机驱动
void motor_init(void);

// 控制电机运行 (speed: -100 to 100)
void motor_run(int speed);

//左转
void motor_turnleft(void);

// 右转
void motor_turnright(void);

//滑行
void motor_coast(void);

// 刹车
void motor_stop(void);


#endif /* __MOTOR_H */