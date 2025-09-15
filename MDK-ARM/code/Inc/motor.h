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


// 左电机参数结构体
typedef struct{
    volatile int8_t target_lspeed;
    volatile uint8_t ldir;
}LeftMotor;

// 右电机参数结构体
typedef struct{
    volatile int8_t target_rspeed;
    volatile uint8_t rdir;
}RightMotor;

// 电机参数结构体
typedef struct{
    LeftMotor left;
    RightMotor right;
}Motor;

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