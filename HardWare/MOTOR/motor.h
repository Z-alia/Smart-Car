#ifndef __MOTOR_H
#define __MOTOR_H
#include "stdint.h"
#include "Element_recognition.h"

//本工程的PWM分辨率为1000
#define tgtspd 300 //700改70
#define tgtspd_curve 175
#define TL_tgtspd 250
#define TR_tgtspd 250
#define weight_up 0.15//下部为0-30 中部为30-60 上部为60-120
#define weight_md 0.45
#define weight_dw 0.40
#define weight_curve_up 1.00
#define weight_curve_down 0.00

// PID控制结构体
typedef struct {
    volatile float kp;           // 比例系数
    volatile float ki;           // 积分系数
    volatile float kd;           // 微分系数
	volatile float kff;			 // 前馈系数
	volatile float ahead;		 // 前馈曲率
    volatile float error;        // 当前误差
    volatile float last_error;   // 上次误差
    volatile float integral;     // 积分项
    volatile float derivative;   // 微分项
    volatile float output;       // 输出值
    float integral_limit;        // 积分限幅，初始化控制器时赋值
    float output_limit;          // 输出限幅
	
} PIDController;

extern PIDController PID_image;
extern PIDController PID_speed;

// 单电机控制结构体
typedef struct {
    volatile int32_t speed;     // 速度  这里的速度储存编码器测速值 其实暂时没什么用 后面可以试试与target_speed做差进行分段速度变化
    volatile uint8_t dir;      // 方向（1正,0反）
    volatile int32_t target_speed;   // 目标速度  目标速度用于接收pid系统返值并传参给电机控制函数
    uint8_t lor;                // 左或右电机标识（0左,1右）
} Motor;

// PID控制器初始化
void pid_init(PIDController* pid, float kp, float ki, float kd, float kff);

//pid计算
float pid_calculate(PIDController* pid);//外环计算
float pid_speed_calculate(PIDController* pid,Motor *motor);//内环计算

//pid前馈
float PID_pre_calculate(PIDController *pid);

// 初始化电机驱动
void motor_init(void);

// 控制电机运行 (speed: -1000 to 1000)
void motor_run(Motor *motor_ptr, int32_t speed);

//pid循迹
void run_follow(PIDController* pid,Motor *motor_left,Motor *motor_right);
void run_follow_v0(PIDController* pid);//开环循迹

//左转
void motor_turnleft(void);

// 右转
void motor_turnright(void);

//滑行
void motor_coast(void);

// 刹车
void motor_stop(void);

//循迹
//void motor_follow_line_straight(PIDController* pid);

//void straight_error_get(PIDController *PID,struct lineinfo_s lineinfo[],struct watch_o *watch,float derta);
extern Motor leftmotor;
extern Motor rightmotor;
extern volatile int32_t delta_v;

#endif /* __MOTOR_H */