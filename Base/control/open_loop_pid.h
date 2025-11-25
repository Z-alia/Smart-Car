/*
 * open_loop_pid.h
 * 开环PID控制器 - 基于图像误差的差速控制
 * Created on: 2025-11-23
 */

#ifndef CONTROL_OPEN_LOOP_PID_H_
#define CONTROL_OPEN_LOOP_PID_H_

#include <stdint.h>

// ==================== 开环PID结构体定义 ====================

/**
 * @brief 开环PID控制器结构体
 * @note 基于位置式PID算法
 */
typedef struct {
    // PID参数
    float kp;              // 比例增益
    float ki;              // 积分增益
    float kd;              // 微分增益
    
    // PID状态
    float integral;        // 积分累积值
    float prev_error;      // 上次误差值
    
    // 限幅参数
    float integral_max;    // 积分限幅,防止积分饱和
    float output_max;      // 输出限幅
} OpenLoopPID;

/**
 * @brief 开环PID控制器状态结构体
 */
typedef struct {
    OpenLoopPID pid;           // PID控制器
    float error;               // 当前误差值
    float control_output;      // 控制输出值(差速)
    float left_target;         // 左轮目标速度
    float right_target;        // 右轮目标速度
} OpenLoopPIDState;

// ==================== 全局变量声明 ====================
extern OpenLoopPIDState g_open_loop_pid_state;

// ==================== 函数声明 ====================

/**
 * @brief 初始化开环PID控制器
 * @param kp 比例增益
 * @param ki 积分增益
 * @param kd 微分增益
 * @param integral_max 积分限幅
 * @param output_max 输出限幅
 */
void open_loop_pid_init(float kp, float ki, float kd, 
                        float integral_max, float output_max);

/**
 * @brief 重置开环PID控制器状态
 */
void open_loop_pid_reset(void);

/**
 * @brief 开环PID控制计算
 * @param v_forward 前进基准速度(m/s)
 * @param left_target_out 左轮目标速度输出指针
 * @param right_target_out 右轮目标速度输出指针
 * @return 控制输出值(差速)
 * @note 内部调用 straight_error_get() 获取误差,并取反值
 */
float open_loop_pid_calculate(float v_forward, 
                               float* left_target_out, 
                               float* right_target_out);

/**
 * @brief 在线更新PID参数
 * @param kp 比例增益
 * @param ki 积分增益
 * @param kd 微分增益
 */
void open_loop_pid_update_params(float kp, float ki, float kd);

/**
 * @brief 获取当前误差值
 * @return 当前误差值
 */
float open_loop_pid_get_error(void);

/**
 * @brief 获取当前控制输出值
 * @return 当前控制输出值
 */
float open_loop_pid_get_output(void);

#endif /* CONTROL_OPEN_LOOP_PID_H_ */
