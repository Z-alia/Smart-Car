/**
 * @file 电机控制修改指南.c
 * @brief 如何正确处理带符号的PWM输出，支持电机反转
 */

#include "control_pid.h"
#include "control.h"
#include "motor.h"

// ==================== 正确的电机输出处理方式 ====================

/**
 * @brief 方式1: 在主循环中直接处理PWM符号
 * @note 推荐方式 - 简单直接
 */
void example_motor_output_method1(void)
{
    float pwm_L, pwm_R;
    
    // 调用速度环PID（输出带符号PWM）
    cascade_pid_inner_loop(control.left_speed, 
                          control.right_speed, 
                          &pwm_L, &pwm_R);
    
    // ========== 关键：处理PWM符号，分离方向和幅值 ==========
    
    // 左轮
    if (pwm_L >= 0) {
        control.left_target_speed = (int16_t)pwm_L;
        control.left_dir = 1;  // 正转
    } else {
        control.left_target_speed = (int16_t)(-pwm_L);  // 取绝对值
        control.left_dir = 0;  // 反转
    }
    
    // 右轮
    if (pwm_R >= 0) {
        control.right_target_speed = (int16_t)pwm_R;
        control.right_dir = 1;  // 正转
    } else {
        control.right_target_speed = (int16_t)(-pwm_R);  // 取绝对值
        control.right_dir = 0;  // 反转
    }
    
    // 输出到电机驱动
    motor_output(control.left_target_speed, control.left_dir,
                control.right_target_speed, control.right_dir);
}

/**
 * @brief 方式2: 使用辅助函数封装
 * @note 推荐方式 - 代码更清晰
 */
void motor_set_pwm_signed(float pwm_L, float pwm_R)
{
    // 左轮
    control.left_target_speed = (int16_t)((pwm_L >= 0) ? pwm_L : -pwm_L);
    control.left_dir = (pwm_L >= 0) ? 1 : 0;
    
    // 右轮
    control.right_target_speed = (int16_t)((pwm_R >= 0) ? pwm_R : -pwm_R);
    control.right_dir = (pwm_R >= 0) ? 1 : 0;
    
    // 输出到电机
    motor_output(control.left_target_speed, control.left_dir,
                control.right_target_speed, control.right_dir);
}

void example_motor_output_method2(void)
{
    float pwm_L, pwm_R;
    
    cascade_pid_inner_loop(control.left_speed, 
                          control.right_speed, 
                          &pwm_L, &pwm_R);
    
    motor_set_pwm_signed(pwm_L, pwm_R);
}

/**
 * @brief 方式3: 完整的定时器中断示例
 * @note 实际使用推荐方式
 */
void TIM_SpeedControl_IRQHandler(void)
{
    static uint32_t counter = 0;
    counter++;
    
    // 1. 读取编码器速度（假设已更新）
    // get_speed();  // 更新control.left_speed和control.right_speed
    
    // 2. 速度环PID控制
    float pwm_L, pwm_R;
    cascade_pid_inner_loop(control.left_speed, 
                          control.right_speed, 
                          &pwm_L, &pwm_R);
    
    // 3. 处理PWM符号并输出
    // 左轮
    if (pwm_L >= 0) {
        control.left_target_speed = (int16_t)pwm_L;
        control.left_dir = 1;
    } else {
        control.left_target_speed = (int16_t)(-pwm_L);
        control.left_dir = 0;
    }
    
    // 右轮
    if (pwm_R >= 0) {
        control.right_target_speed = (int16_t)pwm_R;
        control.right_dir = 1;
    } else {
        control.right_target_speed = (int16_t)(-pwm_R);
        control.right_dir = 0;
    }
    
    // 4. 输出到电机
    motor_output(control.left_target_speed, control.left_dir,
                control.right_target_speed, control.right_dir);
    
    // 5. 调试输出（每100次打印一次）
    if (counter % 100 == 0) {
        printf("vL_tgt=%.2f, vL_act=%.2f, pwmL=%.0f(%s)\n",
               g_cascade_pid.vL_target, control.left_speed, 
               (pwm_L >= 0) ? pwm_L : -pwm_L,
               (pwm_L >= 0) ? "FWD" : "REV");
    }
}

// ==================== 测试场景示例 ====================

/**
 * @brief 测试场景1: 快速减速（会触发反转）
 */
void test_rapid_deceleration(void)
{
    printf("测试：快速减速\n");
    
    // 初始状态：高速前进
    cascade_pid_outer_loop(2.0f);  // 目标速度2.0 m/s
    // 假设当前实际速度也接近2.0 m/s
    
    // 突然降速到0.2 m/s
    cascade_pid_outer_loop(0.2f);
    
    // 此时目标速度突然降低，但实际速度还很高
    // vL_target ≈ 0.2 m/s
    // vL_actual ≈ 2.0 m/s
    // error = 0.2 - 2.0 = -1.8 m/s (负误差)
    // PID输出 = 负PWM → 电机反转制动！
    
    printf("预期：电机输出负PWM，反转制动\n");
}

/**
 * @brief 测试场景2: 原地转向（左右轮反向）
 */
void test_in_place_rotation(void)
{
    printf("测试：原地转向\n");
    
    // 设置大差速，一轮正转一轮反转
    cascade_pid_outer_loop(0.0f);  // 前进速度=0
    
    // 手动设置目标（模拟大转弯）
    g_cascade_pid.vL_target = -0.3f;  // 左轮反转
    g_cascade_pid.vR_target = 0.3f;   // 右轮正转
    
    // 假设当前速度都为0
    // vL_error = -0.3 - 0 = -0.3 → 负PWM → 左轮反转
    // vR_error = 0.3 - 0 = 0.3 → 正PWM → 右轮正转
    
    printf("预期：左轮反转，右轮正转，原地旋转\n");
}

/**
 * @brief 测试场景3: 倒车
 */
void test_backward(void)
{
    printf("测试：倒车\n");
    
    // 设置负速度
    cascade_pid_outer_loop(-1.0f);  // 倒车1.0 m/s
    
    // vL_target = -1.0 m/s
    // vR_target = -1.0 m/s
    
    // 如果当前速度为0:
    // vL_error = -1.0 - 0 = -1.0 → 负PWM → 反转
    // vR_error = -1.0 - 0 = -1.0 → 负PWM → 反转
    
    printf("预期：两轮都反转，小车倒退\n");
}

// ==================== 关键修改总结 ====================

/*
 * 【修改要点】
 * 
 * 1. 删除目标速度的非负限制
 *    - 旧代码: clamp(v, 0.0f, v_max)  ❌
 *    - 新代码: clamp(v, -v_max, v_max) ✓
 * 
 * 2. 速度环PID输出带符号
 *    - 旧代码: *pwm_out += pwm  ❌ (累加导致混乱)
 *    - 新代码: *pwm_out = pwm   ✓ (直接赋值)
 * 
 * 3. 处理PWM符号
 *    - 旧代码: control.left_target_speed = (int16_t)fabs(pwm)  ❌
 *    - 新代码: 
 *      if (pwm >= 0) {
 *          speed = pwm; dir = 1;
 *      } else {
 *          speed = -pwm; dir = 0;
 *      }  ✓
 * 
 * 4. 增大渐变率限制
 *    - 旧代码: max_delta_v = 0.5 m/s  ❌ (太小，减速慢)
 *    - 新代码: max_delta_v = 1.0 m/s  ✓ (支持快速减速)
 * 
 * 5. 降低积分限幅
 *    - 积分限幅: 100 → 50  (防止饱和)
 * 
 * 【控制逻辑】
 * 
 * 正常加速:
 *   vL_target=1.0, vL_actual=0.5
 *   error = 0.5 → PID输出 +500 PWM → 正转加速 ✓
 * 
 * 快速减速（关键！）:
 *   vL_target=0.2, vL_actual=2.0
 *   error = -1.8 → PID输出 -800 PWM → 反转制动 ✓
 * 
 * 倒车:
 *   vL_target=-1.0, vL_actual=0.0
 *   error = -1.0 → PID输出 -500 PWM → 反转倒车 ✓
 * 
 * 【优势】
 * 
 * ✓ 快速减速: 利用反向PWM主动制动
 * ✓ 精确控制: 误差大时PID输出大，快速收敛
 * ✓ 四象限控制: 支持前进/后退/加速/减速
 * ✓ 防止失控: 速度过大时自动反转制动
 */

// ==================== 调试技巧 ====================

/**
 * @brief 实时监控速度环状态
 */
void debug_speed_loop(void)
{
    CascadePID_Controller* pid = cascade_pid_get_state();
    
    printf("========== 速度环状态 ==========\n");
    printf("目标: vL=%.2f, vR=%.2f\n", 
           pid->vL_target, pid->vR_target);
    printf("实际: vL=%.2f, vR=%.2f\n",
           control.left_speed, control.right_speed);
    printf("误差: eL=%.2f, eR=%.2f\n",
           pid->vL_target - control.left_speed,
           pid->vR_target - control.right_speed);
    printf("PWM: pwmL=%.0f, pwmR=%.0f\n",
           pid->pwm_L, pid->pwm_R);
    printf("方向: dirL=%d, dirR=%d\n",
           control.left_dir, control.right_dir);
    printf("积分: iL=%.2f, iR=%.2f\n",
           pid->left_speed_pid.integral,
           pid->right_speed_pid.integral);
    printf("================================\n");
}

/**
 * @brief 检查是否收敛
 */
int check_convergence(void)
{
    float vL_error = g_cascade_pid.vL_target - control.left_speed;
    float vR_error = g_cascade_pid.vR_target - control.right_speed;
    
    float abs_vL_error = (vL_error >= 0) ? vL_error : -vL_error;
    float abs_vR_error = (vR_error >= 0) ? vR_error : -vR_error;
    
    // 误差小于0.05 m/s认为收敛
    if (abs_vL_error < 0.05f && abs_vR_error < 0.05f) {
        printf("速度环已收敛!\n");
        return 1;
    } else {
        printf("速度环未收敛: eL=%.3f, eR=%.3f\n", vL_error, vR_error);
        return 0;
    }
}
