/**
 * @file control_pid.c
 * @brief 简洁版图像误差-速度串级PID控制器实现
 * @note 仿照现有speed_pid_calculate逻辑，结构简单易调参
 */

#include "control_pid.h"
#include "control.h"  // 引入straight_error_get()声明
#include <string.h>
#include <stdint.h>

// ==================== 全局实例定义 ====================

CascadePID_Controller g_cascade_pid = {0};

// ==================== 私有辅助函数 ====================

/**
 * @brief 浮点数限幅
 */
static inline float clamp(float x, float min, float max)
{
    return x < min ? min : (x > max ? max : x);
}

/**
 * @brief 通用PID控制器计算(位置式)
 * @param pid PID结构体指针
 * @param error 误差输入
 * @return PID输出
 * @note 完全仿照control.c中的speed_pid_calculate逻辑
 */
static float pid_calculate(PID_Controller* pid, float error)
{
    if (!pid) return 0.0f;
    
    // 微分(一阶差分) - 先计算微分避免使用限幅后的误差
    float derivative = error - pid->prev_error;
    pid->prev_error = error;
    
    // 积分分离: 误差大时只用PD控制，误差小时才加入积分
    // 优点: 大误差快速响应，小误差精确控制
    float abs_error = (error >= 0) ? error : -error;
    float output_unlimited;
    
    if (abs_error > pid->error_threshold) {
        // 误差大: 只用PD控制 (快速响应)
        output_unlimited = pid->kp * error + pid->kd * derivative;
    } else {
        // 误差小: 完整PID控制 (精确消除静差)
        output_unlimited = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    }
    
    // 输出限幅
    float output = clamp(output_unlimited, -pid->output_max, pid->output_max);
    
    // 抗积分饱和: 只有当输出未饱和时才累积积分
    // 这样可以防止积分累积过大导致超调
    float abs_output = (output_unlimited >= 0) ? output_unlimited : -output_unlimited;
    if (abs_output < pid->output_max * 0.95f) {
        pid->integral += error;
        pid->integral = clamp(pid->integral, -pid->integral_max, pid->integral_max);
    }
    
    return output;
}

/**
 * @brief 重置单个PID控制器状态
 */
static void pid_reset(PID_Controller* pid)
{
    if (!pid) return;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

/**
 * @brief 初始化单个PID控制器参数
 */
static void pid_init(PID_Controller* pid, 
                     float kp, float ki, float kd,
                     float integral_max, float output_max,
                     float error_threshold)
{
    if (!pid) return;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->integral_max = integral_max;
    pid->output_max = output_max;
    pid->error_threshold = error_threshold;
}

// ==================== 对外接口实现 ====================

/**
 * @brief 初始化串级PID控制器
 */
void cascade_pid_init(float half_track,
                      float image_kp, float image_ki, float image_kd,
                      float speed_kp, float speed_ki, float speed_kd)
{
    memset(&g_cascade_pid, 0, sizeof(CascadePID_Controller));
    
    // 运动学参数
    g_cascade_pid.half_track = half_track;
    g_cascade_pid.max_omega = 15.0f;        // 最大角速度15 rad/s (约860°/s) - 保留用于监控
    g_cascade_pid.max_diff_speed = 5.0f;    // 最大差速5.0 m/s (允许较大转弯)
    
    // 前馈补偿参数 (默认25, 可通过接口调节)
    g_cascade_pid.feedforward_pwm = 25.0f;  // 对抗重力和摩擦力的基准PWM
    
    // 初始化图像环PID
    // 图像误差范围约±100(像素偏差), 输出为差速(m/s)
    pid_init(&g_cascade_pid.image_pid,
             image_kp,   // 建议0.001~0.01
             image_ki,   // 建议0.0~0.001
             image_kd,   // 建议0.0~0.005
             50.0f,      // 积分限幅
             g_cascade_pid.max_diff_speed,  // 输出限幅=最大差速
             10.0f);     // 积分分离阈值(图像误差>10时只用PD)
    
    // 初始化左轮速度环PID
    // 速度误差范围约±2.0 m/s, 输出为PWM(±1000)
    // 关键: 降低积分限幅防止饱和, 设置积分分离阈值
    pid_init(&g_cascade_pid.left_speed_pid,
             speed_kp,   // 建议10~50
             speed_ki,   // 建议0.1~2.0
             speed_kd,   // 建议0.0~1.0
             50.0f,      // 积分限幅(降低防止饱和)
             1000.0f,    // 输出限幅(PWM)
             0.1f);      // 积分分离阈值(速度误差>0.1m/s时只用PD)
    
    // 初始化右轮速度环PID (参数与左轮相同)
    pid_init(&g_cascade_pid.right_speed_pid,
             speed_kp,
             speed_ki,
             speed_kd,
             50.0f,      // 积分限幅
             1000.0f,    // 输出限幅
             0.1f);      // 积分分离阈值
}

/**
 * @brief 串级PID控制主循环(单频率版本)
 * @note 适合在同一中断中同时执行图像环和速度环
 */
void cascade_pid_control(float v_forward, 
                        float vL_actual, 
                        float vR_actual,
                        float* pwm_L_out, 
                        float* pwm_R_out)
{
    // 外环: 更新速度目标
    cascade_pid_outer_loop(v_forward);
    
    // 内环: 计算PWM输出
    cascade_pid_inner_loop(vL_actual, vR_actual, pwm_L_out, pwm_R_out);
}

// ==================== 双频率控制接口实现 ====================

/**
 * @brief 图像环PID计算(低频中断, 10ms~50ms)
 * @note 只更新速度目标, 不计算PWM
 */
void cascade_pid_outer_loop(float v_forward)
{
    // ==================== 外环: 图像误差 → 差速 ====================
    
    // 步骤1: 获取图像误差
    // straight_error_get()返回: sum - 94.0 (sum为中线x坐标的加权平均)
    // 正值 = 中线在右侧(需要右转), 负值 = 中线在左侧(需要左转)
   float image_error = straight_error_get();
    
    // 步骤2: 图像环PID计算 → 差速
    // 注意: 对误差取反，因为中线在右侧应该右转(左轮快)
    // 正误差(中线右) → 负差速 → vL > vR → 右转 ✓
    // 负误差(中线左) → 正差速 → vR > vL → 左转 ✓
   float speed_diff = pid_calculate(&g_cascade_pid.image_pid, -image_error);
    
    // 保存角速度用于监控(从差速反推, 仅用于调试)
   //g_cascade_pid.omega_target = speed_diff / (2.0f * g_cascade_pid.half_track);
    
    // ==================== 差速 → 左右轮目标速度 ====================
    
    // 步骤3: 分配左右轮目标速度 (保存到全局状态)
    // speed_diff > 0: 右轮快 → 左转
    // speed_diff < 0: 左轮快 → 右转
    g_cascade_pid.vL_target = v_forward - speed_diff * 0.5f;
    g_cascade_pid.vR_target = v_forward + speed_diff * 0.5f;
    
    // 步骤4: 目标速度限幅 (允许负值，支持反转)
    // 关键改进: 不再强制限制为非负，允许电机反转
    float v_max_abs = v_forward * 2.0f + 1.0f;  // 允许最大速度为2倍前进速度+1m/s的安全余量
    g_cascade_pid.vL_target = clamp(g_cascade_pid.vL_target, -v_max_abs, v_max_abs);
    g_cascade_pid.vR_target = clamp(g_cascade_pid.vR_target, -v_max_abs, v_max_abs);
    
    // 步骤5: 渐变率限制(防止目标速度突变)
    // 关键改进: 增大最大变化量，支持快速减速和反转
    float max_delta_v = 1.0f;  // 每次最大变刖1.0 m/s (原0.5太小)
    float delta_vL = g_cascade_pid.vL_target - g_cascade_pid.vL_target_last;
    float delta_vR = g_cascade_pid.vR_target - g_cascade_pid.vR_target_last;
    
    float abs_delta_vL = (delta_vL >= 0) ? delta_vL : -delta_vL;
    if (abs_delta_vL > max_delta_v) {
        g_cascade_pid.vL_target = g_cascade_pid.vL_target_last + 
            (delta_vL > 0 ? max_delta_v : -max_delta_v);
    }
    
    float abs_delta_vR = (delta_vR >= 0) ? delta_vR : -delta_vR;
    if (abs_delta_vR > max_delta_v) {
        g_cascade_pid.vR_target = g_cascade_pid.vR_target_last + 
            (delta_vR > 0 ? max_delta_v : -max_delta_v);
    }
    
    // 保存本次目标速度供下次使用
    g_cascade_pid.vL_target_last = g_cascade_pid.vL_target;
    g_cascade_pid.vR_target_last = g_cascade_pid.vR_target;
}

/**
 * @brief 速度环PID计算(高频中断, 1ms~5ms)
 * @note 使用上次外环计算的速度目标
 * @note 关键改进: PWM输出带符号，支持电机反转
 */
void cascade_pid_inner_loop(float vL_actual, 
                            float vR_actual,
                            float* pwm_L_out, 
                            float* pwm_R_out)
{
    if (!pwm_L_out || !pwm_R_out) return;
    
    // ==================== 内环: 速度误差 → PWM ====================
    
    // 左轮速度环PID
    // 误差 = 目标 - 实际
    // 正误差 (实际过慢) → 正PWM (加速)
    // 负误差 (实际过快) → 负PWM (减速/反转)
    float vL_error = g_cascade_pid.vL_target - vL_actual;
    float pid_L = pid_calculate(&g_cascade_pid.left_speed_pid, vL_error);
    
    // 右轮速度环PID
    float vR_error = g_cascade_pid.vR_target - vR_actual;
    float pid_R = pid_calculate(&g_cascade_pid.right_speed_pid, vR_error);
    
    // 前馈补偿 + PID修正 (组合方案核心)
    // 基准PWM对抗重力/摩擦, PID修正误差
    g_cascade_pid.pwm_L = g_cascade_pid.feedforward_pwm + pid_L;
    g_cascade_pid.pwm_R = g_cascade_pid.feedforward_pwm + pid_R;
    
    // 直接输出PWM (带符号)
    // 正值 = 正转, 负值 = 反转
    *pwm_L_out = g_cascade_pid.pwm_L;
    *pwm_R_out = g_cascade_pid.pwm_R;
}

/**
 * @brief 重置串级PID所有状态
 */
void cascade_pid_reset(void)
{
    pid_reset(&g_cascade_pid.image_pid);
    pid_reset(&g_cascade_pid.left_speed_pid);
    pid_reset(&g_cascade_pid.right_speed_pid);
    
    g_cascade_pid.omega_target = 0.0f;
    g_cascade_pid.vL_target = 0.0f;
    g_cascade_pid.vR_target = 0.0f;
    g_cascade_pid.pwm_L = 0.0f;
    g_cascade_pid.pwm_R = 0.0f;
}

/**
 * @brief 在线修改图像环PID参数
 */
void cascade_pid_set_image_params(float kp, float ki, float kd)
{
    g_cascade_pid.image_pid.kp = kp;
    g_cascade_pid.image_pid.ki = ki;
    g_cascade_pid.image_pid.kd = kd;
    pid_reset(&g_cascade_pid.image_pid);  // 清除残留状态
}

/**
 * @brief 在线修改速度环PID参数
 */
void cascade_pid_set_speed_params(float kp, float ki, float kd)
{
    // 左右轮使用相同参数
    g_cascade_pid.left_speed_pid.kp = kp;
    g_cascade_pid.left_speed_pid.ki = ki;
    g_cascade_pid.left_speed_pid.kd = kd;
    
    g_cascade_pid.right_speed_pid.kp = kp;
    g_cascade_pid.right_speed_pid.ki = ki;
    g_cascade_pid.right_speed_pid.kd = kd;
    
    pid_reset(&g_cascade_pid.left_speed_pid);
    pid_reset(&g_cascade_pid.right_speed_pid);
}

/**
 * @brief 在线修改运动学限幅参数
 */
void cascade_pid_set_limits(float max_omega, float max_diff_speed)
{
    g_cascade_pid.max_omega = max_omega;
    g_cascade_pid.max_diff_speed = max_diff_speed;
    g_cascade_pid.image_pid.output_max = max_diff_speed;  // 同步更新图像环输出限幅
}

/**
 * @brief 设置前馈补偿PWM值
 */
void cascade_pid_set_feedforward(float feedforward_pwm)
{
    g_cascade_pid.feedforward_pwm = feedforward_pwm;
}

/**
 * @brief 获取当前前馈补偿值
 */
float cascade_pid_get_feedforward(void)
{
    return g_cascade_pid.feedforward_pwm;
}

/**
 * @brief 设置速度环积分分离阈值
 */
void cascade_pid_set_integral_separation_threshold(float error_threshold)
{
    g_cascade_pid.left_speed_pid.error_threshold = error_threshold;
    g_cascade_pid.right_speed_pid.error_threshold = error_threshold;
}

/**
 * @brief 获取控制器状态
 */
CascadePID_Controller* cascade_pid_get_state(void)
{
    return &g_cascade_pid;
}

// ==================== 使用说明 ====================
/*
 * 【快速上手】
 * 
 * 1. 初始化(main函数中调用一次):
 *    cascade_pid_init(0.008f,           // 半轮距8mm
 *                     0.5f, 0.01f, 0.1f, // 图像环PID
 *                     20.0f, 0.5f, 0.2f); // 速度环PID
 * 
 * ==================== 方式1: 单频率控制(简单) ====================
 * 适用于图像采集和速度控制在同一周期
 * 
 * 2a. 控制循环(定时器中断, 建议10ms~50ms):
 *    void TIM_Control_IRQHandler(void) {
 *        get_speed();  // 更新编码器速度
 *        float pwm_L, pwm_R;
 *        
 *        cascade_pid_control(0.3f,               // 前进速度0.3m/s
 *                           control.left_speed,   // 左轮实际速度
 *                           control.right_speed,  // 右轮实际速度
 *                           &pwm_L, &pwm_R);
 *        
 *        // 输出到电机
 *        control.left_target_speed = (int16_t)fabs(pwm_L);
 *        control.left_dir = (pwm_L >= 0) ? 1 : 0;
 *        control.right_target_speed = (int16_t)fabs(pwm_R);
 *        control.right_dir = (pwm_R >= 0) ? 1 : 0;
 *    }
 * 
 * ==================== 方式2: 双频率控制(推荐) ====================
 * 适用于需要高频速度控制的场景
 * 
 * 2b. 低频中断 - 图像环(10ms~50ms, 与图像采集同步):
 *    void TIM_ImageProcess_IRQHandler(void) {
 *        // 图像处理完成后调用
 *        cascade_pid_outer_loop(0.3f);  // 更新速度目标
 *    }
 * 
 * 2c. 高频中断 - 速度环(1ms~5ms, 快速响应):
 *    void TIM_SpeedControl_IRQHandler(void) {
 *        get_speed();  // 更新编码器速度
 *        float pwm_L, pwm_R;
 *        
 *        cascade_pid_inner_loop(control.left_speed,
 *                              control.right_speed,
 *                              &pwm_L, &pwm_R);
 *        
 *        // 输出到电机
 *        control.left_target_speed = (int16_t)fabs(pwm_L);
 *        control.left_dir = (pwm_L >= 0) ? 1 : 0;
 *        control.right_target_speed = (int16_t)fabs(pwm_R);
 *        control.right_dir = (pwm_R >= 0) ? 1 : 0;
 *    }
 * 
 * ==================== 频率配置建议 ====================
 * 
 * - 图像采集: 20Hz~100Hz (10ms~50ms)
 *   → 图像环调用频率与此同步
 * 
 * - 速度控制: 200Hz~1000Hz (1ms~5ms)
 *   → 速度环调用频率应为图像环的5~50倍
 * 
 * - 典型配置:
 *   图像环: 50ms (20Hz) - 在图像处理完成中断中调用
 *   速度环: 2ms (500Hz) - 在定时器中断中调用
 *   频率比: 25:1
 * 
 * 3. 运行时调参:
 *    cascade_pid_set_image_params(0.8f, 0.02f, 0.15f);  // 调整图像环
 *    cascade_pid_set_speed_params(25.0f, 0.8f, 0.3f);   // 调整速度环
 * 
 * 【调参建议】
 * 
 * 阶段1: 先调内环(速度环)
 *   - 固定图像环Kp=0, Ki=0, Kd=0 (关闭外环)
 *   - 手动设置 g_cascade_pid.vL_target = 0.3f (或通过调试器)
 *   - 观察速度跟踪响应，调节速度环参数
 *   - 目标: 快速、稳定、无超调
 *   - 推荐初值: Kp=20, Ki=0.5, Kd=0.2
 * 
 * 阶段2: 再调外环(图像环)
 *   - 启用图像环, 从纯比例开始: Kp=0.005, Ki=0, Kd=0
 *   - 观察转向响应, 逐步增大Kp直到出现轻微振荡
 *   - 减小Kp到振荡消失, 然后加入微分Kd抑制超调
 *   - 最后加入小量积分Ki消除稳态误差
 *   - 推荐初值: Kp=0.001~0.01, Ki=0.0~0.001, Kd=0.0~0.005
 *   - 注意: 图像环增益比原来小100倍(因为直接输出差速而非角速度)
 * 
 * 【调试技巧】
 * 
 * - 查看实时状态:
 *   CascadePID_Controller* state = cascade_pid_get_state();
 *   printf("图像误差: %.2f\n", straight_error_get());
 *   printf("差速输出: %.4f m/s\n", state->image_pid积分后的输出);
 *   printf("期望角速度: %.2f rad/s (监控用)\n", state->omega_target);
 *   printf("左轮目标速度: %.3f m/s\n", state->vL_target);
 * 
 * - 图像误差方向验证:
 *   straight_error_get() > 0 → 中线在右 → -error < 0 → speed_diff < 0 → vL > vR → 右转 ✓
 *   straight_error_get() < 0 → 中线在左 → -error > 0 → speed_diff > 0 → vR > vL → 左转 ✓
 * 
 * - 检查饱和:
 *   如果image_pid.integral持续达到±50, 说明积分饱和, 需要:
 *   1) 减小Ki
 *   2) 增大integral_max
 *   3) 检查是否有持续偏差(机械/传感器问题)
 * 
 * - 检查限幅:
 *   如果omega_target持续在±max_omega边界, 说明转向受限, 可以:
 *   1) 增大max_omega (但注意机械安全)
 *   2) 减小图像环Kp (降低转向增益)
 * 
 * - 双频率控制优势:
 *   1) 速度环高频响应，提高抗干扰能力
 *   2) 图像环低频更新，节省CPU资源
 *   3) 图像处理延迟不影响速度控制实时性
 */
