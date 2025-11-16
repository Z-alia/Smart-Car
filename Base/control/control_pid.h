/**
 * @file control_pid.h
 * @brief 简洁版图像误差-速度串级PID控制器
 * @note 控制结构: 
 *       外环(图像环): straight_error_get() → PID → 期望角速度ω
 *       内环(速度环): ω转差速 → 左右轮速度目标 → PID → PWM
 */

#ifndef CONTROL_PID_H_
#define CONTROL_PID_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// ==================== PID结构体定义 ====================

/**
 * @brief 标准PID控制器结构体
 */
typedef struct {
    float kp;           // 比例系数
    float ki;           // 积分系数
    float kd;           // 微分系数
    float integral;     // 积分累积
    float prev_error;   // 上次误差(用于微分)
    float integral_max; // 积分限幅
    float output_max;   // 输出限幅
    float error_threshold; // 积分分离阈值(误差大于此值时只用PD控制)
} PID_Controller;

// ==================== 串级PID控制器结构体 ====================

/**
 * @brief 串级PID控制器完整状态
 */
typedef struct {
    // 外环: 图像误差 → 角速度
    PID_Controller image_pid;  // 图像误差PID
    
    // 内环: 左右轮速度控制
    PID_Controller left_speed_pid;   // 左轮速度PID
    PID_Controller right_speed_pid;  // 右轮速度PID
    
    // 运动学参数
    float half_track;       // 半轮距(m)
    float max_omega;        // 最大角速度限幅(rad/s)
    float max_diff_speed;   // 最大差速限幅(m/s)
    
    // 内部状态
    float omega_target;     // 期望角速度(rad/s)
    float vL_target;        // 左轮目标速度(m/s)
    float vR_target;        // 右轮目标速度(m/s)
    float vL_target_last;   // 左轮上次目标速度(用于渐变限制)
    float vR_target_last;   // 右轮上次目标速度(用于渐变限制)
    float pwm_L;            // 左轮PWM输出
    float pwm_R;            // 右轮PWM输出
    
    // 前馈补偿参数 (上坡/摩擦力对抗)
    float feedforward_pwm;  // 前馈基准PWM (建议10~50, 对抗重力和摩擦)
    
} CascadePID_Controller;

// ==================== 全局实例 ====================

extern CascadePID_Controller g_cascade_pid;

// ==================== 核心接口函数 ====================

/**
 * @brief 初始化串级PID控制器
 * @param half_track 半轮距(m), 例如轮距16mm则输入0.008
 * @param image_kp 图像环比例系数, 建议初值0.1~1.0
 * @param image_ki 图像环积分系数, 建议初值0.0~0.1
 * @param image_kd 图像环微分系数, 建议初值0.0~0.5
 * @param speed_kp 速度环比例系数, 建议初值10~50
 * @param speed_ki 速度环积分系数, 建议初值0.1~2.0
 * @param speed_kd 速度环微分系数, 建议初值0.0~1.0
 * @note 使用示例:
 *       cascade_pid_init(0.008f,  // 半轮距8mm
 *                        0.5f, 0.01f, 0.1f,  // 图像环PID
 *                        20.0f, 0.5f, 0.2f); // 速度环PID
 */
void cascade_pid_init(float half_track,
                      float image_kp, float image_ki, float image_kd,
                      float speed_kp, float speed_ki, float speed_kd);

/**
 * @brief 串级PID控制主循环(每个控制周期调用一次)
 * @param v_forward 前进基准速度(m/s)
 * @param vL_actual 左轮实际速度(m/s), 来自编码器
 * @param vR_actual 右轮实际速度(m/s), 来自编码器
 * @param pwm_L_out 左轮PWM输出指针
 * @param pwm_R_out 右轮PWM输出指针
 * @note 内部流程:
 *       1. 调用 straight_error_get() 获取图像误差
 *       2. 图像环PID: err → ω (角速度)
 *       3. 运动学转换: ω → vL_target, vR_target
 *       4. 速度环PID: (vL_target-vL_actual) → PWM_L
 *       5. 速度环PID: (vR_target-vR_actual) → PWM_R
 * @note 调用频率: 建议10ms~50ms (与图像采集周期一致)
 */
void cascade_pid_control(float v_forward, 
                        float vL_actual, 
                        float vR_actual,
                        float* pwm_L_out, 
                        float* pwm_R_out);

// ==================== 双频率控制接口 ====================

/**
 * @brief 图像环PID计算(低频中断调用, 如10ms~50ms)
 * @param v_forward 前进基准速度(m/s)
 * @note 执行流程:
 *       1. 调用 straight_error_get() 获取图像误差
 *       2. 图像环PID: err → ω (角速度)
 *       3. 运动学转换: ω → vL_target, vR_target (保存到内部状态)
 * @note 调用频率: 与图像采集同步, 建议10ms~50ms
 * @note 配合使用: 在图像处理完成后的中断中调用
 * @example
 *       // 在图像处理中断中(50ms周期):
 *       void TIM_ImageProcess_IRQHandler(void) {
 *           process_image();  // 图像处理
 *           cascade_pid_outer_loop(0.3f);  // 更新速度目标
 *       }
 */
void cascade_pid_outer_loop(float v_forward);

/**
 * @brief 速度环PID计算(高频中断调用, 如1ms~5ms)
 * @param vL_actual 左轮实际速度(m/s), 来自编码器
 * @param vR_actual 右轮实际速度(m/s), 来自编码器
 * @param pwm_L_out 左轮PWM输出指针
 * @param pwm_R_out 右轮PWM输出指针
 * @note 执行流程:
 *       1. 使用上次外环计算的 vL_target, vR_target
 *       2. 速度环PID: (vL_target-vL_actual) → PWM_L
 *       3. 速度环PID: (vR_target-vR_actual) → PWM_R
 * @note 调用频率: 建议1ms~5ms (快速响应速度变化)
 * @note 配合使用: 在定时器中断中周期性调用
 * @example
 *       // 在高频定时器中断中(1ms周期):
 *       void TIM_SpeedControl_IRQHandler(void) {
 *           get_speed();  // 更新编码器速度
 *           float pwm_L, pwm_R;
 *           cascade_pid_inner_loop(control.left_speed, 
 *                                  control.right_speed,
 *                                  &pwm_L, &pwm_R);
 *           motor_set_pwm(pwm_L, pwm_R);  // 输出到电机
 *       }
 */
void cascade_pid_inner_loop(float vL_actual, 
                            float vR_actual,
                            float* pwm_L_out, 
                            float* pwm_R_out);

// ==================== 辅助函数 ====================

/**
 * @brief 重置串级PID所有状态(清除积分、微分)
 * @note 在启动/停止/切换控制方案时调用
 */
void cascade_pid_reset(void);

/**
 * @brief 在线修改图像环PID参数
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 */
void cascade_pid_set_image_params(float kp, float ki, float kd);

/**
 * @brief 在线修改速度环PID参数
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 */
void cascade_pid_set_speed_params(float kp, float ki, float kd);

/**
 * @brief 在线修改运动学参数
 * @param max_omega 最大角速度(rad/s), 建议5~20
 * @param max_diff_speed 最大差速(m/s), 建议0.05~0.2
 */
void cascade_pid_set_limits(float max_omega, float max_diff_speed);

/**
 * @brief 设置前馈补偿PWM值 (组合方案核心参数)
 * @param feedforward_pwm 基准PWM值, 建议10~50
 * @note 作用: 对抗重力和摩擦力, 提高上坡能力
 * @note 调试方法:
 *       - 上坡无力: 逐步增大 (25 -> 30 -> 35)
 *       - 平路过快: 逐步减小 (25 -> 20 -> 15)
 *       - 默认值: 25.0f
 */
void cascade_pid_set_feedforward(float feedforward_pwm);

/**
 * @brief 获取当前前馈补偿值
 * @return 当前的前馈PWM值
 */
float cascade_pid_get_feedforward(void);

/**
 * @brief 设置速度环积分分离阈值
 * @param error_threshold 误差阈值(m/s), 建议0.05~0.2
 * @note 误差 > 阈值时只用PD控制 (快速响应)
 * @note 误差 <= 阈值时用完整PID (精确控制)
 * @note 默认值: 0.1 m/s
 */
void cascade_pid_set_integral_separation_threshold(float error_threshold);

/**
 * @brief 获取当前控制器状态(调试用)
 * @return 指向全局控制器的指针
 * @note 可读取内部变量:
 *       g_cascade_pid.omega_target  - 期望角速度
 *       g_cascade_pid.vL_target     - 左轮目标速度
 *       g_cascade_pid.vR_target     - 右轮目标速度
 *       g_cascade_pid.image_pid.integral - 图像环积分值
 */
CascadePID_Controller* cascade_pid_get_state(void);

#ifdef __cplusplus
}
#endif

#endif // CONTROL_PID_H_
