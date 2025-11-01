#ifndef CONTROL_H_
#define CONTROL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// 控制方案选择枚举
typedef enum {
    CONTROL_SCHEME_CASCADE_PID = 1,  // 方案一:图像误差PID外环 + 速度环PID内环(串级PID)
    CONTROL_SCHEME_ADRC        = 2   // 方案二:图像误差ADRC外环 + 速度环PID内环
} ControlScheme;

// 速度环PID参数结构体
typedef struct {
    float kp;           // 比例系数
    float ki;           // 积分系数
    float kd;           // 微分系数
    float integral;     // 积分累积
    float prev_error;   // 上次误差(用于微分)
    float integral_max; // 积分限幅
    float output_max;   // 输出限幅
} SpeedPID;

// 差速映射参数(用于方案一)
typedef struct {
    float half_track;       // 半轮距(m)
    float speed_ratio;      // 差速映射比例系数
    float max_diff_speed;   // 最大差速(m/s)
    float smooth_alpha;     // 平滑系数 0..1
    
    // 非线性增益参数(可选)
    int   enable_nonlinear; // 是否启用非线性增益 0=禁用, 1=启用
    float nonlinear_k;      // 非线性增益系数,建议1e-7~1e-5
} DiffMapParams;

// 差速映射状态
typedef struct {
    float vL_prev;
    float vR_prev;
} DiffMapState;

// ADRC参数结构体(用于方案二)
typedef struct {
    float b;            // 系统增益(需标定)
    float h;            // 采样时间(秒)
    float u_max;        // 最大差速(m/s)
    float r;            // TD跟踪速度
    float w0;           // ESO带宽
    float kp;           // 位置增益
    float kd;           // 速度增益
    float delta;        // fal线性区宽度
    float a1;           // fal指数(位置)
    float a2;           // fal指数(速度)
} ADRC_Params;

// ADRC内部状态
typedef struct {
    float x1, x2;       // TD状态
    float z1, z2, z3;   // ESO状态
    float u_last;       // 上次控制量
} ADRC_State;

// 控制器状态结构体
typedef struct {
    // 当前启用的控制方案
    ControlScheme active_scheme;
    
    // 方案一:串级PID相关
    DiffMapParams  cascade_diff_params;
    DiffMapState   cascade_diff_state;
    SpeedPID       cascade_speed_left;
    SpeedPID       cascade_speed_right;
    
    // 方案二:ADRC相关
    ADRC_Params    adrc_params;
    ADRC_State     adrc_state;
    SpeedPID       adrc_speed_left;
    SpeedPID       adrc_speed_right;
    
    // 输出值
    float vL_target;    // 左轮目标速度
    float vR_target;    // 右轮目标速度
    float pwm_L;        // 左轮PWM输出
    float pwm_R;        // 右轮PWM输出
} ControlState;

// 全局控制器状态
extern ControlState g_control;

// ==================== 对外接口函数 ====================

// 初始化控制器(选择方案并初始化参数)
void control_init(ControlScheme scheme);

// 切换控制方案(运行时切换)
void control_switch_scheme(ControlScheme scheme);

// 主控制循环(每个控制周期调用一次)
// 参数:
//   v_forward - 目标前进速度
//   vL_actual - 左轮实际速度(需外部提供)
//   vR_actual - 右轮实际速度(需外部提供)
// 输出:
//   pwm_L_out - 左轮PWM输出
//   pwm_R_out - 右轮PWM输出
void control_loop(float v_forward, 
                  float vL_actual, 
                  float vR_actual,
                  float* pwm_L_out, 
                  float* pwm_R_out);

// ==================== 需要外部实现的接口 ====================

// 计算图像误差(仿照cam_err_calculation的逻辑)
// 返回:横向偏差误差,范围约[-200, 200]
extern float cam_err_calculation(void);

// ==================== 辅助函数 ====================

// 速度环PID计算
float speed_pid_calculate(SpeedPID* pid, float target, float actual);

// 重置速度环PID状态
void speed_pid_reset(SpeedPID* pid);

// 初始化速度环PID参数
void speed_pid_init(SpeedPID* pid, float kp, float ki, float kd, 
                    float integral_max, float output_max);

#ifdef __cplusplus
}
#endif

#endif // CONTROL_H_
