#ifndef CONTROL_H_
#define CONTROL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

//定义段
#define Radius 0.0034f //车轮半径，单位m
#define Half_track 0.008f //半轮距，单位m
#define Speed_ratio 10.0f //差速映射比例系数
#define Max_diff_speed 100.0f //最大差速(m/s)
#define Smooth_alpha 0.20f //平滑系数 0..1
#define Integral_max 100.0f //积分限幅
#define Output_max 1000.0f //输出限幅 (PWM)
#define Encoder_PPR 256.0f*4.0f //编码器每转脉冲数 256线 四倍频
#define IMAGE_H 120  // 图像高度
#define IMAGE_W 188  // 图像宽度
//获取误差
#define weight_up 0.17//下部为0-30 中部为30-60 上部为60-120
#define weight_md 0.30
#define weight_dw 0.53

//车身状态结构体(整合编码器和电机状态)
typedef struct
{
    // 速度反馈(来自编码器)
    float left_speed;           // 左轮速度(m/s)
    float right_speed;          // 右轮速度(m/s)
    int32_t lencoder_count;     // 左编码器当前计数 (32-bit to match 32-bit TIM counters)
    int32_t lencoder_count_last;// 左编码器上次计数
    int32_t rencoder_count;     // 右编码器当前计数 (32-bit to match 32-bit TIM counters)
    int32_t rencoder_count_last;// 右编码器上次计数
    
    // 电机控制输出(传给motor模块)
    int16_t left_target_speed;  // 左轮目标速度(PWM值)
    int16_t right_target_speed; // 右轮目标速度(PWM值)
    uint8_t left_dir;           // 左轮方向(1正,0反)
    uint8_t right_dir;          // 右轮方向(1正,0反)
	float error;
} control_t;

extern control_t control;
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

// ==================== 便捷调参接口 ====================

/**
 * @brief 串级PID参数配置结构体(用于外部调参)
 */
typedef struct {
    // 外环差速映射参数
    float half_track;           // 半轮距(m), 默认0.065
    float speed_ratio;          // 误差→角速度增益, 默认10.0, 范围5~15
    float max_diff_speed;       // 最大差速(m/s), 默认1.5
    float smooth_alpha;         // 平滑系数, 默认0.20, 范围0.05~1.0
    
    // 非线性增益(高级特性)
    int   enable_nonlinear;     // 是否启用, 0=禁用, 1=启用
    float nonlinear_k;          // 非线性系数, 默认1e-6
    
    // 内环速度PID参数(左轮)
    float speed_left_kp;        // 左轮比例, 默认10.0
    float speed_left_ki;        // 左轮积分, 默认0.5
    float speed_left_kd;        // 左轮微分, 默认0.1
    
    // 内环速度PID参数(右轮)
    float speed_right_kp;       // 右轮比例, 默认10.0
    float speed_right_ki;       // 右轮积分, 默认0.5
    float speed_right_kd;       // 右轮微分, 默认0.1
    
    // PID限幅参数
    float integral_max;         // 积分限幅, 默认100.0
    float output_max;           // PWM输出限幅, 默认1000.0
} CascadePIDConfig;

extern CascadePIDConfig cascade_pid_config;

/**
 * @brief ADRC参数配置结构体(用于外部调参)
 */
typedef struct {
    // 必须标定的参数
    float b;                    // 系统增益, 需标定, 范围3~10
    float h;                    // 采样时间(秒), 必须与实际周期一致
    float u_max;                // 最大角速度(rad/s), 默认15.0
    
    // 可调节参数
    float r;                    // TD跟踪速度, 默认50.0, 范围30~150
    float w0;                   // ESO带宽, 默认20.0, 范围10~100
    float kp;                   // 位置增益, 默认1.0
    float kd;                   // 速度增益, 默认0.5
    
    // 高级参数(一般不需调节)
    float delta;                // fal线性区宽度, 默认0.01
    float a1;                   // fal指数(位置), 默认0.5
    float a2;                   // fal指数(速度), 默认1.5
    
    // 内环速度PID参数(左轮)
    float speed_left_kp;        // 左轮比例, 默认10.0
    float speed_left_ki;        // 左轮积分, 默认0.5
    float speed_left_kd;        // 左轮微分, 默认0.1
    
    // 内环速度PID参数(右轮)
    float speed_right_kp;       // 右轮比例, 默认10.0
    float speed_right_ki;       // 右轮积分, 默认0.5
    float speed_right_kd;       // 右轮微分, 默认0.1
    
    // PID限幅参数
    float integral_max;         // 积分限幅, 默认100.0
    float output_max;           // PWM输出限幅, 默认1000.0
} ADRCConfig;
//自定义参数初始化串级PID配置结构体
void control_init_cascade_pid_config(CascadePIDConfig* config,
                                     float kp_left, float ki_left, float kd_left,
                                     float kp_right, float ki_right, float kd_right);
// 使用默认参数初始化串级PID
void control_init_cascade_pid_default(void);

// 使用自定义参数初始化串级PID
void control_init_cascade_pid(const CascadePIDConfig* config);

// 使用默认参数初始化ADRC
void control_init_adrc_default(void);

// 使用自定义参数初始化ADRC
void control_init_adrc(const ADRCConfig* config);

// 在线修改串级PID外环参数(热调参)
void control_update_cascade_outer(float speed_ratio, float max_diff_speed, float smooth_alpha);

// 在线修改串级PID速度环参数(热调参)
void control_update_cascade_speed_pid(float kp_left, float ki_left, float kd_left,
                                      float kp_right, float ki_right, float kd_right);

// 在线修改ADRC核心参数(热调参)
void control_update_adrc_core(float b, float w0, float r, float kp, float kd);

// 启用/禁用非线性增益(串级PID专用)
void control_set_nonlinear_gain(int enable, float nonlinear_k);

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

// ==================== 图像处理接口(直接引用image模块数据) ====================

// 声明外部图像数据(由image.c提供,避免重复定义)
extern uint8_t l_border[120];      // 左线数组(来自image.c)
extern uint8_t r_border[120];      // 右线数组(来自image.c)
extern uint8_t left_lost[120];     // 左线丢失标志数组(来自image.c)
extern uint8_t right_lost[120];    // 右线丢失标志数组(来自image.c)
extern uint8_t left_lost_num;      // 左线丢失总行数(来自image.c)
extern uint8_t right_lost_num;     // 右线丢失总行数(来自image.c)

// 声明外部逆透视坐标数组(由image_process或persp模块填充)
// 每行存储左右边线逆透视变换后的坐标
extern int16_t persp_lx[120];      // 左线逆透视x坐标(透视变换后)
extern int16_t persp_rx[120];      // 右线逆透视x坐标(透视变换后)
extern uint8_t persp_ly[120];      // 左线逆透视y坐标(透视变换后)
extern uint8_t persp_ry[120];      // 右线逆透视y坐标(透视变换后)

// 摄像头误差计算参数(简化版,直接使用image数据)
typedef struct {
    int far_line;              // 远端行(打角最远行), 默认80
    int near_line;             // 近端行(打角最近行), 默认20
    float SteerKpchange;       // 非线性增益系数, 0=禁用
    int track_count;           // 赛道计数(用于丢线判断)
    
    // 车轮位置参数(用于计算切线斜率)
    uint8_t camwl;             // 左轮在图像中的x位置, 默认76
    uint8_t camwr;             // 右轮在图像中的x位置, 默认97
    uint8_t camwf;             // 前轮在图像中的y位置, 默认0
} CamErrParams;

// 摄像头误差计算状态
typedef struct {
    float angle_target;        // 当前目标角度
    float angle_target_last;   // 上次目标角度
    int watchleft;             // 左切线切点所在行
    int watchright;            // 右切线切点所在行
} CamErrState;

// 全局摄像头误差计算数据(简化版)
typedef struct {
    CamErrParams params;       // 参数配置
    CamErrState state;         // 内部状态
    int forward_near;          // 近端距离阈值
    int forward_far;           // 远端距离阈值
} CamErrData;

extern CamErrData g_cam_err_data;  // 全局摄像头数据

// 初始化摄像头误差计算模块(简化接口)
void cam_err_init(int near_line, int far_line, int forward_near, int forward_far);

// ==================== 需要外部实现的接口 ====================

// 计算图像误差(直接使用image模块的l_border/r_border数据)
// 返回:横向偏差误差,范围约[-200, 200]
float cam_err_calculation(void);
float straight_error_get(void);
// ==================== 完整控制链路接口 ====================

/**
 * @brief 完整控制流程(图像→误差→控制→输出)
 * @param v_forward 前进基准速度(m/s)
 * @note 内部流程:
 *       1. 从 image 模块读取 l_border/r_border
 *       2. cam_err_calculation() 计算横向误差
 *       3. control_loop() 执行控制算法
 *       4. 输出PWM到 control.left_target_speed / control.right_target_speed
 *       5. motor模块读取target_speed执行电机控制
 */
void control_execute(float v_forward);

// ==================== 辅助函数 ====================

// 速度环PID计算
float speed_pid_calculate(SpeedPID* pid, float target, float actual);

// 重置速度环PID状态
void speed_pid_reset(SpeedPID* pid);

// 初始化速度环PID参数
void speed_pid_init(SpeedPID* pid, float kp, float ki, float kd, 
                    float integral_max, float output_max);

/*-----------------编码器配套--------------------*/
float get_speed(void);

#ifdef __cplusplus
}
#endif

#endif // CONTROL_H_
