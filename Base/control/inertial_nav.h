/**
 * @file inertial_nav.h
 * @brief 三轮差速小车惯性导航系统
 * @note 功能:
 *       1. 轨迹记录: IMU+编码器融合定位
 *       2. 轨迹复现: 闭环控制高精度跟踪
 *       3. 数据融合: 互补滤波消除累积误差
 */

#ifndef INERTIAL_NAV_H_
#define INERTIAL_NAV_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// ==================== 配置参数 ====================

#define NAV_MAX_WAYPOINTS   3000    // 最大航点数量 (50Hz × 60s = 3000)
#define NAV_SAMPLE_RATE_HZ  50      // 采样频率(Hz)
#define NAV_DT              0.02f   // 采样周期(s) = 1/50

// ==================== 数据结构定义 ====================

/**
 * @brief 位姿状态(Pose)
 */
typedef struct {
    float x;            // X坐标(m)
    float y;            // Y坐标(m)
    float theta;        // 航向角(rad), 0=正东, 逆时针为正
    float v;            // 线速度(m/s)
    float omega;        // 角速度(rad/s)
} Pose_t;

/**
 * @brief 航点(Waypoint) - 记录轨迹的离散点
 * @note 优化版：使用定点数压缩存储，8字节/航点
 *       x, y: 范围±32m, 精度1mm (int16 / 1000)
 *       theta: 范围±180°, 精度0.01° (int16 / 100)
 *       v: 范围0~6.5m/s, 精度0.001m/s (uint16 / 1000)
 */
typedef struct {
    int16_t x;          // X坐标(mm), 范围±32m
    int16_t y;          // Y坐标(mm), 范围±32m
    int16_t theta;      // 航向角(0.01°), 范围±180°
    uint16_t v;         // 线速度(mm/s), 范围0~65m/s
} Waypoint_t;

/**
 * @brief 轨迹缓冲区
 */
typedef struct {
    Waypoint_t points[NAV_MAX_WAYPOINTS];  // 航点数组
    uint16_t count;                         // 当前航点数量
    uint16_t current_index;                 // 当前播放索引
    uint8_t is_recording;                   // 录制状态
    uint8_t is_replaying;                   // 复现状态
} Trajectory_t;

/**
 * @brief IMU数据(来自ICM42688P)
 */
typedef struct {
    float gyro_z;       // Z轴角速度(rad/s), 车体坐标系
    float accel_x;      // X轴加速度(m/s²), 前进方向
    float accel_y;      // Y轴加速度(m/s²), 左侧方向
} IMU_Data_t;

/**
 * @brief 编码器数据
 */
typedef struct {
    float v_left;       // 左轮速度(m/s)
    float v_right;      // 右轮速度(m/s)
} Encoder_Data_t;

/**
 * @brief 轨迹跟踪PID控制器
 */
typedef struct {
    // 位置环PID (横向误差)
    float pos_kp;
    float pos_ki;
    float pos_kd;
    float pos_integral;
    float pos_prev_error;
    
    // 航向环PID (角度误差)
    float heading_kp;
    float heading_ki;
    float heading_kd;
    float heading_integral;
    float heading_prev_error;
    
    // 速度前馈
    float velocity_ff;      // 速度前馈系数 0~1
    
    // 限幅
    float max_lateral_speed;    // 最大横向校正速度(m/s)
    float max_angular_speed;    // 最大角速度校正(rad/s)
} TrackingPID_t;

/**
 * @brief 惯导系统主结构体
 */
typedef struct {
    // 当前状态
    Pose_t current_pose;        // 当前位姿
    Pose_t initial_pose;        // 初始位姿(用于复位)
    
    // 轨迹数据
    Trajectory_t trajectory;    // 轨迹缓冲区
    
    // 传感器数据
    IMU_Data_t imu;            // IMU数据
    Encoder_Data_t encoder;    // 编码器数据
    
    // 控制器
    TrackingPID_t tracking_pid; // 轨迹跟踪控制器
    
    // 运动学参数
    float wheel_base;           // 轮距(m)
    float wheel_radius;         // 轮半径(m)
    
    // 融合参数
    float gyro_trust;           // 陀螺仪信任度 0~1 (互补滤波系数)
    
    // 统计信息
    float max_position_error;   // 最大位置误差(m)
    float avg_position_error;   // 平均位置误差(m)
    float max_heading_error;    // 最大航向误差(rad)
    
} InertialNav_t;

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
} control_t;

extern control_t control;
// ==================== 全局实例 ====================

extern InertialNav_t g_inertial_nav;

// ==================== 核心接口函数 ====================

/**
 * @brief 初始化惯性导航系统
 * @param wheel_base 轮距(m), 例如0.16m
 * @param wheel_radius 轮半径(m), 例如0.034m
 * @param gyro_trust 陀螺仪信任度(0~1), 建议0.7
 *                   越高越相信陀螺仪, 越低越相信编码器
 * @note 使用示例:
 *       inertial_nav_init(0.16f, 0.034f, 0.7f);
 */
void inertial_nav_init(float wheel_base, float wheel_radius, float gyro_trust);

/**
 * @brief 更新传感器数据
 * @param gyro_z Z轴角速度(rad/s), 来自IMU
 * @param v_left 左轮速度(m/s), 来自编码器
 * @param v_right 右轮速度(m/s), 来自编码器
 * @note 调用频率: 50Hz (每20ms调用一次)
 * @note 使用示例:
 *       // 在定时器中断中(20ms周期):
 *       float gyro_z = imu_data.gyro_z * (3.14159f / 180.0f);  // 转换为rad/s
 *       inertial_nav_update(gyro_z, 
 *                          control.left_speed, 
 *                          control.right_speed);
 */
void inertial_nav_update(float gyro_z, float v_left, float v_right);

// ==================== 轨迹记录接口 ====================

/**
 * @brief 开始记录轨迹
 * @note 清空缓冲区, 复位位姿到原点
 */
void inertial_nav_start_recording(void);

/**
 * @brief 停止记录轨迹
 * @return 记录的航点数量
 */
uint16_t inertial_nav_stop_recording(void);

/**
 * @brief 清除已记录的轨迹
 */
void inertial_nav_clear_trajectory(void);

/**
 * @brief 获取轨迹信息
 * @param waypoint_count 输出航点数量
 * @param duration_ms 输出轨迹总时长(ms)
 * @param total_distance 输出轨迹总长度(m)
 */
void inertial_nav_get_trajectory_info(uint16_t* waypoint_count, 
                                      uint32_t* duration_ms,
                                      float* total_distance);

// ==================== 轨迹复现接口 ====================

/**
 * @brief 开始轨迹复现
 * @return 0=成功, 1=无轨迹数据, 2=已在复现中
 * @note 复位位姿到初始点, 开始闭环跟踪
 */
uint8_t inertial_nav_start_replay(void);

/**
 * @brief 停止轨迹复现
 */
void inertial_nav_stop_replay(void);

/**
 * @brief 暂停/继续轨迹复现
 * @param pause 1=暂停, 0=继续
 */
void inertial_nav_pause_replay(uint8_t pause);

/**
 * @brief 轨迹跟踪控制器(在复现模式下调用)
 * @param v_out 输出线速度指令(m/s)
 * @param omega_out 输出角速度指令(rad/s)
 * @note 调用频率: 与inertial_nav_update相同(100Hz)
 * @note 使用示例:
 *       float v_cmd, omega_cmd;
 *       inertial_nav_tracking_control(&v_cmd, &omega_cmd);
 *       // 将v_cmd, omega_cmd转换为左右轮速度
 *       float vL = v_cmd - omega_cmd * wheel_base / 2;
 *       float vR = v_cmd + omega_cmd * wheel_base / 2;
 */
void inertial_nav_tracking_control(float* v_out, float* omega_out);

// ==================== 位姿管理接口 ====================

/**
 * @brief 复位位姿到原点
 */
void inertial_nav_reset_pose(void);

/**
 * @brief 设置当前位姿
 * @param x X坐标(m)
 * @param y Y坐标(m)
 * @param theta 航向角(rad)
 */
void inertial_nav_set_pose(float x, float y, float theta);

/**
 * @brief 获取当前位姿
 * @return 当前位姿指针(只读)
 */
const Pose_t* inertial_nav_get_pose(void);

/**
 * @brief 获取目标位姿(复现模式下的目标点)
 * @return 目标位姿指针, 如果不在复现模式返回NULL
 */
const Pose_t* inertial_nav_get_target_pose(void);

// ==================== 参数调整接口 ====================

/**
 * @brief 设置位置环PID参数
 * @param kp 比例系数, 建议0.5~2.0
 * @param ki 积分系数, 建议0.0~0.1
 * @param kd 微分系数, 建议0.0~0.5
 */
void inertial_nav_set_position_pid(float kp, float ki, float kd);

/**
 * @brief 设置航向环PID参数
 * @param kp 比例系数, 建议1.0~5.0
 * @param ki 积分系数, 建议0.0~0.2
 * @param kd 微分系数, 建议0.0~1.0
 */
void inertial_nav_set_heading_pid(float kp, float ki, float kd);

/**
 * @brief 设置速度前馈系数
 * @param ff 前馈系数 0~1, 建议0.8~0.95
 *           越大越依赖记录的速度, 越小越依赖PID校正
 */
void inertial_nav_set_velocity_feedforward(float ff);

/**
 * @brief 设置陀螺仪信任度(互补滤波系数)
 * @param trust 信任度 0~1, 建议0.6~0.8
 *              1.0=完全信任陀螺仪, 0.0=完全信任编码器
 */
void inertial_nav_set_gyro_trust(float trust);

// ==================== 调试与监控接口 ====================

/**
 * @brief 获取跟踪误差统计
 * @param max_pos_error 最大位置误差(m)
 * @param avg_pos_error 平均位置误差(m)
 * @param max_heading_error 最大航向误差(rad)
 */
void inertial_nav_get_error_stats(float* max_pos_error, 
                                  float* avg_pos_error,
                                  float* max_heading_error);

/**
 * @brief 重置误差统计
 */
void inertial_nav_reset_error_stats(void);

/**
 * @brief 获取系统状态
 * @return 指向惯导系统结构体的指针(调试用)
 */
InertialNav_t* inertial_nav_get_state(void);

/**
 * @brief 轨迹数据导出(通过串口或其他方式)
 * @param callback 回调函数, 每个航点调用一次
 * @note 使用示例:
 *       void print_waypoint(const Waypoint_t* wp, uint16_t index) {
 *           printf("%d, %.3f, %.3f, %.3f\n", 
 *                  index, wp->x, wp->y, wp->theta);
 *       }
 *       inertial_nav_export_trajectory(print_waypoint);
 */
void inertial_nav_export_trajectory(void (*callback)(const Waypoint_t* wp, uint16_t index));

#ifdef __cplusplus
}
#endif

#endif /* INERTIAL_NAV_H_ */

// ==================== 内存优化说明 ====================
/*
 * 【优化前】
 *   Waypoint_t: 24 bytes (5×float + 1×uint32_t)
 *   缓冲区: 6000 航点 × 24 bytes = 144 KB (占RAM 18%)
 * 
 * 【优化后】
 *   Waypoint_t: 8 bytes (3×int16_t + 1×uint16_t)
 *   缓冲区: 3000 航点 × 8 bytes = 24 KB (占RAM 3%)
 * 
 * 【优化效果】
 *   内存占用减少 83% (140.6 KB → 23.4 KB)
 *   录制时长保持 60 秒 (采样率 100Hz → 50Hz)
 *   精度完全满足需求:
 *     位置: ±32m 范围, 1mm 精度
 *     角度: ±180° 范围, 0.01° 精度
 *     速度: 0~65 m/s 范围, 1mm/s 精度
 * 
 * 【技术方案】
 *   1. 定点数压缩: float → int16/uint16
 *   2. 去除冗余字段: omega 和 timestamp 不存储
 *   3. 降低采样率: 100Hz → 50Hz (控制周期仍足够)
 *   4. 运行时转换: compress/decompress 函数透明处理
 */

// ==================== Flash优化说明 ====================
/*
 * 【删除的复杂函数】
 *   sqrtf   → fast_sqrt (牛顿迭代法, 精度0.1%)
 *   sinf    → fast_sincos (泰勒展开, 精度0.5%)
 *   cosf    → fast_sincos (泰勒展开, 精度0.5%)
 *   atan2f  → 向量叉积 (dx*sin - dy*cos, 完全等价)
 *   fabsf   → 三元运算符 (x>=0 ? x : -x)
 * 
 * 【优化技术】
 *   1. fast_sqrt: 位操作初值 + 2次牛顿迭代
 *   2. fast_sincos: 对称性映射 + 3阶泰勒展开
 *   3. 横向误差: 向量叉积代替atan2+sin组合
 *   4. 删除math.h依赖: 避免链接浮点数学库
 * 
 * 【Flash节省】
 *   浮点数学库: 约 10-16 KB
 * 
 * 【精度保证】
 *   位置更新误差: <0.5% (三角函数近似)
 *   距离计算误差: <0.1% (平方根近似)
 *   控制性能: 无明显影响 (误差远小于控制精度)
 */
