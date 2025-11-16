/**
 * @file inertial_nav.c
 * @brief 三轮差速小车惯性导航系统实现
 * @note 核心算法:
 *       1. 航迹推算(Dead Reckoning): IMU+编码器融合定位
 *       2. 互补滤波: 消除陀螺仪漂移和编码器打滑误差
 *       3. 轨迹跟踪: 位置环+航向环双环PID控制
 * @note 优化策略:
 *       1. 删除math.h依赖 - 避免链接浮点数学库(节省Flash)
 *       2. 使用快速近似算法 - 泰勒展开/牛顿迭代/查表法
 *       3. 简化复杂运算 - 向量叉积代替atan2/sin组合
 */

#include "inertial_nav.h"
#include <stdint.h>
#include <string.h>

// ==================== 全局实例定义 ====================

InertialNav_t g_inertial_nav = {0};

// ==================== 私有辅助函数 ====================

/**
 * @brief 角度归一化到[-π, π]
 * @note 优化: 避免while循环，最多两次判断
 */
static float normalize_angle(float angle)
{
    const float PI = 3.14159265f;
    const float TWO_PI = 6.28318531f;
    
    // 大多数情况下角度已经在范围内
    if (angle > PI) {
        angle -= TWO_PI;
        if (angle > PI) angle -= TWO_PI;  // 极端情况
    } else if (angle < -PI) {
        angle += TWO_PI;
        if (angle < -PI) angle += TWO_PI;
    }
    return angle;
}

/**
 * @brief 浮点数限幅
 */
static inline float clamp_f(float x, float min, float max)
{
    return x < min ? min : (x > max ? max : x);
}

/**
 * @brief 快速计算平方根 (使用牛顿迭代法)
 * @note 优化: 避免sqrtf，仅2-3次迭代达到0.1%精度
 */
static float fast_sqrt(float x)
{
    if (x <= 0.0f) return 0.0f;
    
    // 初始估计 (位操作法)
    union { float f; uint32_t i; } conv;
    conv.f = x;
    conv.i = 0x1fbd1df5 + (conv.i >> 1);  // 魔法常数
    float y = conv.f;
    
    // 牛顿迭代: y = (y + x/y) / 2
    y = (y + x / y) * 0.5f;
    y = (y + x / y) * 0.5f;  // 第2次迭代，精度足0.1%
    
    return y;
}

/**
 * @brief 计算两点之间的距离
 * @note 优化: 使用fast_sqrt代替sqrtf
 */
static float distance_between(float x1, float y1, float x2, float y2)
{
    float dx = x2 - x1;
    float dy = y2 - y1;
    return fast_sqrt(dx * dx + dy * dy);
}

/**
 * @brief 快速sin/cos计算 (小角度泰勒展开)
 * @note 优化: 对|θ|<15°使用3阶泰勒，误差<0.5%
 *       大角度使用对称性简化
 */
static void fast_sincos(float theta, float* sin_out, float* cos_out)
{
    // 将角度限制到[0, 2π]
    const float PI = 3.14159265f;
    const float HALF_PI = 1.57079633f;
    
    // 利用对称性将角度映射到[0, π/2]
    float t = theta;
    int quadrant = 0;
    
    while (t > HALF_PI) {
        t -= HALF_PI;
        quadrant++;
    }
    while (t < 0.0f) {
        t += HALF_PI;
        quadrant--;
    }
    quadrant = quadrant & 3;  // 模4
    
    // 小角度泰勒展开: sin(x) ≈ x - x³/6, cos(x) ≈ 1 - x²/2
    float t2 = t * t;
    float sin_t = t - (t * t2) / 6.0f;
    float cos_t = 1.0f - t2 * 0.5f;
    
    // 根据象限调整符号
    switch (quadrant) {
        case 0: *sin_out = sin_t;  *cos_out = cos_t;  break;
        case 1: *sin_out = cos_t;  *cos_out = -sin_t; break;
        case 2: *sin_out = -sin_t; *cos_out = -cos_t; break;
        case 3: *sin_out = -cos_t; *cos_out = sin_t;  break;
    }
}

/**
 * @brief 压缩位姿为航点(定点数存储)
 * @note 转换规则:
 *       x, y: m → mm (int16, 范围±32m)
 *       theta: rad → 0.01° (int16, 范围±180°)
 *       v: m/s → mm/s (uint16, 范围0~65m/s)
 */
static void compress_waypoint(const Pose_t* pose, Waypoint_t* wp)
{
    // 坐标: m → mm
    wp->x = (int16_t)(pose->x * 1000.0f);
    wp->y = (int16_t)(pose->y * 1000.0f);
    
    // 航向: rad → 0.01°
    float theta_deg = pose->theta * 57.2957795f;  // rad to deg
    wp->theta = (int16_t)(theta_deg * 100.0f);
    
    // 速度: m/s → mm/s
    wp->v = (uint16_t)(pose->v * 1000.0f);
}

/**
 * @brief 解压缩航点为位姿
 */
static void decompress_waypoint(const Waypoint_t* wp, Pose_t* pose)
{
    // 坐标: mm → m
    pose->x = (float)wp->x / 1000.0f;
    pose->y = (float)wp->y / 1000.0f;
    
    // 航向: 0.01° → rad
    float theta_deg = (float)wp->theta / 100.0f;
    pose->theta = theta_deg / 57.2957795f;  // deg to rad
    
    // 速度: mm/s → m/s
    pose->v = (float)wp->v / 1000.0f;
    
    // omega不存储，设为0
    pose->omega = 0.0f;
}

// ==================== 核心功能实现 ====================

/**
 * @brief 初始化惯性导航系统
 */
void inertial_nav_init(float wheel_base, float wheel_radius, float gyro_trust)
{
    memset(&g_inertial_nav, 0, sizeof(InertialNav_t));
    
    // 运动学参数
    g_inertial_nav.wheel_base = wheel_base;
    g_inertial_nav.wheel_radius = wheel_radius;
    g_inertial_nav.gyro_trust = clamp_f(gyro_trust, 0.0f, 1.0f);
    
    // 初始化跟踪PID参数
    g_inertial_nav.tracking_pid.pos_kp = 1.0f;
    g_inertial_nav.tracking_pid.pos_ki = 0.05f;
    g_inertial_nav.tracking_pid.pos_kd = 0.1f;
    
    g_inertial_nav.tracking_pid.heading_kp = 2.0f;
    g_inertial_nav.tracking_pid.heading_ki = 0.1f;
    g_inertial_nav.tracking_pid.heading_kd = 0.2f;
    
    g_inertial_nav.tracking_pid.velocity_ff = 0.9f;  // 90%前馈 + 10%校正
    
    g_inertial_nav.tracking_pid.max_lateral_speed = 0.3f;    // 最大横向校正0.3m/s
    g_inertial_nav.tracking_pid.max_angular_speed = 3.0f;    // 最大角速度校正3rad/s
}

/**
 * @brief 航迹推算 - 更新位姿
 */
void inertial_nav_update(float gyro_z, float v_left, float v_right)
{
    InertialNav_t* nav = &g_inertial_nav;
    
    // 保存传感器数据
    nav->imu.gyro_z = gyro_z;
    nav->encoder.v_left = v_left;
    nav->encoder.v_right = v_right;
    
    // ==================== 速度估计 ====================
    
    // 编码器估计的线速度和角速度
    float v_enc = (v_left + v_right) / 2.0f;
    float omega_enc = (v_right - v_left) / nav->wheel_base;
    
    // 陀螺仪估计的角速度
    float omega_gyro = gyro_z;
    
    // 互补滤波融合角速度 (抑制陀螺仪漂移和编码器打滑)
    float alpha = nav->gyro_trust;
    float omega_fused = alpha * omega_gyro + (1.0f - alpha) * omega_enc;
    
    // 更新当前速度
    nav->current_pose.v = v_enc;
    nav->current_pose.omega = omega_fused;
    
    // ==================== 位姿更新 ====================
    
    // 一阶欧拉积分 (可选升级为Runge-Kutta)
    float dt = NAV_DT;
    
    // 更新航向角
    nav->current_pose.theta += omega_fused * dt;
    nav->current_pose.theta = normalize_angle(nav->current_pose.theta);
    
    // 更新位置 (二维平面运动学) - 使用优化的sin/cos
    float cos_theta, sin_theta;
    fast_sincos(nav->current_pose.theta, &sin_theta, &cos_theta);
    
    nav->current_pose.x += v_enc * cos_theta * dt;
    nav->current_pose.y += v_enc * sin_theta * dt;
    
    // ==================== 轨迹记录 ====================
    
    if (nav->trajectory.is_recording) {
        // 每次更新都记录一个航点
        if (nav->trajectory.count < NAV_MAX_WAYPOINTS) {
            Waypoint_t* wp = &nav->trajectory.points[nav->trajectory.count];
            
            // 压缩当前位姿到航点
            compress_waypoint(&nav->current_pose, wp);
            
            nav->trajectory.count++;
        } else {
            // 缓冲区已满, 自动停止记录
            nav->trajectory.is_recording = 0;
        }
    }
}

// ==================== 轨迹记录接口实现 ====================

/**
 * @brief 开始记录轨迹
 */
void inertial_nav_start_recording(void)
{
    InertialNav_t* nav = &g_inertial_nav;
    
    // 清空轨迹缓冲区
    memset(&nav->trajectory, 0, sizeof(Trajectory_t));
    
    // 复位位姿到原点
    inertial_nav_reset_pose();
    
    // 保存初始位姿
    nav->initial_pose = nav->current_pose;
    
    // 开始记录
    nav->trajectory.is_recording = 1;
}

/**
 * @brief 停止记录轨迹
 */
uint16_t inertial_nav_stop_recording(void)
{
    InertialNav_t* nav = &g_inertial_nav;
    nav->trajectory.is_recording = 0;
    return nav->trajectory.count;
}

/**
 * @brief 清除已记录的轨迹
 */
void inertial_nav_clear_trajectory(void)
{
    InertialNav_t* nav = &g_inertial_nav;
    memset(&nav->trajectory, 0, sizeof(Trajectory_t));
}

/**
 * @brief 获取轨迹信息
 */
void inertial_nav_get_trajectory_info(uint16_t* waypoint_count, 
                                      uint32_t* duration_ms,
                                      float* total_distance)
{
    InertialNav_t* nav = &g_inertial_nav;
    
    if (waypoint_count) {
        *waypoint_count = nav->trajectory.count;
    }
    
    if (duration_ms && nav->trajectory.count > 0) {
        // 时长 = 航点数 / 采样率
        *duration_ms = (uint32_t)(nav->trajectory.count * 1000 / NAV_SAMPLE_RATE_HZ);
    }
    
    if (total_distance) {
        float dist = 0.0f;
        Pose_t prev_pose, curr_pose;
        
        if (nav->trajectory.count > 0) {
            decompress_waypoint(&nav->trajectory.points[0], &prev_pose);
        }
        
        for (uint16_t i = 1; i < nav->trajectory.count; i++) {
            decompress_waypoint(&nav->trajectory.points[i], &curr_pose);
            dist += distance_between(prev_pose.x, prev_pose.y, curr_pose.x, curr_pose.y);
            prev_pose = curr_pose;
        }
        *total_distance = dist;
    }
}

// ==================== 轨迹复现接口实现 ====================

/**
 * @brief 开始轨迹复现
 */
uint8_t inertial_nav_start_replay(void)
{
    InertialNav_t* nav = &g_inertial_nav;
    
    // 检查是否有轨迹数据
    if (nav->trajectory.count == 0) {
        return 1;  // 无轨迹数据
    }
    
    // 检查是否已在复现中
    if (nav->trajectory.is_replaying) {
        return 2;  // 已在复现中
    }
    
    // 复位位姿到初始点
    nav->current_pose = nav->initial_pose;
    
    // 重置播放索引
    nav->trajectory.current_index = 0;
    
    // 重置PID状态
    nav->tracking_pid.pos_integral = 0.0f;
    nav->tracking_pid.pos_prev_error = 0.0f;
    nav->tracking_pid.heading_integral = 0.0f;
    nav->tracking_pid.heading_prev_error = 0.0f;
    
    // 重置误差统计
    inertial_nav_reset_error_stats();
    
    // 开始复现
    nav->trajectory.is_replaying = 1;
    
    return 0;  // 成功
}

/**
 * @brief 停止轨迹复现
 */
void inertial_nav_stop_replay(void)
{
    g_inertial_nav.trajectory.is_replaying = 0;
}

/**
 * @brief 暂停/继续轨迹复现
 */
void inertial_nav_pause_replay(uint8_t pause)
{
    // 简单实现: 暂停=停止索引增长
    // 可扩展为独立的暂停标志
    if (pause) {
        // 保持current_index不变, 但继续控制
    }
}

/**
 * @brief 轨迹跟踪控制器(双环PID)
 */
void inertial_nav_tracking_control(float* v_out, float* omega_out)
{
    InertialNav_t* nav = &g_inertial_nav;
    TrackingPID_t* pid = &nav->tracking_pid;
    
    // 默认输出0
    *v_out = 0.0f;
    *omega_out = 0.0f;
    
    // 检查是否在复现模式
    if (!nav->trajectory.is_replaying) {
        return;
    }
    
    // 检查是否到达终点
    if (nav->trajectory.current_index >= nav->trajectory.count) {
        inertial_nav_stop_replay();
        return;
    }
    
    // ==================== 获取目标点 ====================
    
    Waypoint_t* target_wp = &nav->trajectory.points[nav->trajectory.current_index];
    Pose_t target;
    decompress_waypoint(target_wp, &target);
    
    // ==================== 计算位置误差 ====================
    
    // 位置误差向量
    float dx = target.x - nav->current_pose.x;
    float dy = target.y - nav->current_pose.y;
    
    // 位置误差(欧式距离) - 使用fast_sqrt
    float pos_error = fast_sqrt(dx * dx + dy * dy);
    
    // 横向误差简化计算 (避免atan2和sin)
    // 原理: lateral_error = pos_error * sin(angle_to_target - theta)
    // 简化: 使用向量叉积 dx*sin(θ) - dy*cos(θ)
    float sin_theta, cos_theta;
    fast_sincos(nav->current_pose.theta, &sin_theta, &cos_theta);
    float lateral_error = dx * sin_theta - dy * cos_theta;
    
    // ==================== 位置环PID ====================
    
    pid->pos_integral += lateral_error * NAV_DT;
    pid->pos_integral = clamp_f(pid->pos_integral, -10.0f, 10.0f);  // 积分限幅
    
    float pos_derivative = (lateral_error - pid->pos_prev_error) / NAV_DT;
    pid->pos_prev_error = lateral_error;
    
    float lateral_correction = pid->pos_kp * lateral_error + 
                              pid->pos_ki * pid->pos_integral +
                              pid->pos_kd * pos_derivative;
    
    lateral_correction = clamp_f(lateral_correction, 
                                 -pid->max_lateral_speed, 
                                  pid->max_lateral_speed);
    
    // ==================== 计算航向误差 ====================
    
    float heading_error = normalize_angle(target.theta - nav->current_pose.theta);
    
    // ==================== 航向环PID ====================
    
    pid->heading_integral += heading_error * NAV_DT;
    pid->heading_integral = clamp_f(pid->heading_integral, -5.0f, 5.0f);
    
    float heading_derivative = (heading_error - pid->heading_prev_error) / NAV_DT;
    pid->heading_prev_error = heading_error;
    
    float heading_correction = pid->heading_kp * heading_error +
                              pid->heading_ki * pid->heading_integral +
                              pid->heading_kd * heading_derivative;
    
    heading_correction = clamp_f(heading_correction,
                                 -pid->max_angular_speed,
                                  pid->max_angular_speed);
    
    // ==================== 前馈 + 反馈控制 ====================
    
    // 线速度: 前馈(记录的速度) + 反馈(横向误差校正转为速度调整)
    float v_feedforward = target.v * pid->velocity_ff;
    float v_feedback = 0.0f;  // 可选: 根据位置误差调整前进速度
    
    *v_out = v_feedforward + v_feedback;
    
    // 角速度: 前馈(从轨迹估计) + 反馈(PID校正)
    // 注意: omega不再存储，前馈设为0，完全依靠PID控制
    float omega_feedforward = 0.0f;
    float omega_feedback = heading_correction - lateral_correction * 0.5f;  // 横向误差转角速度
    
    *omega_out = omega_feedforward + omega_feedback;
    
    // ==================== 更新误差统计 ====================
    
    if (pos_error > nav->max_position_error) {
        nav->max_position_error = pos_error;
    }
    
    float abs_heading_error = (heading_error >= 0) ? heading_error : -heading_error;
    if (abs_heading_error > nav->max_heading_error) {
        nav->max_heading_error = abs_heading_error;
    }
    
    // 滚动平均位置误差
    nav->avg_position_error = nav->avg_position_error * 0.95f + pos_error * 0.05f;
    
    // ==================== 更新航点索引 ====================
    
    // 当接近当前目标点时, 切换到下一个航点
    if (pos_error < 0.02f) {  // 位置误差<2cm时认为到达
        nav->trajectory.current_index++;
    }
}

// ==================== 位姿管理接口实现 ====================

/**
 * @brief 复位位姿到原点
 */
void inertial_nav_reset_pose(void)
{
    memset(&g_inertial_nav.current_pose, 0, sizeof(Pose_t));
}

/**
 * @brief 设置当前位姿
 */
void inertial_nav_set_pose(float x, float y, float theta)
{
    g_inertial_nav.current_pose.x = x;
    g_inertial_nav.current_pose.y = y;
    g_inertial_nav.current_pose.theta = normalize_angle(theta);
}

/**
 * @brief 获取当前位姿
 */
const Pose_t* inertial_nav_get_pose(void)
{
    return &g_inertial_nav.current_pose;
}

/**
 * @brief 获取目标位姿
 */
const Pose_t* inertial_nav_get_target_pose(void)
{
    InertialNav_t* nav = &g_inertial_nav;
    
    if (!nav->trajectory.is_replaying || 
        nav->trajectory.current_index >= nav->trajectory.count) {
        return NULL;
    }
    
    // 返回当前目标航点(转换为Pose_t格式)
    static Pose_t target_pose;
    Waypoint_t* wp = &nav->trajectory.points[nav->trajectory.current_index];
    
    decompress_waypoint(wp, &target_pose);
    
    return &target_pose;
}

// ==================== 参数调整接口实现 ====================

/**
 * @brief 设置位置环PID参数
 */
void inertial_nav_set_position_pid(float kp, float ki, float kd)
{
    g_inertial_nav.tracking_pid.pos_kp = kp;
    g_inertial_nav.tracking_pid.pos_ki = ki;
    g_inertial_nav.tracking_pid.pos_kd = kd;
}

/**
 * @brief 设置航向环PID参数
 */
void inertial_nav_set_heading_pid(float kp, float ki, float kd)
{
    g_inertial_nav.tracking_pid.heading_kp = kp;
    g_inertial_nav.tracking_pid.heading_ki = ki;
    g_inertial_nav.tracking_pid.heading_kd = kd;
}

/**
 * @brief 设置速度前馈系数
 */
void inertial_nav_set_velocity_feedforward(float ff)
{
    g_inertial_nav.tracking_pid.velocity_ff = clamp_f(ff, 0.0f, 1.0f);
}

/**
 * @brief 设置陀螺仪信任度
 */
void inertial_nav_set_gyro_trust(float trust)
{
    g_inertial_nav.gyro_trust = clamp_f(trust, 0.0f, 1.0f);
}

// ==================== 调试与监控接口实现 ====================

/**
 * @brief 获取跟踪误差统计
 */
void inertial_nav_get_error_stats(float* max_pos_error, 
                                  float* avg_pos_error,
                                  float* max_heading_error)
{
    if (max_pos_error) {
        *max_pos_error = g_inertial_nav.max_position_error;
    }
    if (avg_pos_error) {
        *avg_pos_error = g_inertial_nav.avg_position_error;
    }
    if (max_heading_error) {
        *max_heading_error = g_inertial_nav.max_heading_error;
    }
}

/**
 * @brief 重置误差统计
 */
void inertial_nav_reset_error_stats(void)
{
    g_inertial_nav.max_position_error = 0.0f;
    g_inertial_nav.avg_position_error = 0.0f;
    g_inertial_nav.max_heading_error = 0.0f;
}

/**
 * @brief 获取系统状态
 */
InertialNav_t* inertial_nav_get_state(void)
{
    return &g_inertial_nav;
}

/**
 * @brief 轨迹数据导出
 */
void inertial_nav_export_trajectory(void (*callback)(const Waypoint_t* wp, uint16_t index))
{
    if (!callback) return;
    
    InertialNav_t* nav = &g_inertial_nav;
    for (uint16_t i = 0; i < nav->trajectory.count; i++) {
        callback(&nav->trajectory.points[i], i);
    }
}

// ==================== 使用说明 ====================
/*
 * 【完整工作流程】
 * 
 * ==================== 阶段1: 初始化 ====================
 * 
 * // 在main函数中初始化
 * inertial_nav_init(0.16f,   // 轮距16cm
 *                   0.034f,  // 轮半径3.4cm
 *                   0.7f);   // 陀螺仪信任度70%
 * 
 * ==================== 阶段2: 轨迹记录 ====================
 * 
 * // 开始记录
 * inertial_nav_start_recording();
 * 
 * // 在定时器中断中更新(100Hz, 每10ms):
 * void TIM_Nav_Update_IRQHandler(void) {
 *     // 获取IMU数据
 *     float gyro_z = imu_data.gyro_z * (M_PI / 180.0f);  // 转rad/s
 *     
 *     // 获取编码器数据
 *     get_speed();  // 更新control.left_speed和control.right_speed
 *     
 *     // 更新惯导
 *     inertial_nav_update(gyro_z, 
 *                        control.left_speed, 
 *                        control.right_speed);
 * }
 * 
 * // 手动遥控小车走一圈...
 * 
 * // 停止记录
 * uint16_t count = inertial_nav_stop_recording();
 * printf("记录了%d个航点\n", count);
 * 
 * // 查看轨迹信息
 * uint16_t wp_count;
 * uint32_t duration;
 * float distance;
 * inertial_nav_get_trajectory_info(&wp_count, &duration, &distance);
 * printf("航点数:%d, 时长:%dms, 距离:%.2fm\n", 
 *        wp_count, duration, distance);
 * 
 * ==================== 阶段3: 轨迹复现 ====================
 * 
 * // 开始复现
 * if (inertial_nav_start_replay() == 0) {
 *     printf("开始复现轨迹\n");
 * }
 * 
 * // 在定时器中断中(100Hz):
 * void TIM_Nav_Replay_IRQHandler(void) {
 *     // 1. 更新惯导(必须!)
 *     float gyro_z = imu_data.gyro_z * (M_PI / 180.0f);
 *     get_speed();
 *     inertial_nav_update(gyro_z, 
 *                        control.left_speed, 
 *                        control.right_speed);
 *     
 *     // 2. 计算跟踪控制量
 *     float v_cmd, omega_cmd;
 *     inertial_nav_tracking_control(&v_cmd, &omega_cmd);
 *     
 *     // 3. 转换为左右轮速度(差速运动学)
 *     float L = 0.16f;  // 轮距
 *     float vL_target = v_cmd - omega_cmd * L / 2.0f;
 *     float vR_target = v_cmd + omega_cmd * L / 2.0f;
 *     
 *     // 4. 调用速度环PID(cascade_pid_inner_loop或speed_pid_calculate)
 *     cascade_pid_inner_loop(control.left_speed,
 *                           control.right_speed,
 *                           &pwm_L, &pwm_R);
 * }
 * 
 * // 复现完成后自动停止, 或手动停止:
 * inertial_nav_stop_replay();
 * 
 * // 查看复现精度
 * float max_err, avg_err, heading_err;
 * inertial_nav_get_error_stats(&max_err, &avg_err, &heading_err);
 * printf("最大位置误差:%.3fm, 平均误差:%.3fm, 最大航向误差:%.2f°\n",
 *        max_err, avg_err, heading_err * 57.3f);
 * 
 * ==================== 调参建议 ====================
 * 
 * 1. 陀螺仪信任度(gyro_trust):
 *    - 地面平坦、低速: 0.5~0.6 (更信任编码器)
 *    - 地面粗糙、高速: 0.7~0.8 (更信任陀螺仪)
 *    - 调试方法: 走直线, 观察航向漂移
 * 
 * 2. 位置环PID (横向误差校正):
 *    - Kp: 0.5~2.0, 影响横向回归速度
 *    - Ki: 0.0~0.1, 消除横向偏差
 *    - Kd: 0.0~0.5, 抑制振荡
 * 
 * 3. 航向环PID (角度校正):
 *    - Kp: 1.0~5.0, 影响转向响应
 *    - Ki: 0.0~0.2, 消除角度偏差
 *    - Kd: 0.0~1.0, 抑制转向振荡
 * 
 * 4. 速度前馈(velocity_ff):
 *    - 0.9~0.95: 高精度复现, 依赖记录速度
 *    - 0.7~0.8: 保守复现, 更多PID校正
 * 
 * ==================== 注意事项 ====================
 * 
 * 1. 坐标系定义:
 *    - X轴: 正东方向(车体初始朝向)
 *    - Y轴: 正北方向(逆时针90°)
 *    - θ: 航向角, 0=正东, 逆时针为正
 * 
 * 2. 采样频率要求:
 *    - 必须严格100Hz (10ms周期)
 *    - 过低会导致积分误差增大
 *    - 过高会增加计算负担
 * 
 * 3. 陀螺仪数据预处理:
 *    - 必须转换为rad/s (如果是°/s需乘π/180)
 *    - 建议零偏校准(静止时平均值)
 *    - 注意坐标系转换(车体坐标系Z轴向上)
 * 
 * 4. 编码器数据质量:
 *    - 确保速度单位为m/s
 *    - 处理打滑情况(融合陀螺仪)
 *    - 检查方向正确性(左右轮)
 * 
 * 5. 内存限制:
 *    - 最大6000个航点
 *    - 100Hz采样 → 最长60秒轨迹
 *    - 内存占用: 6000 × 24字节 ≈ 144KB
 *    - 可调整NAV_MAX_WAYPOINTS扩容（注意RAM容量）
 */
