/**
 * @file obstacle_auto_replay.h
 * @brief 红色障碍物触发的惯导自动复现模块
 * @note 功能:
 *       1. 监听红色障碍物标志位 (Red_obstacle_flag)
 *       2. 自动切换到惯导复现模式
 *       3. 复现完成后返回正常巡线
 * @date 2025-11-20
 */

#ifndef OBSTACLE_AUTO_REPLAY_H_
#define OBSTACLE_AUTO_REPLAY_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "inertial_nav.h"

// ==================== 状态机定义 ====================

/**
 * @brief 自动复现状态机
 */
typedef enum {
    AUTO_REPLAY_IDLE = 0,           // 空闲状态 (正常巡线)
    AUTO_REPLAY_OBSTACLE_DETECTED,  // 检测到红色障碍物
    AUTO_REPLAY_PREPARING,          // 准备复现 (停车/减速)
    AUTO_REPLAY_REPLAYING,          // 正在复现轨迹
    AUTO_REPLAY_COMPLETED,          // 复现完成
    AUTO_REPLAY_RETURNING,          // 返回巡线模式
} AutoReplayState_t;

/**
 * @brief 自动复现配置参数
 */
typedef struct {
    uint16_t obstacle_distance_threshold;   // 障碍物触发距离阈值 (单位根据Red_obstacle_flag定义)
    uint16_t obstacle_confirm_count;        // 障碍物确认计数 (连续检测N次才触发)
    
    float prepare_decel_distance;           // 准备阶段减速距离 (m)
    float prepare_target_speed;             // 准备阶段目标速度 (m/s)
    
    uint8_t enable_auto_return;             // 复现完成后是否自动返回巡线
    float return_delay_time;                // 返回巡线前的延迟时间 (s)
    
} AutoReplayConfig_t;

/**
 * @brief 自动复现控制器
 */
typedef struct {
    AutoReplayState_t state;                // 当前状态
    AutoReplayConfig_t config;              // 配置参数
    
    // 障碍物检测
    uint16_t obstacle_confirm_counter;      // 障碍物连续检测计数器
    uint16_t last_obstacle_distance;        // 上次检测到的障碍物距离
    
    // 时间控制
    uint32_t state_start_time;              // 当前状态开始时间 (ms)
    uint32_t replay_start_time;             // 复现开始时间 (ms)
    uint32_t replay_end_time;               // 复现结束时间 (ms)
    
    // 位置记录
    float obstacle_detected_x;              // 检测到障碍物时的X坐标
    float obstacle_detected_y;              // 检测到障碍物时的Y坐标
    
    // 统计信息
    uint32_t total_replay_count;            // 总复现次数
    uint32_t success_replay_count;          // 成功复现次数
    float last_replay_duration;             // 上次复现持续时间 (s)
    
} AutoReplayController_t;

// ==================== 全局实例 ====================

extern AutoReplayController_t g_auto_replay;

// ==================== 核心接口函数 ====================

/**
 * @brief 初始化自动复现模块
 * @param obstacle_threshold 障碍物距离阈值
 * @param confirm_count 连续检测确认次数 (建议3~5)
 * @param enable_auto_return 是否自动返回巡线 (1=是, 0=否)
 * @note 使用示例:
 *       obstacle_auto_replay_init(100, 3, 1);  // 距离<100, 连续3次, 自动返回
 */
void obstacle_auto_replay_init(uint16_t obstacle_threshold, 
                               uint16_t confirm_count,
                               uint8_t enable_auto_return);

/**
 * @brief 更新自动复现状态机 (需周期性调用)
 * @param red_obstacle_flag 红色障碍物距离标志位 (来自Element_recognition.h)
 * @param current_time_ms 当前系统时间 (ms)
 * @param pwm_L_out 左轮PWM输出指针 (用于控制电机)
 * @param pwm_R_out 右轮PWM输出指针
 * @return 当前状态
 * @note 调用频率: 建议10ms~50ms (与图像处理同步)
 * @note 使用示例:
 *       // 在主循环中调用
 *       float pwm_L, pwm_R;
 *       AutoReplayState_t state = obstacle_auto_replay_update(
 *           watch.Red_obstacle_flag,
 *           HAL_GetTick(),
 *           &pwm_L,
 *           &pwm_R
 *       );
 */
AutoReplayState_t obstacle_auto_replay_update(uint16_t red_obstacle_flag,
                                               uint32_t current_time_ms,
                                               float* pwm_L_out,
                                               float* pwm_R_out);

/**
 * @brief 强制停止复现，返回巡线模式
 * @note 紧急情况下使用
 */
void obstacle_auto_replay_abort(void);

/**
 * @brief 重置自动复现模块
 */
void obstacle_auto_replay_reset(void);

/**
 * @brief 设置准备阶段参数
 * @param decel_distance 减速距离 (m)
 * @param target_speed 目标速度 (m/s)
 */
void obstacle_auto_replay_set_prepare_params(float decel_distance, float target_speed);

/**
 * @brief 设置返回巡线延迟时间
 * @param delay_time 延迟时间 (s)
 */
void obstacle_auto_replay_set_return_delay(float delay_time);

/**
 * @brief 获取当前状态
 * @return 状态字符串
 */
const char* obstacle_auto_replay_get_state_name(AutoReplayState_t state);

/**
 * @brief 获取统计信息
 * @param total_count 总复现次数输出指针
 * @param success_count 成功次数输出指针
 * @param last_duration 上次持续时间输出指针 (s)
 */
void obstacle_auto_replay_get_statistics(uint32_t* total_count,
                                         uint32_t* success_count,
                                         float* last_duration);

/**
 * @brief 检查是否正在复现
 * @return 1=正在复现, 0=未复现
 */
uint8_t obstacle_auto_replay_is_active(void);

/**
 * @brief 检查是否可以使用正常巡线控制
 * @return 1=可以巡线, 0=应使用惯导控制
 */
uint8_t obstacle_auto_replay_can_use_line_control(void);

#ifdef __cplusplus
}
#endif

#endif /* OBSTACLE_AUTO_REPLAY_H_ */
