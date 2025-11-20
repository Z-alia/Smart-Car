/**
 * @file obstacle_auto_replay.c
 * @brief 红色障碍物触发的惯导自动复现模块实现
 * @note 核心流程:
 *       1. 监听Red_obstacle_flag → 检测到障碍物
 *       2. 减速准备 → 切换到惯导复现模式
 *       3. 播放存储的轨迹 → 闭环跟踪控制
 *       4. 复现完成 → 返回正常巡线
 * @date 2025-11-20
 */

#include "obstacle_auto_replay.h"
#include "inertial_nav.h"
#include <string.h>

// ==================== 全局实例定义 ====================

AutoReplayController_t g_auto_replay = {0};

// ==================== 私有辅助函数 ====================

/**
 * @brief 浮点数限幅
 */
static inline float clamp_f(float x, float min, float max)
{
    return x < min ? min : (x > max ? max : x);
}

/**
 * @brief 获取经过的时间 (ms)
 */
static inline uint32_t time_elapsed(uint32_t start_time, uint32_t current_time)
{
    // 处理溢出情况
    if (current_time >= start_time) {
        return current_time - start_time;
    } else {
        return (0xFFFFFFFF - start_time) + current_time + 1;
    }
}

/**
 * @brief 状态切换
 */
static void transition_to_state(AutoReplayState_t new_state, uint32_t current_time_ms)
{
    g_auto_replay.state = new_state;
    g_auto_replay.state_start_time = current_time_ms;
}

// ==================== 核心功能实现 ====================

/**
 * @brief 初始化自动复现模块
 */
void obstacle_auto_replay_init(uint16_t obstacle_threshold, 
                               uint16_t confirm_count,
                               uint8_t enable_auto_return)
{
    memset(&g_auto_replay, 0, sizeof(AutoReplayController_t));
    
    // 配置参数
    g_auto_replay.config.obstacle_distance_threshold = obstacle_threshold;
    g_auto_replay.config.obstacle_confirm_count = confirm_count;
    g_auto_replay.config.enable_auto_return = enable_auto_return;
    
    // 默认参数
    g_auto_replay.config.prepare_decel_distance = 0.5f;     // 提前0.5m减速
    g_auto_replay.config.prepare_target_speed = 0.1f;       // 减速到0.1m/s
    g_auto_replay.config.return_delay_time = 1.0f;          // 延迟1s返回巡线
    
    // 初始状态
    g_auto_replay.state = AUTO_REPLAY_IDLE;
}

/**
 * @brief 处理空闲状态 (正常巡线)
 */
static void handle_idle_state(uint16_t red_obstacle_flag, uint32_t current_time_ms)
{
    // 检测红色障碍物
    if (red_obstacle_flag > 0 && red_obstacle_flag <= g_auto_replay.config.obstacle_distance_threshold) {
        // 连续检测确认
        g_auto_replay.obstacle_confirm_counter++;
        g_auto_replay.last_obstacle_distance = red_obstacle_flag;
        
        if (g_auto_replay.obstacle_confirm_counter >= g_auto_replay.config.obstacle_confirm_count) {
            // 确认检测到障碍物，切换状态
            transition_to_state(AUTO_REPLAY_OBSTACLE_DETECTED, current_time_ms);
            
            // 记录检测位置
            InertialNav_t* nav = inertial_nav_get_state();
            g_auto_replay.obstacle_detected_x = nav->current_pose.x;
            g_auto_replay.obstacle_detected_y = nav->current_pose.y;
        }
    } else {
        // 未检测到障碍物，重置计数器
        g_auto_replay.obstacle_confirm_counter = 0;
    }
}

/**
 * @brief 处理障碍物检测状态
 */
static void handle_obstacle_detected_state(uint32_t current_time_ms, 
                                           float* pwm_L_out, 
                                           float* pwm_R_out)
{
    // 立即切换到准备阶段
    transition_to_state(AUTO_REPLAY_PREPARING, current_time_ms);
}

/**
 * @brief 处理准备复现状态 (减速/停车)
 */
static void handle_preparing_state(uint32_t current_time_ms,
                                   float* pwm_L_out,
                                   float* pwm_R_out)
{
    // 获取当前速度
    InertialNav_t* nav = inertial_nav_get_state();
    float current_speed = nav->current_pose.v;
    
    // 渐进减速
    float target_speed = g_auto_replay.config.prepare_target_speed;
    
    if (current_speed > target_speed) {
        // 简单减速控制 (可以优化为PID)
        float speed_error = target_speed - current_speed;
        float pwm_adjustment = speed_error * 50.0f;  // 简单比例控制
        
        *pwm_L_out = pwm_adjustment;
        *pwm_R_out = pwm_adjustment;
    } else {
        // 速度已达到目标，准备开始复现
        
        // 检查是否有存储的轨迹
        if (nav->trajectory.count > 0) {
            // 停止录制 (如果正在录制)
            if (nav->trajectory.is_recording) {
                inertial_nav_stop_recording();
            }
            
            // 开始复现
            inertial_nav_start_replay();
            
            transition_to_state(AUTO_REPLAY_REPLAYING, current_time_ms);
            g_auto_replay.replay_start_time = current_time_ms;
            g_auto_replay.total_replay_count++;
        } else {
            // 没有轨迹数据，直接返回巡线
            transition_to_state(AUTO_REPLAY_RETURNING, current_time_ms);
        }
    }
}

/**
 * @brief 处理正在复现状态
 */
static void handle_replaying_state(uint32_t current_time_ms,
                                   float* pwm_L_out,
                                   float* pwm_R_out)
{
    InertialNav_t* nav = inertial_nav_get_state();
    
    // 检查是否还在复现
    if (nav->trajectory.is_replaying) {
        // 调用惯导跟踪更新 (内部会计算PWM输出)
        // 注意: 这里假设惯导系统会更新控制输出，实际使用时需要调整
        
        // 复现过程中，PWM由惯导系统控制
        // 这里只是监控状态
        
        // 可选: 添加超时保护
        uint32_t elapsed = time_elapsed(g_auto_replay.replay_start_time, current_time_ms);
        uint32_t max_replay_time = 120000;  // 最大复现时间120秒
        
        if (elapsed > max_replay_time) {
            // 超时，强制停止
            inertial_nav_stop_replay();
            transition_to_state(AUTO_REPLAY_COMPLETED, current_time_ms);
        }
    } else {
        // 复现已完成
        g_auto_replay.replay_end_time = current_time_ms;
        g_auto_replay.last_replay_duration = 
            (float)time_elapsed(g_auto_replay.replay_start_time, current_time_ms) / 1000.0f;
        g_auto_replay.success_replay_count++;
        
        transition_to_state(AUTO_REPLAY_COMPLETED, current_time_ms);
    }
}

/**
 * @brief 处理复现完成状态
 */
static void handle_completed_state(uint32_t current_time_ms)
{
    // 检查是否需要自动返回巡线
    if (g_auto_replay.config.enable_auto_return) {
        uint32_t elapsed = time_elapsed(g_auto_replay.state_start_time, current_time_ms);
        uint32_t delay_ms = (uint32_t)(g_auto_replay.config.return_delay_time * 1000.0f);
        
        if (elapsed >= delay_ms) {
            transition_to_state(AUTO_REPLAY_RETURNING, current_time_ms);
        }
    }
    // 如果不自动返回，等待手动控制
}

/**
 * @brief 处理返回巡线状态
 */
static void handle_returning_state(uint32_t current_time_ms)
{
    // 重置所有状态，返回空闲
    g_auto_replay.obstacle_confirm_counter = 0;
    g_auto_replay.last_obstacle_distance = 0;
    
    transition_to_state(AUTO_REPLAY_IDLE, current_time_ms);
}

/**
 * @brief 主更新函数 - 状态机调度
 */
AutoReplayState_t obstacle_auto_replay_update(uint16_t red_obstacle_flag,
                                               uint32_t current_time_ms,
                                               float* pwm_L_out,
                                               float* pwm_R_out)
{
    if (!pwm_L_out || !pwm_R_out) {
        return g_auto_replay.state;
    }
    
    // 根据当前状态执行对应逻辑
    switch (g_auto_replay.state) {
        case AUTO_REPLAY_IDLE:
            handle_idle_state(red_obstacle_flag, current_time_ms);
            break;
            
        case AUTO_REPLAY_OBSTACLE_DETECTED:
            handle_obstacle_detected_state(current_time_ms, pwm_L_out, pwm_R_out);
            break;
            
        case AUTO_REPLAY_PREPARING:
            handle_preparing_state(current_time_ms, pwm_L_out, pwm_R_out);
            break;
            
        case AUTO_REPLAY_REPLAYING:
            handle_replaying_state(current_time_ms, pwm_L_out, pwm_R_out);
            break;
            
        case AUTO_REPLAY_COMPLETED:
            handle_completed_state(current_time_ms);
            break;
            
        case AUTO_REPLAY_RETURNING:
            handle_returning_state(current_time_ms);
            break;
            
        default:
            // 异常状态，重置
            transition_to_state(AUTO_REPLAY_IDLE, current_time_ms);
            break;
    }
    
    return g_auto_replay.state;
}

/**
 * @brief 强制停止复现
 */
void obstacle_auto_replay_abort(void)
{
    // 停止惯导复现
    InertialNav_t* nav = inertial_nav_get_state();
    if (nav->trajectory.is_replaying) {
        inertial_nav_stop_replay();
    }
    
    // 重置状态
    g_auto_replay.state = AUTO_REPLAY_IDLE;
    g_auto_replay.obstacle_confirm_counter = 0;
}

/**
 * @brief 重置模块
 */
void obstacle_auto_replay_reset(void)
{
    AutoReplayConfig_t saved_config = g_auto_replay.config;
    memset(&g_auto_replay, 0, sizeof(AutoReplayController_t));
    g_auto_replay.config = saved_config;  // 保留配置
    g_auto_replay.state = AUTO_REPLAY_IDLE;
}

/**
 * @brief 设置准备阶段参数
 */
void obstacle_auto_replay_set_prepare_params(float decel_distance, float target_speed)
{
    g_auto_replay.config.prepare_decel_distance = decel_distance;
    g_auto_replay.config.prepare_target_speed = clamp_f(target_speed, 0.0f, 1.0f);
}

/**
 * @brief 设置返回延迟
 */
void obstacle_auto_replay_set_return_delay(float delay_time)
{
    g_auto_replay.config.return_delay_time = clamp_f(delay_time, 0.0f, 10.0f);
}

/**
 * @brief 获取状态名称
 */
const char* obstacle_auto_replay_get_state_name(AutoReplayState_t state)
{
    static const char* state_names[] = {
        "IDLE",
        "OBSTACLE_DETECTED",
        "PREPARING",
        "REPLAYING",
        "COMPLETED",
        "RETURNING"
    };
    
    if (state < sizeof(state_names) / sizeof(state_names[0])) {
        return state_names[state];
    }
    return "UNKNOWN";
}

/**
 * @brief 获取统计信息
 */
void obstacle_auto_replay_get_statistics(uint32_t* total_count,
                                         uint32_t* success_count,
                                         float* last_duration)
{
    if (total_count) *total_count = g_auto_replay.total_replay_count;
    if (success_count) *success_count = g_auto_replay.success_replay_count;
    if (last_duration) *last_duration = g_auto_replay.last_replay_duration;
}

/**
 * @brief 检查是否正在复现
 */
uint8_t obstacle_auto_replay_is_active(void)
{
    return (g_auto_replay.state == AUTO_REPLAY_REPLAYING) ? 1 : 0;
}

/**
 * @brief 检查是否可以使用巡线控制
 */
uint8_t obstacle_auto_replay_can_use_line_control(void)
{
    // 只有在IDLE和RETURNING状态下才能使用巡线控制
    return (g_auto_replay.state == AUTO_REPLAY_IDLE || 
            g_auto_replay.state == AUTO_REPLAY_RETURNING) ? 1 : 0;
}

// ==================== 调试辅助函数 ====================

/**
 * @brief 打印当前状态信息 (调试用)
 * @note 需要包含stdio.h，实际使用时可注释掉
 */
#ifdef OBSTACLE_AUTO_REPLAY_DEBUG
#include <stdio.h>

void obstacle_auto_replay_print_status(void)
{
    printf("=== Auto Replay Status ===\n");
    printf("State: %s\n", obstacle_auto_replay_get_state_name(g_auto_replay.state));
    printf("Obstacle counter: %u/%u\n", 
           g_auto_replay.obstacle_confirm_counter,
           g_auto_replay.config.obstacle_confirm_count);
    printf("Last obstacle distance: %u\n", g_auto_replay.last_obstacle_distance);
    printf("Total replays: %lu\n", g_auto_replay.total_replay_count);
    printf("Success replays: %lu\n", g_auto_replay.success_replay_count);
    printf("Last duration: %.2f s\n", g_auto_replay.last_replay_duration);
    printf("==========================\n");
}
#endif
