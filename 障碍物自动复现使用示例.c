/**
 * @file 障碍物自动复现使用示例.c
 * @brief 红色障碍物触发惯导自动复现的完整使用指南
 * @date 2025-11-20
 * @note 本文件展示如何集成obstacle_auto_replay模块到智能车主程序
 */

/*
================================================================================
                            功能说明
================================================================================

核心功能:
  当摄像头检测到红色障碍物时，自动切换到惯导复现模式，播放之前录制的轨迹

工作流程:
  1. 正常巡线阶段 → 摄像头实时监测红色障碍物
  2. 检测到Red_obstacle_flag ≠ 0 → 连续确认3次
  3. 确认障碍物 → 减速准备 (0.5m减速到0.1m/s)
  4. 开始复现 → 播放存储的惯导轨迹
  5. 复现完成 → 延迟1秒后返回正常巡线

适用场景:
  - 比赛中遇到障碍物需要绕行
  - 预先录制绕障轨迹，检测到障碍物时自动复现
  - 无需人工干预，全自动避障

================================================================================
                            系统架构
================================================================================

模块组成:
  
  1. Element_recognition (摄像头模块)
     - 提供: watch.Red_obstacle_flag (红色障碍物距离)
     - 作用: 实时检测障碍物
  
  2. inertial_nav (惯性导航模块)
     - 提供: 轨迹记录/复现功能
     - 作用: 存储和播放轨迹数据
  
  3. obstacle_auto_replay (本模块)
     - 作用: 连接摄像头和惯导，实现自动切换
     - 状态机: IDLE → DETECTED → PREPARING → REPLAYING → COMPLETED → RETURNING

数据流:
  
  摄像头 → Red_obstacle_flag → obstacle_auto_replay → 惯导复现 → 电机PWM
    ↓                              ↓                      ↓
  障碍物检测                    状态机调度              轨迹跟踪控制

================================================================================
                            使用示例
================================================================================
*/

#include "obstacle_auto_replay.h"
#include "inertial_nav.h"
#include "Element_recognition.h"  // watch.Red_obstacle_flag
#include "control_pid.h"
#include "motor.h"
#include "main.h"

// ==================== 示例1: 基础初始化和集成 ====================

void example1_basic_setup(void)
{
    /*
    完整初始化流程:
    1. 初始化惯导系统
    2. 录制一段绕障轨迹 (提前录制)
    3. 初始化自动复现模块
    4. 在主循环中周期性调用
    */
    
    // 步骤1: 初始化惯导系统
    inertial_nav_init(
        0.16f,      // 轮距16cm
        0.033f,     // 轮半径3.3cm
        0.7f        // 陀螺仪信任度70%
    );
    
    // 步骤2: (提前录制绕障轨迹 - 手动驾驶一次)
    // 在实际比赛前，需要先录制一次绕障轨迹:
    //   a) 启动录制: inertial_nav_start_recording();
    //   b) 手动驾驶小车完成绕障动作
    //   c) 停止录制: inertial_nav_stop_recording();
    //   d) 轨迹数据已保存在 g_inertial_nav.trajectory 中
    
    // 步骤3: 初始化自动复现模块
    obstacle_auto_replay_init(
        100,        // 障碍物距离阈值 (根据Red_obstacle_flag定义)
        3,          // 连续检测3次才触发
        1           // 复现完成后自动返回巡线
    );
    
    // 步骤4: 设置可选参数
    obstacle_auto_replay_set_prepare_params(0.5f, 0.1f);  // 提前0.5m减速到0.1m/s
    obstacle_auto_replay_set_return_delay(1.0f);          // 延迟1秒返回巡线
}

void example1_main_loop(void)
{
    /*
    主循环中的调用方式
    */
    
    while (1) {
        // 获取当前系统时间
        uint32_t current_time = HAL_GetTick();
        
        // 更新自动复现状态机
        float pwm_L, pwm_R;
        AutoReplayState_t replay_state = obstacle_auto_replay_update(
            watch.Red_obstacle_flag,    // 摄像头检测的障碍物距离
            current_time,               // 当前时间 (ms)
            &pwm_L,                     // 左轮PWM输出
            &pwm_R                      // 右轮PWM输出
        );
        
        // 根据状态选择控制策略
        if (obstacle_auto_replay_can_use_line_control()) {
            // 正常巡线模式 - 使用图像PID控制
            cascade_pid_outer_loop(0.3f);  // 图像环
            cascade_pid_inner_loop(
                control.left_speed,
                control.right_speed,
                &pwm_L,
                &pwm_R
            );
        } else {
            // 自动复现模式 - 使用惯导控制
            // PWM已由obstacle_auto_replay_update计算
            // 如果需要，可以在这里调用惯导更新
            inertial_nav_update(
                imu_gyro_z,         // 从IMU读取
                control.left_speed,
                control.right_speed
            );
        }
        
        // 输出PWM到电机
        motor_set_pwm(pwm_L, pwm_R);
        
        // 延迟 (根据实际控制周期调整)
        HAL_Delay(10);
    }
}

// ==================== 示例2: 完整的状态机集成 ====================

void example2_state_machine_integration(void)
{
    /*
    更完善的集成方式，带状态指示和错误处理
    */
    
    // 在定时器中断中调用 (建议10ms~50ms)
    void TIM_Control_IRQHandler(void)
    {
        static uint32_t last_time = 0;
        uint32_t current_time = HAL_GetTick();
        
        // 更新编码器速度
        get_speed();
        
        // 更新IMU数据
        float gyro_z = read_imu_gyro_z();
        
        // 更新惯导位姿
        inertial_nav_update(
            gyro_z,
            control.left_speed,
            control.right_speed
        );
        
        // 自动复现状态机
        float pwm_L, pwm_R;
        AutoReplayState_t state = obstacle_auto_replay_update(
            watch.Red_obstacle_flag,
            current_time,
            &pwm_L,
            &pwm_R
        );
        
        // 根据状态执行不同控制
        switch (state) {
            case AUTO_REPLAY_IDLE:
                // 正常巡线
                cascade_pid_control(0.3f, 
                                   control.left_speed,
                                   control.right_speed,
                                   &pwm_L, &pwm_R);
                break;
                
            case AUTO_REPLAY_OBSTACLE_DETECTED:
                // 检测到障碍物，准备切换
                // 可以发出蜂鸣器提示音
                // buzzer_beep(100);
                break;
                
            case AUTO_REPLAY_PREPARING:
                // 正在减速准备
                // PWM已由状态机计算
                break;
                
            case AUTO_REPLAY_REPLAYING:
                // 正在复现轨迹
                // 调用惯导跟踪控制
                inertial_nav_tracking_update(&pwm_L, &pwm_R);
                break;
                
            case AUTO_REPLAY_COMPLETED:
                // 复现完成，等待返回
                pwm_L = 0;
                pwm_R = 0;
                break;
                
            case AUTO_REPLAY_RETURNING:
                // 返回巡线模式
                break;
        }
        
        // 输出PWM
        motor_set_pwm(pwm_L, pwm_R);
        
        last_time = current_time;
    }
}

// ==================== 示例3: 录制绕障轨迹流程 ====================

void example3_record_obstacle_trajectory(void)
{
    /*
    提前录制绕障轨迹的完整流程
    
    场地准备:
      1. 放置红色障碍物 (或标记物)
      2. 规划绕障路径
      3. 确保小车能完整绕过障碍物
    
    录制步骤:
      1. 启动录制
      2. 手动/遥控驾驶小车绕过障碍物
      3. 停止录制
      4. 验证轨迹数据
    */
    
    // 步骤1: 启动录制
    void start_recording_button_callback(void)
    {
        inertial_nav_start_recording();
        printf("Recording started...\n");
    }
    
    // 步骤2: 手动驾驶 (通过遥控器或手动控制)
    void manual_control_loop(void)
    {
        while (is_recording()) {
            // 读取遥控器输入
            float forward_cmd = get_rc_forward();    // -1.0 ~ 1.0
            float turn_cmd = get_rc_turn();          // -1.0 ~ 1.0
            
            // 计算左右轮速度
            float v_L = forward_cmd + turn_cmd;
            float v_R = forward_cmd - turn_cmd;
            
            // 输出到电机
            motor_set_speed(v_L * 100, v_R * 100);
            
            // 更新惯导 (自动记录航点)
            inertial_nav_update(
                read_imu_gyro_z(),
                control.left_speed,
                control.right_speed
            );
            
            HAL_Delay(20);  // 50Hz采样
        }
    }
    
    // 步骤3: 停止录制
    void stop_recording_button_callback(void)
    {
        inertial_nav_stop_recording();
        
        // 获取录制统计
        InertialNav_t* nav = inertial_nav_get_state();
        printf("Recording stopped.\n");
        printf("Total waypoints: %u\n", nav->trajectory.count);
        printf("Duration: %.1f s\n", (float)nav->trajectory.count / NAV_SAMPLE_RATE_HZ);
    }
    
    // 步骤4: 验证轨迹 (可选)
    void verify_trajectory(void)
    {
        InertialNav_t* nav = inertial_nav_get_state();
        
        if (nav->trajectory.count < 10) {
            printf("ERROR: Trajectory too short!\n");
            return;
        }
        
        printf("Trajectory validation:\n");
        printf("  Start point: (%.2f, %.2f)\n", 
               nav->trajectory.points[0].x / 1000.0f,
               nav->trajectory.points[0].y / 1000.0f);
        printf("  End point: (%.2f, %.2f)\n",
               nav->trajectory.points[nav->trajectory.count-1].x / 1000.0f,
               nav->trajectory.points[nav->trajectory.count-1].y / 1000.0f);
        printf("  Total distance: %.2f m\n", calculate_trajectory_length());
    }
}

// ==================== 示例4: 障碍物检测逻辑 ====================

void example4_obstacle_detection_logic(void)
{
    /*
    Red_obstacle_flag的含义和使用
    
    根据Element_recognition.h:
      - Red_obstacle_flag是unsigned short类型
      - 表示红色障碍物的距离 (单位待确认，可能是像素或cm)
      - 0表示未检测到障碍物
      - >0表示检测到障碍物，数值越大距离越近 (或越远，取决于具体实现)
    
    检测逻辑:
      1. Red_obstacle_flag > 0 → 检测到障碍物
      2. Red_obstacle_flag <= threshold → 距离足够近，需要绕行
      3. 连续检测N次 → 确认是真实障碍物 (排除误检)
    */
    
    // 示例: 根据实际定义调整阈值
    void setup_obstacle_detection(void)
    {
        // 假设Red_obstacle_flag表示距离 (单位: 像素或cm)
        // 距离 < 100 时触发
        obstacle_auto_replay_init(
            100,    // 阈值
            3,      // 连续3次确认
            1       // 自动返回
        );
    }
    
    // 调试: 打印障碍物检测信息
    void debug_obstacle_detection(void)
    {
        if (watch.Red_obstacle_flag > 0) {
            printf("Red obstacle detected! Distance: %u\n", watch.Red_obstacle_flag);
            
            AutoReplayState_t state = obstacle_auto_replay_update(
                watch.Red_obstacle_flag,
                HAL_GetTick(),
                NULL, NULL
            );
            
            printf("Auto replay state: %s\n", 
                   obstacle_auto_replay_get_state_name(state));
        }
    }
}

// ==================== 示例5: 紧急停止和手动控制 ====================

void example5_emergency_control(void)
{
    /*
    紧急情况处理:
      1. 手动停止复现
      2. 重置状态机
      3. 强制返回巡线
    */
    
    // 按钮中断: 紧急停止
    void emergency_stop_button_callback(void)
    {
        // 停止复现
        obstacle_auto_replay_abort();
        
        // 停止电机
        motor_set_pwm(0, 0);
        
        printf("Emergency stop!\n");
    }
    
    // 按钮中断: 重置系统
    void reset_button_callback(void)
    {
        // 重置自动复现模块
        obstacle_auto_replay_reset();
        
        // 重置惯导系统
        inertial_nav_reset();
        
        printf("System reset.\n");
    }
    
    // 按钮中断: 强制返回巡线
    void force_return_button_callback(void)
    {
        // 检查当前状态
        if (obstacle_auto_replay_is_active()) {
            printf("Forcing return to line tracking...\n");
            obstacle_auto_replay_abort();
        }
    }
}

// ==================== 示例6: 参数调优指南 ====================

void example6_parameter_tuning(void)
{
    /*
    参数调优建议:
    
    1. obstacle_distance_threshold (障碍物距离阈值)
       - 太小: 反应太晚，可能撞上障碍物
       - 太大: 反应太早，误触发
       - 建议: 根据摄像头视野和车速调整
         - 低速 (0.3m/s): 100~150
         - 高速 (0.5m/s): 150~200
    
    2. obstacle_confirm_count (确认次数)
       - 太小: 容易误触发
       - 太大: 反应慢
       - 建议: 3~5次
    
    3. prepare_decel_distance (减速距离)
       - 太小: 减速不及时
       - 太大: 提前减速影响效率
       - 建议: 根据车速计算
         - v=0.3m/s: 0.3~0.5m
         - v=0.5m/s: 0.5~0.8m
    
    4. prepare_target_speed (准备速度)
       - 太快: 可能冲出轨迹
       - 太慢: 影响复现效果
       - 建议: 0.1~0.2 m/s
    
    5. return_delay_time (返回延迟)
       - 太短: 可能还在障碍物区域
       - 太长: 浪费时间
       - 建议: 1~2秒
    */
    
    // 低速巡线配置
    void config_for_low_speed(void)
    {
        obstacle_auto_replay_init(100, 3, 1);
        obstacle_auto_replay_set_prepare_params(0.3f, 0.1f);
        obstacle_auto_replay_set_return_delay(1.0f);
    }
    
    // 高速巡线配置
    void config_for_high_speed(void)
    {
        obstacle_auto_replay_init(200, 5, 1);
        obstacle_auto_replay_set_prepare_params(0.8f, 0.2f);
        obstacle_auto_replay_set_return_delay(1.5f);
    }
}

// ==================== 示例7: 统计信息和调试 ====================

void example7_statistics_and_debug(void)
{
    /*
    统计信息获取和调试输出
    */
    
    // 定期打印统计信息
    void print_statistics(void)
    {
        uint32_t total, success;
        float duration;
        
        obstacle_auto_replay_get_statistics(&total, &success, &duration);
        
        printf("=== Auto Replay Statistics ===\n");
        printf("Total replays: %lu\n", total);
        printf("Success replays: %lu\n", success);
        printf("Success rate: %.1f%%\n", (float)success / total * 100.0f);
        printf("Last duration: %.2f s\n", duration);
        printf("==============================\n");
    }
    
    // 实时状态监控
    void monitor_realtime_status(void)
    {
        AutoReplayController_t* ctrl = &g_auto_replay;
        
        printf("Current state: %s\n", 
               obstacle_auto_replay_get_state_name(ctrl->state));
        printf("Obstacle counter: %u/%u\n",
               ctrl->obstacle_confirm_counter,
               ctrl->config.obstacle_confirm_count);
        printf("Last obstacle distance: %u\n", ctrl->last_obstacle_distance);
        
        if (obstacle_auto_replay_is_active()) {
            InertialNav_t* nav = inertial_nav_get_state();
            printf("Replay progress: %u/%u waypoints\n",
                   nav->trajectory.current_index,
                   nav->trajectory.count);
        }
    }
}

/*
================================================================================
                            集成检查清单
================================================================================

□ 1. 硬件准备
   □ IMU (ICM42688P) 已连接并初始化
   □ 编码器已连接并测试
   □ 摄像头能检测红色障碍物
   □ 电机驱动正常

□ 2. 软件初始化
   □ 调用 inertial_nav_init()
   □ 调用 obstacle_auto_replay_init()
   □ 设置准备参数
   □ 设置返回延迟

□ 3. 录制轨迹
   □ 启动录制: inertial_nav_start_recording()
   □ 手动驾驶完成绕障
   □ 停止录制: inertial_nav_stop_recording()
   □ 验证轨迹数据

□ 4. 主循环集成
   □ 周期性调用 obstacle_auto_replay_update()
   □ 根据状态选择控制策略
   □ 正确输出PWM到电机

□ 5. 调试验证
   □ 测试障碍物检测
   □ 测试自动切换
   □ 测试复现精度
   □ 测试返回巡线

□ 6. 异常处理
   □ 实现紧急停止
   □ 实现手动重置
   □ 添加超时保护

================================================================================
                            常见问题
================================================================================

Q1: Red_obstacle_flag一直是0，无法触发复现？
A1: 检查:
    1. 摄像头是否正确识别红色障碍物
    2. watch结构体是否正确更新
    3. 障碍物是否在摄像头视野内

Q2: 触发复现后小车不动？
A2: 检查:
    1. 是否提前录制了轨迹 (trajectory.count > 0)
    2. 惯导系统是否正确初始化
    3. PWM输出是否正确传递到电机

Q3: 复现轨迹偏差很大？
A3: 优化:
    1. 调整惯导融合参数 (gyro_trust)
    2. 调整跟踪PID参数
    3. 检查编码器精度

Q4: 复现完成后无法返回巡线？
A4: 检查:
    1. enable_auto_return是否设置为1
    2. return_delay_time是否合理
    3. 巡线控制是否正常

Q5: 误触发复现怎么办？
A5: 解决:
    1. 增大obstacle_confirm_count (3 → 5)
    2. 调整obstacle_distance_threshold
    3. 优化摄像头红色检测算法

================================================================================
*/
