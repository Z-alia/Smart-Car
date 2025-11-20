/**
 * @file Flash轨迹存储使用示例.c
 * @brief 惯导轨迹Flash存储的完整使用指南
 * @date 2025-11-20
 * @note 本文件展示如何使用Flash存储模块实现掉电保持的轨迹数据管理
 */

/*
================================================================================
                            功能概述
================================================================================

核心功能:
  1. 轨迹数据持久化存储 (掉电后数据仍然存在)
  2. 支持最多8条轨迹
  3. 自动CRC32校验保证数据完整性
  4. 索引管理方便查询和操作

硬件需求:
  - STM32H750VBT6 (带QSPI接口)
  - W25Q64/W25Q128 外部Flash芯片
  - QSPI连接正常

存储空间:
  - 每条轨迹: 24KB (3000航点 × 8字节)
  - 总占用: 192KB (8条轨迹)
  - Flash容量: 8MB (W25Q64)
  - 剩余空间: 7.8MB可用于其他数据

================================================================================
                            使用流程
================================================================================
*/

#include "trajectory_flash_storage.h"
#include "inertial_nav.h"
#include "obstacle_auto_replay.h"
#include "main.h"

// ==================== 示例1: 基础初始化 ====================

void example1_basic_initialization(void)
{
    /*
    系统启动时初始化Flash存储
    */
    
    // 步骤1: 初始化QSPI硬件 (在main.c中调用)
    MX_QUADSPI_Init();
    
    // 步骤2: 初始化惯导系统
    inertial_nav_init(0.16f, 0.033f, 0.7f);
    
    // 步骤3: 初始化Flash存储模块
    FlashStorageStatus_t status = trajectory_flash_init();
    
    if (status == FLASH_STORAGE_OK) {
        printf("Flash storage initialized successfully.\n");
        
        // 查看已有轨迹
        TrajectoryMetadata_t* list;
        uint16_t count;
        trajectory_flash_get_list(&list, &count);
        
        printf("Found %d trajectories in Flash:\n", count);
        for (uint16_t i = 0; i < count; i++) {
            printf("  [%d] %s - %d waypoints, %.2f m\n",
                   list[i].trajectory_id,
                   list[i].name,
                   list[i].waypoint_count,
                   list[i].total_distance);
        }
    } else {
        printf("Flash init failed: %s\n", 
               trajectory_flash_get_error_string(status));
    }
}

// ==================== 示例2: 录制并保存轨迹 ====================

void example2_record_and_save_trajectory(void)
{
    /*
    完整的轨迹录制和保存流程
    */
    
    // 步骤1: 开始录制
    printf("Start recording trajectory...\n");
    inertial_nav_start_recording();
    
    // 步骤2: 录制过程 (手动驾驶或自动巡线)
    // 在主循环中持续调用:
    while (is_recording) {
        // 更新IMU和编码器数据
        float gyro_z = read_imu_gyro_z();
        get_speed();
        
        // 惯导更新 (自动记录航点)
        inertial_nav_update(
            gyro_z,
            control.left_speed,
            control.right_speed
        );
        
        HAL_Delay(20);  // 50Hz采样
    }
    
    // 步骤3: 停止录制
    uint16_t waypoint_count = inertial_nav_stop_recording();
    printf("Recording stopped. Total waypoints: %d\n", waypoint_count);
    
    // 步骤4: 保存到Flash (槽位0)
    FlashStorageStatus_t status = trajectory_flash_save(
        0,                          // 轨迹ID
        "Obstacle Avoidance",       // 名称
        "Recorded on 2025-11-20"    // 描述
    );
    
    if (status == FLASH_STORAGE_OK) {
        printf("Trajectory saved successfully to Flash slot 0.\n");
        printf("Data is now persistent (survives power loss).\n");
    } else {
        printf("Save failed: %s\n", trajectory_flash_get_error_string(status));
    }
}

// ==================== 示例3: 掉电重启后加载轨迹 ====================

void example3_load_trajectory_after_power_cycle(void)
{
    /*
    演示掉电保持功能：
      1. 录制并保存轨迹
      2. 断电
      3. 重新上电
      4. 加载轨迹
      5. 复现轨迹
    */
    
    // === 系统重启后 ===
    
    // 初始化Flash存储
    trajectory_flash_init();
    
    // 检查是否有保存的轨迹
    if (trajectory_flash_exists(0)) {
        printf("Found saved trajectory 0, loading...\n");
        
        // 加载轨迹
        FlashStorageStatus_t status = trajectory_flash_load(0);
        
        if (status == FLASH_STORAGE_OK) {
            printf("Trajectory loaded successfully.\n");
            
            // 获取轨迹信息
            uint16_t count;
            uint32_t duration;
            float distance;
            inertial_nav_get_trajectory_info(&count, &duration, &distance);
            
            printf("Loaded trajectory info:\n");
            printf("  Waypoints: %d\n", count);
            printf("  Duration: %.1f s\n", duration / 1000.0f);
            printf("  Distance: %.2f m\n", distance);
            
            // 开始复现
            inertial_nav_start_replay();
            printf("Replay started!\n");
            
        } else {
            printf("Load failed: %s\n", 
                   trajectory_flash_get_error_string(status));
        }
    } else {
        printf("No trajectory found in slot 0.\n");
    }
}

// ==================== 示例4: 多条轨迹管理 ====================

void example4_multiple_trajectories(void)
{
    /*
    管理多条轨迹
    */
    
    // 场景1: 保存不同场景的轨迹
    
    // 录制轨迹0: 避障路线
    record_obstacle_avoidance_route();
    trajectory_flash_save(0, "Obstacle Route", "Main obstacle avoidance path");
    
    // 录制轨迹1: 快速冲刺
    record_sprint_route();
    trajectory_flash_save(1, "Sprint Route", "Fast straight line");
    
    // 录制轨迹2: 复杂弯道
    record_curve_route();
    trajectory_flash_save(2, "Curve Route", "S-curve navigation");
    
    // 场景2: 查看所有轨迹
    void list_all_trajectories(void)
    {
        TrajectoryMetadata_t* list;
        uint16_t count;
        
        trajectory_flash_get_list(&list, &count);
        
        printf("\n=== Saved Trajectories ===\n");
        for (uint16_t i = 0; i < count; i++) {
            printf("[%d] %s\n", list[i].trajectory_id, list[i].name);
            printf("    Description: %s\n", list[i].description);
            printf("    Waypoints: %d\n", list[i].waypoint_count);
            printf("    Distance: %.2f m\n", list[i].total_distance);
            printf("    Duration: %.1f s\n", list[i].duration_ms / 1000.0f);
            printf("    Start: (%.2f, %.2f, %.1f°)\n",
                   list[i].start_x, list[i].start_y, 
                   list[i].start_theta * 57.3f);
            printf("    End: (%.2f, %.2f, %.1f°)\n",
                   list[i].end_x, list[i].end_y,
                   list[i].end_theta * 57.3f);
            printf("\n");
        }
    }
    
    // 场景3: 根据场景选择轨迹
    void select_trajectory_by_scenario(void)
    {
        if (detected_obstacle) {
            // 检测到障碍物，加载避障轨迹
            trajectory_flash_load(0);
        } else if (need_sprint) {
            // 需要冲刺，加载快速轨迹
            trajectory_flash_load(1);
        } else if (complex_curve_detected) {
            // 检测到复杂弯道
            trajectory_flash_load(2);
        }
        
        inertial_nav_start_replay();
    }
}

// ==================== 示例5: 与自动复现模块集成 ====================

void example5_integration_with_auto_replay(void)
{
    /*
    完整集成：Flash存储 + 自动复现
    
    工作流程:
      1. 提前录制绕障轨迹并保存到Flash
      2. 比赛时加载轨迹到内存
      3. 检测到红色障碍物自动触发复现
      4. 复现完成返回巡线
    */
    
    // === 准备阶段 (比赛前) ===
    
    void preparation_phase(void)
    {
        printf("=== Preparation Phase ===\n");
        
        // 初始化系统
        MX_QUADSPI_Init();
        inertial_nav_init(0.16f, 0.033f, 0.7f);
        trajectory_flash_init();
        
        // 录制绕障轨迹
        printf("Recording obstacle avoidance trajectory...\n");
        inertial_nav_start_recording();
        
        // 手动驾驶完成绕障
        manual_drive_obstacle_avoidance();
        
        inertial_nav_stop_recording();
        
        // 保存到Flash槽位0
        FlashStorageStatus_t status = trajectory_flash_save(
            0,
            "Competition Obstacle Route",
            "Official obstacle avoidance path"
        );
        
        if (status == FLASH_STORAGE_OK) {
            printf("✓ Trajectory saved to Flash.\n");
            printf("✓ Ready for competition!\n");
        }
    }
    
    // === 比赛阶段 (上电后) ===
    
    void competition_phase(void)
    {
        printf("=== Competition Phase ===\n");
        
        // 初始化系统
        MX_QUADSPI_Init();
        inertial_nav_init(0.16f, 0.033f, 0.7f);
        trajectory_flash_init();
        
        // 从Flash加载预先保存的轨迹
        printf("Loading trajectory from Flash...\n");
        if (trajectory_flash_load(0) == FLASH_STORAGE_OK) {
            printf("✓ Trajectory loaded successfully.\n");
        } else {
            printf("✗ Failed to load trajectory!\n");
            return;
        }
        
        // 初始化自动复现模块
        obstacle_auto_replay_init(100, 3, 1);
        
        // 主循环
        while (1) {
            // 更新传感器
            process_image();
            get_speed();
            
            // 自动复现状态机
            float pwm_L, pwm_R;
            AutoReplayState_t state = obstacle_auto_replay_update(
                watch.Red_obstacle_flag,
                HAL_GetTick(),
                &pwm_L,
                &pwm_R
            );
            
            // 根据状态选择控制
            if (obstacle_auto_replay_can_use_line_control()) {
                // 正常巡线
                cascade_pid_control(0.3f, 
                                   control.left_speed,
                                   control.right_speed,
                                   &pwm_L, &pwm_R);
            } else {
                // 自动复现 (使用Flash中加载的轨迹)
                inertial_nav_tracking_update(&pwm_L, &pwm_R);
            }
            
            motor_set_pwm(pwm_L, pwm_R);
            
            HAL_Delay(10);
        }
    }
}

// ==================== 示例6: 数据导出和备份 ====================

void example6_export_and_backup(void)
{
    /*
    数据导出和备份功能
    */
    
    // 场景1: 导出轨迹数据到串口 (用于分析)
    void export_trajectory_to_uart(void)
    {
        printf("Exporting trajectory 0 to UART...\n");
        printf("Index,X(m),Y(m),Theta(deg),V(m/s)\n");
        
        void export_callback(const Waypoint_t* wp, uint16_t index) {
            printf("%d,%.3f,%.3f,%.3f,%.3f\n",
                   index,
                   wp->x / 1000.0f,
                   wp->y / 1000.0f,
                   wp->theta / 100.0f,
                   wp->v / 1000.0f);
        }
        
        trajectory_flash_export(0, export_callback);
    }
    
    // 场景2: 备份重要轨迹
    void backup_important_trajectory(void)
    {
        // 将轨迹0备份到槽位7
        FlashStorageStatus_t status = trajectory_flash_backup(0, 7);
        
        if (status == FLASH_STORAGE_OK) {
            printf("Trajectory 0 backed up to slot 7.\n");
        }
    }
    
    // 场景3: 验证数据完整性
    void verify_all_trajectories(void)
    {
        printf("Verifying all trajectories...\n");
        
        FlashStorageStatus_t status = trajectory_flash_verify(0xFF);  // 0xFF = all
        
        if (status == FLASH_STORAGE_OK) {
            printf("✓ All trajectories verified OK.\n");
        } else if (status == FLASH_STORAGE_CRC_ERROR) {
            printf("✗ CRC error detected!\n");
        }
    }
}

// ==================== 示例7: 错误处理 ====================

void example7_error_handling(void)
{
    /*
    完善的错误处理
    */
    
    // 保存轨迹时的错误处理
    void save_with_error_handling(void)
    {
        FlashStorageStatus_t status = trajectory_flash_save(0, "Test", NULL);
        
        switch (status) {
            case FLASH_STORAGE_OK:
                printf("✓ Save successful.\n");
                break;
                
            case FLASH_STORAGE_INIT_FAILED:
                printf("✗ Flash not initialized. Call trajectory_flash_init() first.\n");
                break;
                
            case FLASH_STORAGE_WRITE_FAILED:
                printf("✗ Write failed. Check QSPI connection.\n");
                break;
                
            case FLASH_STORAGE_ERASE_FAILED:
                printf("✗ Erase failed. Flash may be damaged.\n");
                break;
                
            case FLASH_STORAGE_FULL:
                printf("✗ Storage full. Delete old trajectories.\n");
                break;
                
            case FLASH_STORAGE_INVALID_PARAM:
                printf("✗ Invalid parameter. Check trajectory ID and data.\n");
                break;
                
            default:
                printf("✗ Unknown error: %s\n", 
                       trajectory_flash_get_error_string(status));
                break;
        }
    }
    
    // 加载轨迹时的错误处理
    void load_with_error_handling(void)
    {
        // 先检查轨迹是否存在
        if (!trajectory_flash_exists(0)) {
            printf("Trajectory 0 not found.\n");
            return;
        }
        
        // 加载轨迹
        FlashStorageStatus_t status = trajectory_flash_load(0);
        
        if (status == FLASH_STORAGE_CRC_ERROR) {
            printf("✗ CRC error! Data corrupted.\n");
            printf("Attempting to load backup...\n");
            
            // 尝试加载备份
            status = trajectory_flash_load(7);
            if (status == FLASH_STORAGE_OK) {
                printf("✓ Loaded from backup slot 7.\n");
            }
        }
    }
}

// ==================== 示例8: 存储空间管理 ====================

void example8_storage_management(void)
{
    /*
    存储空间管理和优化
    */
    
    // 查看存储使用情况
    void check_storage_usage(void)
    {
        uint16_t used, total;
        uint32_t used_bytes, total_bytes;
        
        trajectory_flash_get_usage(&used, &total, &used_bytes, &total_bytes);
        
        printf("=== Flash Storage Usage ===\n");
        printf("Trajectories: %d / %d (%.1f%%)\n",
               used, total, (float)used / total * 100.0f);
        printf("Space: %lu / %lu bytes (%.1f%%)\n",
               used_bytes, total_bytes,
               (float)used_bytes / total_bytes * 100.0f);
    }
    
    // 清理旧轨迹
    void cleanup_old_trajectories(void)
    {
        // 删除不再需要的轨迹
        trajectory_flash_delete(5);
        trajectory_flash_delete(6);
        
        printf("Old trajectories deleted.\n");
    }
    
    // 格式化Flash (紧急情况)
    void emergency_format(void)
    {
        printf("WARNING: This will erase all trajectories!\n");
        printf("Press any key to confirm...\n");
        getchar();
        
        FlashStorageStatus_t status = trajectory_flash_format();
        
        if (status == FLASH_STORAGE_OK) {
            printf("Flash formatted successfully.\n");
        }
    }
}

/*
================================================================================
                            最佳实践
================================================================================

1. 初始化顺序:
   ✓ MX_QUADSPI_Init() 必须最先调用
   ✓ trajectory_flash_init() 在使用存储前调用
   ✓ inertial_nav_init() 在录制/复现前调用

2. 保存轨迹:
   ✓ 录制完成后立即保存
   ✓ 使用有意义的名称和描述
   ✓ 重要轨迹做好备份

3. 加载轨迹:
   ✓ 系统启动时提前加载
   ✓ 加载后验证数据完整性
   ✓ 准备备份方案

4. 错误处理:
   ✓ 检查每次操作的返回值
   ✓ CRC错误时尝试加载备份
   ✓ 记录错误日志

5. 性能优化:
   ✓ Flash操作耗时，不要在实时控制中调用
   ✓ 批量操作合并执行
   ✓ 使用缓存减少读取次数

================================================================================
                            常见问题
================================================================================

Q1: Flash初始化失败？
A1: 检查:
    - QSPI硬件连接
    - MX_QUADSPI_Init() 是否已调用
    - Flash芯片型号是否匹配

Q2: 保存后再次上电无法加载？
A2: 检查:
    - Flash供电是否正常
    - 数据是否正确保存 (验证CRC)
    - 索引表是否损坏

Q3: CRC校验失败？
A3: 可能原因:
    - Flash芯片损坏
    - 数据写入不完整
    - 电压波动导致数据翻转
    解决: 使用备份槽位

Q4: Flash空间不足？
A4: 解决方案:
    - 删除不需要的轨迹
    - 使用更大容量Flash (W25Q128)
    - 优化航点采样率

Q5: 读写速度慢？
A5: 优化方法:
    - 使用Quad SPI模式 (四线)
    - 提高QSPI时钟频率
    - 批量读写减少命令开销

================================================================================
*/
