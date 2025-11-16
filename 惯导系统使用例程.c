/**
 * @file 惯导系统使用例程.c
 * @brief STM32H750 惯性导航系统完整使用示例
 * @note 包含初始化、轨迹记录、轨迹复现的完整流程
 */

#include "main.h"
#include "inertial_nav.h"
#include "control.h"
#include "control_pid.h"
#include "ICM-42688P.h"
#include "motor.h"
#include "lcd_spi_200.h"

// ==================== 全局变量 ====================

// 系统状态机
typedef enum {
    STATE_IDLE = 0,         // 空闲状态
    STATE_RECORDING,        // 录制轨迹
    STATE_REPLAYING,        // 复现轨迹
    STATE_MANUAL_CONTROL    // 手动控制
} SystemState_t;

SystemState_t g_system_state = STATE_IDLE;

// 按键标志（根据你的硬件调整）
volatile uint8_t g_button_start_recording = 0;  // 开始录制按键
volatile uint8_t g_button_stop_recording = 0;   // 停止录制按键
volatile uint8_t g_button_start_replay = 0;     // 开始复现按键
volatile uint8_t g_button_stop_replay = 0;      // 停止复现按键

// ==================== 初始化函数 ====================

/**
 * @brief 惯导系统初始化
 * @note 在main函数中调用一次
 */
void inertial_nav_system_init(void)
{
    // 1. 初始化惯性导航系统
    inertial_nav_init(
        0.16f,      // 轮距 16cm
        0.034f,     // 轮半径 3.4cm
        0.7f        // 陀螺仪信任度 70%
    );
    
    // 2. 设置PID参数（可选，使用默认值也行）
    inertial_nav_set_position_pid(1.0f, 0.05f, 0.1f);    // 位置环
    inertial_nav_set_heading_pid(2.0f, 0.1f, 0.2f);      // 航向环
    inertial_nav_set_velocity_feedforward(0.9f);          // 90%前馈
    
    // 3. 初始化串级PID控制器（用于速度控制）
    cascade_pid_init(
        0.008f,             // 半轮距 8mm
        5.5f, 0.0f, 0.0f,   // 图像环PID (如果不用图像可以都设为0)
        50.0f, 0.0f, 0.0f   // 速度环PID
    );
    
    printf("惯导系统初始化完成!\r\n");
}

// ==================== 定时器中断 - 50Hz更新 ====================

/**
 * @brief 定时器中断回调 (20ms周期, 50Hz)
 * @note 在TIM中断处理函数中调用
 */
void TIM_NavUpdate_IRQHandler(void)
{
    // 1. 读取IMU数据
    ICM42688P_Read();  // 读取陀螺仪数据
    float gyro_z_deg = icm42688p.gyro_z;  // 角速度(°/s)
    float gyro_z_rad = gyro_z_deg * 0.0174533f;  // 转换为rad/s
    
    // 2. 读取编码器速度（假设已在其他地方更新）
    float v_left = control.left_speed;   // m/s
    float v_right = control.right_speed; // m/s
    
    // 3. 更新惯导系统（航迹推算 + 轨迹记录）
    inertial_nav_update(gyro_z_rad, v_left, v_right);
    
    // 4. 如果在复现模式，执行轨迹跟踪控制
    if (g_system_state == STATE_REPLAYING) {
        float v_cmd, omega_cmd;
        
        // 获取跟踪控制器输出的速度指令
        inertial_nav_tracking_control(&v_cmd, &omega_cmd);
        
        // 转换为左右轮速度
        float wheel_base = 0.16f;
        float vL_target = v_cmd - omega_cmd * wheel_base / 2.0f;
        float vR_target = v_cmd + omega_cmd * wheel_base / 2.0f;
        
        // 调用速度环PID
        float pwm_L, pwm_R;
        cascade_pid_inner_loop(v_left, v_right, &pwm_L, &pwm_R);
        
        // 输出到电机
        control.left_target_speed = (int16_t)((pwm_L >= 0) ? pwm_L : -pwm_L);
        control.left_dir = (pwm_L >= 0) ? 1 : 0;
        control.right_target_speed = (int16_t)((pwm_R >= 0) ? pwm_R : -pwm_R);
        control.right_dir = (pwm_R >= 0) ? 1 : 0;
        
        motor_output(control.left_target_speed, control.left_dir,
                    control.right_target_speed, control.right_dir);
    }
}

// ==================== 主循环状态机 ====================

/**
 * @brief 主循环中的状态机处理
 * @note 在main函数的while(1)中调用
 */
void inertial_nav_state_machine(void)
{
    switch (g_system_state) {
        
        // ========== 空闲状态 ==========
        case STATE_IDLE:
            // 等待按键指令
            if (g_button_start_recording) {
                g_button_start_recording = 0;
                
                // 开始录制轨迹
                inertial_nav_start_recording();
                g_system_state = STATE_RECORDING;
                
                printf("开始录制轨迹...\r\n");
                LCD_ShowString(10, 10, "Recording...", RED, WHITE, 16, 0);
            }
            
            if (g_button_start_replay) {
                g_button_start_replay = 0;
                
                // 开始复现轨迹
                uint8_t ret = inertial_nav_start_replay();
                if (ret == 0) {
                    g_system_state = STATE_REPLAYING;
                    printf("开始复现轨迹...\r\n");
                    LCD_ShowString(10, 10, "Replaying...  ", BLUE, WHITE, 16, 0);
                } else {
                    printf("无法复现: 错误码 %d\r\n", ret);
                }
            }
            break;
        
        // ========== 录制状态 ==========
        case STATE_RECORDING:
            // 显示录制进度
            static uint32_t last_display_time = 0;
            if (HAL_GetTick() - last_display_time > 500) {  // 每500ms更新一次
                last_display_time = HAL_GetTick();
                
                uint16_t count;
                uint32_t duration_ms;
                float distance;
                inertial_nav_get_trajectory_info(&count, &duration_ms, &distance);
                
                printf("录制中: %d航点, %.1fs, %.2fm\r\n", 
                       count, duration_ms/1000.0f, distance);
                
                char buf[32];
                snprintf(buf, sizeof(buf), "Points:%d  ", count);
                LCD_ShowString(10, 30, buf, RED, WHITE, 16, 0);
            }
            
            // 停止录制
            if (g_button_stop_recording) {
                g_button_stop_recording = 0;
                
                uint16_t count = inertial_nav_stop_recording();
                g_system_state = STATE_IDLE;
                
                printf("录制完成! 总共 %d 个航点\r\n", count);
                LCD_ShowString(10, 10, "Rec Done!     ", GREEN, WHITE, 16, 0);
                
                // 显示轨迹信息
                uint32_t duration_ms;
                float distance;
                inertial_nav_get_trajectory_info(&count, &duration_ms, &distance);
                printf("轨迹时长: %.1f秒\r\n", duration_ms/1000.0f);
                printf("轨迹长度: %.2f米\r\n", distance);
            }
            break;
        
        // ========== 复现状态 ==========
        case STATE_REPLAYING:
            // 显示复现进度
            static uint32_t last_replay_display = 0;
            if (HAL_GetTick() - last_replay_display > 500) {
                last_replay_display = HAL_GetTick();
                
                // 获取当前位姿和目标位姿
                const Pose_t* current = inertial_nav_get_pose();
                const Pose_t* target = inertial_nav_get_target_pose();
                
                if (target) {
                    float dx = target->x - current->x;
                    float dy = target->y - current->y;
                    float error = fast_sqrt(dx*dx + dy*dy);
                    
                    printf("复现中: 位置误差=%.3fm\r\n", error);
                    
                    char buf[32];
                    snprintf(buf, sizeof(buf), "Err:%.2fm  ", error);
                    LCD_ShowString(10, 30, buf, BLUE, WHITE, 16, 0);
                }
                
                // 获取误差统计
                float max_pos_err, avg_pos_err, max_heading_err;
                inertial_nav_get_error_stats(&max_pos_err, &avg_pos_err, &max_heading_err);
                printf("最大误差:%.3fm, 平均误差:%.3fm\r\n", max_pos_err, avg_pos_err);
            }
            
            // 停止复现
            if (g_button_stop_replay) {
                g_button_stop_replay = 0;
                
                inertial_nav_stop_replay();
                g_system_state = STATE_IDLE;
                
                printf("复现结束!\r\n");
                LCD_ShowString(10, 10, "Replay Done!  ", GREEN, WHITE, 16, 0);
                
                // 显示最终误差统计
                float max_pos_err, avg_pos_err, max_heading_err;
                inertial_nav_get_error_stats(&max_pos_err, &avg_pos_err, &max_heading_err);
                printf("=== 复现精度统计 ===\r\n");
                printf("最大位置误差: %.3f m\r\n", max_pos_err);
                printf("平均位置误差: %.3f m\r\n", avg_pos_err);
                printf("最大航向误差: %.2f °\r\n", max_heading_err * 57.2958f);
            }
            
            // 检查是否自动结束
            InertialNav_t* nav = inertial_nav_get_state();
            if (!nav->trajectory.is_replaying) {
                // 轨迹已播放完毕
                g_system_state = STATE_IDLE;
                printf("轨迹播放完成!\r\n");
            }
            break;
        
        default:
            g_system_state = STATE_IDLE;
            break;
    }
}

// ==================== 按键处理（中断或轮询） ====================

/**
 * @brief 按键扫描函数
 * @note 在main函数中定期调用，或在按键中断中设置标志
 */
void button_scan(void)
{
    // 示例：假设有4个按键
    // 根据实际硬件修改GPIO读取方式
    
    static uint8_t key1_last = 1, key2_last = 1, key3_last = 1, key4_last = 1;
    
    // 按键1: 开始录制 (下降沿触发)
    uint8_t key1 = HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin);
    if (key1_last == 1 && key1 == 0) {
        g_button_start_recording = 1;
    }
    key1_last = key1;
    
    // 按键2: 停止录制
    uint8_t key2 = HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin);
    if (key2_last == 1 && key2 == 0) {
        g_button_stop_recording = 1;
    }
    key2_last = key2;
    
    // 按键3: 开始复现
    uint8_t key3 = HAL_GPIO_ReadPin(KEY3_GPIO_Port, KEY3_Pin);
    if (key3_last == 1 && key3 == 0) {
        g_button_start_replay = 1;
    }
    key3_last = key3;
    
    // 按键4: 停止复现
    uint8_t key4 = HAL_GPIO_ReadPin(KEY4_GPIO_Port, KEY4_Pin);
    if (key4_last == 1 && key4 == 0) {
        g_button_stop_replay = 1;
    }
    key4_last = key4;
}

// ==================== 调试函数 ====================

/**
 * @brief 串口打印当前位姿
 */
void print_current_pose(void)
{
    const Pose_t* pose = inertial_nav_get_pose();
    printf("当前位姿: x=%.3f m, y=%.3f m, θ=%.1f°, v=%.2f m/s\r\n",
           pose->x, pose->y, pose->theta * 57.2958f, pose->v);
}

/**
 * @brief 轨迹数据导出（通过串口）
 */
void export_trajectory_callback(const Waypoint_t* wp, uint16_t index)
{
    // 解压缩航点数据
    float x = (float)wp->x / 1000.0f;
    float y = (float)wp->y / 1000.0f;
    float theta = (float)wp->theta / 100.0f;
    float v = (float)wp->v / 1000.0f;
    
    // 输出CSV格式，方便导入Excel/Matlab分析
    printf("%d,%.3f,%.3f,%.2f,%.3f\r\n", index, x, y, theta, v);
}

void export_trajectory(void)
{
    printf("=== 轨迹数据导出 ===\r\n");
    printf("Index,X(m),Y(m),Theta(deg),V(m/s)\r\n");
    inertial_nav_export_trajectory(export_trajectory_callback);
    printf("=== 导出完成 ===\r\n");
}

// ==================== Main函数集成示例 ====================

/*
int main(void)
{
    // 硬件初始化
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM2_Init();  // 编码器1
    MX_TIM5_Init();  // 编码器2
    MX_USART1_UART_Init();  // 串口调试
    
    // 外设初始化
    LCD_Init();
    ICM42688P_Init();
    motor_init();
    
    // 启动编码器
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
    
    // 惯导系统初始化
    inertial_nav_system_init();
    
    // 启动定时器中断 (20ms周期, 50Hz)
    HAL_TIM_Base_Start_IT(&htim6);
    
    printf("系统启动完成!\r\n");
    printf("按键1: 开始录制\r\n");
    printf("按键2: 停止录制\r\n");
    printf("按键3: 开始复现\r\n");
    printf("按键4: 停止复现\r\n");
    
    while (1)
    {
        // 1. 按键扫描
        button_scan();
        
        // 2. 状态机处理
        inertial_nav_state_machine();
        
        // 3. 其他任务
        // ...
        
        HAL_Delay(10);  // 主循环10ms
    }
}
*/

// ==================== 定时器中断配置 ====================

/*
// 在stm32h7xx_it.c中添加:

void TIM6_DAC_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim6, TIM_FLAG_UPDATE)) {
        __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);
        
        // 调用惯导更新函数
        TIM_NavUpdate_IRQHandler();
    }
}

// 在main.c中配置定时器:
void MX_TIM6_Init(void)
{
    // 配置为20ms周期 (50Hz)
    // 假设APB1时钟 = 120MHz
    // 预分频 = 12000-1 (120MHz/12000 = 10kHz)
    // 计数周期 = 200-1 (10kHz/200 = 50Hz)
    
    htim6.Instance = TIM6;
    htim6.Init.Prescaler = 12000 - 1;
    htim6.Init.Period = 200 - 1;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&htim6);
    
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
}
*/

// ==================== 高级应用示例 ====================

/**
 * @brief 示例1: 录制矩形轨迹
 */
void example_record_rectangle(void)
{
    printf("开始录制矩形轨迹...\r\n");
    
    inertial_nav_start_recording();
    
    // 前进 1米
    motor_set_speed(0.3f, 0.3f);
    HAL_Delay(3333);  // 1m / 0.3m/s = 3.33s
    
    // 左转90度
    motor_set_speed(0.2f, -0.2f);
    HAL_Delay(1000);
    
    // 前进 0.5米
    motor_set_speed(0.3f, 0.3f);
    HAL_Delay(1667);
    
    // 左转90度
    motor_set_speed(0.2f, -0.2f);
    HAL_Delay(1000);
    
    // 前进 1米
    motor_set_speed(0.3f, 0.3f);
    HAL_Delay(3333);
    
    // 左转90度
    motor_set_speed(0.2f, -0.2f);
    HAL_Delay(1000);
    
    // 前进 0.5米 (回到起点)
    motor_set_speed(0.3f, 0.3f);
    HAL_Delay(1667);
    
    motor_stop();
    
    uint16_t count = inertial_nav_stop_recording();
    printf("录制完成! 共 %d 个航点\r\n", count);
}

/**
 * @brief 示例2: 参数调优
 */
void example_pid_tuning(void)
{
    // 如果复现精度不够，可以调整PID参数
    
    // 如果横向偏差大 -> 增大位置环Kp
    inertial_nav_set_position_pid(2.0f, 0.05f, 0.1f);
    
    // 如果航向偏差大 -> 增大航向环Kp
    inertial_nav_set_heading_pid(3.0f, 0.1f, 0.2f);
    
    // 如果速度跟踪不好 -> 减小前馈系数
    inertial_nav_set_velocity_feedforward(0.7f);
    
    // 如果陀螺仪漂移严重 -> 降低陀螺仪信任度
    inertial_nav_set_gyro_trust(0.5f);
}

/**
 * @brief 示例3: 实时监控位姿
 */
void example_real_time_monitor(void)
{
    while (1) {
        const Pose_t* pose = inertial_nav_get_pose();
        
        // LCD显示
        char buf[32];
        snprintf(buf, sizeof(buf), "X:%.2fm  ", pose->x);
        LCD_ShowString(10, 50, buf, BLACK, WHITE, 16, 0);
        
        snprintf(buf, sizeof(buf), "Y:%.2fm  ", pose->y);
        LCD_ShowString(10, 70, buf, BLACK, WHITE, 16, 0);
        
        snprintf(buf, sizeof(buf), "θ:%.1f°  ", pose->theta * 57.2958f);
        LCD_ShowString(10, 90, buf, BLACK, WHITE, 16, 0);
        
        HAL_Delay(100);
    }
}
