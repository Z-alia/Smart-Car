/**
 * @file red_obstacle_navigation.c
 * @brief 红色障碍完全出赛道避障实现（基于航位推算）
 * @note 本实现不依赖图像，完全基于编码器和陀螺仪的航位推算
 */

#include "red_obstacle.h"
#include "integral.h"
#include "motor.h"
#include <stdbool.h>

// ==================== 避障参数定义 ====================
#define RED_OBS_FORWARD_DIST     20.0f   // 第一段直行距离(cm) - 脱离检测区
#define RED_OBS_LATERAL_DIST     30.0f   // 横向移动距离H(cm) - 离开赛道
#define RED_OBS_BYPASS_DIST      40.0f   // 绕行距离L(cm) - 超过障碍物长度
#define RED_OBS_TURN_ANGLE       90.0f   // 转向角度(度)
#define RED_OBS_FORWARD_SPEED    500.0f  // 前进速度(PWM: -1000 to 1000)
#define RED_OBS_TURN_SPEED       300.0f  // 转向速度(PWM: -1000 to 1000)

// ==================== 避障状态定义 ====================
typedef enum {
    RED_NAV_IDLE = 0,      // 未进入避障
    RED_NAV_ENTER,         // 检测到障碍，准备避障
    RED_NAV_PHASE_1,       // 阶段1: 直行脱离检测区
    RED_NAV_PHASE_2_TURN,  // 阶段2-转向: 左转90°
    RED_NAV_PHASE_2_FWD,   // 阶段2-前进: 前进H(离开赛道)
    RED_NAV_PHASE_3_TURN,  // 阶段3-转向: 右转90°
    RED_NAV_PHASE_3_FWD,   // 阶段3-前进: 前进L(绕过障碍)
    RED_NAV_PHASE_4_TURN,  // 阶段4-转向: 右转90°
    RED_NAV_PHASE_4_FWD,   // 阶段4-前进: 前进H(回到赛道)
    RED_NAV_PHASE_5,       // 阶段5: 左转90°(回到平行)
    RED_NAV_OUT            // 避障完成
} RedNavigationState;

// ==================== 内部状态变量 ====================
static RedNavigationState nav_state = RED_NAV_IDLE;

// ==================== 外部电机对象 ====================
extern Motor leftmotor;
extern Motor rightmotor;

// ==================== 电机控制辅助函数 ====================
/**
 * @brief 设置左右轮速度（使用motor_run）
 * @param left_speed  左轮PWM速度(-1000 to 1000)
 * @param right_speed 右轮PWM速度(-1000 to 1000)
 */
static void set_motor_speed(int16_t left_speed, int16_t right_speed)
{
    motor_run(&leftmotor, left_speed, 0);
    motor_run(&rightmotor, right_speed, 0);
}

// ==================== 避障核心函数 ====================

/**
 * @brief 红色障碍航位推算避障主函数
 * @note 在Element_recognition()的case red_obstacle中调用
 */
void red_obstacle_navigation_avoid(void)
{
    switch(nav_state)
    {
        case RED_NAV_IDLE:
            // 等待外部触发（由red_obstacle_enter设置为RED_NAV_ENTER）
            break;
            
        case RED_NAV_ENTER:
            // 初始化积分器，准备开始避障
            clear_distant_integeral();
            clear_angle_integeral();
            nav_state = RED_NAV_PHASE_1;
            break;
            
        // ========== 阶段1: 直行20cm（脱离检测区） ==========
        case RED_NAV_PHASE_1:
            // 首次进入此状态时启动路径积分
            if(get_integeral_state(&distance_integral) == 0) {
                begin_distant_integeral(RED_OBS_FORWARD_DIST);
            }
            
            // 直行前进
            set_motor_speed((int16_t)RED_OBS_FORWARD_SPEED, (int16_t)RED_OBS_FORWARD_SPEED);
            
            // 完成检测
            if(get_integeral_state(&distance_integral) == 2) {
                clear_distant_integeral();
                nav_state = RED_NAV_PHASE_2_TURN;
            }
            break;
            
        // ========== 阶段2-转向: 左转90° ==========
        case RED_NAV_PHASE_2_TURN:
            if(get_integeral_state(&angle_integral) == 0) {
                begin_angle_integeral(-RED_OBS_TURN_ANGLE);  // 负值=左转
            }
            
            // 原地左转（左轮后退，右轮前进）
            set_motor_speed(-(int16_t)RED_OBS_TURN_SPEED, (int16_t)RED_OBS_TURN_SPEED);
            
            if(get_integeral_state(&angle_integral) == 2) {
                clear_angle_integeral();
                nav_state = RED_NAV_PHASE_2_FWD;
            }
            break;
            
        // ========== 阶段2-前进: 前进H(30cm，离开赛道) ==========
        case RED_NAV_PHASE_2_FWD:
            if(get_integeral_state(&distance_integral) == 0) {
                begin_distant_integeral(RED_OBS_LATERAL_DIST);
            }
            
            set_motor_speed((int16_t)RED_OBS_FORWARD_SPEED, (int16_t)RED_OBS_FORWARD_SPEED);
            
            if(get_integeral_state(&distance_integral) == 2) {
                clear_distant_integeral();
                nav_state = RED_NAV_PHASE_3_TURN;
            }
            break;
            
        // ========== 阶段3-转向: 右转90° ==========
        case RED_NAV_PHASE_3_TURN:
            if(get_integeral_state(&angle_integral) == 0) {
                begin_angle_integeral(RED_OBS_TURN_ANGLE);  // 正值=右转
            }
            
            // 原地右转（左轮前进，右轮后退）
            set_motor_speed((int16_t)RED_OBS_TURN_SPEED, -(int16_t)RED_OBS_TURN_SPEED);
            
            if(get_integeral_state(&angle_integral) == 2) {
                clear_angle_integeral();
                nav_state = RED_NAV_PHASE_3_FWD;
            }
            break;
            
        // ========== 阶段3-前进: 前进L(40cm，绕过障碍) ==========
        case RED_NAV_PHASE_3_FWD:
            if(get_integeral_state(&distance_integral) == 0) {
                begin_distant_integeral(RED_OBS_BYPASS_DIST);
            }
            
            set_motor_speed((int16_t)RED_OBS_FORWARD_SPEED, (int16_t)RED_OBS_FORWARD_SPEED);
            
            if(get_integeral_state(&distance_integral) == 2) {
                clear_distant_integeral();
                nav_state = RED_NAV_PHASE_4_TURN;
            }
            break;
            
        // ========== 阶段4-转向: 右转90° ==========
        case RED_NAV_PHASE_4_TURN:
            if(get_integeral_state(&angle_integral) == 0) {
                begin_angle_integeral(RED_OBS_TURN_ANGLE);
            }
            
            set_motor_speed((int16_t)RED_OBS_TURN_SPEED, -(int16_t)RED_OBS_TURN_SPEED);
            
            if(get_integeral_state(&angle_integral) == 2) {
                clear_angle_integeral();
                nav_state = RED_NAV_PHASE_4_FWD;
            }
            break;
            
        // ========== 阶段4-前进: 前进H(30cm，回到赛道) ==========
        case RED_NAV_PHASE_4_FWD:
            if(get_integeral_state(&distance_integral) == 0) {
                begin_distant_integeral(RED_OBS_LATERAL_DIST);
            }
            
            set_motor_speed((int16_t)RED_OBS_FORWARD_SPEED, (int16_t)RED_OBS_FORWARD_SPEED);
            
            if(get_integeral_state(&distance_integral) == 2) {
                clear_distant_integeral();
                nav_state = RED_NAV_PHASE_5;
            }
            break;
            
        // ========== 阶段5: 左转90°（回到与赛道平行） ==========
        case RED_NAV_PHASE_5:
            if(get_integeral_state(&angle_integral) == 0) {
                begin_angle_integeral(-RED_OBS_TURN_ANGLE);
            }
            
            set_motor_speed(-(int16_t)RED_OBS_TURN_SPEED, (int16_t)RED_OBS_TURN_SPEED);
            
            if(get_integeral_state(&angle_integral) == 2) {
                clear_angle_integeral();
                nav_state = RED_NAV_OUT;
            }
            break;
            
        case RED_NAV_OUT:
            // 避障完成，停止电机，等待外部重置状态
            set_motor_speed(0, 0);
            // 状态将由red_obstacle_out()重置
            break;
    }
}

/**
 * @brief 启动航位推算避障（由red_obstacle_enter调用）
 */
void red_obstacle_navigation_start(void)
{
    nav_state = RED_NAV_ENTER;
}

/**
 * @brief 检查航位推算避障是否完成
 * @return true: 避障完成; false: 避障进行中
 */
bool red_obstacle_navigation_is_finished(void)
{
    return (nav_state == RED_NAV_OUT);
}

/**
 * @brief 重置航位推算避障状态
 */
void red_obstacle_navigation_reset(void)
{
    nav_state = RED_NAV_IDLE;
    clear_distant_integeral();
    clear_angle_integeral();
}

// ==================== 使用说明 ====================
/*
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  航位推算避障使用指南
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

一、在red_obstacle.c中集成
──────────────────────────────────────────────
修改red_obstacle_avoid()函数:

void red_obstacle_avoid(void)
{
    // 使用航位推算避障
    red_obstacle_navigation_avoid();
}

修改red_obstacle_enter()函数:

void red_obstacle_enter(void)
{
    int16_t red_x = 0, red_y = 0;
    if (detect_red_area(&red_x, &red_y))
    {
        enter_element(red_obstacle);
        red_obstacle_navigation_start();  // 启动航位推算
    }
}

修改red_obstacle_out()函数:

void red_obstacle_out(void)
{
    if (red_obstacle_navigation_is_finished())
    {
        out_element();
        red_obstacle_navigation_reset();  // 重置状态
    }
}

二、电机控制说明
──────────────────────────────────────────────
本模块使用motor_run()函数控制电机:

void motor_run(Motor *motor_ptr, int16_t speed, uint8_t v)
参数说明:
  - motor_ptr: 电机对象指针 (&leftmotor 或 &rightmotor)
  - speed: PWM速度值 (-1000 to 1000)
  - v: 速度偏置值 (通常为0)

速度参数已调整为PWM值:
  - RED_OBS_FORWARD_SPEED = 500 (前进PWM)
  - RED_OBS_TURN_SPEED = 300 (转向PWM)

三、调试流程
──────────────────────────────────────────────
1. 验证编码器和陀螺仪数据正常
2. 测试单阶段动作（例如先只测试直行20cm）
3. 逐步增加阶段，观察轨迹
4. 调整参数（速度、距离、角度）

四、参数调整说明
──────────────────────────────────────────────
#define RED_OBS_FORWARD_DIST     20.0f   // 增大→更远离检测区
#define RED_OBS_LATERAL_DIST     30.0f   // 增大→离开赛道更远
#define RED_OBS_BYPASS_DIST      40.0f   // 增大→绕行更长距离
#define RED_OBS_TURN_ANGLE       90.0f   // 根据陀螺仪标定调整
#define RED_OBS_FORWARD_SPEED    500.0f  // 降低→提高精度 (PWM)
#define RED_OBS_TURN_SPEED       300.0f  // 降低→转向更稳定 (PWM)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
*/
