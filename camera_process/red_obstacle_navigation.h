/**
 * @file red_obstacle_navigation.h
 * @brief 红色障碍航位推算避障头文件
 */

#ifndef RED_OBSTACLE_NAVIGATION_H
#define RED_OBSTACLE_NAVIGATION_H

#include <stdint.h>
#include <stdbool.h>

// ==================== 函数声明 ====================

/**
 * @brief 红色障碍航位推算避障主函数
 * @note 在Element_recognition()的case red_obstacle中调用
 */
void red_obstacle_navigation_avoid(void);

/**
 * @brief 启动航位推算避障
 */
void red_obstacle_navigation_start(void);

/**
 * @brief 检查航位推算避障是否完成
 * @return true: 避障完成; false: 避障进行中
 */
bool red_obstacle_navigation_is_finished(void);

/**
 * @brief 重置航位推算避障状态
 */
void red_obstacle_navigation_reset(void);

#endif /* RED_OBSTACLE_NAVIGATION_H */
