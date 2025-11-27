/**
 * @file element_state_machine.h
 * @brief 简洁元素识别状态机（基于watch标志位）
 * @note 使用switch结构实现，专注于环岛处理
 * @date 2025-11-23
 */

#ifndef ELEMENT_STATE_MACHINE_H_
#define ELEMENT_STATE_MACHINE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// ==================== 状态定义 ====================

typedef enum {
    STATE_NORMAL = 0,           // 正常巡线
    STATE_LOOP_ENTRY,           // 环岛入环
    STATE_LOOP_IN,              // 环内行驶
    STATE_LOOP_EXIT,            // 环岛出环
    STATE_CROSS,                // 十字路口
    STATE_ZEBRA,                // 斑马线
    STATE_OBSTACLE_BLACK,       // 黑色障碍物
    STATE_OBSTACLE_RED,         // 红色障碍物
    STATE_RAMP                  // 上坡
} ElementState_t;

// ==================== 全局变量 ====================

extern ElementState_t g_element_state;

// ==================== 核心接口 ====================

/**
 * @brief 初始化元素识别状态机
 */
void element_state_machine_init(void);

/**
 * @brief 状态机主循环（在图像处理后调用）
 * @note 自动读取watch结构体中的标志位，更新状态
 */
void element_state_machine_update(void);

/**
 * @brief 获取当前状态
 * @return 当前元素状态
 */
ElementState_t element_state_machine_get_state(void);

/**
 * @brief 手动重置状态机到正常状态
 */
void element_state_machine_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* ELEMENT_STATE_MACHINE_H_ */
