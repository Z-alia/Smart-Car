/*
 * red_obstacle.h
 * 红色障碍物识别模块
 * Created on: 2025-11-23
 */

#ifndef CAMERA_PROCESS_RED_OBSTACLE_H_
#define CAMERA_PROCESS_RED_OBSTACLE_H_

#include "main.h"
#include "Element_recognition.h"

// ==================== 红色障碍物检测参数 ====================

/**
 * @brief 红色障碍物配置参数
 */
typedef struct {
    uint16_t min_red_area;      // 最小红色区域面积(像素)
    uint16_t max_red_area;      // 最大红色区域面积
    uint8_t red_threshold_h;    // 红色H通道阈值上限
    uint8_t red_threshold_s;    // 红色S通道阈值下限
    uint8_t red_threshold_v;    // 红色V通道阈值下限
    uint8_t detection_line_start; // 检测起始行
    uint8_t detection_line_end;   // 检测结束行
} red_obstacle_config_t;

// ==================== 全局变量声明 ====================

extern red_obstacle_config_t red_obstacle_config;

// ==================== 接口函数声明 ====================

/**
 * @brief 初始化红色障碍物检测模块
 */
void red_obstacle_init(void);

/**
 * @brief 红色障碍物进入检测(在None状态下调用)
 * @note 扫描图像中的红色区域,判断是否为红色障碍物
 *       检测到后调用 enter_element(red_obstacle)
 */
void red_obstacle_enter(void);

/**
 * @brief 红色障碍物避障处理
 * @note 根据障碍物位置决定避障策略(左绕/右绕/停车)
 */
void red_obstacle_avoid(void);

/**
 * @brief 红色障碍物退出处理
 * @note 完成避障后调用 out_element() 恢复正常巡线
 */
void red_obstacle_out(void);

/**
 * @brief 红色区域检测函数
 * @param red_x 检测到的红色区域中心X坐标(输出)
 * @param red_y 检测到的红色区域中心Y坐标(输出)
 * @return 1=检测到红色障碍物, 0=未检测到
 * @note 用户需要实现此函数,根据实际颜色传感器/图像处理算法
 */
uint8_t detect_red_area(int16_t* red_x, int16_t* red_y);

#endif /* CAMERA_PROCESS_RED_OBSTACLE_H_ */
