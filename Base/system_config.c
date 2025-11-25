/*
 * system_config.c
 * 系统配置文件 - 全局变量定义
 * Created on: 2025-11-23
 */

#include "system_config.h"

// ==================== 全局变量定义 ====================

setpara_struct setpara;
mycar_struct mycar;
//imu_struct imu;
vofa_struct vofa;

// 激光测距数据
uint16_t dl1b_distance_mm = 9999;  // 默认无障碍物

// ==================== PID占位变量 ====================
// 这些变量在Element_recognition.c中引用,需要定义但可能不使用
uint8_t CAM_Turn = 0;
uint8_t CAM_FUZZY_PID = 0;
uint8_t Speed_middle = 0;
uint8_t Speed_left = 0;
uint8_t Speed_right = 0;
