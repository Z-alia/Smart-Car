/*
 * system_config.h
 * 系统配置文件 - 定义全局结构体和缺失的依赖
 * Created on: 2025-11-23
 */

#ifndef SYSTEM_CONFIG_H_
#define SYSTEM_CONFIG_H_

#include <stdint.h>

// ==================== 参数配置结构体 ====================

/**
 * @brief 系统参数配置结构体
 */
typedef struct {
    // 启动模式
    uint8_t start_mode;          // 1=左出库 2=右出库 3/4=其他模式
    
    // 元素序列配置
    uint8_t set_element[21];     // 元素序列 (1=左圆环,2=右圆环,3=坡道...)
    uint8_t loop_data[4];        // 圆环类型 (0=小圆环,1=大圆环)
    
    // 时间配置(ms)
    uint32_t begin_time;         // 开始识别元素的时间
    uint32_t slope_begin_time;   // 开始识别坡道的时间
    uint32_t zebra_begin_time;   // 开始识别斑马线的时间
    
    // 速度配置
    float speed_min;             // 最小速度
    float loop_target_speed;     // 圆环目标速度
    float slope_speed;           // 坡道速度
    float big_loop_speed;        // 大圆环速度
    float cross_speed;           // 十字路口速度
    float zebra_speed;           // 斑马线速度
    float com_target_speed;      // 普通道路目标速度
    
    // 距离配置(cm)
    float loop_out_distance;     // 小圆环出圆距离
    float big_loop_out_distance; // 大圆环出圆距离
    float zebra_distance;        // 斑马线停车距离
    
    // 斑马线计数配置
    uint8_t zebra_line_count;    // 斑马线线条数量阈值
    uint8_t stop_over_count;     // 停车计数阈值
    
    // 功能开关
    uint8_t cross_open_flag;     // 十字路口识别开关 >=1启用
    uint8_t bla_obs_open_flag;   // 黑色障碍物识别开关 >=1启用
    
    // 扫线参数
    uint8_t far_line;            // 远端扫线行数 (默认115)
    
    // 其他参数
    uint8_t mode;                // 出环模式 (OutLeft/OutRight)
    uint8_t obstacle_dir;        // 避障方向 0=左 1=右
    
    // PID参数(占位,实际使用control.h中的配置)
    uint8_t com_speed_PID;       // 通用速度PID索引
    uint8_t loop_turn_PID;       // 圆环转弯PID索引
    uint8_t stop_PID;            // 停车PID索引
    uint8_t big_loop_PID;        // 大圆环转弯PID索引
    uint8_t com_turn_PID;        // 普通转向PID索引
    
} setpara_struct;

/**
 * @brief 小车状态结构体
 */
typedef struct {
    // 运行状态
    uint32_t RUNTIME;            // 运行时间(ms)
    uint8_t car_running;         // 车辆运行状态 0=停止 1=运行
    uint8_t car_stop;            // 停车标志
    
    // 速度控制
    float target_speed;          // 目标速度
    float present_speed;         // 当前速度
    uint8_t speed_ctrl;          // 速度控制使能 0=禁用 1=启用
    uint8_t pid_ctrl;            // PID控制使能 0=禁用 1=启用
    
    // 其他状态
    uint8_t tracking_mode;       // 循线模式
    uint8_t CircCount;           // 圆环计数
    
} mycar_struct;

/**
 * @brief IMU数据结构体
 */
typedef struct {
    float pitch;                 // 俯仰角(度)
    float roll;                  // 横滚角(度)
    float yaw;                   // 航向角(度)
    float gyro_x;                // X轴角速度(度/秒)
    float gyro_y;                // Y轴角速度(度/秒)
    float gyro_z;                // Z轴角速度(度/秒) - 用于角度积分
} imu_struct;

/**
 * @brief 视觉监控结构体 - 存储各元素识别的中间状态
 */
typedef struct {
    // 二值化阈值
    uint8_t threshold;           // 动态阈值
    
    // 圆环相关
    uint8_t InLoop;              // 入环标志
    uint8_t InLoopAngleL;        // 入环左拐点行
    uint8_t InLoopAngleR;        // 入环右拐点行
    uint8_t InLoopCirc;          // 入环圆弧行
    uint8_t InLoopAngle2;        // 入环第二拐点行
    uint8_t OutLoop;             // 出环标志
    uint8_t OutLoopAngle1;       // 出环第一拐点行
    uint8_t OutLoopAngle2;       // 出环第二拐点行
    
    // 十字路口相关
    uint8_t cross_flag;          // 十字路口标志
    uint8_t cross_line;          // 十字路口行
    uint8_t Garge_line;          // 车库行
    uint8_t cross_RD_angle;      // 十字右下拐点行
    uint8_t cross_LD_angle;      // 十字左下拐点行
    uint8_t cross_AngleL;        // 十字左拐点行
    uint8_t cross_AngleR;        // 十字右拐点行
    uint8_t cross_AngleL_x;      // 十字左拐点列
    uint8_t cross_AngleR_x;      // 十字右拐点列
    
    // 断路标志
    uint8_t broken_circuit_flag; // 断路标志
    
    // 坡道相关
    uint8_t slope_flag;          // 坡道标志 0=未检测 1=上坡 2=下坡
    
    // 巡线模式
    uint8_t Line_patrol_mode;    // 巡线模式
    
    // 斑马线相关
    uint8_t zebra_flag;          // 斑马线标志
    uint8_t Zebra_Angle;         // 斑马线拐点行
    uint8_t Zebra_Angle2;        // 斑马线第二拐点行
    uint8_t ZebraLine;           // 斑马线行
    uint16_t stop_count;         // 停车计数
    
    // 车库相关
    uint8_t out_garage_flag;     // 出库标志
    uint8_t garage_stop;         // 车库停车标志
    uint8_t garage_flag;         // 车库标志
    
    // 障碍物相关
    uint8_t obstacle_flag;       // 红色障碍物标志
    uint8_t black_obstacle_flag; // 黑色障碍物标志
    uint8_t black_obstacle_line; // 障碍物行
    uint8_t left_obstacle_x;     // 左侧障碍物列
    uint8_t right_obstacle_x;    // 右侧障碍物列
    
    // 扫线范围
    uint8_t angle_near_line;     // 近端扫线行
    uint8_t angle_far_line;      // 远端扫线行
    
} watch_struct;

/**
 * @brief VOFA调试数据结构体(可选)
 */
typedef struct {
    float loop[10];              // 圆环调试数据
    float zebra[10];             // 斑马线调试数据
} vofa_struct;

// ==================== 全局变量声明 ====================

extern setpara_struct setpara;
extern mycar_struct mycar;
extern imu_struct imu;
//extern struct watch_o watch;
extern vofa_struct vofa;

// ==================== 外部依赖函数替代 ====================

/**
 * @brief 激光测距数据(mm)
 * @note 如果没有激光传感器,可以设置为较大值禁用
 */
extern uint16_t dl1b_distance_mm;

/**
 * @brief 蜂鸣器函数(可选,可以注释掉)
 * @param times 蜂鸣次数
 * @param duration 持续时间(ms)
 */
static inline void beep2(uint8_t times, uint16_t duration) {
    // TODO: 实现蜂鸣器功能,或者留空
    // 示例: HAL_GPIO_TogglePin(BEEP_GPIO_Port, BEEP_Pin);
}

/**
 * @brief 设置目标速度
 * @param speed 目标速度(m/s)
 */
static inline void set_speed(float speed) {
    mycar.target_speed = speed;
}

/**
 * @brief PID参数切换函数(占位)
 * @note 实际使用control.h中的PID参数
 */
static inline void change_pid_para(void* pid1, void* pid2) {
    // 占位函数,实际根据需要实现PID参数切换
}

// ==================== 默认参数初始化 ====================

/**
 * @brief 初始化系统参数为默认值
 */
static inline void system_config_init(void) {
    // 启动模式
    setpara.start_mode = 1;  // 默认左出库
    
    // 元素序列 (0=自动识别所有元素)
    setpara.set_element[0] = 0;  
    for (int i = 1; i < 21; i++) {
        setpara.set_element[i] = 0;
    }
    
    // 圆环类型
    setpara.loop_data[0] = 0;  // 小圆环
    setpara.loop_data[1] = 0;
    setpara.loop_data[2] = 0;
    setpara.loop_data[3] = 0;
    
    // 时间配置
    setpara.begin_time = 2000;        // 2秒后开始识别元素
    setpara.slope_begin_time = 3000;  // 3秒后开始识别坡道
    setpara.zebra_begin_time = 3000;  // 3秒后开始识别斑马线
    
    // 速度配置
    setpara.speed_min = 1.0f;          // 最小速度1m/s
    setpara.loop_target_speed = 1.2f;  // 圆环速度1.2m/s
    setpara.slope_speed = 0.8f;        // 坡道速度0.8m/s
    setpara.big_loop_speed = 1.5f;     // 大圆环速度1.5m/s
    setpara.cross_speed = 1.0f;        // 十字路口速度1.0m/s
    setpara.zebra_speed = 0.6f;        // 斑马线速度0.6m/s
    setpara.com_target_speed = 1.2f;   // 普通道路目标速度1.2m/s
    
    // 距离配置
    setpara.loop_out_distance = 30.0f;      // 小圆环出圆距离30cm
    setpara.big_loop_out_distance = 40.0f;  // 大圆环出圆距离40cm
    setpara.zebra_distance = 20.0f;         // 斑马线停车距离20cm
    
    // 斑马线计数配置
    setpara.zebra_line_count = 3;      // 斑马线线条数量阈值
    setpara.stop_over_count = 50;      // 停车计数阈值
    
    // 功能开关
    setpara.cross_open_flag = 1;       // 启用十字路口识别
    setpara.bla_obs_open_flag = 1;     // 启用黑色障碍物识别
    
    // 扫线参数
    setpara.far_line = 115;            // 远端扫线行数
    
    // 其他参数
    setpara.mode = 0;                  // 默认出环模式
    setpara.obstacle_dir = 0;          // 默认左避障
    
    // 小车初始状态
    mycar.RUNTIME = 0;
    mycar.car_running = 1;
    mycar.car_stop = 0;
    mycar.target_speed = 1.0f;
    mycar.present_speed = 0.0f;
    mycar.speed_ctrl = 1;
    mycar.pid_ctrl = 1;
    mycar.tracking_mode = 0;
    mycar.CircCount = 0;
    
    // IMU初始状态
//    imu.pitch = 0.0f;
//    imu.roll = 0.0f;
//    imu.yaw = 0.0f;
//    imu.gyro_x = 0.0f;
//    imu.gyro_y = 0.0f;
//    imu.gyro_z = 0.0f;
//    
//    // 视觉监控初始状态
//    watch.threshold = 128;
//    watch.InLoop = 0;
//    watch.InLoopAngleL = 120;
//    watch.InLoopAngleR = 120;
//    watch.InLoopCirc = 120;
//    watch.InLoopAngle2 = 120;
//    watch.OutLoop = 0;
//    watch.OutLoopAngle2 = 120;
//    watch.OutLoopAngle1 = 120;
//    watch.cross_flag = 0;
//    watch.cross_line = 120;
//    watch.Garge_line = 120;
//    watch.cross_RD_angle = 120;
//    watch.cross_LD_angle = 120;
//    watch.cross_AngleL = 120;
//    watch.cross_AngleR = 120;
//    watch.cross_AngleL_x = 0;
//    watch.cross_AngleR_x = 187;
//    watch.broken_circuit_flag = 0;
//    watch.slope_flag = 0;
//    watch.Line_patrol_mode = 0;
//    watch.zebra_flag = 0;
//    watch.Zebra_Angle = 120;
//    watch.Zebra_Angle2 = 120;
//    watch.ZebraLine = 120;
//    watch.stop_count = 0;
//    watch.out_garage_flag = 0;
//    watch.garage_stop = 0;
//    watch.garage_flag = 0;
//    watch.obstacle_flag = 0;
//    watch.black_obstacle_flag = 0;
//    watch.black_obstacle_line = 120;
//    watch.left_obstacle_x = 0;
//    watch.right_obstacle_x = 187;
//    watch.angle_near_line = 30;
//    watch.angle_far_line = 115;
    
    // 激光测距(如果没有传感器,设置为大值)
    dl1b_distance_mm = 9999;  // 9999mm表示无障碍物
}

#endif /* SYSTEM_CONFIG_H_ */
