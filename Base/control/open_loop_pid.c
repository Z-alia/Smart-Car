/*
 * open_loop_pid.c
 * 开环PID控制器实现 - 基于图像误差的差速控制
 * Created on: 2025-11-23
 */

#include "open_loop_pid.h"
#include <string.h>
#include <math.h>

// ==================== 外部依赖声明 ====================
// 这些数据结构需要从图像处理模块获取

/**
 * @brief 扫线信息结构体(来自 scan_line.h)
 * @note 包含每行的左右边线、丢线标志等信息
 */
struct lineinfo_s {
    int16_t y;          // 行号(0~119)
    int16_t left;       // 左边线位置(0~188)
    int16_t right;      // 右边线位置(0~188)
    int16_t edge_count; // 边线数量
    int left_lost;      // 左边线丢失标志
    int right_lost;     // 右边线丢失标志
    int16_t whole_lost; // 整行丢失标志
    // ... 其他字段省略
};

/**
 * @brief 扫线结果数组(来自 scan_line.c)
 * @note 每行的扫线信息,lineinfo[119-y]对应图像第y行
 */
extern struct lineinfo_s lineinfo[120];

/**
 * @brief 权重参数(可调节)
 * @note 调整这些权重可以改变不同区域对误差的影响
 */
#ifndef weight_dw
#define weight_dw 0.2f  // 下段权重(行1-30,近处)
#endif

#ifndef weight_md
#define weight_md 0.3f  // 中段权重(行30-60,中距离)
#endif

#ifndef weight_up
#define weight_up 0.5f  // 上段权重(行60-120,远处)
#endif

// ==================== 全局变量定义 ====================
OpenLoopPIDState g_open_loop_pid_state = {0};

// ==================== 私有辅助函数 ====================

/**
 * @brief 浮点数限幅函数
 * @param x 输入值
 * @param lo 下限
 * @param hi 上限
 * @return 限幅后的值
 */
static inline float clampf(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

/**
 * @brief 直线误差获取函数(基于扫线结果)
 * @return 横向偏差误差值
 * @note 计算加权平均中线位置与图像中心(94.0)的偏差
 *       使用 lineinfo[] 数组计算中线位置
 *       误差正值表示车偏右(中线在右侧),负值表示车偏左
 */
static float straight_error_get(void)
{
    float sum = 0.0f, temp = 0.0f;
    uint8_t length = 0;
    
    // 下段(行1-30,近处)
    for (uint8_t i = 1; i < 30; i++) {
        // 检查该行是否有有效边线(未完全丢失)
        if (!lineinfo[i].whole_lost && 
            !lineinfo[i].left_lost && 
            !lineinfo[i].right_lost) {
            length++;
            // 计算中线位置: (左边线 + 右边线) / 2
            float center = (lineinfo[i].left + lineinfo[i].right) / 2.0f;
            temp += center;
        }
    }
    if (length > 0) {
        sum += (temp / length) * weight_dw;
    }
    
    // 中段(行30-60,中距离)
    length = 0;
    temp = 0.0f;
    for (uint8_t i = 30; i < 60; i++) {
        if (!lineinfo[i].whole_lost && 
            !lineinfo[i].left_lost && 
            !lineinfo[i].right_lost) {
            length++;
            float center = (lineinfo[i].left + lineinfo[i].right) / 2.0f;
            temp += center;
        }
    }
    if (length > 0) {
        sum += (temp / length) * weight_md;
    }
    
    // 上段(行60-120,远处)
    temp = 0.0f;
    length = 0;
    for (uint8_t i = 60; i < 120; i++) {
        if (!lineinfo[i].whole_lost && 
            !lineinfo[i].left_lost && 
            !lineinfo[i].right_lost) {
            length++;
            float center = (lineinfo[i].left + lineinfo[i].right) / 2.0f;
            temp += center;
        }
    }
    if (length > 0) {
        sum += (temp / (float)length) * weight_up;
    }
    
    // 返回误差: 当前位置 - 目标位置(94.0)
    // 94.0 是图像中心位置(188/2 = 94)
    return sum - 94.0f;
}

// ==================== 公共函数实现 ====================

/**
 * @brief 初始化开环PID控制器
 * @param kp 比例增益
 * @param ki 积分增益
 * @param kd 微分增益
 * @param integral_max 积分限幅
 * @param output_max 输出限幅
 */
void open_loop_pid_init(float kp, float ki, float kd, 
                        float integral_max, float output_max)
{
    memset(&g_open_loop_pid_state, 0, sizeof(OpenLoopPIDState));
    
    g_open_loop_pid_state.pid.kp = kp;
    g_open_loop_pid_state.pid.ki = ki;
    g_open_loop_pid_state.pid.kd = kd;
    g_open_loop_pid_state.pid.integral_max = integral_max;
    g_open_loop_pid_state.pid.output_max = output_max;
}

/**
 * @brief 重置开环PID控制器状态
 */
void open_loop_pid_reset(void)
{
    g_open_loop_pid_state.pid.integral = 0.0f;
    g_open_loop_pid_state.pid.prev_error = 0.0f;
    g_open_loop_pid_state.error = 0.0f;
    g_open_loop_pid_state.control_output = 0.0f;
}

/**
 * @brief 开环PID控制计算
 * @param v_forward 前进基准速度(m/s)
 * @param left_target_out 左轮目标速度输出指针
 * @param right_target_out 右轮目标速度输出指针
 * @return 控制输出值(差速)
 * @note 内部调用 straight_error_get() 获取误差,并取反值
 *       执行流程:
 *       1. 获取图像误差并取反(因为误差定义方向)
 *       2. PID计算: u(k) = Kp·e(k) + Ki·Σe(k) + Kd·[e(k)-e(k-1)]
 *       3. 输出限幅
 *       4. 差速分配: vL = v - Δv/2, vR = v + Δv/2
 */
float open_loop_pid_calculate(float v_forward, 
                               float* left_target_out, 
                               float* right_target_out)
{
    if (!left_target_out || !right_target_out) {
        return 0.0f;
    }
    
    OpenLoopPID* pid = &g_open_loop_pid_state.pid;
    
    // 1. 获取误差并取反
    // straight_error_get() 返回正值表示偏右,负值表示偏左
    // 取反后: 正值需要右转(右轮快),负值需要左转(左轮快)
    float error = -straight_error_get();
    g_open_loop_pid_state.error = error;
    
    // 2. 积分计算(抗饱和)
    pid->integral += error;
    pid->integral = clampf(pid->integral, -pid->integral_max, pid->integral_max);
    
    // 3. 微分计算(一阶差分)
    float derivative = error - pid->prev_error;
    pid->prev_error = error;
    
    // 4. PID输出(位置式PID)
    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    
    // 5. 输出限幅
    output = clampf(output, -pid->output_max, pid->output_max);
    g_open_loop_pid_state.control_output = output;
    
    // 6. 差速分配
    // output > 0: 需要右转,右轮加速,左轮减速
    // output < 0: 需要左转,左轮加速,右轮减速
    float left_target = v_forward - output * 0.5f;
    float right_target = v_forward + output * 0.5f;
    
    g_open_loop_pid_state.left_target = left_target;
    g_open_loop_pid_state.right_target = right_target;
    
    *left_target_out = left_target;
    *right_target_out = right_target;
    
    return output;
}

/**
 * @brief 在线更新PID参数
 * @param kp 比例增益
 * @param ki 积分增益
 * @param kd 微分增益
 */
void open_loop_pid_update_params(float kp, float ki, float kd)
{
    g_open_loop_pid_state.pid.kp = kp;
    g_open_loop_pid_state.pid.ki = ki;
    g_open_loop_pid_state.pid.kd = kd;
}

/**
 * @brief 获取当前误差值
 * @return 当前误差值
 */
float open_loop_pid_get_error(void)
{
    return g_open_loop_pid_state.error;
}

/**
 * @brief 获取当前控制输出值
 * @return 当前控制输出值
 */
float open_loop_pid_get_output(void)
{
    return g_open_loop_pid_state.control_output;
}

// ==================== 完整使用说明 ====================
/*
╔══════════════════════════════════════════════════════════════════════════════╗
║                     开环PID控制器 - 完整调用流程                              ║
╚══════════════════════════════════════════════════════════════════════════════╝

一、依赖模块准备
─────────────────────────────────────────────────────────────────────────────
在使用本控制器之前,需要确保以下模块已正常工作:

1. 图像采集模块(dcmi_ov2640.c)
   - 初始化摄像头: OV2640_Init()
   - 启动DMA传输: OV2640_DMA_Transmit_Continuous()
   
2. 图像处理模块(Binarization.c)
   - 计算阈值: watch.threshold = img_otsu(...)
   - 二值化: Binarization()
   
3. 扫线模块(scan_line.c)
   - 执行扫线: scan_line()
   - 填充 lineinfo[] 数组


二、在 main.c 中的完整调用示例
─────────────────────────────────────────────────────────────────────────────
#include "open_loop_pid.h"
#include "scan_line.h"
#include "Binarization.h"
#include "dcmi_ov2640.h"

int main(void)
{
    // 1. 硬件初始化
    HAL_Init();
    SystemClock_Config();
    
    // 2. 外设初始化
    OV2640_Init();              // 初始化摄像头
    OV2640_DMA_Transmit_Continuous(Camera_Buffer, OV2640_BufferSize);
    LCD_Init();                 // 显示屏初始化
    
    // 3. 初始化开环PID控制器
    open_loop_pid_init(
        1.0f,    // kp: 比例增益,建议0.5~2.0
        0.1f,    // ki: 积分增益,建议0.05~0.2
        0.05f,   // kd: 微分增益,建议0.01~0.1
        50.0f,   // integral_max: 积分限幅
        2.0f     // output_max: 输出限幅(差速,m/s)
    );
    
    // 4. 主循环
    while (1)
    {
        // 4.1 等待一帧图像采集完成
        if (DCMI_FrameState == 1)
        {
            DCMI_FrameState = 0;
            
            // 4.2 图像处理流程
            watch.threshold = img_otsu((uint16_t *)mt9v03x_image[30], 
                                      60, Display_Width, 10);
            Binarization();      // 二值化
            scan_line();         // 扫线 → 填充 lineinfo[] 数组
            
            // 4.3 调用开环PID计算
            float v_forward = 1.0f;  // 前进速度 1m/s
            float left_target, right_target;
            
            open_loop_pid_calculate(v_forward, &left_target, &right_target);
            
            // 4.4 输出到电机(需要自己实现电机控制函数)
            set_motor_speed(MOTOR_LEFT, left_target);
            set_motor_speed(MOTOR_RIGHT, right_target);
            
            // 4.5 显示图像(可选)
            draw_edge();
            show_ov2640_image_int8(0, 0, imo[0], 
                                  Display_Width, Display_Height, 
                                  Display_Width, Display_Height);
        }
    }
}


三、参数调试指南
─────────────────────────────────────────────────────────────────────────────
调试步骤:
1. 先调Kp (比例增益)
   - 从小到大逐渐增加(0.5 → 1.0 → 1.5 → 2.0)
   - 观察车辆响应速度,Kp越大响应越快
   - 如果出现左右摆动(振荡),说明Kp过大,需要减小
   
2. 再调Kd (微分增益)
   - 在Kp基础上增加Kd,抑制振荡
   - 建议从0.05开始,逐渐增加到0.1~0.2
   - Kd可以提前预测误差变化,增加系统阻尼
   
3. 最后调Ki (积分增益)
   - 消除稳态误差(长时间偏离中线)
   - 建议从0.05开始,逐渐增加
   - 如果车辆出现"打转"现象,说明Ki过大或积分饱和
   
运行时调参:
   open_loop_pid_update_params(1.2f, 0.15f, 0.08f);


四、重要数据结构说明
─────────────────────────────────────────────────────────────────────────────
1. lineinfo[] 数组 (来自 scan_line.c)
   - lineinfo[0~119]: 每行的扫线结果
   - lineinfo[i].left: 第i行的左边线位置(0~188)
   - lineinfo[i].right: 第i行的右边线位置(0~188)
   - lineinfo[i].left_lost: 左边线丢失标志(1=丢失)
   - lineinfo[i].right_lost: 右边线丢失标志
   - lineinfo[i].whole_lost: 整行丢失标志
   
2. 误差计算逻辑
   - 中线位置 = (左边线 + 右边线) / 2
   - 误差 = 加权平均中线 - 94.0(图像中心)
   - 误差 > 0: 车偏右,需要左转
   - 误差 < 0: 车偏左,需要右转


五、调试信息获取
─────────────────────────────────────────────────────────────────────────────
float error = open_loop_pid_get_error();    // 当前误差值
float output = open_loop_pid_get_output();  // 控制输出值

// 串口发送调试信息
printf("Error: %.2f, Output: %.2f, L: %.2f, R: %.2f\r\n", 
       error, output, 
       g_open_loop_pid_state.left_target, 
       g_open_loop_pid_state.right_target);


六、注意事项
─────────────────────────────────────────────────────────────────────────────
1. 必须先执行 scan_line() 才能调用 open_loop_pid_calculate()
2. 权重调节: 修改 weight_dw, weight_md, weight_up 以适应不同赛道
3. 输出限幅: output_max 应根据车辆最大转弯能力设置
4. 积分限幅: integral_max 建议设为稳态误差的5~10倍
5. 切换赛道时调用 open_loop_pid_reset() 清除积分残留

═══════════════════════════════════════════════════════════════════════════════
*/
