/*
 * control.c
 * 差速小车双控制方案实现
 * 仿照cam_err_calculation的逻辑设计
 */

#include "control.h"
#include <math.h>
#include <string.h>
#include "image.h"
#include "motor.h"
/*-----------------工具函数---------------------*/
int my_abs_c(int value)
{
if(value>=0) return value;
else return -value;
}

// 全局控制器状态
ControlState g_control = {0};
control_t control = {0};

// 全局摄像头误差计算数据(简化版)
CamErrData g_cam_err_data = {0};

// ==================== 摄像头误差计算实现(直接使用image数据) ====================
float straight_error_get(void)
{
	float sum=0.0,temp=0.0; // 平均值，可后加加权
	uint8_t length=0;
	for(uint8_t i=1;i<30;i++)
    {
		if(center_line[i]!=0)
		length++;
       temp+=center_line[i];
		
    }
	sum+=(temp/length)*weight_dw;
	length=0;
	temp=0.0;
	for(uint8_t i=30;i<60;i++)
    {
	   if(center_line[i]!=0)
	   length++;
       temp+=center_line[i];
    }
	sum+=(temp/length)*weight_md;
	temp=0.0;
	length=0;
	for(uint8_t i=60;i<120;i++)
    {
	    if(center_line[i]!=0)
	   length++;
       temp+=center_line[i];
    }
	sum+=(temp/(float)length)*weight_up;
    return sum- (94.0);
}
/**
 * @brief 摄像头误差计算函数(完全恢复原版算法)
 * @return 横向偏差误差值(带非线性增益)
 * @note 核心逻辑(与原版cam_err_calculation完全一致):
 *       1. 使用逆透视坐标计算每行边界点到车轮的切线斜率
 *          公式: angel = 125 × (persp_x - camw_x) / (persp_y - camw_y)
 *       2. 遍历图像行,寻找左右切线斜率(AngleLeft最小, AngleRight最大)
 *       3. 左右斜率之和作为原始误差
 *       4. 变化率限幅(防止突变)
 *       5. 可选非线性增益: output = (1 + K·err²)·err
 * 
 * @note 数据来源: 
 *       - 边线: l_border/r_border/left_lost/right_lost (来自image.c)
 *       - 逆透视: persp_lx/ly/rx/ry (来自persp模块)
 *       - 车轮位置: camwl/camwr/camwf (配置参数)
 * 
 * @note 距离判断: 使用逆透视坐标 persp_ly/ry 判断行的远近
 */
float cam_err_calculation(void)
{
    #define CAM_D_ERR_LIMIT 30     // 变化率限幅
    #define CAM_OUTPUT_LIMIT 200   // 输出限幅
    
    CamErrData* cam = &g_cam_err_data;
    CamErrParams* params = &cam->params;
    CamErrState* state = &cam->state;
    
    // 初始化切线斜率为极值
    int AngleLeft = (int)0x80000000;   // 负无穷(寻找最小值,即左切线)
    int AngleRight = 0x7FFFFFFF;        // 正无穷(寻找最大值,即右切线)
    int AngleLeftLast = 0, AngleRightLast = 0;
    
    int watchleft = params->far_line;
    int watchright = params->far_line;
    int left_lost_count = 0, right_lost_count = 0;
    
    // 遍历图像行,计算切线斜率并寻找最优切线
    for (int y = params->near_line; y < params->far_line && y < IMAGE_H - 1; y++)
    {
        // 使用逆透视坐标判断是否跳过过近的行
        // 原逻辑: 0.625 * (persp_ly + persp_ry) < forward_near
        if (/*0.625f * ((int)persp_ly[y] + (int)persp_ry[y]) < cam->forward_near*/0)
        {
            continue;
        }
        
        // ========== 原版算法：计算边界点到车轮的切线斜率 ==========
        // 公式: angel_left = 125 × (persp_lx - camwl) / (persp_ly - camwf)
        // 含义: 左边界点到左轮的斜率倒数×1000 (实际公式为×125，原注释有误)
        int angel_left = 0;
        int angel_right = 0;
        
        // 避免除零
//        int denom_left = (int)persp_ly[y] - (int)params->camwf;
//        int denom_right = (int)persp_ry[y] - (int)params->camwf;
				
				int denom_left = y - (int)params->camwf;
        int denom_right = y - (int)params->camwf;
        
        if (denom_left != 0) {
            angel_left = 125 * ((int)l_border[y] - (int)params->camwl) / denom_left;
        }
        
        if (denom_right != 0) {
            angel_right = 125 * ((int)r_border[y] - (int)params->camwr) / denom_right;
        }
        // ========== 原版算法结束 ==========
        
        // 左边线处理:寻找斜率最小的切线(左切线)
        if (left_lost_count < 10)
        {
            // 统计连续丢线次数
            if (y > 40) {
                if (left_lost[y] && params->track_count < 75)
                    left_lost_count++;
                else
                    left_lost_count = 0;
            }
            
            // 更新左切线斜率(取最小值,且变化量不超过400)
            if (AngleLeft < angel_left &&
                !left_lost[y] &&
                (my_abs_c(AngleLeft - AngleLeftLast) < 400))
            {
                AngleLeft = angel_left;
                watchleft = y;
            }
        }
        
        // 右边线处理:寻找斜率最大的切线(右切线)
        if (right_lost_count < 10)
        {
            // 统计连续丢线次数
            if (y > 40) {
                if (right_lost[y] && params->track_count < 75)
                    right_lost_count++;
                else
                    right_lost_count = 0;
            }
            
            // 更新右切线斜率(取最大值,且变化量不超过400)
            if (AngleRight > angel_right &&
                !right_lost[y] &&
                (my_abs_c(AngleRight - AngleRightLast) < 400))
            {
                AngleRight = angel_right;
                watchright = y;
            }
        }
        
        // 终止条件1:左斜率大于右斜率(某行已越过垂直线)
        // 终止条件2:该行太远(基于逆透视坐标判断)
        // 原逻辑: 0.4 * (persp_ly + persp_ry) > forward_far
        if (/*AngleLeft > AngleRight || 
            0.4f * ((int)persp_ly[y] + (int)persp_ry[y]) > cam->forward_far*/0)
        {
            break;
        }
        
        AngleLeftLast = AngleLeft;
        AngleRightLast = AngleRight;
    }
    
    // 记录切点位置(用于调试显示)
    state->watchleft = watchleft + 1;
    state->watchright = watchright + 1;
    
    // 原始误差 = 左斜率 + 右斜率
    float angle_target = (float)(AngleLeftLast + AngleRightLast);
    
    // 变化率限幅(防止误差突变)
    float d_err = angle_target - state->angle_target_last;
    if (d_err > CAM_D_ERR_LIMIT)
        angle_target = state->angle_target_last + CAM_D_ERR_LIMIT;
    else if (d_err < -CAM_D_ERR_LIMIT)
        angle_target = state->angle_target_last - CAM_D_ERR_LIMIT;
    
    // 输出限幅
    if (angle_target > CAM_OUTPUT_LIMIT)
        angle_target = CAM_OUTPUT_LIMIT;
    else if (angle_target < -CAM_OUTPUT_LIMIT)
        angle_target = -CAM_OUTPUT_LIMIT;
    
    // 保存本次结果供下次使用
    state->angle_target_last = angle_target;
    state->angle_target = angle_target;
    
    // 非线性增益处理: output = (1 + K·err²)·err
    // K = 1e-7 × SteerKpchange
    float output = (1.0f + 1e-7f * params->SteerKpchange * angle_target * angle_target) * angle_target;
    
    return output;
    
    #undef CAM_D_ERR_LIMIT
    #undef CAM_OUTPUT_LIMIT
}

/**
 * @brief 初始化摄像头误差计算模块(简化版)
 * @param near_line 近端行(打角起始行)
 * @param far_line 远端行(打角终止行)
 * @param forward_near 近端距离阈值
 * @param forward_far 远端距离阈值
 * @note 使用示例:
 *       cam_err_init(20, 80, 10, 100);  // 从第20行到第80行打角
 */
void cam_err_init(int near_line, int far_line, int forward_near, int forward_far)
{
    memset(&g_cam_err_data, 0, sizeof(CamErrData));
    
    // 设置参数
    g_cam_err_data.params.near_line = near_line;
    g_cam_err_data.params.far_line = far_line;
    g_cam_err_data.forward_near = forward_near;
    g_cam_err_data.forward_far = forward_far;
    
    // 默认参数
    g_cam_err_data.params.SteerKpchange = 0.0f;  // 默认不启用非线性增益
    g_cam_err_data.params.track_count = 0;
    
    // 车轮位置参数(原版默认值)
    g_cam_err_data.params.camwl = 76;   // 左轮x位置
    g_cam_err_data.params.camwr = 97;   // 右轮x位置
    g_cam_err_data.params.camwf = 0;    // 前轮y位置
}

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
 * @brief 符号函数
 * @param x 输入值
 * @return 1.0(正), -1.0(负), 0.0(零)
 */
static inline float sign(float x)
{
    if (x > 0.0f) return 1.0f;
    else if (x < 0.0f) return -1.0f;
    else return 0.0f;
}

/**
 * @brief 符号切换函数(ADRC专用)
 * @param x 输入值
 * @param d 切换阈值
 * @return 切换函数值,用于平滑过渡
 */
static inline float fsg(float x, float d)
{
    return (sign(x + d) - sign(x - d));
}

/**
 * @brief 最速跟踪微分器(ADRC-TD核心函数)
 * @param x1 位置误差
 * @param x2 速度误差
 * @param r 跟踪速度因子,越大响应越快
 * @param h 采样时间(秒)
 * @return 最优加速度控制量
 * @note 实现快速无超调跟踪,韩京清教授经典算法
 */
static float fhan(float x1, float x2, float r, float h)
{
    float d, a0, y, a, a1, a2;
    d = r * powf(h, 2);
    a0 = h * x2;
    y = x1 + a0;
    a1 = sqrtf(d * (d + 8.0f * fabsf(y)));
    a2 = a0 + sign(y) * (a1 - d) / 2.0f;
    a = (a0 + y) * fsg(y, d) + a2 * (1.0f - fsg(y, d));
    return -r * (a / d) * fsg(y, d) - r * sign(a) * (1.0f - fsg(a, d));
}

/**
 * @brief 非线性幂次函数(ADRC-ESO/NLSEF核心函数)
 * @param e 误差输入
 * @param alpha 幂次指数(0~1),决定非线性特性
 * @param delta 线性区宽度,|e|<delta时为线性段,避免抖振
 * @return 非线性函数值
 * @note 小误差线性处理(减少抖动),大误差幂次放大(加快响应)
 *       alpha=0.5时类似sqrt,alpha=1.0时为线性
 */
static float fal(float e, float alpha, float delta)
{
    float result = 0.0f, fabsf_e = 0.0f;
    fabsf_e = fabsf(e);
    if (delta >= fabsf_e)
        result = e / powf(delta, 1.0f - alpha);
    else
        result = powf(fabsf_e, alpha) * sign(e);
    return result;
}

// ==================== 速度环PID实现 ====================

/**
 * @brief 初始化速度环PID控制器
 * @param pid PID结构体指针
 * @param kp 比例增益(Position)
 * @param ki 积分增益(Integral)
 * @param kd 微分增益(Derivative)
 * @param integral_max 积分限幅,防止积分饱和
 * @param output_max 输出限幅(PWM最大值)
 */
void speed_pid_init(SpeedPID* pid, float kp, float ki, float kd, 
                    float integral_max, float output_max)
{
    if (!pid) return;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->integral_max = integral_max;
    pid->output_max = output_max;
}

/**
 * @brief 重置PID控制器状态
 * @param pid PID结构体指针
 * @note 切换控制方案或重新启动时调用,清除积分残留
 */
void speed_pid_reset(SpeedPID* pid)
{
    if (!pid) return;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

/**
 * @brief PID控制器计算函数(增量式)
 * @param pid PID结构体指针
 * @param target 目标速度(m/s)
 * @param actual 实际速度(m/s),由编码器测量
 * @return PWM控制量,需直接输出到电机驱动
 * @note 位置式PID: u(k) = Kp·e(k) + Ki·Σe(k) + Kd·[e(k)-e(k-1)]
 */
float speed_pid_calculate(SpeedPID* pid, float target, float actual)
{
    if (!pid) return 0.0f;
    
    float error = target - actual;
    
    // 积分(抗饱和)
    pid->integral += error;
    pid->integral = clampf(pid->integral, -pid->integral_max, pid->integral_max);
    
    // 微分(一阶差分)
    float derivative = error - pid->prev_error;
    pid->prev_error = error;
    
    // PID输出
    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    
    // 输出限幅
    output = clampf(output, -pid->output_max, pid->output_max);
    
    return output;
}

// ==================== 方案一:串级PID ====================

/**
 * @brief 串级PID方案初始化
 * @note 控制结构: 图像误差 → 差速PID → 左右轮速度目标 → 速度PID → PWM
 *       外环:横向误差比例控制,仿照cam_err_calculation的非线性逻辑
 *       内环:双轮独立速度环PID
 */
static void cascade_pid_init(void)
{
    // 初始化差速映射参数(基于物理运动学模型)
    g_control.cascade_diff_params.half_track = 0.065f;      // 半轮距(m) - 测量值,如13cm轮距则=0.065
    g_control.cascade_diff_params.speed_ratio = 10.0f;      // 误差→角速度增益(rad/s per 千分之一),建议5~15
    g_control.cascade_diff_params.max_diff_speed = 1.5f;    // 最大差速(m/s)
    g_control.cascade_diff_params.smooth_alpha = 0.20f;     // 平滑系数,仿照变化率限幅思想
    
    // 非线性增益参数(高级特性,默认禁用)
    g_control.cascade_diff_params.enable_nonlinear = 0;     // 0=禁用, 1=启用
    g_control.cascade_diff_params.nonlinear_k = 1e-6f;      // 非线性系数(仿照cam_err的1e-7)
    
    memset(&g_control.cascade_diff_state, 0, sizeof(DiffMapState));
    
    // 初始化左右轮速度环PID参数
    speed_pid_init(&g_control.cascade_speed_left, 
                   10.0f,   // kp
                   0.5f,    // ki
                   0.1f,    // kd
                   100.0f,  // integral_max
                   1000.0f);// output_max (PWM)
    
    speed_pid_init(&g_control.cascade_speed_right, 
                   10.0f,   // kp
                   0.5f,    // ki
                   0.1f,    // kd
                   100.0f,  // integral_max
                   1000.0f);// output_max (PWM)
}

/**
 * @brief 串级PID控制主函数
 * @param v_forward 前进基准速度(m/s)
 * @param vL_actual 左轮实际速度(m/s),编码器反馈
 * @param vR_actual 右轮实际速度(m/s),编码器反馈
 * @param pwm_L_out 左轮PWM输出指针
 * @param pwm_R_out 右轮PWM输出指针
 * @note 执行流程:
 *       1. cam_err_calculation() 获取横向误差
 *       2. [可选]非线性增益处理: output=(1+K·err²)·err
 *       3. 物理映射: 误差→角速度→差速(基于运动学)
 *       4. 差速+前进速度 → 左右轮目标速度
 *       5. 双轮独立PID → PWM输出
 */
static void cascade_pid_control(float v_forward, 
                                float vL_actual, 
                                float vR_actual,
                                float* pwm_L_out, 
                                float* pwm_R_out)
{
    // 外环:图像误差 → 差速目标
    // 调用cam_err_calculation()获取误差(仿照原逻辑)
    //float err = cam_err_calculation();
    float err = straight_error_get();
    // 【可选】非线性增益处理(仿照cam_err_calculation的二次项)
    // output = (1 + K·err²)·err, 小误差线性,大误差增强
    float err_processed = err;
    if (g_control.cascade_diff_params.enable_nonlinear) {
        float K = g_control.cascade_diff_params.nonlinear_k;
        err_processed = (1.0f + K * err * err) * err;
    }
    
    // 物理模型映射公式:
    // cam_err = AngleLeft + AngleRight (左右切线斜率之和,单位:千分之一弧度)
    // 将切线斜率误差转换为期望角速度 ω (rad/s)
    // ω = K_err × err / 1000 (除以1000是因为cam_err的单位是千分之一)
    // 再转换为差速: Δv = ω × L (L为半轮距)
    float half_track = g_control.cascade_diff_params.half_track;
    float err_to_omega = g_control.cascade_diff_params.speed_ratio; // 误差→角速度增益
    
    // 角速度 = 误差增益 × 归一化误差
    float omega = err_to_omega * (err_processed );
    
    // 差速 = 角速度 × 半轮距 (根据运动学: vL=v-ω·L, vR=v+ω·L)
    float speed_diff = omega * half_track;
    
    // 限幅(仿照cam_err的cam_limit)
    speed_diff = clampf(speed_diff, 
                       -g_control.cascade_diff_params.max_diff_speed,
                       g_control.cascade_diff_params.max_diff_speed);
    
    // 差速 → 左右轮目标速度
    // speed_diff > 0:右转,右轮快,左轮慢
    // speed_diff < 0:左转,左轮快,右轮慢
    float vL_target_raw = v_forward - speed_diff * 0.5f;
    float vR_target_raw = v_forward + speed_diff * 0.5f;
    
    // 平滑输出(仿照cam_err的变化率限幅思想)
    float alpha = g_control.cascade_diff_params.smooth_alpha;
    g_control.vL_target = g_control.cascade_diff_state.vL_prev + 
                          alpha * (vL_target_raw - g_control.cascade_diff_state.vL_prev);
    g_control.vR_target = g_control.cascade_diff_state.vR_prev + 
                          alpha * (vR_target_raw - g_control.cascade_diff_state.vR_prev);
    
    g_control.cascade_diff_state.vL_prev = g_control.vL_target;
    g_control.cascade_diff_state.vR_prev = g_control.vR_target;
    
    // 内环:速度环PID → PWM输出
    g_control.pwm_L = speed_pid_calculate(&g_control.cascade_speed_left,
                                          g_control.vL_target,
                                          vL_actual);
    
    g_control.pwm_R = speed_pid_calculate(&g_control.cascade_speed_right,
                                          g_control.vR_target,
                                          vR_actual);
    
    *pwm_L_out = g_control.pwm_L;
    *pwm_R_out = g_control.pwm_R;
}

// ==================== 方案二:ADRC ====================

/**
 * @brief ADRC方案初始化
 * @note 控制结构: 图像误差 → ADRC → 差速控制量 → 左右轮速度目标 → 速度PID → PWM
 *       外环:横向误差ADRC自抗扰控制,自动补偿扰动
 *       内环:双轮独立速度环PID
 *       核心优势:无需精确模型,自动估计和补偿总扰动(摩擦、滑移、坡度等)
 */
static void adrc_init(void)
{
    // 初始化ADRC参数(基于差速小车运动学模型)
    // 【必须标定的参数】
    // 物理建模: 输入u为角速度ω(rad/s), 输出y为横向误差err
    // 简化模型: err' = -K·ω, 其中K是系统响应增益
    // ADRC的b参数: b ≈ 系统响应增益(根据开环测试标定)
    g_control.adrc_params.b = 5.0f;         // 系统增益(需开环标定),建议3~10
    g_control.adrc_params.h = 0.001f;       // 采样时间(秒)
    g_control.adrc_params.u_max = 15.0f;    // 最大角速度(rad/s),约860度/秒
    
    // 【可调节参数】(有默认值)
    g_control.adrc_params.r = 50.0f;        // TD跟踪速度
    g_control.adrc_params.w0 = 20.0f;       // ESO带宽
    g_control.adrc_params.kp = 1.0f;        // 位置增益
    g_control.adrc_params.kd = 0.5f;        // 速度增益
    g_control.adrc_params.delta = 0.01f;    // fal线性区宽度
    g_control.adrc_params.a1 = 0.5f;        // fal指数(位置)
    g_control.adrc_params.a2 = 1.5f;        // fal指数(速度)
    
    // 初始化ADRC状态
    memset(&g_control.adrc_state, 0, sizeof(ADRC_State));
    
    // 初始化左右轮速度环PID参数
    speed_pid_init(&g_control.adrc_speed_left, 
                   10.0f,   // kp
                   0.5f,    // ki
                   0.1f,    // kd
                   100.0f,  // integral_max
                   1000.0f);// output_max (PWM)
    
    speed_pid_init(&g_control.adrc_speed_right, 
                   10.0f,   // kp
                   0.5f,    // ki
                   0.1f,    // kd
                   100.0f,  // integral_max
                   1000.0f);// output_max (PWM)
}

/**
 * @brief ADRC控制器核心算法(三阶ADRC)
 * @param y 系统输出(当前横向误差)
 * @param v 期望输出(横向误差期望=0)
 * @return 控制量u(差速,m/s)
 * @note 三大核心模块:
 *       1. TD(跟踪微分器):安排过渡过程,提取微分信号
 *       2. ESO(扩张状态观测器):估计系统状态+总扰动
 *       3. NLSEF(非线性状态误差反馈):非线性PD+扰动补偿
 *       数学模型: y'' = b·u + f (f为总扰动)
 *                u = u0 - f_est/b (扰动主动补偿)
 */
static float adrc_control_core(float y, float v)
{
    ADRC_Params* p = &g_control.adrc_params;
    ADRC_State* s = &g_control.adrc_state;
    
    float u0 = 0.0f;
    float e1 = 0.0f;
    float e2 = 0.0f;
    float e = 0.0f;
    
    // 计算ESO带宽参数(3阶Butterworth配置)
    float belta01 = 3.0f * p->w0;
    float belta02 = 3.0f * p->w0 * p->w0;
    float belta03 = p->w0 * p->w0 * p->w0;
    
    // ==================== TD(跟踪微分器) ====================
    // 输入:期望值v, 输出:过渡过程x1及其微分x2
    // 作用:快速无超调跟踪,避免目标突变引起振荡
    s->x1 = s->x1 + p->h * s->x2;
    s->x2 = s->x2 + p->h * fhan(s->x1 - v, s->x2, p->r, p->h);
    
    // ==================== ESO(扩张状态观测器) ====================
    // 观测状态: z1≈y(位置), z2≈y'(速度), z3≈f(总扰动)
    // 作用:实时估计系统状态和扰动,无需建立精确数学模型
    e = s->z1 - y;  // 观测误差
    
    s->z1 = s->z1 + p->h * (s->z2 - belta01 * e);
    s->z2 = s->z2 + p->h * (s->z3 - belta02 * fal(e, 0.5f, p->delta) + p->b * s->u_last);
    s->z3 = s->z3 + p->h * (-belta03 * fal(e, 0.25f, p->delta));  // 扰动估计
    
    // ==================== NLSEF(非线性状态误差反馈) ====================
    e1 = s->x1 - s->z1;  // 位置误差(期望-观测)
    e2 = s->x2 - s->z2;  // 速度误差(期望-观测)
    
    // 非线性PD控制(小误差精细,大误差快速)
    u0 = p->kp * fal(e1, p->a1, p->delta) + p->kd * fal(e2, p->a2, p->delta);
    
    // 加上扰动补偿(ADRC核心思想:主动抗扰)
    float u = u0 - s->z3 / p->b;
    
    // 输出限幅
    u = clampf(u, -p->u_max, p->u_max);
    
    // 保存控制量供下次ESO使用
    s->u_last = u;
    
    return u;
}

/**
 * @brief ADRC控制主函数
 * @param v_forward 前进基准速度(m/s)
 * @param vL_actual 左轮实际速度(m/s),编码器反馈
 * @param vR_actual 右轮实际速度(m/s),编码器反馈
 * @param pwm_L_out 左轮PWM输出指针
 * @param pwm_R_out 右轮PWM输出指针
 * @note 执行流程:
 *       1. cam_err_calculation() 获取横向误差
 *       2. ADRC控制器:自动估计扰动并补偿 → 差速控制量
 *       3. 差速+前进速度 → 左右轮目标速度
 *       4. 双轮独立PID → PWM输出
 *       优势:对模型误差、参数变化、外界扰动具有强鲁棒性
 */
static void adrc_control(float v_forward, 
                        float vL_actual, 
                        float vR_actual,
                        float* pwm_L_out, 
                        float* pwm_R_out)
{
    // 外环:图像误差 → ADRC → 差速控制量
    // 调用cam_err_calculation()获取误差
    float err = cam_err_calculation();
    
    // 调用ADRC控制器
    // y = err:当前横向误差(系统输出,范围约±200)
    // v = 0:期望横向误差为0(目标)
    // 返回:角速度控制量(rad/s)
    float omega = adrc_control_core(err, 0.0f);
    
    // ADRC输出的是角速度,需要转换为差速
    // 注意:ADRC的b参数物理意义是 b ≈ 1/(响应时间常数)
    // 这里的 u_max 已经是角速度单位,直接用于差速计算
    float speed_diff = omega;
    
    // 差速 → 左右轮目标速度
    g_control.vL_target = v_forward - speed_diff * 0.5f;
    g_control.vR_target = v_forward + speed_diff * 0.5f;
    
    // 限幅(防止一侧速度过大或反转)
    float vmax = g_control.adrc_params.u_max + v_forward;
    g_control.vL_target = clampf(g_control.vL_target, -vmax, vmax);
    g_control.vR_target = clampf(g_control.vR_target, -vmax, vmax);
    
    // 内环:速度环PID → PWM输出
    g_control.pwm_L = speed_pid_calculate(&g_control.adrc_speed_left,
                                          g_control.vL_target,
                                          vL_actual);
    
    g_control.pwm_R = speed_pid_calculate(&g_control.adrc_speed_right,
                                          g_control.vR_target,
                                          vR_actual);
    
    *pwm_L_out = g_control.pwm_L;
    *pwm_R_out = g_control.pwm_R;
}

// ==================== 对外接口实现 ====================

/**
 * @brief 控制系统初始化
 * @param scheme 控制方案选择
 *               - CONTROL_SCHEME_CASCADE_PID: 串级PID(简单易调)
 *               - CONTROL_SCHEME_ADRC: 自抗扰控制(鲁棒性强)
 * @note 在main函数开始时调用一次,选择一种方案初始化
 *       两种方案独立运行,互不干扰
 */
void control_init(ControlScheme scheme)
{
    memset(&g_control, 0, sizeof(ControlState));
    g_control.active_scheme = scheme;
    
    switch (scheme) {
        case CONTROL_SCHEME_CASCADE_PID:
            cascade_pid_init();
            break;
        
        case CONTROL_SCHEME_ADRC:
            adrc_init();
            break;
        
        default:
            // 默认使用串级PID
            g_control.active_scheme = CONTROL_SCHEME_CASCADE_PID;
            cascade_pid_init();
            break;
    }
}

/**
 * @brief 运行时切换控制方案
 * @param scheme 目标控制方案
 * @note 切换时自动重置所有状态(PID积分、ADRC观测器等),避免残留影响
 *       可在按键中断或上位机指令中调用,实现动态切换
 */
void control_switch_scheme(ControlScheme scheme)
{
    if (scheme == g_control.active_scheme) {
        return;  // 已经是当前方案,无需切换
    }
    
    // 重置PID状态,避免积分残留
    speed_pid_reset(&g_control.cascade_speed_left);
    speed_pid_reset(&g_control.cascade_speed_right);
    speed_pid_reset(&g_control.adrc_speed_left);
    speed_pid_reset(&g_control.adrc_speed_right);
    
    // 重置差速映射状态
    memset(&g_control.cascade_diff_state, 0, sizeof(DiffMapState));
    
    // 重置ADRC状态
    memset(&g_control.adrc_state, 0, sizeof(ADRC_State));
    
    g_control.active_scheme = scheme;
    
    switch (scheme) {
        case CONTROL_SCHEME_CASCADE_PID:
            cascade_pid_init();
            break;
        
        case CONTROL_SCHEME_ADRC:
            adrc_init();
            break;
        
        default:
            break;
    }
}

/**
 * @brief 控制主循环(周期性调用)
 * @param v_forward 前进基准速度(m/s),通常固定值如1.0m/s
 * @param vL_actual 左轮实际速度(m/s),由编码器测量并计算得到
 * @param vR_actual 右轮实际速度(m/s),由编码器测量并计算得到
 * @param pwm_L_out 左轮PWM输出指针,范围建议[-1000,1000]
 * @param pwm_R_out 右轮PWM输出指针,范围建议[-1000,1000]
 * 
 * @note 调用频率:建议1ms(1000Hz),必须与ADRC参数h一致
 * 
 * @note 使用示例:
 *       while(1) {
 *           float vL = get_left_speed();   // 读编码器
 *           float vR = get_right_speed();
 *           float pwm_L, pwm_R;
 *           
 *           control_loop(1.0f, vL, vR, &pwm_L, &pwm_R);
 *           
 *           set_motor_pwm(pwm_L, pwm_R);   // 输出PWM
 *           delay_ms(1);  // 1ms周期
 *       }
 * 
 * @note 内部执行流程:
 *       图像采集 → cam_err_calculation() → 外环控制 → 差速分配 → 内环PID → PWM
 */
void control_loop(float v_forward, 
                  float vL_actual, 
                  float vR_actual,
                  float* pwm_L_out, 
                  float* pwm_R_out)
{
    if (!pwm_L_out || !pwm_R_out) {
        return;
    }
    
    switch (g_control.active_scheme) {
        case CONTROL_SCHEME_CASCADE_PID:
            cascade_pid_control(v_forward, vL_actual, vR_actual, pwm_L_out, pwm_R_out);
            break;
        
        case CONTROL_SCHEME_ADRC:
            adrc_control(v_forward, vL_actual, vR_actual, pwm_L_out, pwm_R_out);
            break;
        
        default:
            // 安全保护:输出0
            *pwm_L_out = 0.0f;
            *pwm_R_out = 0.0f;
            break;
    }
}


// ==================== 便捷调参接口实现 ==================== 

CascadePIDConfig cascade_pid_config;

/**
 * @brief 使用默认参数初始化串级PID
 * @note 最简单的初始化方式,适合快速测试
 */
void control_init_cascade_pid_default(void)
{
    memset(&g_control, 0, sizeof(ControlState));
    g_control.active_scheme = CONTROL_SCHEME_CASCADE_PID;
    cascade_pid_init();
}

/**
 * @brief 使用自定义参数初始化串级PID
 * @param config 全部参数配置结构体指针
 * @note 允许完全自定义所有参数,适合精细调参
 */
void control_init_cascade_pid(const CascadePIDConfig* config)
{
    if (!config) {
        control_init_cascade_pid_default();
        return;
    }
    
    memset(&g_control, 0, sizeof(ControlState));
    g_control.active_scheme = CONTROL_SCHEME_CASCADE_PID;
    
    // 配置外环差速映射参数
    g_control.cascade_diff_params.half_track = config->half_track;
    g_control.cascade_diff_params.speed_ratio = config->speed_ratio;
    g_control.cascade_diff_params.max_diff_speed = config->max_diff_speed;
    g_control.cascade_diff_params.smooth_alpha = config->smooth_alpha;
    g_control.cascade_diff_params.enable_nonlinear = config->enable_nonlinear;
    g_control.cascade_diff_params.nonlinear_k = config->nonlinear_k;
    
    memset(&g_control.cascade_diff_state, 0, sizeof(DiffMapState));
    
    // 配置内环速度PID(左轮)
    speed_pid_init(&g_control.cascade_speed_left, 
                   config->speed_left_kp,
                   config->speed_left_ki,
                   config->speed_left_kd,
                   config->integral_max,
                   config->output_max);
    
    // 配置内环速度PID(右轮)
    speed_pid_init(&g_control.cascade_speed_right, 
                   config->speed_right_kp,
                   config->speed_right_ki,
                   config->speed_right_kd,
                   config->integral_max,
                   config->output_max);
}
/**
 * @brief 使用自定义参数初始化串级PID配置结构体
 * @param config 参数配置结构体指针 lkp,lki,lkd:左轮PID参数 rkp,rki,rkd:右轮PID参数
 * @note 允许完全自定义所有参数,适合精细调参
 */
void control_init_cascade_pid_config(CascadePIDConfig* config,float lkp,float lki,float lkd,
                                     float rkp,float rki,float rkd)
{
    if (config==NULL) {
        return;
    }
    
    // 配置外环差速映射参数
    config->half_track = Half_track;      // 半轮距(m)
    config->speed_ratio = Speed_ratio;      // 误差→角速度增益(rad/s per 千分之一)
    config->max_diff_speed = Max_diff_speed;    // 最大差速(m/s)
    config->smooth_alpha = Smooth_alpha;     // 平滑系数
    config->enable_nonlinear = 0;     // 非线性增益 0=禁用, 1=启用
    config->nonlinear_k = 1e-6f;      // 非线性系数
    
    // 配置内环速度PID参数(左轮)
    config->speed_left_kp = lkp;
    config->speed_left_ki = lki;
    config->speed_left_kd = lkd;
    
    // 配置内环速度PID参数(右轮)
    config->speed_right_kp = rkp;
    config->speed_right_ki = rki;
    config->speed_right_kd = rkd;
    
    config->integral_max = Integral_max;   // 积分限幅
    config->output_max = Output_max;     // 输出限幅 (PWM)

}

/**
 * @brief 使用默认参数初始化ADRC
 * @note 最简单的初始化方式,适合快速测试
 */
void control_init_adrc_default(void)
{
    memset(&g_control, 0, sizeof(ControlState));
    g_control.active_scheme = CONTROL_SCHEME_ADRC;
    adrc_init();
}

/**
 * @brief 使用自定义参数初始化ADRC
 * @param config 参数配置结构体指针
 * @note 允许完全自定义所有参数,适合精细调参
 */
void control_init_adrc(const ADRCConfig* config)
{
    if (!config) {
        control_init_adrc_default();
        return;
    }
    
    memset(&g_control, 0, sizeof(ControlState));
    g_control.active_scheme = CONTROL_SCHEME_ADRC;
    
    // 配置ADRC参数
    g_control.adrc_params.b = config->b;
    g_control.adrc_params.h = config->h;
    g_control.adrc_params.u_max = config->u_max;
    g_control.adrc_params.r = config->r;
    g_control.adrc_params.w0 = config->w0;
    g_control.adrc_params.kp = config->kp;
    g_control.adrc_params.kd = config->kd;
    g_control.adrc_params.delta = config->delta;
    g_control.adrc_params.a1 = config->a1;
    g_control.adrc_params.a2 = config->a2;
    
    memset(&g_control.adrc_state, 0, sizeof(ADRC_State));
    
    // 配置内环速度PID(左轮)
    speed_pid_init(&g_control.adrc_speed_left, 
                   config->speed_left_kp,
                   config->speed_left_ki,
                   config->speed_left_kd,
                   config->integral_max,
                   config->output_max);
    
    // 配置内环速度PID(右轮)
    speed_pid_init(&g_control.adrc_speed_right, 
                   config->speed_right_kp,
                   config->speed_right_ki,
                   config->speed_right_kd,
                   config->integral_max,
                   config->output_max);
}

/**
 * @brief 在线修改串级PID外环参数
 * @param speed_ratio 误差→角速度增益
 * @param max_diff_speed 最大差速(m/s)
 * @param smooth_alpha 平滑系数
 * @note 允许运行时动态调节,无需重启
 */
void control_update_cascade_outer(float speed_ratio, float max_diff_speed, float smooth_alpha)
{
    if (g_control.active_scheme != CONTROL_SCHEME_CASCADE_PID) {
        return; // 只在串级PID模式下有效
    }
    
    g_control.cascade_diff_params.speed_ratio = speed_ratio;
    g_control.cascade_diff_params.max_diff_speed = max_diff_speed;
    g_control.cascade_diff_params.smooth_alpha = smooth_alpha;
}

/**
 * @brief 在线修改串级PID速度环参数
 * @param kp_left 左轮比例系数
 * @param ki_left 左轮积分系数
 * @param kd_left 左轮微分系数
 * @param kp_right 右轮比例系数
 * @param ki_right 右轮积分系数
 * @param kd_right 右轮微分系数
 * @note 允许运行时动态调节,修改后自动重置PID状态
 */
void control_update_cascade_speed_pid(float kp_left, float ki_left, float kd_left,
                                      float kp_right, float ki_right, float kd_right)
{
    if (g_control.active_scheme != CONTROL_SCHEME_CASCADE_PID) {
        return; // 只在串级PID模式下有效
    }
    
    // 更新左轮PID参数
    g_control.cascade_speed_left.kp = kp_left;
    g_control.cascade_speed_left.ki = ki_left;
    g_control.cascade_speed_left.kd = kd_left;
    speed_pid_reset(&g_control.cascade_speed_left);
    
    // 更新右轮PID参数
    g_control.cascade_speed_right.kp = kp_right;
    g_control.cascade_speed_right.ki = ki_right;
    g_control.cascade_speed_right.kd = kd_right;
    speed_pid_reset(&g_control.cascade_speed_right);
}

/**
 * @brief 在线修改ADRC核心参数
 * @param b 系统增益
 * @param w0 ESO带宽
 * @param r TD跟踪速度
 * @param kp 位置增益
 * @param kd 速度增益
 * @note 允许运行时动态调节,修改后自动重置ADRC状态
 */
void control_update_adrc_core(float b, float w0, float r, float kp, float kd)
{
    if (g_control.active_scheme != CONTROL_SCHEME_ADRC) {
        return; // 只在ADRC模式下有效
    }
    
    g_control.adrc_params.b = b;
    g_control.adrc_params.w0 = w0;
    g_control.adrc_params.r = r;
    g_control.adrc_params.kp = kp;
    g_control.adrc_params.kd = kd;
    
    // 重置ADRC状态
    memset(&g_control.adrc_state, 0, sizeof(ADRC_State));
}

/**
 * @brief 启用/禁用非线性增益
 * @param enable 0=禁用, 1=启用
 * @param nonlinear_k 非线性系数(仅在enable=1时有效)
 * @note 串级PID专用,允许运行时切换
 */
void control_set_nonlinear_gain(int enable, float nonlinear_k)
{
    if (g_control.active_scheme != CONTROL_SCHEME_CASCADE_PID) {
        return; // 只在串级PID模式下有效
    }
    
    g_control.cascade_diff_params.enable_nonlinear = enable;
    if (enable) {
        g_control.cascade_diff_params.nonlinear_k = nonlinear_k;
    }
}

/*-----------------编码器配套--------------------*/
float get_speed(void)
{
    control.left_speed = 10.0f * (float)(control.lencoder_count - control.lencoder_count_last) * 6.28f * Radius / Encoder_PPR;
    control.right_speed = 10.0f * (float)(control.rencoder_count - control.rencoder_count_last) * 6.28f * Radius / Encoder_PPR;
    return (control.left_speed + control.right_speed) / 2.0f; // 返回平均速度 近似车身速度
}


// ==================== 完整控制链路实现 ====================

/**
 * @brief 完整控制流程(图像→误差→控制→输出)
 * @param v_forward 前进基准速度(m/s)
 * @note 执行流程:
 *       1. 使用 get_speed() 获取编码器速度
 *       2. 调用 control_loop() 执行控制算法
 *       3. 将PWM输出写入 control.left_target_speed / control.right_target_speed
 *       4. 设置方向标志 control.left_dir / control.right_dir
 * @note 外部调用: 在定时器中断或主循环中周期性调用(建议1ms)
 * @note 配合使用: motor_run(&leftmotor, control.left_target_speed)
 */
void control_execute(float v_forward)
{
    // 1. 获取编码器速度(已通过get_speed更新control.left_speed和control.right_speed)
    float vL_actual = control.left_speed;
    float vR_actual = control.right_speed;
    
    // 2. 执行控制循环(内部调用cam_err_calculation)
    float pwm_L, pwm_R;
    control_loop(v_forward, vL_actual, vR_actual, &pwm_L, &pwm_R);
    
    // 3. 将PWM输出写入control结构体(供motor模块使用)
    // PWM转换: float → int16_t, 并设置方向
    if (pwm_L >= 0) {
        control.left_target_speed = (int16_t)pwm_L;
        control.left_dir = 1;  // 正向
    } else {
        control.left_target_speed = (int16_t)(-pwm_L);
        control.left_dir = 0;  // 反向
    }
    
    if (pwm_R >= 0) {
        control.right_target_speed = (int16_t)pwm_R;
        control.right_dir = 1;  // 正向
    } else {
        control.right_target_speed = (int16_t)(-pwm_R);
        control.right_dir = 0;  // 反向
    }
}

// ==================== 使用说明 ====================
/*
 * cam_err_calculation() 使用流程:
 * 
 * 1. 初始化(在main函数中调用一次):
 *    cam_err_init(20, 80, 10, 100);  // 设置打角范围和距离阈值
 * 
 * 2. 每帧图像处理后,填充 g_cam_err_data.lineinfo[]:
 *    for (int y = 0; y < 120; y++) {
 *        g_cam_err_data.lineinfo[y].angel_left = ...;    // 左切线斜率(千分之一)
 *        g_cam_err_data.lineinfo[y].angel_right = ...;   // 右切线斜率(千分之一)
 *        g_cam_err_data.lineinfo[y].persp_ly = ...;      // 左边线y坐标
 *        g_cam_err_data.lineinfo[y].persp_ry = ...;      // 右边线y坐标
 *        g_cam_err_data.lineinfo[y].left_lost = ...;     // 左线丢失标志
 *        g_cam_err_data.lineinfo[y].right_lost = ...;    // 右线丢失标志
 *    }
 * 
 * 3. 在控制循环中调用(已集成到control_loop中):
 *    float err = cam_err_calculation();  // 获取横向误差
 * 
 * 4. 可选:调整非线性增益(高级特性):
 *    g_cam_err_data.params.SteerKpchange = 1.0f;  // 非线性系数,0为禁用
 * 
 * 注意事项:
 * - lineinfo[y].angel_left/right 单位是千分之一(如1000表示斜率1.0)
 * - 返回值范围约±200,已包含非线性增益
 * - 需要外部图像处理模块提供 lineinfo 数据
 */
