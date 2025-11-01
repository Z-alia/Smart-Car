/*
 * control.c
 * 差速小车双控制方案实现
 * 仿照cam_err_calculation的逻辑设计
 */

#include "control.h"
#include <math.h>
#include <string.h>

// 全局控制器状态
ControlState g_control = {0};

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
    // 初始化差速映射参数(仿照cam_err_calculation的非线性逻辑)
    g_control.cascade_diff_params.half_track = 0.065f;      // 半轮距(m)
    g_control.cascade_diff_params.speed_ratio = 0.01f;      // 差速映射比例,类似cam_err的输出缩放
    g_control.cascade_diff_params.max_diff_speed = 1.5f;    // 最大差速(m/s)
    g_control.cascade_diff_params.smooth_alpha = 0.20f;     // 平滑系数,仿照变化率限幅思想
    
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
 *       2. 比例映射到差速控制量(仿照cam_err的非线性增益)
 *       3. 差速+前进速度 → 左右轮目标速度
 *       4. 双轮独立PID → PWM输出
 */
static void cascade_pid_control(float v_forward, 
                                float vL_actual, 
                                float vR_actual,
                                float* pwm_L_out, 
                                float* pwm_R_out)
{
    // 外环:图像误差 → 差速目标
    // 调用cam_err_calculation()获取误差(仿照原逻辑)
    float err = cam_err_calculation();
    
    // 仿照cam_err_calculation最后的非线性放大:
    // return (1.f + 1e-7f * SteerKpchange * angle_target^2) * angle_target
    // 这里简化处理,直接用比例映射到差速
    float speed_diff = err * g_control.cascade_diff_params.speed_ratio;
    
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
    // 初始化ADRC参数(仿照ADRC.c的逻辑)
    // 【必须标定的参数】
    g_control.adrc_params.b = 7.7f;         // 系统增益:b ≈ 1/轮距
    g_control.adrc_params.h = 0.001f;       // 采样时间(秒)
    g_control.adrc_params.u_max = 1.5f;     // 最大差速(m/s)
    
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
    // y = err:当前横向误差(系统输出)
    // v = 0:期望横向误差为0(目标)
    // 返回:差速控制量(m/s),正值=右转,负值=左转
    float speed_diff = adrc_control_core(err, 0.0f);
    
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
