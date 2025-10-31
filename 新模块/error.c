#include "error.h"
#include <math.h>

// 由原工程提供（err_calculation.c），此处仅做外部声明以复用其有效逻辑
extern float cam_err_calculation(void);
extern int   Steer_PWM_Cal(int track_pos);

static inline float clampf_local(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

// |a|<=~0.6rad（~34°）范围内的快速 tan 近似：tan(a)≈a + a^3/3
static inline float fast_tan_small(float a)
{
    float a2 = a * a;
    return a + (a * a2) * (1.0f / 3.0f);
}

void diff_map_from_servo_cmd_curvature(int steer_cmd,
                                       float v_forward,
                                       const DiffMapParams* p,
                                       DiffMapState* s,
                                       float* vL_out,
                                       float* vR_out)
{
    if (!p || !s || !vL_out || !vR_out) {
        return;
    }

    // 1) 指令→等效角度（rad）并限幅
    float a = clampf_local((float)steer_cmd * p->steer_to_rad_k,
                           -p->max_angle_rad, p->max_angle_rad);

    // 2) 角度→曲率 κ；小角快近似，大角限幅保护
    float abs_a = a > 0.f ? a : -a;
    float kappa;
    if (abs_a <= 0.6f) {
        kappa = fast_tan_small(a) / p->L_virt;
    } else {
        float kappa_max = fast_tan_small(p->max_angle_rad) / p->L_virt;
        kappa = (a > 0.f ? 1.f : -1.f) * kappa_max * (abs_a / p->max_angle_rad);
    }

    // 3) 曲率→角速度→左右轮速度
    float omega  = v_forward * kappa;
    float vL_cmd = v_forward - omega * p->half_track;
    float vR_cmd = v_forward + omega * p->half_track;

    // 4) 幅值归一化到 vmax，保持几何比例
    float m = fabsf(vL_cmd);
    float mr = fabsf(vR_cmd);
    if (mr > m) m = mr;
    if (m > p->vmax && m > 1e-6f) {
        float sc = p->vmax / m;
        vL_cmd *= sc;
        vR_cmd *= sc;
    }

    // 5) 输出平滑
    vL_cmd = s->vL_prev + p->smooth_alpha * (vL_cmd - s->vL_prev);
    vR_cmd = s->vR_prev + p->smooth_alpha * (vR_cmd - s->vR_prev);

    s->vL_prev = vL_cmd;
    s->vR_prev = vR_cmd;

    *vL_out = vL_cmd;
    *vR_out = vR_cmd;
}

void error_pipeline_curvature(float v_forward,
                              const DiffMapParams* p,
                              DiffMapState* s,
                              float* vL_out,
                              float* vR_out)
{
    if (!p || !s || !vL_out || !vR_out) {
        return;
    }

    // 1) 计算图像转向误差（来自原工程 cam_err_calculation）
    float err = cam_err_calculation();

    // 2) 由误差得到舵机指令（来自原工程 Steer_PWM_Cal）
    int steer_cmd = Steer_PWM_Cal((int)err);

    // 3) 曲率法映射到左右轮
    diff_map_from_servo_cmd_curvature(steer_cmd, v_forward, p, s, vL_out, vR_out);
}
