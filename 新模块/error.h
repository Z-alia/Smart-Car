#ifndef ERROR_H_
#define ERROR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// 差速映射参数（曲率法）
typedef struct {
    float half_track;       // b：半轮距（m 或等效单位）
    float L_virt;           // 虚拟轴距（m），用于“角度→曲率”
    float max_angle_rad;    // 等效最大转角（rad）
    float vmax;             // 左右轮最大目标速度（与整车速度单位一致）
    float smooth_alpha;     // 平滑系数 0..1（越大响应越快）
    float steer_to_rad_k;   // 舵机指令→角度的系数（rad/指令单位）
} DiffMapParams;

// 差速映射内部状态（用于平滑）
typedef struct {
    float vL_prev;          // 上一周期左轮目标
    float vR_prev;          // 上一周期右轮目标
} DiffMapState;

// 方案一：曲率法，将“舵机指令/转向强度”映射到左右轮目标速度
// 参数：
//   steer_cmd   - 舵机指令（例如 Steer_PWM_Cal 输出，范围约 ±500 或你的实际范围）
//   v_forward   - 目标前进速度（你的速度单位）
//   p           - 参数指针
//   s           - 状态（平滑记忆）
// 输出：
//   vL_out/vR_out - 左右轮目标速度
void diff_map_from_servo_cmd_curvature(int steer_cmd,
                                       float v_forward,
                                       const DiffMapParams* p,
                                       DiffMapState* s,
                                       float* vL_out,
                                       float* vR_out);

// 一体化流水线：
// 使用现有 cam_err_calculation 和 Steer_PWM_Cal（在原工程中定义）
// 先计算图像误差 -> 舵机指令，再用曲率法得到左右轮目标速度。
// 注意：本函数通过 extern 声明调用原文件中的实现，无需改动原逻辑。
void error_pipeline_curvature(float v_forward,
                              const DiffMapParams* p,
                              DiffMapState* s,
                              float* vL_out,
                              float* vR_out);

#ifdef __cplusplus
}
#endif

#endif // ERROR_H_
