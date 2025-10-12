#ifndef KALMAN_H_
#define KALMAN_H_
#include "scan_line.h"
// 曲线参数结构体 (用于函数返回值)
typedef struct
{
    float a; // 二次项系数
    float b; // 一次项系数
    float c; // 常数项
} CurveParams_t;

// 卡尔曼滤波器状态结构体 (用于存储持久化数据)
typedef struct
{
    // 状态向量 X = [a, b, c]^T
    float X[3];
    // 状态协方差矩阵 P (3x3)
    float P[3][3];
    // 过程噪声协方差矩阵 Q (3x3)
    float Q[3][3];
    // 测量噪声协方差矩阵 R (3x3)
    float R[3][3];
    // 初始化标志位
    int is_initialized;
} KalmanState_CurveFit_t;

CurveParams_t ProcessLineWithKalman(struct lineinfo_s *lineinfo, int point_count);
void PopulatePredictedLine(const CurveParams_t* curve, struct lineinfo_s* lineinfo, int screen_width, int array_size);
extern CurveParams_t stable_curve_params; // 用于存储滤波后的稳定曲线参数

#endif /* KALMAN_H_ */