#include <stdio.h> // 用于 NULL
#include <math.h>  // 用于 fabsf

#include "Kalman.h"
#include "Element_recognition.h"
CurveParams_t stable_curve_params={0.0f,0.0f,0.0f };

/**
 * @brief  对中线进行二次曲线拟合，并使用卡尔曼滤波器进行平滑。
 * @param  lineinfo      指向存储中线点的数组的指针。
 * @param  point_count   实际有效的中线点数量。
 * @return CurveParams_t 滤波后得到的稳定曲线参数 {a, b, c}。
 */
CurveParams_t ProcessLineWithKalman(struct lineinfo_s *lineinfo, int point_count)
{
    // 使用静态变量存储卡尔曼滤波器的状态，使其在函数调用之间持久存在
    static KalmanState_CurveFit_t kf_state;
    
    // ------------------ 1. 卡尔曼滤波器初始化 (仅在第一次运行时执行) ------------------
    if (!kf_state.is_initialized)
    {
        // 状态向量 X 初始化为0
        kf_state.X[0] = 0.0f; kf_state.X[1] = 0.0f; kf_state.X[2] = 0.0f;
        
        // 状态协方差矩阵 P 初始化为单位矩阵，表示初始状态不确定性较大
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                kf_state.P[i][j] = (i == j) ? 1.0f : 0.0f;
            }
        }
        
        // --- [关键调参区] ---
        // 过程噪声 Q: 模型预测的不确定性。值越小，越相信上一时刻的预测，曲线越平滑但响应越慢。
        float q_val_a = 0.01f; // a参数(曲率)变化噪声
        float q_val_b = 0.1f;  // b参数(角度)变化噪声
        float q_val_c = 10.0f;    // c参数(偏移)变化噪声
        kf_state.Q[0][0] = q_val_a; kf_state.Q[0][1] = 0; kf_state.Q[0][2] = 0;
        kf_state.Q[1][0] = 0; kf_state.Q[1][1] = q_val_b; kf_state.Q[1][2] = 0;
        kf_state.Q[2][0] = 0; kf_state.Q[2][1] = 0; kf_state.Q[2][2] = q_val_c;

        // 测量噪声 R: 最小二乘法拟合结果的不确定性。值越小，越相信当前的拟合结果，曲线响应快但噪声大。
        float r_val = 0.05f;
        kf_state.R[0][0] = r_val; kf_state.R[0][1] = 0; kf_state.R[0][2] = 0;
        kf_state.R[1][0] = 0; kf_state.R[1][1] = r_val; kf_state.R[1][2] = 0;
        kf_state.R[2][0] = 0; kf_state.R[2][1] = 0; kf_state.R[2][2] = r_val;
        // --- [调参区结束] ---

        kf_state.is_initialized = 1;
    }
    
    CurveParams_t measured_params = {0, 0, 0};

    // ------------------ 2. 最小二乘法拟合 (获取"测量值") ------------------
    // 至少需要3个点才能拟合二次曲线
    if (lineinfo != NULL && point_count >= 3)
    {
        float sum_x = 0, sum_y = 0, sum_y2 = 0, sum_y3 = 0, sum_y4 = 0;
        float sum_xy = 0, sum_xy2 = 0;

        for (int i = 0; i < point_count; ++i)
        {
            float x = (float)lineinfo[i].mid;
            float y = (float)lineinfo[i].y;
            float y2 = y * y;
            
            sum_x += x;
            sum_y += y;
            sum_y2 += y2;
            sum_y3 += y2 * y;
            sum_y4 += y2 * y2;
            sum_xy += x * y;
            sum_xy2 += x * y2;
        }

        float N = (float)point_count;
        float A[3][3] = {
            {sum_y4, sum_y3, sum_y2},
            {sum_y3, sum_y2, sum_y},
            {sum_y2, sum_y,  N}
        };
        float B[3] = {sum_xy2, sum_xy, sum_x};

        // 求解 3x3 线性方程组 Ax=B (使用克莱姆法则)
        float det_A = A[0][0]*(A[1][1]*A[2][2] - A[1][2]*A[2][1]) - 
                      A[0][1]*(A[1][0]*A[2][2] - A[1][2]*A[2][0]) + 
                      A[0][2]*(A[1][0]*A[2][1] - A[1][1]*A[2][0]);
                      
        // 避免除以0
        if (fabsf(det_A) > 1e-6)
        {
            float inv_det_A = 1.0f / det_A;
            // 计算伴随矩阵与B的乘积
            measured_params.a = inv_det_A * (B[0]*(A[1][1]*A[2][2]-A[1][2]*A[2][1]) - B[1]*(A[0][1]*A[2][2]-A[0][2]*A[2][1]) + B[2]*(A[0][1]*A[1][2]-A[0][2]*A[1][1]));
            measured_params.b = inv_det_A * -(B[0]*(A[1][0]*A[2][2]-A[1][2]*A[2][0]) - B[1]*(A[0][0]*A[2][2]-A[0][2]*A[2][0]) + B[2]*(A[0][0]*A[1][2]-A[0][2]*A[1][0]));
            measured_params.c = inv_det_A * (B[0]*(A[1][0]*A[2][1]-A[1][1]*A[2][0]) - B[1]*(A[0][0]*A[2][1]-A[0][1]*A[2][0]) + B[2]*(A[0][0]*A[1][1]-A[0][1]*A[1][0]));
        }
    }
    // 如果点数不够，则测量值使用上一时刻的预测值，避免突变
    else
    {
        measured_params.a = kf_state.X[0];
        measured_params.b = kf_state.X[1];
        measured_params.c = kf_state.X[2];
    }
    
    // ------------------ 3. 卡尔曼滤波: 预测与更新 ------------------
    // A=I, H=I 的简化模型
    
    // 1. 预测
    // X_pred = X; (状态预测不变)
    float P_pred[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            P_pred[i][j] = kf_state.P[i][j] + kf_state.Q[i][j];
        }
    }
    
    // 2. 更新
    float S[3][3]; // S = P_pred + R
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            S[i][j] = P_pred[i][j] + kf_state.R[i][j];
        }
    }
    
    // 计算 S 的逆 S_inv
    float det_S = S[0][0]*(S[1][1]*S[2][2]-S[1][2]*S[2][1]) - S[0][1]*(S[1][0]*S[2][2]-S[1][2]*S[2][0]) + S[0][2]*(S[1][0]*S[2][1]-S[1][1]*S[2][0]);
    if (fabsf(det_S) > 1e-6)
    {
        float inv_det_S = 1.0f / det_S;
        float S_inv[3][3];
        S_inv[0][0] = (S[1][1]*S[2][2] - S[1][2]*S[2][1]) * inv_det_S;
        S_inv[0][1] = (S[0][2]*S[2][1] - S[0][1]*S[2][2]) * inv_det_S;
        S_inv[0][2] = (S[0][1]*S[1][2] - S[0][2]*S[1][1]) * inv_det_S;
        S_inv[1][0] = (S[1][2]*S[2][0] - S[1][0]*S[2][2]) * inv_det_S;
        S_inv[1][1] = (S[0][0]*S[2][2] - S[0][2]*S[2][0]) * inv_det_S;
        S_inv[1][2] = (S[0][2]*S[1][0] - S[0][0]*S[1][2]) * inv_det_S;
        S_inv[2][0] = (S[1][0]*S[2][1] - S[1][1]*S[2][0]) * inv_det_S;
        S_inv[2][1] = (S[0][1]*S[2][0] - S[0][0]*S[2][1]) * inv_det_S;
        S_inv[2][2] = (S[0][0]*S[1][1] - S[0][1]*S[1][0]) * inv_det_S;

        // K = P_pred * S_inv (卡尔曼增益)
        float K[3][3];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                K[i][j] = P_pred[i][0]*S_inv[0][j] + P_pred[i][1]*S_inv[1][j] + P_pred[i][2]*S_inv[2][j];
            }
        }

        // 更新状态 X
        float z[3] = {measured_params.a, measured_params.b, measured_params.c};
        for (int i = 0; i < 3; ++i) {
            kf_state.X[i] = kf_state.X[i] + K[i][0]*(z[0]-kf_state.X[0]) + K[i][1]*(z[1]-kf_state.X[1]) + K[i][2]*(z[2]-kf_state.X[2]);
        }
        
        // 更新协方差 P = (I - K) * P_pred
        float I_minus_K[3][3];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                I_minus_K[i][j] = ((i==j)?1.0f:0.0f) - K[i][j];
            }
        }
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                kf_state.P[i][j] = I_minus_K[i][0]*P_pred[0][j] + I_minus_K[i][1]*P_pred[1][j] + I_minus_K[i][2]*P_pred[2][j];
            }
        }
    }
    // 如果矩阵奇异，则不更新状态，防止计算错误
    
    // ------------------ 4. 准备返回值 ------------------
    CurveParams_t estimated_params;
    estimated_params.a = kf_state.X[0];
    estimated_params.b = kf_state.X[1];
    estimated_params.c = kf_state.X[2];

    return estimated_params;
}


/**
 * @brief  使用平滑的曲线参数，计算并填充lineinfo数组中的midpredict成员。
 * @param  curve         指向包含稳定曲线参数 {a, b, c} 的结构体指针。
 * @param  lineinfo      指向要填充的lineinfo数组的指针 (注意：不是const，因为我们要修改它)。
 * @param  array_size    lineinfo数组的总大小 (例如 120)。
 * @param  screen_width  屏幕的宽度 (例如 188)，用于边界检查。
 */
void PopulatePredictedLine(const CurveParams_t* curve, struct lineinfo_s* lineinfo, int screen_width, int array_size)
{
    if (curve == NULL || lineinfo == NULL) {
        return;
    }

    watch.PredictTopMidline = array_size - 1; // 预测中线最大行数

    float a = curve->a;
    float b = curve->b;
    float c = curve->c;

    // 遍历整个lineinfo数组的每一行
    for (int i = 0; i < array_size; ++i)
    {
        // lineinfo[i].y 存储了当前行的y坐标，我们用它来计算
        int y = lineinfo[i].y; 

        // 1. 根据公式计算x的精确浮点坐标
        float x_float = a * y * y + b * y + c;

        // 2. 四舍五入到最近的整数，这比直接强制类型转换更精确
        int x_predict = (int)roundf(x_float);
        
        // 3. (可选但强烈推荐) 边界裁剪，防止预测值超出屏幕范围
        if (x_predict <= 0) {
            x_predict = 0;
            watch.PredictTopMidline = y;
            return; // 触边立即返回
        } else if (x_predict >= screen_width) {
            x_predict = screen_width - 1;
            watch.PredictTopMidline = y;
            return; // 触边立即返回
        }


        // 4. 将计算出的安全、准确的预测值存入结构体
        lineinfo[i].midpredict = x_predict;
    }
}