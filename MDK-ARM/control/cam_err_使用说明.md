# cam_err_calculation() 移植使用说明

## 一、概述

已将原始的 `cam_err_calculation()` 函数逻辑完整移植到 `control.c` 中，保留了所有核心算法，但使用了 control 模块的数据结构。

## 二、核心算法逻辑

### 1. 切线斜率法
- **左切线**: 遍历图像行，寻找斜率最小的线（`AngleLeft`）
- **右切线**: 遍历图像行，寻找斜率最大的线（`AngleRight`）
- **原始误差**: `err = AngleLeft + AngleRight`（左右斜率之和）

### 2. 关键特性
- ✅ **变化率限幅**: 防止误差突变（`d_err_limit = 30`）
- ✅ **输出限幅**: 误差范围 `[-200, 200]`
- ✅ **丢线处理**: 统计连续丢线次数，超过10行停止更新
- ✅ **距离判断**: 基于透视变换坐标过滤过近/过远的行
- ✅ **非线性增益**: `output = (1 + K·err²)·err`（可选）

## 三、数据结构

### 1. LineInfo（单行图像信息）
```c
typedef struct {
    int angel_left;      // 左切线斜率(千分之一)
    int angel_right;     // 右切线斜率(千分之一)
    uint8_t persp_ly;    // 左边线y坐标(透视变换后)
    uint8_t persp_ry;    // 右边线y坐标
    uint8_t persp_lx;    // 左边线x坐标
    uint8_t persp_rx;    // 右边线x坐标
    uint8_t left_lost;   // 左边线丢失标志 0=存在, 1=丢失
    uint8_t right_lost;  // 右边线丢失标志
} LineInfo;
```

### 2. CamErrParams（参数配置）
```c
typedef struct {
    int far_line;              // 远端行(打角最远行), 默认80
    int near_line;             // 近端行(打角最近行), 默认20
    int angle_far_line;        // 实际使用的远端行
    int angle_near_line;       // 实际使用的近端行
    float SteerKpchange;       // 非线性增益系数, 0=禁用
    int track_count;           // 赛道计数(丢线判断用)
    uint8_t camwl;             // 左轮x位置(默认76)
    uint8_t camwr;             // 右轮x位置(默认97)
    uint8_t camwf;             // 前轮y位置(默认0)
} CamErrParams;
```

### 3. 全局数据
```c
extern CamErrData g_cam_err_data;  // 包含120行LineInfo和参数
```

## 四、使用流程

### 步骤1：初始化（main函数中调用一次）
```c
void main(void)
{
    // ... 其他初始化代码 ...
    
    // 初始化cam_err模块
    // 参数: near_line, far_line, forward_near, forward_far
    cam_err_init(20, 80, 10, 100);
    
    // 可选: 调整非线性增益
    g_cam_err_data.params.SteerKpchange = 0.0f;  // 0=禁用, 1.0=原始强度
    
    // 初始化控制系统
    control_init_cascade_pid_default();
    
    // ... 启动定时器/中断 ...
}
```

### 步骤2：图像处理后填充数据（每帧调用）
```c
void image_process_callback(void)
{
    // 假设已完成边线识别和透视变换
    
    for (int y = 0; y < 120; y++) {
        // 从你的图像处理模块获取数据
        g_cam_err_data.lineinfo[y].angel_left = calculate_left_slope(y);   // 计算左切线斜率(千分之一)
        g_cam_err_data.lineinfo[y].angel_right = calculate_right_slope(y); // 计算右切线斜率(千分之一)
        g_cam_err_data.lineinfo[y].persp_ly = left_line_y[y];              // 左边线y坐标
        g_cam_err_data.lineinfo[y].persp_ry = right_line_y[y];             // 右边线y坐标
        g_cam_err_data.lineinfo[y].persp_lx = left_line_x[y];              // 左边线x坐标
        g_cam_err_data.lineinfo[y].persp_rx = right_line_x[y];             // 右边线x坐标
        g_cam_err_data.lineinfo[y].left_lost = is_left_lost[y];            // 0=找到, 1=丢失
        g_cam_err_data.lineinfo[y].right_lost = is_right_lost[y];          // 0=找到, 1=丢失
    }
    
    // 更新距离阈值(可选)
    g_cam_err_data.forward_near = 10;
    g_cam_err_data.forward_far = 100;
}
```

### 步骤3：控制循环调用（已自动集成）
```c
void control_timer_1ms(void)
{
    // 读取编码器速度
    float vL = get_left_wheel_speed();
    float vR = get_right_wheel_speed();
    
    float pwm_L, pwm_R;
    
    // control_loop 内部会自动调用 cam_err_calculation()
    control_loop(1.0f, vL, vR, &pwm_L, &pwm_R);
    
    // 输出PWM
    set_motor_pwm(pwm_L, pwm_R);
}
```

## 五、参数调节

### 1. 打角范围调节
```c
// 调整打角的行范围(影响控制灵敏度)
g_cam_err_data.params.angle_near_line = 20;  // 近端行(值越大越看远处)
g_cam_err_data.params.angle_far_line = 80;   // 远端行(值越大看得越远)
```

### 2. 非线性增益调节
```c
// 启用非线性增益(大误差加强响应)
g_cam_err_data.params.SteerKpchange = 1.0f;  // 范围0~2.0
// 公式: output = (1 + 1e-7 × SteerKpchange × err²) × err
// 效果: err=100时增益1.001倍, err=200时增益1.004倍
```

### 3. 距离阈值调节
```c
// 调整远近距离判断(过滤无效行)
g_cam_err_data.forward_near = 10;   // 太近的行不处理
g_cam_err_data.forward_far = 100;   // 太远的行不处理
```

## 六、与原函数的对应关系

| 原函数变量/参数 | 移植后位置 | 说明 |
|----------------|-----------|------|
| `lineinfo[y]` | `g_cam_err_data.lineinfo[y]` | 每行图像信息 |
| `forward_near` | `g_cam_err_data.forward_near` | 近端距离阈值 |
| `forward_far` | `g_cam_err_data.forward_far` | 远端距离阈值 |
| `setpara.far_line` | `g_cam_err_data.params.far_line` | 远端行 |
| `setpara.SteerKpchange` | `g_cam_err_data.params.SteerKpchange` | 非线性系数 |
| `watch.angle_far_line` | `g_cam_err_data.params.angle_far_line` | 实际远端行 |
| `d_can_err_limit` | `CAM_D_ERR_LIMIT (30)` | 变化率限幅 |
| `cam_limit` | `CAM_OUTPUT_LIMIT (200)` | 输出限幅 |

## 七、调试技巧

### 1. 查看切点位置
```c
// 在每次调用后查看切点
int left_tangent_row = g_cam_err_data.state.watchleft;   // 左切线所在行
int right_tangent_row = g_cam_err_data.state.watchright; // 右切线所在行
```

### 2. 查看原始误差
```c
float raw_err = g_cam_err_data.state.angle_target;  // 未经非线性处理的误差
```

### 3. 输出调试数据
```c
// 使用串口或VOFA+输出关键数据
printf("err=%f, left_row=%d, right_row=%d\n", 
       cam_err_calculation(),
       g_cam_err_data.state.watchleft,
       g_cam_err_data.state.watchright);
```

## 八、常见问题

### Q1: 误差值始终为0？
**A**: 检查 `g_cam_err_data.lineinfo[]` 是否正确填充，特别是 `angel_left` 和 `angel_right` 的值。

### Q2: 误差突变剧烈？
**A**: 增大变化率限幅或调整打角范围:
```c
// 修改源码中的 CAM_D_ERR_LIMIT 宏，或增加平滑处理
g_cam_err_data.params.angle_far_line = 60;  // 减小打角范围
```

### Q3: 非线性增益没有效果？
**A**: 确认 `SteerKpchange` 已设置为非0值:
```c
g_cam_err_data.params.SteerKpchange = 1.0f;  // 必须>0才启用
```

### Q4: 如何适配不同分辨率？
**A**: 调整 `CamErrData` 中的 `lineinfo[120]` 数组大小:
```c
// 在control.h中修改
LineInfo lineinfo[YOUR_IMAGE_HEIGHT];  // 根据实际分辨率调整
```

## 九、完整示例

```c
// main.c
#include "control.h"

int main(void)
{
    // 1. 硬件初始化
    HAL_Init();
    SystemClock_Config();
    
    // 2. 初始化摄像头误差计算
    cam_err_init(20, 80, 10, 100);
    g_cam_err_data.params.SteerKpchange = 0.0f;  // 禁用非线性增益
    
    // 3. 初始化控制系统
    control_init_cascade_pid_default();
    
    // 4. 启动定时器(1ms周期)
    HAL_TIM_Base_Start_IT(&htim1);
    
    while(1)
    {
        // 主循环处理图像
        if (new_image_ready) {
            process_image();  // 填充 g_cam_err_data.lineinfo
            new_image_ready = 0;
        }
    }
}

// 1ms控制中断
void TIM1_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE)) {
        __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
        
        // 读取编码器
        float vL = get_speed();  // 左轮速度
        float vR = get_speed();  // 右轮速度
        
        float pwm_L, pwm_R;
        
        // 控制循环(内部调用cam_err_calculation)
        control_loop(1.0f, vL, vR, &pwm_L, &pwm_R);
        
        // 输出PWM
        set_motor_pwm(pwm_L, pwm_R);
    }
}

// 图像处理函数(填充lineinfo)
void process_image(void)
{
    // 边线识别 + 透视变换
    for (int y = 0; y < 120; y++) {
        // 假设已提取边线坐标
        int left_x = find_left_edge(y);
        int right_x = find_right_edge(y);
        
        // 计算切线斜率(相邻行的x坐标差值 × 1000)
        if (y > 0) {
            g_cam_err_data.lineinfo[y].angel_left = 
                (left_x - g_cam_err_data.lineinfo[y-1].persp_lx) * 1000;
            g_cam_err_data.lineinfo[y].angel_right = 
                (right_x - g_cam_err_data.lineinfo[y-1].persp_rx) * 1000;
        }
        
        // 填充坐标
        g_cam_err_data.lineinfo[y].persp_lx = left_x;
        g_cam_err_data.lineinfo[y].persp_rx = right_x;
        g_cam_err_data.lineinfo[y].persp_ly = y;  // 透视变换后的y
        g_cam_err_data.lineinfo[y].persp_ry = y;
        
        // 丢线判断
        g_cam_err_data.lineinfo[y].left_lost = (left_x < 0) ? 1 : 0;
        g_cam_err_data.lineinfo[y].right_lost = (right_x > IMAGE_WIDTH) ? 1 : 0;
    }
}
```

## 十、总结

✅ **已完成**: 
- 完整移植 `cam_err_calculation()` 核心逻辑
- 保留所有算法特性(切线法、变化率限幅、非线性增益)
- 使用 control 模块的数据结构
- 自动集成到 `control_loop()` 中

✅ **需要外部提供**:
- 每帧图像的 `lineinfo[]` 数据(边线坐标和斜率)
- 编码器速度反馈

✅ **优势**:
- 算法逻辑与原函数完全一致
- 参数可灵活配置
- 便于调试和优化
