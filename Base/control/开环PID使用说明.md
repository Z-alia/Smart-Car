# 开环PID控制器 - 完整使用指南

## 📋 目录
1. [系统架构](#系统架构)
2. [依赖关系](#依赖关系)
3. [完整调用流程](#完整调用流程)
4. [参数调试指南](#参数调试指南)
5. [故障排查](#故障排查)

---

## 🏗️ 系统架构

```
摄像头图像采集 → 图像二值化 → 扫线算法 → 开环PID控制 → 电机输出
    ↓                ↓            ↓            ↓            ↓
OV2640_Init()   Binarization()  scan_line()  open_loop_   set_motor_
                                             pid_calculate  speed()
```

### 数据流向

```
mt9v03x_image[120][188]  ← 摄像头原始图像(16bit灰度)
         ↓
Grayscale[120][188]      ← 二值化图像(0或255)
         ↓
lineinfo[120]            ← 扫线结果(每行左右边线位置)
         ↓
straight_error_get()     ← 计算横向误差
         ↓
open_loop_pid_calculate() ← PID控制计算
         ↓
left_target, right_target ← 左右轮目标速度
```

---

## 🔗 依赖关系

### 1. 硬件模块
- **OV2640摄像头** (HardWare/OV2640/)
  - `dcmi_ov2640.c/h` - DCMI接口和DMA传输
  - `mt9v03x_image[120]` - 图像数据存储(uint16_t*)

### 2. 图像处理模块
- **二值化模块** (camera_process/Binarization.c)
  ```c
  extern uint8 Grayscale[120][188];  // 灰度图
  extern uint8 imo[120][188];        // 二值化图像
  
  // 关键函数
  int img_otsu(uint16_t *img, uint8_t img_v, uint8_t img_h, uint8_t step);
  void Binarization();
  void draw_edge();
  ```

- **扫线模块** (camera_process/scan_line.c)
  ```c
  // 核心数据结构
  struct lineinfo_s {
      int16 y;           // 行号(0~119)
      int16 left;        // 左边线位置(0~188)
      int16 right;       // 右边线位置(0~188)
      int16 edge_count;  // 边线数量
      int left_lost;     // 左边线丢失标志
      int right_lost;    // 右边线丢失标志
      int16 whole_lost;  // 整行丢失标志
  };
  
  extern struct lineinfo_s lineinfo[120];
  
  // 关键函数
  void scan_line();  // 执行扫线算法
  ```

- **全局状态** (camera_process/Element_recognition.h)
  ```c
  struct watch_o {
      uint8 threshold;  // 二值化阈值
      int watch_line;   // 扫线观测行
      int watch_lost;   // 丢线统计
      // ... 更多字段
  };
  
  extern struct watch_o watch;
  ```

### 3. 控制模块
- **开环PID** (Base/control/open_loop_pid.c/h)
  ```c
  // 初始化
  void open_loop_pid_init(float kp, float ki, float kd, 
                          float integral_max, float output_max);
  
  // 控制计算
  float open_loop_pid_calculate(float v_forward, 
                                 float* left_target_out, 
                                 float* right_target_out);
  
  // 参数更新
  void open_loop_pid_update_params(float kp, float ki, float kd);
  
  // 状态重置
  void open_loop_pid_reset(void);
  ```

---

## 🚀 完整调用流程

### main.c 完整示例

```c
/* Includes */
#include "main.h"
#include "dcmi.h"
#include "lcd_spi_200.h"
#include "dcmi_ov2640.h"
#include "scan_line.h"
#include "Binarization.h"
#include "Element_recognition.h"
#include "open_loop_pid.h"  // ← 添加这个头文件

int main(void)
{
    /* 1. 系统初始化 */
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_DCMI_Init();
    
    /* 2. 摄像头初始化 */
    OV2640_Init();
    OV2640_DMA_Transmit_Continuous(Camera_Buffer, OV2640_BufferSize);
    
    /* 3. 显示屏初始化 */
    LCD_Init();
    
    /* 4. 开环PID控制器初始化 */
    open_loop_pid_init(
        1.0f,    // kp: 比例增益
        0.1f,    // ki: 积分增益
        0.05f,   // kd: 微分增益
        50.0f,   // integral_max: 积分限幅
        2.0f     // output_max: 输出限幅(m/s)
    );
    
    /* 5. 主循环 */
    while (1)
    {
        if (DCMI_FrameState == 1)  // 等待图像采集完成
        {
            DCMI_FrameState = 0;
            
            /* 5.1 计算二值化阈值(OTSU算法) */
            watch.threshold = img_otsu((uint16_t *)mt9v03x_image[30], 
                                      60, Display_Width, 10);
            
            /* 5.2 阈值限幅(防止过曝或过暗) */
            if (watch.threshold > 120)
                watch.threshold = 120;
            else if (watch.threshold < 80)
                watch.threshold = 80;
            
            /* 5.3 二值化处理 */
            Binarization();  // mt9v03x_image → Grayscale
            
            /* 5.4 执行扫线算法 */
            scan_line();     // Grayscale → lineinfo[]
            
            /* 5.5 调用开环PID控制 */
            float v_forward = 1.0f;  // 前进速度 1m/s
            float left_target, right_target;
            
            open_loop_pid_calculate(v_forward, &left_target, &right_target);
            
            /* 5.6 输出到电机(需要自己实现) */
            set_motor_speed(MOTOR_LEFT, left_target);
            set_motor_speed(MOTOR_RIGHT, right_target);
            
            /* 5.7 显示处理结果(可选) */
            draw_edge();  // 在imo上绘制边线
            show_ov2640_image_int8(0, 0, imo[0], 
                                  Display_Width, Display_Height, 
                                  Display_Width, Display_Height);
        }
    }
}
```

---

## 🎯 参数调试指南

### 第一步: 调整Kp (比例增益)

```c
// 测试不同的Kp值
open_loop_pid_update_params(0.5f, 0.0f, 0.0f);  // 响应慢,稳定
open_loop_pid_update_params(1.0f, 0.0f, 0.0f);  // 适中
open_loop_pid_update_params(2.0f, 0.0f, 0.0f);  // 响应快,可能振荡
```

**现象观察:**
- Kp太小: 车辆反应慢,转弯不足,容易冲出赛道
- Kp合适: 车辆快速回到中线,无明显摆动
- Kp太大: 车辆左右摆动(振荡),无法稳定在中线

**建议:** 从0.5开始,逐渐增加到1.5~2.0,找到开始振荡的临界值,然后回退20%

---

### 第二步: 添加Kd (微分增益)

```c
// 在Kp基础上添加Kd
open_loop_pid_update_params(1.5f, 0.0f, 0.05f);  // 添加少量Kd
open_loop_pid_update_params(1.5f, 0.0f, 0.10f);  // 增加Kd抑制振荡
```

**作用:**
- 提前预测误差变化,增加系统阻尼
- 抑制Kp引起的振荡
- 提高快速过弯能力

**建议:** Kd约为Kp的5%~10%,不宜过大(会放大噪声)

---

### 第三步: 添加Ki (积分增益)

```c
// 消除稳态误差
open_loop_pid_update_params(1.5f, 0.05f, 0.10f);  // 添加少量Ki
open_loop_pid_update_params(1.5f, 0.10f, 0.10f);  // 增加Ki
```

**作用:**
- 消除长时间存在的偏差(稳态误差)
- 适应赛道宽度变化

**注意:**
- Ki过大会导致积分饱和,车辆"打转"
- 切换赛道时务必调用 `open_loop_pid_reset()` 清除积分

**建议:** Ki约为Kp的5%~10%

---

### 参数调试模板

| 赛道类型 | Kp   | Ki   | Kd   | 说明 |
|---------|------|------|------|------|
| 直线较多 | 1.0  | 0.05 | 0.05 | 稳定优先 |
| 弯道较多 | 1.5  | 0.10 | 0.10 | 响应快速 |
| 急弯赛道 | 2.0  | 0.15 | 0.15 | 激进控制 |
| 宽赛道   | 0.8  | 0.08 | 0.08 | 降低灵敏度 |

---

## 🔧 故障排查

### 问题1: 编译错误 `未定义标识符 "lineinfo"`

**原因:** 未包含扫线模块头文件

**解决:**
```c
// 在 open_loop_pid.c 中添加
#include "scan_line.h"  // 如果项目中有这个头文件
```

或者在 `scan_line.h` 中添加:
```c
extern struct lineinfo_s lineinfo[120];
```

---

### 问题2: 车辆不动或乱跑

**检查清单:**
1. 确认 `scan_line()` 是否在 `open_loop_pid_calculate()` 之前调用
2. 查看 `lineinfo[]` 数组是否有有效数据
3. 检查误差值是否合理:
   ```c
   float error = open_loop_pid_get_error();
   printf("Error: %.2f\r\n", error);  // 应该在 ±50 范围内
   ```

---

### 问题3: 车辆左右摆动严重

**原因:** Kp或Kd过大

**解决:**
1. 减小Kp值(减少20%)
2. 适当增加Kd抑制振荡
3. 检查采样周期是否稳定(建议10ms)

---

### 问题4: 车辆转弯不足

**原因:** 
- Kp太小
- output_max限幅过小

**解决:**
1. 增加Kp值
2. 增加output_max:
   ```c
   open_loop_pid_init(1.5f, 0.1f, 0.1f, 50.0f, 3.0f);  // 增加到3.0
   ```

---

### 问题5: 直线上偏离中线

**原因:** 存在稳态误差

**解决:**
1. 增加Ki值消除稳态误差
2. 检查权重配置:
   ```c
   #define weight_dw 0.2f  // 下段(近处)权重
   #define weight_md 0.3f  // 中段权重
   #define weight_up 0.5f  // 上段(远处)权重
   ```
3. 可以增加远处权重(weight_up),提前预判

---

## 📊 调试信息输出

### 串口打印关键数据

```c
// 在控制循环中添加
float error = open_loop_pid_get_error();
float output = open_loop_pid_get_output();

printf("E:%.2f O:%.2f L:%.2f R:%.2f\r\n", 
       error, output, 
       g_open_loop_pid_state.left_target, 
       g_open_loop_pid_state.right_target);
```

### VOFA+上位机波形显示

```c
// 发送4个浮点数到VOFA+(JustFloat协议)
void send_to_vofa(float f1, float f2, float f3, float f4)
{
    uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};
    HAL_UART_Transmit(&huart1, (uint8_t*)&f1, 4, 10);
    HAL_UART_Transmit(&huart1, (uint8_t*)&f2, 4, 10);
    HAL_UART_Transmit(&huart1, (uint8_t*)&f3, 4, 10);
    HAL_UART_Transmit(&huart1, (uint8_t*)&f4, 4, 10);
    HAL_UART_Transmit(&huart1, tail, 4, 10);
}

// 在主循环中
send_to_vofa(error, output, left_target, right_target);
```

---

## ✅ 快速检查清单

- [ ] 摄像头采集正常 (`DCMI_FrameState == 1`)
- [ ] 二值化阈值合理 (80~120)
- [ ] 扫线结果有效 (`lineinfo[].whole_lost == 0`)
- [ ] 误差值合理 (±50范围内)
- [ ] PID参数已初始化
- [ ] 调用顺序正确 (Binarization → scan_line → PID)
- [ ] 电机控制函数已实现

---

## 📞 技术支持

如有问题,请检查:
1. `lineinfo[]` 数组是否有有效数据
2. 权重参数是否适应当前赛道
3. PID参数是否合理
4. 电机输出是否正常

**祝您调试顺利!** 🎉
