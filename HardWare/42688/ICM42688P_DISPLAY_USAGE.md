# ICM42688P LCD显示功能使用指南

## 概述

本文档介绍如何使用ICM42688P传感器的LCD显示功能。新增的函数提供了从硬件检测到数据实时显示的完整解决方案。

## 新增函数

### 1. `ICM42688P_DetectAndDisplay()` - 完整检测和显示

**功能**: 执行完整的传感器检测流程并在LCD上显示结果和实时数据

**包含步骤**:
1. LCD初始化检查
2. 硬件连接检测（CS引脚控制测试）
3. 传感器初始化
4. WHOAMI验证
5. 电源管理状态检查
6. 实时IMU数据显示（100帧）

**使用场景**: 
- 调试ICM42688P硬件连接
- 验证传感器功能
- 演示传感器数据

**使用示例**:
```c
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI1_Init();  // LCD
    MX_SPI4_Init();  // ICM
    
    LCD_Init();
    ICM42688P_DetectAndDisplay();  // 一键检测和显示
    
    while(1) {
        // 继续其他任务
    }
}
```

**显示内容**:
```
=== ICM42688P Test ===

Hardware Check:
CS H:1 L:0         OK

Initializing...
Init Time: 350 ms
Init SUCCESS!

WHOAMI: 0x47       OK
PWR_MGMT: 0x0F     ON

Press to continue...

=== IMU Data ===
Accel(g):
X:  0.02
Y:  0.01
Z:  0.98

Gyro(dps):
X:   1.2
Y:  -0.5
Z:   0.3

Temp(C):  25.3
Rate(Hz):  48.5
Running...
```

---

### 2. `ICM42688P_QuickCheck()` - 快速状态检测

**功能**: 快速检测ICM传感器状态，不执行初始化

**返回值**:
- `0` : 传感器工作正常
- `1` : WHOAMI错误（通信问题或芯片错误）
- `2` : 传感器未启动（需要初始化）

**使用场景**:
- 在主循环中周期性检查传感器状态
- 系统自检
- 故障诊断

**使用示例**:
```c
void SystemHealthCheck(void)
{
    uint8_t status = ICM42688P_QuickCheck();
    
    switch(status) {
        case 0:
            printf("ICM: OK\n");
            break;
        case 1:
            printf("ICM: Communication Error\n");
            break;
        case 2:
            printf("ICM: Need Initialization\n");
            ICM42688P_Init();  // 重新初始化
            break;
    }
}
```

---

### 3. `ICM42688P_DisplayCompact()` - 紧凑数据显示

**功能**: 在指定位置显示紧凑的IMU数据（占用3行，36像素高度）

**参数**:
- `x` : 显示起始X坐标
- `y` : 显示起始Y坐标

**显示格式**:
```
A: 0.0, 0.0, 1.0
G:  2, -1,  0
T:25.3C
```

**使用场景**:
- 需要在屏幕上同时显示多种信息
- 主循环中周期性更新显示
- 数据记录和监控

**使用示例**:
```c
int main(void)
{
    // 初始化
    LCD_Init();
    ICM42688P_Init();
    
    LCD_Clear();
    LCD_DisplayString(10, 10, "System Status");
    LCD_DisplayString(10, 30, "Motor: OK");
    LCD_DisplayString(10, 50, "Camera: OK");
    
    while(1) {
        // 在(10, 70)位置显示IMU数据
        ICM42688P_DisplayCompact(10, 70);
        
        // 其他显示内容
        LCD_DisplayString(10, 110, "Speed: 1.2 m/s");
        
        HAL_Delay(50);  // 20Hz刷新
    }
}
```

---

## 完整集成示例

### 方案1: 调试模式（详细诊断）

```c
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI1_Init();
    MX_SPI4_Init();
    
    LCD_Init();
    
    // 执行完整检测流程
    ICM42688P_DetectAndDisplay();
    
    // 继续正常程序
    while(1) {
        // 你的代码...
    }
}
```

### 方案2: 生产模式（快速启动）

```c
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI1_Init();
    MX_SPI4_Init();
    
    LCD_Init();
    LCD_Clear();
    
    // 快速初始化
    uint8_t init_result = ICM42688P_Init();
    
    if(init_result == 0) {
        LCD_DisplayString(10, 10, "ICM: Ready");
    } else {
        LCD_SetColor(LCD_RED);
        LCD_DisplayString(10, 10, "ICM: Error");
        LCD_SetColor(LCD_WHITE);
    }
    
    // 主循环
    while(1) {
        // 紧凑显示IMU数据
        ICM42688P_DisplayCompact(10, 30);
        
        // 使用IMU数据进行控制
        ICM42688P_ReadIMUData(&imu_data);
        float yaw_rate = imu_data.gyro_z;
        // ... 控制算法
        
        HAL_Delay(20);
    }
}
```

### 方案3: 只用于控制（不显示）

```c
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI4_Init();
    
    // 初始化ICM（不需要LCD）
    ICM42688P_Init();
    
    while(1) {
        // 读取IMU数据
        ICM42688P_ReadIMUData(&imu_data);
        
        // 使用数据进行姿态控制
        PID_Control(imu_data.gyro_z, imu_data.accel_x);
        
        HAL_Delay(10);  // 100Hz控制频率
    }
}
```

---

## 数据结构

```c
typedef struct
{
    float accel_x;      // X轴加速度 (g)
    float accel_y;      // Y轴加速度 (g)
    float accel_z;      // Z轴加速度 (g)
    
    float gyro_x;       // X轴角速度 (dps)
    float gyro_y;       // Y轴角速度 (dps)
    float gyro_z;       // Z轴角速度 (dps)
    
    float q0, q1, q2, q3;  // 四元数
    
    float temperature;  // 温度 (°C)
    
    uint16_t timestamp; // 时间戳
} IMU_Data;

extern IMU_Data imu_data;  // 全局IMU数据
```

---

## 错误代码

### 初始化错误码 (`ICM42688P_Init()` 返回值)

| 错误码 | 说明 | 可能原因 | 解决方法 |
|--------|------|----------|----------|
| 0 | 成功 | - | - |
| 1 | WHOAMI读取失败 | SPI通信问题、芯片未连接 | 检查硬件连接、SPI配置 |
| 2 | 配置后通信失败 | 配置破坏了通信 | 检查寄存器配置 |
| 3 | 传感器未正确启动 | PWR_MGMT0配置错误 | 检查电源管理配置 |

### 快速检测错误码 (`ICM42688P_QuickCheck()` 返回值)

| 错误码 | 说明 | 解决方法 |
|--------|------|----------|
| 0 | 正常 | - |
| 1 | WHOAMI错误 | 检查硬件连接和SPI通信 |
| 2 | 传感器未启动 | 调用 `ICM42688P_Init()` |

---

## 性能参数

- **初始化时间**: 约300-500ms
- **数据读取周期**: 最快20ms（50Hz）
- **LCD刷新率**: 推荐20-50Hz
- **温度分辨率**: 0.1°C
- **加速度计量程**: ±16g
- **陀螺仪量程**: ±2000 dps

---

## 注意事项

1. **LCD必须先初始化**: 在调用任何显示函数前，必须先调用 `LCD_Init()`

2. **SPI冲突**: LCD和ICM使用不同的SPI接口（SPI1和SPI4），不会冲突

3. **刷新率控制**: 建议使用20-50Hz的刷新率，过快会影响性能

4. **内存使用**: 
   - `ICM42688P_DetectAndDisplay()`: 约200字节栈空间
   - `ICM42688P_DisplayCompact()`: 约50字节栈空间

5. **阻塞时间**:
   - `ICM42688P_DetectAndDisplay()`: 约5-6秒（包含延时）
   - `ICM42688P_ReadIMUData()`: 约3-5ms
   - `ICM42688P_DisplayCompact()`: 约10-15ms

---

## 常见问题

### Q1: 显示"CS H:0 L:0"或"CS FAIL"

**原因**: CS引脚控制失败

**解决**:
1. 检查PE11引脚配置（应为GPIO输出）
2. 检查引脚是否被其他外设占用
3. 使用万用表测量PE11电平

### Q2: 显示"Init FAIL! Code:1"

**原因**: WHOAMI读取失败，通信问题

**解决**:
1. 检查SPI4接线（PE12/13/14）
2. 检查ICM42688P供电（3.3V）
3. 测量MISO引脚是否有数据

### Q3: 显示"Init FAIL! Code:3"

**原因**: 传感器未正确启动

**解决**:
1. 增加初始化后的延时
2. 检查PWR_MGMT0寄存器配置
3. 尝试多次复位

### Q4: 数据显示为0或异常值

**原因**: 传感器配置错误或读取失败

**解决**:
1. 使用 `ICM42688P_QuickCheck()` 检查状态
2. 重新初始化传感器
3. 检查Bank选择是否正确

---

## 更多示例

完整的使用示例请参考 `ICM42688P_Display_Example.c` 文件，包含：

- 示例1: 完整检测和显示流程
- 示例2: 快速状态检查
- 示例3: 主循环中的紧凑显示
- 示例4: 自定义显示布局
- 示例5: main.c中的典型用法

---

## 技术支持

如遇到问题，请检查：
1. `ICM42688_DEBUG_GUIDE.md` - ICM调试指南
2. `WHOAMI_0x00_DEBUG.md` - WHOAMI问题排查
3. 硬件原理图和引脚定义

## 更新日志

**2025-11-06**
- 新增 `ICM42688P_DetectAndDisplay()` 函数
- 新增 `ICM42688P_QuickCheck()` 函数  
- 新增 `ICM42688P_DisplayCompact()` 函数
- 添加完整使用示例和文档
