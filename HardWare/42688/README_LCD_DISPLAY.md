# ICM42688P LCD显示功能

## 🎯 一分钟快速开始

在你的 `main.c` 中添加：

```c
#include "ICM-42688P.h"
#include "lcd_spi_200.h"

int main(void)
{
    // 初始化...
    LCD_Init();
    
    // 一键检测和显示ICM数据
    ICM42688P_DetectAndDisplay();
    
    // 继续你的代码...
}
```

## 📖 文档导航

- **[快速开始](QUICK_START.md)** ⭐ - 5分钟上手，推荐首先阅读
- **[完整文档](ICM42688P_DISPLAY_USAGE.md)** - 详细API文档
- **[示例代码](ICM42688P_Display_Example.c)** - 5个完整示例
- **[功能总结](ICM_LCD_DISPLAY_SUMMARY.md)** - 新增功能概览

## 🚀 三个新函数

| 函数 | 作用 | 用途 |
|------|------|------|
| `ICM42688P_DetectAndDisplay()` | 完整检测和显示 | 调试、演示 |
| `ICM42688P_QuickCheck()` | 快速状态检查 | 系统自检 |
| `ICM42688P_DisplayCompact(x,y)` | 紧凑数据显示 | 实时监控 |

## 💡 使用场景

### 调试硬件
```c
ICM42688P_DetectAndDisplay();
```

### 监控数据
```c
while(1) {
    ICM42688P_DisplayCompact(10, 50);
    HAL_Delay(50);
}
```

### 姿态控制
```c
while(1) {
    ICM42688P_ReadIMUData(&imu_data);
    PID_Control(imu_data.gyro_z);
}
```

## 📦 新增文件

```
HardWare/42688/
├── ICM-42688P.h                    (已更新 - 新增函数声明)
├── ICM-426688P.c                   (已更新 - 新增函数实现)
├── README_LCD_DISPLAY.md           (本文件)
├── QUICK_START.md                  (快速开始指南)
├── ICM42688P_DISPLAY_USAGE.md      (完整API文档)
├── ICM42688P_Display_Example.c     (示例代码)
└── ICM_LCD_DISPLAY_SUMMARY.md      (功能总结)
```

## ✅ 主要特性

- ✅ 硬件连接自动检测
- ✅ 初始化过程可视化
- ✅ 实时数据显示（加速度、角速度、温度）
- ✅ 刷新率监控
- ✅ 错误诊断和提示
- ✅ 紧凑显示模式
- ✅ 快速状态检查

## 🎨 显示效果

```
=== ICM42688P Test ===

Hardware Check:
CS H:1 L:0         OK

Initializing...
Init Time: 350 ms
Init SUCCESS!

WHOAMI: 0x47       OK
PWR_MGMT: 0x0F     ON

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
```

## 🔍 快速诊断

**显示"CS FAIL"**
→ 检查PE11引脚

**显示"Init FAIL"**
→ 检查SPI4接线和供电

**数据全是0**
→ 调用 `ICM42688P_QuickCheck()` 检查状态

## 📚 推荐阅读顺序

1. 本文件 - 快速了解
2. [QUICK_START.md](QUICK_START.md) - 学习基本用法
3. [ICM42688P_Display_Example.c](ICM42688P_Display_Example.c) - 查看示例
4. [ICM42688P_DISPLAY_USAGE.md](ICM42688P_DISPLAY_USAGE.md) - 深入学习

---

**快速链接**: [返回项目根目录](../../../../)
