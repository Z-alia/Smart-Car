# 快速开始 - ICM42688P LCD显示

## 最简单的使用方法

在 `main.c` 中添加以下代码：

```c
#include "ICM-42688P.h"
#include "lcd_spi_200.h"

int main(void)
{
    // 系统初始化
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_SPI1_Init();  // LCD使用SPI1
    MX_SPI4_Init();  // ICM使用SPI4
    
    // ============ 方法1: 一键检测和显示（推荐用于调试） ============
    LCD_Init();
    ICM42688P_DetectAndDisplay();
    
    // 该函数会自动完成：
    // 1. 硬件检测
    // 2. 传感器初始化  
    // 3. WHOAMI验证
    // 4. 显示实时数据（约2秒）
    // 然后继续执行后面的代码
    
    // ============ 方法2: 在主循环中显示数据 ============
    // LCD_Init();
    // uint8_t init_result = ICM42688P_Init();
    // 
    // if(init_result == 0) {
    //     LCD_DisplayString(10, 10, "ICM Ready!");
    // }
    // 
    // while(1) {
    //     // 在指定位置显示紧凑的IMU数据
    //     ICM42688P_DisplayCompact(10, 50);
    //     HAL_Delay(50);  // 20Hz刷新
    // }
    
    // ============ 方法3: 只读取数据不显示（用于控制） ============
    // ICM42688P_Init();
    // 
    // while(1) {
    //     ICM42688P_ReadIMUData(&imu_data);
    //     
    //     // 使用数据进行控制
    //     float yaw_rate = imu_data.gyro_z;
    //     // ... 你的控制算法
    //     
    //     HAL_Delay(10);
    // }
    
    // 你的其他代码...
    while(1) {
        // 主循环
    }
}
```

## 三个新增函数

### 1. `ICM42688P_DetectAndDisplay()`
- **作用**: 完整的检测和显示流程
- **耗时**: 约5-6秒
- **适用**: 调试、演示、硬件验证

### 2. `ICM42688P_QuickCheck()`  
- **作用**: 快速检查传感器状态
- **返回**: 0=正常, 1=WHOAMI错误, 2=未启动
- **适用**: 系统自检、故障诊断

### 3. `ICM42688P_DisplayCompact(x, y)`
- **作用**: 在指定位置显示紧凑数据
- **占用**: 3行（36像素高）
- **适用**: 主循环中的实时监控

## 示例：智能车完整代码

```c
#include "main.h"
#include "ICM-42688P.h"
#include "lcd_spi_200.h"
#include "motor.h"
#include "control.h"

int main(void)
{
    // 系统初始化
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI1_Init();
    MX_SPI4_Init();
    MX_TIM_Init();  // 电机PWM
    
    // LCD初始化
    LCD_Init();
    LCD_Clear();
    LCD_DisplayString(10, 10, "Smart Car Init...");
    
    // ICM初始化
    uint8_t icm_status = ICM42688P_Init();
    if(icm_status == 0) {
        LCD_SetColor(LCD_GREEN);
        LCD_DisplayString(10, 30, "ICM: OK");
        LCD_SetColor(LCD_WHITE);
    } else {
        LCD_SetColor(LCD_RED);
        LCD_DisplayString(10, 30, "ICM: FAIL");
        LCD_SetColor(LCD_WHITE);
        // 可以选择继续运行或停止
    }
    
    // 电机初始化
    motor_init();
    LCD_DisplayString(10, 50, "Motor: OK");
    
    HAL_Delay(1000);
    LCD_Clear();
    
    // 显示标题
    LCD_DisplayString(10, 10, "=== Smart Car ===");
    
    // 主循环
    while(1) {
        // 1. 读取IMU数据
        ICM42688P_ReadIMUData(&imu_data);
        
        // 2. 姿态控制（使用陀螺仪Z轴）
        float yaw_rate = imu_data.gyro_z;
        PID_Control(yaw_rate);  // 你的PID控制函数
        
        // 3. 显示数据（每50ms刷新一次，20Hz）
        ICM42688P_DisplayCompact(10, 30);
        
        // 4. 显示速度
        char speed_str[32];
        sprintf(speed_str, "Speed: %.2f m/s", get_speed());
        LCD_DisplayString(10, 70, speed_str);
        
        HAL_Delay(50);
    }
}
```

## 注意事项

1. **必须先初始化LCD**: 调用显示函数前必须先 `LCD_Init()`
2. **SPI不冲突**: LCD用SPI1，ICM用SPI4，可以同时使用
3. **推荐刷新率**: 20-50Hz，太快会影响性能
4. **调试时**: 使用 `ICM42688P_DetectAndDisplay()` 快速验证
5. **生产时**: 使用 `ICM42688P_Init()` + `ICM42688P_DisplayCompact()` 或只读取数据

## 常见问题

**Q: 编译错误 "GPIOE未定义"**  
A: 这是IntelliSense的问题，实际编译能通过。确保包含了`main.h`

**Q: 显示"CS FAIL"**  
A: 检查PE11引脚配置和接线

**Q: 显示"Init FAIL"**  
A: 检查ICM接线和供电，参考`ICM42688_DEBUG_GUIDE.md`

**Q: 数据全是0**  
A: 传感器未启动或读取失败，调用`ICM42688P_QuickCheck()`检查

## 更多信息

- 完整API文档: `ICM42688P_DISPLAY_USAGE.md`
- 使用示例代码: `ICM42688P_Display_Example.c`
- 调试指南: `ICM42688_DEBUG_GUIDE.md`
