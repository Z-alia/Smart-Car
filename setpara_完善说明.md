# setpara 结构体完善说明

## 修改文件
- `Base/system_config.h` - 系统配置头文件
- `Core/Src/main.c` - 主程序文件

## 新增的 setpara_struct 成员

根据对所有元素代码的分析，以下成员已添加到 `setpara_struct` 结构体中：

### 速度配置
```c
float cross_speed;           // 十字路口速度
float zebra_speed;           // 斑马线速度
float com_target_speed;      // 普通道路目标速度
```

### 距离配置
```c
float loop_out_distance;     // 小圆环出圆距离
float big_loop_out_distance; // 大圆环出圆距离
float zebra_distance;        // 斑马线停车距离
```

### 计数配置
```c
uint8_t zebra_line_count;    // 斑马线线条数量阈值
uint8_t stop_over_count;     // 停车计数阈值
```

### 其他参数
```c
uint8_t mode;                // 出环模式 (OutLeft/OutRight)
uint8_t obstacle_dir;        // 避障方向 0=左 1=右
```

### PID参数
```c
uint8_t stop_PID;            // 停车PID索引
uint8_t big_loop_PID;        // 大圆环转弯PID索引
uint8_t com_turn_PID;        // 普通转向PID索引
```

## 新增的 watch_struct 结构体

创建了完整的视觉监控结构体，包含以下类别的成员：

### 二值化阈值
- `threshold` - 动态阈值

### 圆环识别
- `InLoop`, `InLoopAngleL`, `InLoopAngleR`, `InLoopCirc`, `InLoopAngle2`
- `OutLoop`, `OutLoopAngle1`, `OutLoopAngle2`

### 十字路口识别
- `cross_flag`, `cross_line`, `Garge_line`
- `cross_RD_angle`, `cross_LD_angle`, `cross_AngleL`, `cross_AngleR`
- `cross_AngleL_x`, `cross_AngleR_x`

### 坡道识别
- `slope_flag` - 0=未检测, 1=上坡, 2=下坡

### 斑马线识别
- `zebra_flag`, `Zebra_Angle`, `Zebra_Angle2`, `ZebraLine`
- `stop_count`

### 车库识别
- `out_garage_flag`, `garage_stop`, `garage_flag`

### 障碍物识别
- `obstacle_flag`, `black_obstacle_flag`, `black_obstacle_line`
- `left_obstacle_x`, `right_obstacle_x`

### 其他
- `broken_circuit_flag` - 断路标志
- `Line_patrol_mode` - 巡线模式
- `angle_near_line`, `angle_far_line` - 扫线范围

## 全局变量定义

在 `main.c` 中添加了以下全局变量定义：

```c
setpara_struct setpara;           // 参数配置
mycar_struct mycar;               // 小车状态
imu_struct imu;                   // IMU数据
watch_struct watch;               // 视觉监控
vofa_struct vofa;                 // VOFA调试数据
uint16_t dl1b_distance_mm = 9999; // 激光测距数据
```

## 初始化函数更新

`system_config_init()` 函数已更新，包含所有新增成员的默认值初始化：

### setpara 默认值
- `cross_speed = 1.0f` m/s
- `zebra_speed = 0.6f` m/s
- `com_target_speed = 1.2f` m/s
- `loop_out_distance = 30.0f` cm
- `big_loop_out_distance = 40.0f` cm
- `zebra_distance = 20.0f` cm
- `zebra_line_count = 3`
- `stop_over_count = 50`
- `mode = 0` (默认出环模式)
- `obstacle_dir = 0` (默认左避障)

### watch 默认值
所有标志位初始化为0或合理默认值（如120行、187列等）

## 使用示例

### 在元素代码中访问参数
```c
// 设置速度
set_speed(setpara.cross_speed);
set_speed(setpara.zebra_speed);

// 使用距离参数
begin_distant_integeral(setpara.loop_out_distance);
begin_distant_integeral(setpara.big_loop_out_distance);

// 使用计数参数
if(zebra_count > setpara.zebra_line_count) {
    // 检测到足够的斑马线
}

// 使用模式参数
if(setpara.mode == OutLeft) {
    // 左出环模式
}
```

### 在元素代码中设置标志
```c
// 设置坡道标志
watch.slope_flag = 1;  // 上坡

// 设置障碍物位置
watch.left_obstacle_x = detected_x;
watch.black_obstacle_line = detected_y;
```

## 代码使用统计

通过 grep 搜索发现以下使用情况：
- 74 处 `setpara.` 成员访问
- 50+ 处 `watch.` 成员访问（结果截断）

## 注意事项

1. **PID 参数索引**: `com_speed_PID`, `loop_turn_PID`, `stop_PID`, `big_loop_PID`, `com_turn_PID` 是索引值，实际 PID 参数应在 `control.h` 或其他文件中定义

2. **速度单位**: 速度值的单位需要与实际使用保持一致（注意 m/s 和 cm/s 的转换）

3. **距离单位**: 距离配置使用 cm 作为单位

4. **OutLeft/OutRight 定义**: 这些常量在 `patch_line.c` 中定义为宏：
   ```c
   #define OutLeft  1
   #define OutRight 2
   ```

5. **初始化顺序**: 确保在使用任何元素识别功能前调用 `system_config_init()`

## 文件位置
- 结构体定义: `Base/system_config.h`
- 全局变量定义: `Core/Src/main.c`
- 初始化调用: `Core/Src/main.c` 的 `main()` 函数中

## 兼容性
所有现有的元素代码（circle.c, cross.c, slope.c, zebra.c, red_obstacle.c）都已经使用这些成员，现在结构体定义已完整，应该可以正常编译。
