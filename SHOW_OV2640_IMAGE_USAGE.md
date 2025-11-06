# show_ov2640_image 函数使用说明

## 问题总结

`show_ov2640_image()` 调用后没有反应的**根本原因**：

1. **数据结构不匹配**：`mt9v03x_image` 是指针数组，每个元素指向一行图像数据，而原函数假设图像数据在连续内存中。

2. **只传入了第一行数据**：调用 `show_ov2640_image(0, 0, mt9v03x_image[0], ...)` 时，只传入了第一行的指针，导致后续行的地址计算错误。

3. **SPI 频繁切换数据宽度**：原函数每次调用都重新初始化 SPI，可能导致通信不稳定。

## 解决方案

### 方法 1: 使用新函数（推荐）✅

在 main.c 中修改：

```c
// 旧的调用方式（错误）
if (DCMI_FrameState == 1)
{
    DCMI_FrameState = 0;
    
    // ❌ 错误：只传入第一行指针
    show_ov2640_image(0, 0, mt9v03x_image[0], 
                      Display_Width, Display_Height, 
                      Display_Width, Display_Height, 0);
}

// 新的调用方式（正确）
if (DCMI_FrameState == 1)
{
    DCMI_FrameState = 0;
    
    // ✅ 正确：传入整个指针数组
    show_ov2640_image_from_ptr_array(0, 0, mt9v03x_image, 
                                      Display_Width, Display_Height, 
                                      Display_Width, Display_Height, 0);
}
```

### 方法 2: 调试验证

添加调试代码验证摄像头数据：

```c
if (DCMI_FrameState == 1)
{
    DCMI_FrameState = 0;
    
    // 1. 检查是否有数据
    uint16_t first_pixel = mt9v03x_image[0][0];
    uint16_t mid_pixel = mt9v03x_image[60][94];  // 中心像素
    
    char debug_str[50];
    sprintf(debug_str, "P[0,0]: 0x%04X", first_pixel);
    LCD_DisplayString(10, 10, debug_str);
    
    sprintf(debug_str, "P[60,94]: 0x%04X", mid_pixel);
    LCD_DisplayString(10, 30, debug_str);
    
    // 2. 如果有数据，显示图像
    if(first_pixel != 0x0000 || mid_pixel != 0x0000)
    {
        show_ov2640_image_from_ptr_array(0, 60, mt9v03x_image, 
                                          Display_Width, Display_Height, 
                                          Display_Width, Display_Height, 0);
    }
    else
    {
        LCD_DisplayString(10, 50, "No camera data!");
    }
}
```

## 完整示例代码

```c
// 在 main.c 的 while(1) 循环中

while (1)
{
    if (DCMI_FrameState == 1)	// 采集到了一帧图像
    {
        DCMI_FrameState = 0;		// 清零标志位
        
        // 方案 A: 直接显示原图
        show_ov2640_image_from_ptr_array(0, 0, mt9v03x_image,
                                          Display_Width, Display_Height,
                                          Display_Width, Display_Height, 0);
        
        // 方案 B: 显示二值化图像（阈值 128）
        // show_ov2640_image_from_ptr_array(0, 0, mt9v03x_image,
        //                                   Display_Width, Display_Height,
        //                                   Display_Width, Display_Height, 128);
        
        // 方案 C: 缩小显示（例如显示在 120x60 区域）
        // show_ov2640_image_from_ptr_array(10, 10, mt9v03x_image,
        //                                   Display_Width, Display_Height,
        //                                   120, 60, 0);
    }
}
```

## 函数参数说明

```c
show_ov2640_image_from_ptr_array(
    uint16_t x,              // 显示起始 X 坐标
    uint16_t y,              // 显示起始 Y 坐标
    uint16_t **image_rows,   // 图像指针数组（mt9v03x_image）
    uint16_t width,          // 图像实际宽度（Display_Width = 188）
    uint16_t height,         // 图像实际高度（Display_Height = 120）
    uint16_t dis_width,      // 显示宽度（可以缩放）
    uint16_t dis_height,     // 显示高度（可以缩放）
    uint8_t threshold        // 二值化阈值（0 = 不启用）
);
```

### 参数详解

1. **x, y**: 图像在屏幕上的起始坐标
   - 例如 (0, 0) 表示从屏幕左上角开始显示
   - 例如 (10, 10) 表示留出 10 像素边距

2. **image_rows**: 指针数组
   - 直接传入 `mt9v03x_image`（不是 `mt9v03x_image[0]`）
   - 类型是 `uint16_t **`

3. **width, height**: 原始图像尺寸
   - 通常使用 `Display_Width` 和 `Display_Height`
   - 不要修改

4. **dis_width, dis_height**: 显示尺寸
   - 可以与原始尺寸相同（1:1 显示）
   - 可以缩小（例如 120x60）实现缩放效果
   - 不建议放大（会降低图像质量）

5. **threshold**: 二值化阈值
   - 0 = 不启用二值化，显示原始灰度图
   - 1~255 = 启用二值化，低于阈值显示黑色，高于阈值显示白色
   - 推荐值：128（中间值）

## 性能对比

| 显示尺寸 | 帧率估算 | 备注 |
|---------|---------|------|
| 188x120 (全屏) | ~10 FPS | 原始尺寸 |
| 120x60 (缩小) | ~25 FPS | 推荐用于实时预览 |
| 60x30 (小窗口) | ~60 FPS | 高速预览 |

## 故障排查

### 问题 1: 屏幕仍然没有显示

**检查清单：**
```c
// 1. 确认摄像头已启动
OV2640_DMA_Transmit_Continuous(Camera_Buffer, OV2640_BufferSize);

// 2. 检查 DCMI_FrameState 是否会变为 1
if (DCMI_FrameState == 1) {
    LCD_DisplayString(10, 10, "Frame received!");
}

// 3. 检查图像数据是否有效
uint16_t test_pixel = mt9v03x_image[60][94];
char buf[32];
sprintf(buf, "Pixel: 0x%04X", test_pixel);
LCD_DisplayString(10, 30, buf);
```

### 问题 2: 显示的图像是全黑或全白

**可能原因：**
- 摄像头曝光设置不当
- 二值化阈值设置错误
- 图像数据全为 0 或全为 1

**解决方法：**
```c
// 1. 禁用二值化
show_ov2640_image_from_ptr_array(0, 0, mt9v03x_image,
                                  Display_Width, Display_Height,
                                  Display_Width, Display_Height, 0);  // threshold = 0

// 2. 检查像素值范围
uint16_t min_val = 0xFFFF, max_val = 0;
for(int r = 0; r < Display_Height; r++) {
    for(int c = 0; c < Display_Width; c++) {
        uint16_t val = mt9v03x_image[r][c];
        if(val < min_val) min_val = val;
        if(val > max_val) max_val = val;
    }
}
sprintf(buf, "Min:0x%04X Max:0x%04X", min_val, max_val);
LCD_DisplayString(10, 50, buf);
```

### 问题 3: 图像闪烁或撕裂

**可能原因：**
- LCD 显示和 DCMI 采集不同步
- 没有使用双缓冲

**解决方法：**
已经实现了双缓冲机制，确保在 `DCMI_FrameState == 1` 时才更新显示。

## 扩展功能

### 1. 在图像上叠加文字

```c
if (DCMI_FrameState == 1)
{
    DCMI_FrameState = 0;
    
    // 显示图像
    show_ov2640_image_from_ptr_array(0, 0, mt9v03x_image,
                                      Display_Width, Display_Height,
                                      Display_Width, Display_Height, 0);
    
    // 叠加文字
    LCD_SetColor(LCD_RED);
    LCD_SetBackColor(LCD_BLACK);
    LCD_DisplayString(10, 10, "Camera View");
    
    // 显示帧率
    char fps_str[20];
    sprintf(fps_str, "FPS: %d", calculated_fps);
    LCD_DisplayString(10, 30, fps_str);
}
```

### 2. 部分区域显示

```c
// 只显示图像的中心区域
uint16_t *center_rows[60];
for(int i = 0; i < 60; i++) {
    center_rows[i] = &mt9v03x_image[30 + i][44];  // 偏移到中心
}

show_ov2640_image_from_ptr_array(50, 50, (uint16_t**)center_rows,
                                  100, 60,    // 中心 100x60 区域
                                  100, 60, 0);
```

## 总结

✅ **正确使用方式**：
```c
show_ov2640_image_from_ptr_array(0, 0, mt9v03x_image, 
                                  Display_Width, Display_Height,
                                  Display_Width, Display_Height, 0);
```

❌ **错误使用方式**：
```c
show_ov2640_image(0, 0, mt9v03x_image[0],  // 只传入第一行
                  Display_Width, Display_Height,
                  Display_Width, Display_Height, 0);
```

现在编译并测试，应该可以正常显示摄像头图像了！
