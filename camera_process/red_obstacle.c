/*
 * red_obstacle.c
 * 红色障碍物识别与避障实现
 * Created on: 2025-11-23
 */

#include "red_obstacle.h"
#include "scan_line.h"
#include "Binarization.h"
#include "system_config.h"
#include "integral.h"
#include "red_obstacle_navigation.h"  // 航位推算避障

// ==================== 全局变量定义 ====================

red_obstacle_config_t red_obstacle_config;

// ==================== 初始化函数 ====================

/**
 * @brief 初始化红色障碍物检测模块
 */
void red_obstacle_init(void)
{
    // 默认配置参数
    red_obstacle_config.min_red_area = 100;      // 最小100像素
    red_obstacle_config.max_red_area = 5000;     // 最大5000像素
    red_obstacle_config.red_threshold_h = 10;    // H通道阈值(0-180)
    red_obstacle_config.red_threshold_s = 100;   // S通道阈值(0-255)
    red_obstacle_config.red_threshold_v = 100;   // V通道阈值(0-255)
    red_obstacle_config.detection_line_start = 30; // 从第30行开始检测
    red_obstacle_config.detection_line_end = 90;   // 到第90行结束
}

// ==================== 红色区域检测函数(用户实现接口) ====================

/**
 * @brief 红色区域检测函数
 * @param red_x 检测到的红色区域中心X坐标(输出)
 * @param red_y 检测到的红色区域中心Y坐标(输出)
 * @return 1=检测到红色障碍物, 0=未检测到
 * @note TODO: 用户需要根据实际硬件实现此函数
 *       方法1: 使用颜色传感器(TCS34725等)
 *       方法2: 使用OV2640的彩色图像进行HSV颜色空间分割
 *       方法3: 使用OpenMV等视觉模块识别
 */
uint8_t detect_red_area(int16_t* red_x, int16_t* red_y)
{
    // ==================== 示例实现(基于图像扫描) ====================
    // TODO: 这里需要根据实际情况修改为彩色图像处理
    
    uint16_t red_pixel_count = 0;
    int32_t red_x_sum = 0;
    int32_t red_y_sum = 0;
    
    // 简化版:扫描图像检测特定亮度区域(实际应使用颜色信息)
    for (int y = red_obstacle_config.detection_line_start; 
         y < red_obstacle_config.detection_line_end; 
         y++)
    {
        for (int x = 0; x < 188; x++)
        {
            // TODO: 实际应使用彩色图像的HSV颜色判断
            // 这里只是示例:检测高亮度区域
            // if (is_red_color(mt9v03x_image[y][x])) {
            //     red_pixel_count++;
            //     red_x_sum += x;
            //     red_y_sum += y;
            // }
        }
    }
    
    // 判断是否检测到足够大的红色区域
    if (red_pixel_count >= red_obstacle_config.min_red_area && 
        red_pixel_count <= red_obstacle_config.max_red_area)
    {
        *red_x = (int16_t)(red_x_sum / red_pixel_count);
        *red_y = (int16_t)(red_y_sum / red_pixel_count);
        return 1;  // 检测到
    }
    
    return 0;  // 未检测到
}

// ==================== 状态机函数实现 ====================

/**
 * @brief 红色障碍物进入检测
 * @note 在Element==None时调用,扫描图像检测红色障碍物
 */
void red_obstacle_enter(void)
{
    // 时间限制:避免启动时误识别
    if (mycar.RUNTIME < setpara.begin_time) {
        return;
    }
    
    // 只在无元素状态下检测
    if (Element != None) {
        return;
    }
    
    // 检测红色区域
    int16_t red_x = 0, red_y = 0;
    if (detect_red_area(&red_x, &red_y))
    {
        // 记录障碍物位置（供调试使用）
        watch.left_obstacle_x = (red_x < 94) ? red_x : 0;
        watch.right_obstacle_x = (red_x >= 94) ? red_x : 187;
        watch.black_obstacle_line = red_y;
        
        // 进入红色障碍物元素
        enter_element(red_obstacle);
        
        // 启动航位推算避障
        red_obstacle_navigation_start();
        
        // 可选:蜂鸣器提示
        // beep2(2, 100);
    }
}

/**
 * @brief 红色障碍物避障处理
 * @note 使用航位推算避障策略（不依赖图像）
 */
void red_obstacle_avoid(void)
{
    // 使用航位推算避障（完全出赛道策略）
    red_obstacle_navigation_avoid();
}

/**
 * @brief 红色障碍物退出处理
 * @note 完成避障后退出元素
 */
void red_obstacle_out(void)
{
    // 检查航位推算避障是否完成
    if (red_obstacle_navigation_is_finished())
    {
        // 退出红色障碍物元素
        out_element();
        
        // 重置航位推算状态
        red_obstacle_navigation_reset();
        
        // 可选:蜂鸣器提示
        // beep2(1, 50);
    }
}

// ==================== 使用说明 ====================
/*
红色障碍物元素使用指南:

一、硬件要求
────────────────────────────────────────────────
1. 颜色识别方案(选择其一):
   - 方案A: TCS34725颜色传感器 + I2C接口
   - 方案B: OV2640彩色图像 + HSV颜色分割算法
   - 方案C: OpenMV视觉模块 + UART通信

2. 实现 detect_red_area() 函数:
   uint8_t detect_red_area(int16_t* red_x, int16_t* red_y) {
       // TODO: 读取颜色传感器/处理彩色图像
       // 返回1表示检测到红色,同时输出坐标
   }

二、在状态机中集成
────────────────────────────────────────────────
1. 在 Element_recognition.h 中添加枚举:
   typedef enum {
       ...
       red_obstacle = 11,  // 红色障碍物
   } Element_range;

2. 在 Element_recognition() switch中添加:
   case red_obstacle:
       red_obstacle_avoid();  // 避障处理
       red_obstacle_out();    // 退出检测
       break;

3. 在 enter_task() 中添加检测:
   case 0:  // 自动扫描模式
       red_obstacle_enter();  // 检测红色障碍物
       break;

三、参数调整
────────────────────────────────────────────────
red_obstacle_init();  // 初始化时调用

// 调整检测参数
red_obstacle_config.min_red_area = 200;      // 最小面积
red_obstacle_config.detection_line_start = 40; // 检测起始行
red_obstacle_config.detection_line_end = 80;   // 检测结束行

四、避障策略
────────────────────────────────────────────────
- 障碍物在左侧 → 右绕行
- 障碍物在右侧 → 左绕行  
- 障碍物在中间 → 停车等待

五、调试建议
────────────────────────────────────────────────
1. 打印检测信息:
   printf("Red detected at (%d,%d)\r\n", red_x, red_y);

2. 显示检测区域:
   // 在imo图像上标记检测到的红色区域

3. VOFA+波形监控:
   send_to_vofa((float)red_x, (float)red_y, (float)watch.obstacle_flag, 0);

═══════════════════════════════════════════════════════════
*/
