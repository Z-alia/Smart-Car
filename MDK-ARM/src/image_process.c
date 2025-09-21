#include "stm32h7xx_hal.h"
#include "global.h"
//边缘检测
/**
 * @brief  高效的形态学边缘检测 (形态学梯度)
 * @param  output_image 用于存储边缘结果的缓冲区 (平坦数组)
 * @param  src_image    源图像的行指针数组
 * @param  width        图像宽度
 * @param  height       图像高度
 * @note   此函数通过单次遍历计算3x3邻域的(最大值-最小值)来实现边缘检测，
 *         无需额外的图像缓冲区，效率高，内存占用小。
 *         原始图像数据不会被修改。
 */

void image_edge_detection_optimized(Image *image , uint16_t width, uint16_t height)
{
    
    // 遍历每一个像素点 (忽略最外一圈的边界)
    for (int y = 1; y < height - 1; y++)
    {
        for (int x = 1; x < width - 1; x++)
        {
            uint16_t min_val = 0xFFFF; // 邻域最小值 (腐蚀)
            uint16_t max_val = 0x0000; // 邻域最大值 (膨胀)

            // 在一个循环内同时寻找3x3邻域的最大值和最小值
            for (int j = -1; j <= 1; j++)
            {
                for (int i = -1; i <= 1; i++)
                {
                    uint16_t pixel = image->original_image[y + j][x + i];
                    if (pixel < min_val)
                    {
                        min_val = pixel;
                    }
                    if (pixel > max_val)
                    {
                        max_val = pixel;
                    }
                }
            }
            // 目标像素值 = 最大值 - 最小值
            image->output_image[y][x] = max_val - min_val;
        }
    }
    
/*
    // (可选) 清除图像最外圈的边界，因为我们没有计算它们
    for(int x = 0; x < width; x++) {
        output_image[x] = 0; // 第一行
        output_image[(height - 1) * width + x] = 0; // 最后一行
    }
    for(int y = 0; y < height; y++) {
        output_image[y * width] = 0; // 第一列
        output_image[y * width + (width - 1)] = 0; // 最后一列
    }
}
*/
}
//中线拟合,输出边缘位置
void fit_line(Image *image, int width, float *slope, float *intercept) {
    // 存储中线点的数组
    uint8_t x_points[Display_Height];
    uint8_t y_points[Display_Height];
    uint8_t num_points = 0;

    // 遍历每一行，找到边缘像素的中点
    for (int y = scan_start; y < scan_end; y++) {
        int left_edge = -1;
        int right_edge = -1;

        // 从左到右寻找左边缘
        for (int x = 0; x < width; x++) {
            if (image->output_image[y][x] == 1) { // 阈值判断为边缘
                left_edge = x;
                break;
            }
        }

        // 从右到左寻找右边缘
        for (int x = width - 1; x >= 0; x--) {
            if (image->output_image[y][x] == 1) { // 阈值判断为边缘
                right_edge = x;
                break;
            }
        }
        image->x_leftedge[y]=left_edge;
        image->x_rightedge[y]=right_edge;
        // 如果找到了左右边缘，计算中点
        if (left_edge != -1 && right_edge != -1 && right_edge > left_edge) {
            x_points[num_points] = (left_edge + right_edge) / 2;
            num_points++;
        }
    } 
}
//句柄初始化
void image_process_init(Image *image)
{
    image->output_image[120][188]=0;
}
Image image_buf;
//识别赛道类型
void read_track_type(Image *image, uint16_t height) {
    uint16_t track_width=0;
    uint16_t sum=0;
    for(uint8_t i=0;i<(scan_end-scan_start);i++)
    {
        sum+=image->x_rightedge[scan_start+i]-image->x_leftedge[scan_start+i];
    }
    track_width=sum/(scan_end-scan_start);
    if(track_width>50)//参数待调
    {
        image->Type_flag=Straight_path;
    }
    else if(track_width>30 && track_width<=50)
    {
        image->Type_flag=Big_bend;
    }
    else if(track_width>20 && track_width<=30)
    {
        image->Type_flag=Small_bend;
    }
    else if(track_width>10 && track_width<=20)
    {
        image->Type_flag=Island_loop;
    }
    else if(track_width<=10)
    {
        image->Type_flag=Crossroads;
    }
    else if(track_width==0)
    {
        image->Type_flag=End;
    }
}
//误差值反馈beta
float calculate_deviation(Image *image) {
    // 选择一个参考点
    uint8_t reference_y = div_detect_row; // 图像底部某点
    // 计算偏差
    float deviation = image->x_points[reference_y] - 94; // 94为图像中心点横坐标

    return deviation;
}