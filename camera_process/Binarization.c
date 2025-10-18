/*
 * Binarization.c
 *
 *  Created on: 2023年6月21日
 *      Author: Admin
 */
#include "Binarization.h"
#include "Element_recognition.h"
#include "dcmi_ov2640.h"

uint8 Grayscale[120][188];	//灰度图
uint8 imo[120][188]; 		//处理后的图像
uint8 img_threshold[12];

//struct caminfo_s caminfo;
//struct dir_control_struct cardir;
//struct watch_o watch;
//struct FLAG_STRUCT flag;

//大津法
#define _GRAY_SCALE 64
#define _SHIFT_FOR_GRAYSCALE 10
#define _Thresh_Mult 2 //1/2*(256/_GRAY_SCALE)

//大津法，计算二值化阈值
int img_otsu(uint16_t *img, uint8_t img_v, uint8_t img_h, uint8_t step)
{
    uint8_t grayhist[_GRAY_SCALE] = {0}; //灰度直方图
    uint16_t px_sum_all = 0;             //像素点总数
    uint32_t gray_sum_all = 0;           //总灰度积分
    uint16_t px_sum = 0;                 //像素点数量
    uint32_t gray_sum = 0;               //灰度积分

    float fTemp_maxvar = 0;
    uint8_t temp_best_th = 0;
    uint8_t temp_best_th2 = 0;
    uint8_t temp_this_pixel = 0;
    float fCal_var;
    float u0, u1, w0, w1;

    //生成：1. 灰度直方图 2. 像素点总数 3. 总灰度积分
    for (int i = 0; i < img_v; i += step)
    {
        for (int j = 0; j < img_h; j += step)
        {
            temp_this_pixel = (uint8)(img[i * img_h + j] >> _SHIFT_FOR_GRAYSCALE);
            if (temp_this_pixel < _GRAY_SCALE)
                grayhist[temp_this_pixel]++;
            gray_sum_all += temp_this_pixel;
            px_sum_all++;
        }
    }
    //  //总平均灰度
    //  float u = 1.0*gray_sum_all/px_sum_all;
    //迭代求得最大类间方差的阈值
    for (uint8_t k = 0; k < _GRAY_SCALE; k++)
    {
        px_sum += grayhist[k];       //该灰度及以下的像素点数量
        gray_sum += k * grayhist[k]; //该灰度及以下的像素点的灰度和
        w0 = 1.0 * px_sum / px_sum_all;
        w1 = 1.0 - w0;
        if (px_sum > 0)
		{
            u0 = 1.0 * gray_sum / px_sum;
		}
        else
		{
            u0 = 0.0;
		}
        if (px_sum_all - px_sum > 0)
		{
            u1 = 1.0 * (gray_sum_all - gray_sum) / (px_sum_all - px_sum);
		}
        else
		{
            u1 = 0.0;
		}
        //fCal_var = w0*(u0-u)*(u0-u)+w1*(u1-u)*(u1-u);
        fCal_var = w0 * w1 * (u0 - u1) * (u0 - u1);
        if (fCal_var > fTemp_maxvar)
        {
            fTemp_maxvar = fCal_var;
            temp_best_th = k;
            temp_best_th2 = k;
        }
        else if (fCal_var == fTemp_maxvar)
        {
            temp_best_th2 = k;
        }
    }
    return (temp_best_th + temp_best_th2) * _Thresh_Mult;
}





//图形二值化(全局阈值)
void Global_Binarization()
{
			// 大津法计算二值化阈值 
			watch.threshold = img_otsu((uint16_t *)mt9v03x_image[30], 60, Display_Width, 10); 
			
			// 二值化阈值限幅 
			if(watch.threshold>180)
			{
				watch.threshold=180;
			}
			else if(watch.threshold<160)
			{
				watch.threshold=160;
			}
    int row=0,colum;
    for(row=0;row<120;row++)
    {
        for(colum=0;colum<Display_Width;colum++)
        {
            if(mt9v03x_image[row][colum]<(watch.threshold<<8))
			{
                Grayscale[row][colum]=0;
			}
            else
			{
                Grayscale[row][colum]=255;
			}
        }
    }
}

// 自适应阈值二值化 (内存优化版)
// S: 窗口大小, T: 阈值百分比
void Adaptive_Binarization(int S, int T)
{
    int row, col;
    int s2 = S / 2;

    // 列积分缓冲区，用于存储S窗口大小的列像素值之和
    // 只需要一个188宽的一维数组，极大减少内存占用
    static unsigned int col_integral[188];
    
    // 滑动窗口的像素总和
    unsigned long long window_sum = 0;

    // 针对第一行特殊处理，初始化列积分缓冲区和第一个窗口的sum
    // 1. 计算前S/2+1行的列积分
    for (col = 0; col < 188; col++)
    {
        col_integral[col] = 0;
        for (row = 0; row <= s2; row++)
        {
            col_integral[col] += mt9v03x_image[row][col];
        }
    }
    // 2. 计算第一行第一个窗口的sum
    for (col = 0; col <= s2; col++)
    {
        window_sum += col_integral[col];
    }

    // 遍历所有像素
    for (row = 0; row < 120; row++)
    {
        // 确定当前行上下边界，用于更新列积分
        int top_row = row - s2 - 1; // 离开窗口的行
        int bottom_row = row + s2;  // 进入窗口的行

        // 从第二行开始，用滑动的方式更新列积分缓冲区
        if (row > 0)
        {
            for (col = 0; col < 188; col++)
            {
                // 减去离开窗口的行
                if (top_row >= 0)
                {
                    col_integral[col] -= mt9v03x_image[top_row][col];
                }
                // 加上进入窗口的行
                if (bottom_row < 120)
                {
                    col_integral[col] += mt9v03x_image[bottom_row][col];
                }
            }
        }
        
        // 重置第一个窗口的sum
        window_sum = 0;
        for (col = 0; col <= s2; col++)
        {
            window_sum += col_integral[col];
        }

        for (col = 0; col < 188; col++)
        {
            // 使用滑动窗口更新sum
            if (col > 0)
            {
                int left_col = col - s2 - 1;
                int right_col = col + s2;
                if (left_col >= 0)
                {
                    window_sum -= col_integral[left_col];
                }
                if (right_col < 188)
                {
                    window_sum += col_integral[right_col];
                }
            }

            // 计算窗口内的像素数量
            int x1 = (row - s2 > 0) ? row - s2 : 0;
            int x2 = (row + s2 < 119) ? row + s2 : 119;
            int y1 = (col - s2 > 0) ? col - s2 : 0;
            int y2 = (col + s2 < 187) ? col + s2 : 187;
            int count = (x2 - x1 + 1) * (y2 - y1 + 1);

            // 判断当前像素是黑是白
            if ((unsigned long long)mt9v03x_image[row][col] * count < window_sum * (100 - T) / 100)
                Grayscale[row][col] = 0;
            else
                Grayscale[row][col] = 255;
        }
    }
}

