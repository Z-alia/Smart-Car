/*
 * Binarization.c
 *
 *  Created on: 2023年6月21日
 *      Author: Admin
 */
#include "Binarization.h"
#include "Element_recognition.h"
#include "dcmi_ov2640.h"
#include "scan_line.h"

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

//图形二值化
void Binarization()
{
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

