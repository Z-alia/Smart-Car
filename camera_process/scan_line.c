/*
 * scan_line.c
 *
 *  Created on: 2023年6月21日
 *      Author: Admin
 */
 #include <stdlib.h>
#include <math.h>
#include <string.h>
#include "scan_line.h"
#include "Binarization.h"
#include "Element_recognition.h"

struct lineinfo_s lineinfo[120];
#define LINE_WIDTH 188

void scan_line()
{
    //watch.base_line = 0;
    int y=0;
    memset(lineinfo, 0, 120 * sizeof(struct lineinfo_s));
    line_single(&lineinfo[base_line], Grayscale[119-base_line]);   // 寻找基准行

    for (y = base_line-1; y >= forward_near; y--) // 向下搜线到near行
    {
        line_findnext(&lineinfo[y], Grayscale[119-y], &lineinfo[y + 1]);
    }

    for (y = base_line+1; y < forward_far; y++) // 向上搜线到far行
    {
        lineinfo[y].y = y;
        line_findnext(&lineinfo[y], Grayscale[119-y], &lineinfo[y - 1]);
        watch.watch_line = y;
    }
    //line_findnext(&lineinfo[base_line], Grayscale[119-base_line], &lineinfo[base_line - 1]);
}


/*寻找基准行
 * ****************************************************************************
 */
int line_single(struct lineinfo_s *lineinfo, unsigned char *inputimg)
{
    //得到所有凸边沿
    uint8_t edge_store[_EDGE_STORE_SIZE] = {0};
	//扫描inputimg数组（即行数组），并将跳变沿所对应的列数依次放到edge_store数组中
    lineinfo->edge_count = get_orign_edges(inputimg, edge_store);

    //得到最大边沿
    get_max_edge(lineinfo, edge_store);
    return 0;
}


/*
 * 从基准行向下搜线
 */
int line_findnext(struct lineinfo_s *lineinfo, uint8_t *inputimg, struct lineinfo_s *lineinfo_ref)
{
    //得到所有凸边沿
    //uint8_t edge_store[_EDGE_STORE_SIZE] = {0};
    uint8_t *edge_store = lineinfo->edge_store;
    lineinfo->edge_count = get_orign_edges(inputimg, edge_store);
     //watch.addline_y = lineinfo->y;
     if (lineinfo->y > 20 && lineinfo->y < 100)
	{
         zebra_detect(lineinfo, edge_store, inputimg); //斑马线检测
         //apriltag_detect(lineinfo, edge_store, inputimg); //apriltag检测
     }
    //得到最佳边沿
    get_best_edge(lineinfo, edge_store, lineinfo_ref);
    //Junc_detect(lineinfo, edge_store, inputimg); //apriltag检测

    return 0;
}


/*
 *存储所有黑白跳变沿位置到数组edge_store中
 */
uint8_t get_orign_edges(uint8_t *inputimg, uint8_t *edge_store)
{
	#if _EDGE_STORE_SIZE % 2 != 0
	#error "_EDGE_STORE_SIZE must be even!"
	#endif
    //查找并存储所有跳变沿对
    uint8_t edge_store_idx = 0;
    if (inputimg[0] >watch.threshold)//数组第0位大于阈值（图像最左端为白）则认为此处为跳变沿
    {
        edge_store[edge_store_idx] = 0;
        edge_store_idx++;
    }
    for (uint8_t px = 1; px < LINE_WIDTH; px++)
    {
        //if ((inputimg[px - 1]-watch.threshold)*(inputimg[px]-watch.threshold) <= 0) //分布在watch.threshold两侧认为是跳边沿。。不稳定
        if(!((inputimg[px-1]>watch.threshold&&inputimg[px]<watch.threshold)||(inputimg[px-1]<watch.threshold&&inputimg[px]>watch.threshold)))    //可变灰度区分值
            continue;
        if (edge_store_idx >= _EDGE_STORE_SIZE)
            break;
        edge_store[edge_store_idx] = px;   //将跳变沿存入数组
        edge_store_idx++;
    }
    if (inputimg[LINE_WIDTH - 1] >watch.threshold) //数组最后一位大于阈值（图像最右端为白）则认为此处为跳变沿
    {
        if (edge_store_idx < _EDGE_STORE_SIZE)
        {
            edge_store[edge_store_idx] = LINE_WIDTH - 1;
            edge_store_idx++;
        }
    }
    return edge_store_idx;
}


/*得到最大边沿
 * ***************************************************************************
 */
int get_max_edge(struct lineinfo_s *lineinfo, uint8_t *edge_store)
{
    //选取最大的块    --可能的BUG:此处无法忽略噪点
    int max_width = 0;
    int max_idx = 0;

    if (lineinfo->edge_count < 10)
    {
        for (int k = 0; k < lineinfo->edge_count; k += 2)
        {
            if (edge_store[k + 1] - edge_store[k] > max_width)     //获取相差最大的跳变沿
            {
                max_width = edge_store[k + 1] - edge_store[k];
                max_idx = k;
            }
        }
    }
    else
    {
        max_idx = 0;
    }

    lineinfo->left = edge_store[max_idx];
    lineinfo->right = edge_store[lineinfo->edge_count - 1]; //这个地方保持疑问
  // 可能是这个lineinfo->right = edge_store[max_idx + 1];
    //lineinfo->right = edge_store[max_idx + 1];
    //Send_two_DataToVofa(lineinfo->left,lineinfo->right);
    return 0;
}


/*
 * 寻找最佳边沿
 * 根据相邻的一行找到的左边沿和右边沿位置,迭代找出本行与之距离最小的边沿位置
 */

int get_best_edge(struct lineinfo_s *lineinfo, uint8_t *edge_store, struct lineinfo_s *lineinfo_ref)
{
    int min_abs_err;//最小偏差,在迭代过程中这个值越来越小
    uint8_t temp_left_idx = 0, temp_right_idx = 1;
    int temp_err;//本轮迭代下的偏差量
    //获得最近邻的边沿
    min_abs_err = LINE_WIDTH * 2;
    for (uint8_t k = 0; k < lineinfo->edge_count; k += 2)
    {
        temp_err = abs((int)lineinfo_ref->left - (int)edge_store[k]);
        if (temp_err < min_abs_err)
        {
            min_abs_err = temp_err;
            temp_left_idx = k;
        }
        if (edge_store[k] > lineinfo_ref->left + min_abs_err)
            break;
    }
    temp_right_idx = temp_left_idx + 1;
    //Send_four_DataToVofa(min_abs_err,lineinfo_ref->left,lineinfo_ref->y,lineinfo_ref->left_lost*50);
    min_abs_err = LINE_WIDTH * 2;
    for (uint8_t k = temp_right_idx; k < lineinfo->edge_count; k += 2)
    {
        temp_err = abs((int)lineinfo_ref->right - (int)edge_store[k]);
        if (temp_err < min_abs_err)
        {
            min_abs_err = temp_err;
            temp_right_idx = k;
        }
        if (edge_store[k] > lineinfo_ref->right + min_abs_err)
            break;
    }
    //Send_four_DataToVofa(min_abs_err,0,lineinfo_ref->y,0);
   // if (abs((int)edge_store[temp_right_idx] - edge_store[temp_left_idx]) > 4)
    if (lineinfo->y>20&&lineinfo->edge_count>1&&(edge_store[temp_right_idx] - edge_store[temp_left_idx]) > 4  )
    {
        watch.watch_lost = watch.watch_line;
        lineinfo->whole_lost=0;
        // camcfg.watch_lost=  camcfg.watch_line;
    }
    else
    {
        lineinfo->whole_lost=1;
    }

    //判断中间是否有杂点，障碍物，环岛等
    //累加中间的黑色部分求取比例
    int black_pix_count = 0;
    for (int k = temp_left_idx + 1; k < temp_right_idx; k += 2)
	{
        black_pix_count += edge_store[k + 1] - edge_store[k];
	}
    if (8 * black_pix_count > edge_store[temp_right_idx] - edge_store[temp_left_idx]) //如果有不可忽略的杂物
    {
        //选取最大的块    --可能的BUG:此处无法继续忽略噪点
        int min_width = 188;
        int max_idx = temp_left_idx;

        int dst = temp_left_idx + 1;
        uint8_t dst_flag;
        for (int i = temp_left_idx + 1; i < temp_right_idx; i += 2)
        {
            if (edge_store[i + 1] - edge_store[i] < _MIN_EDGE_WIDTH)
            {
                edge_store[dst] = edge_store[i + 2];
                lineinfo->edge_count -= 2;
                dst_flag = 1;
            }
            //  else中内容注释掉现象依然正确  但可能有坑
            else
            {
                if (dst_flag == 1) //如果之前已经出现过被忽略的黑块
                {
                    edge_store[dst + 2] = edge_store[i];
                    edge_store[dst + 3] = edge_store[i + 1];
                }
                else // 当前不可忽略的黑块为第一块
                {
                    edge_store[dst] = edge_store[i];
                    edge_store[dst + 1] = edge_store[i + 1];
                }
                dst += 2;
            }
        }

        for (int k = temp_left_idx; k < lineinfo->edge_count - 1; k += 2)
        {
            if (abs(edge_store[k + 1] + edge_store[k]-188) < min_width)
            {
                min_width = abs(edge_store[k + 1] + edge_store[k]-188);
                max_idx = k;
            }
        }
        //if(watch.scan_line_advantage==0)
        {
            temp_left_idx = max_idx;
            temp_right_idx = max_idx + 1;
        }
//        else if(watch.scan_line_advantage==1)
//        {
//            temp_right_idx=temp_left_idx+1;
//        }
//        else if(watch.scan_line_advantage==2)
//        {
//            temp_left_idx=temp_right_idx-1;
//        }
    }

    lineinfo->left = edge_store[temp_left_idx];
    lineinfo->right = edge_store[temp_right_idx];
    if (lineinfo->right >= 187)
	{
        lineinfo->right_lost = 1;
	}
    else
	{
        lineinfo->right_lost = 0;
	}
    if (lineinfo->left <= 0)
	{
        lineinfo->left_lost = 1;
	}
    else
	{
        lineinfo->left_lost = 0;
	}
    return 0;
}

//斑马线识别函数
int zebra_detect(struct lineinfo_s *lineinfo, uint8_t *edge_store, uint8_t *inputimg)
{
    uint8_t white_width, zobra_white_count, edge_now;
    zobra_white_count = 0;
    if (lineinfo->edge_count > 12)
    {
		for (uint8_t k = 0; k < lineinfo->edge_count; k += 2)
		{
			edge_now = edge_store[k];
			if (edge_now < LINE_WIDTH)
			{
				if ((inputimg[edge_now] & 0x80) == 0x80)
				{
					white_width = edge_store[k + 1] - edge_store[k];
					if (white_width > 1 && white_width < 14)
					{
						zobra_white_count++;
						watch.ZebraInLine=lineinfo->y;//取满足条件时y最大值作为ZebraLine
					}                 //watch.ZebraInLine为看到的斑马线最远端
				}
			}
		}
		if (zobra_white_count > 6)
		{
			lineinfo->zebra_flag = 1;
			//beep(20);
			// if(!mycar.status)
			//ips200_draw_horizon(0, 119 - lineinfo->y, 188, 119 - lineinfo->y, BLUE);
		}
	}
    else
    {
        lineinfo->zebra_flag = 0;
    }
    return 0;
}





