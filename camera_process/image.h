#ifndef _IMAGE_H
#define _IMAGE_H
#include "type_def.h"
#include <stdint.h>
//绘制边界线
void draw_edge();


//宏定义
#define image_h	120//图像高度
#define image_w	188//图像宽度

#define white_pixel	255
#define black_pixel	0

#define bin_jump_num	1//跳过的点数
#define border_max	image_w-2 //边界最大值
#define border_min	1	//边界最小值	

#define USE_num	image_h*3	//定义找点的数组成员个数按理说300个点能放下，但是有些特殊情况确实难顶，多定义了一点

extern void image_process(void); //直接在中断或循环里调用此程序就可以循环执行了
void IPS_ShowEdge(uint8 *p, uint16 width, uint16 height);//牢学长逆透视

extern uint8 l_border[image_h];//左线数组
extern uint8 r_border[image_h];//右线数组
extern uint8 center_line[image_h];//中线数组
extern uint16 dir_r[(uint16)USE_num];//用来存储右边生长方向
extern uint16 dir_l[(uint16)USE_num];//用来存储左边生长方向
extern uint8 hightest;//最高点

#endif /*_IMAGE_H*/

