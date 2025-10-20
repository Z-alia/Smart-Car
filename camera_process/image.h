#ifndef _IMAGE_H
#define _IMAGE_H
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

extern void image_process(void); //直接在中断或循环里调用此程序就可以循环执行了

extern uint8_t l_border[image_h];//左线数组
extern uint8_t r_border[image_h];//右线数组
extern uint8_t center_line[image_h];//中线数组

#endif /*_IMAGE_H*/

