#ifndef _IMAGE_H
#define _IMAGE_H
#include <stdint.h>
#include "control.h"
//绘制边界线
void draw_edge();
/*
//生长方向序列匹配结构体
typedef struct {
    uint16_t start;            // 若匹配到序列 记录起始行号（匹配第一个元素的位置）
    uint16_t end;              // 若匹配到序列 记录终止行号（匹配最后一个元素的位置）
    uint8_t matched;       // 是否完整匹配 (0 = false, 1 = true)
    uint8_t total_gap;     // 实际总间隔数 (越小越好)
    float   confidence;    // 置信度：1.0 = 完美连续, 0.0 = 间隔最大
} match_result;

//生长方向序列结构体
typedef struct{
    uint16_t up[6];
    uint16_t outer[6];
    uint16_t up_inner[6];
    uint16_t inner[6];
    uint16_t corner1[6];
}growth_array;

*/
//宏定义
#define image_h	120//图像高度
#define image_w	188//图像宽度

#define white_pixel	255
#define black_pixel	0

#define bin_jump_num	1//跳过的点数
#define border_max	image_w-2 //边界最大值
#define border_min	1	//边界最小值	

extern void image_process(void); //直接在中断或循环里调用此程序就可以循环执行了
/*
extern match_result match_strict_sequence_with_gaps(
    const uint16_t* input,     // 输入序列
    size_t         input_len,
    const uint16_t* pattern,    //目标模式序列
    size_t         pattern_len,
    uint16_t        max_gap,      // 允许的最大单段间隔
    size_t         start_pos,      // 起始匹配位置（新增参数）
    int8_t         direction
);
extern match_result match_strict_sequence_with_gaps_u8(
    const uint8_t* input,     // 输入序列
    size_t         input_len,
    const uint8_t* pattern,    //目标模式序列
    size_t         pattern_len,
    uint8_t        max_gap,      // 允许的最大单段间隔
    size_t         start_pos,      // 起始匹配位置（新增参数）
    int8_t         direction       // 匹配方向：1=正向，-1=反向
);
*/
extern uint8_t l_border[image_h];//左线数组
extern uint8_t r_border[image_h];//右线数组
extern uint8_t center_line[image_h];//中线数组
extern uint8_t left_lost_num;//左线丢失总行数
extern uint8_t right_lost_num;//右线丢失总行数
extern uint8_t left_lost[image_h];//左线丢失标志数组
extern uint8_t right_lost[image_h];//右线丢失标志数组
extern int16_t persp_lx[120];      // 左线逆透视x坐标(透视变换后)
extern int16_t persp_rx[120];      // 右线逆透视x坐标(透视变换后)
extern uint8_t persp_ly[120];      // 左线逆透视y坐标(透视变换后)
extern uint8_t persp_ry[120];      // 右线逆透视y坐标(透视变换后)
extern uint8_t left_straight;
extern uint8_t right_straight;
extern uint8_t straight;

extern uint8_t last_left_lost_up;
extern uint8_t last_right_lost_up;

#endif /*_IMAGE_H*/

