#ifndef ELEMENT_RECOGNITION_H_
#define ELEMENT_RECOGNITION_H_
#include "main.h"
#include "type_def.h"

//祖传摄像头处理代码的结构体，里面主要是摄像头识别的赛道信息，后续可以在此添加自己的代码
struct watch_o
{
	/* 大津法使用 */
    uint8 threshold;  	//图像二值化阈值

	/* 摄像头视野 */
    int watch_line;
    int watch_lost;//摄像头所能看到赛道的最远端

    /* 统计丢线相关变量 */
    int cross;  			//统计丢线算法中，左边线与右边线都丢线的行数
    int left_lost; 			//统计丢线算法中，左边线丢线的行数
    int right_lost;			//统计丢线算法中，右边线丢线的行数
    int left_near_lost;		//统计丢线算法中,左边线开始丢线的行数
    int right_near_lost;	//统计丢线算法中,右边线开始丢线的行数

	/* 斑马线相关 */
    int ZebraInLine;
};

extern struct watch_o watch;

#endif /* CODE_CAMERA_PROCESS_ELEMENT_RECOGNITION_H_ */
