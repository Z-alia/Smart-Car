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

    /* 赛道类型标志位 */
    uint8_t Straight_flag;	//直道标志位
    uint8_t Curve_right_flag;	//右弯道标志位
    uint8_t Curve_left_flag;	//左弯道标志位
    uint8_t Curve_flag;		//弯道标志位
    uint8_t Cross_flag;		//十字路口标志位

    uint8_t Crossroads_flag_left;	//左环岛标志位
	uint8_t Crossroads_flag_right;//右环岛标志位
    uint8_t Left_Break_flag;	//左断续线标志位
    uint8_t Right_Break_flag;	//右断续线标志位
	
    int16_t CurrentY;           //当前的Y值
	int16_t LastLine;           //如果断线（不论是否接回） 最后有效中点行数
	int16_t CurrentMid;         //当前的的中点X值
	int16_t LastMid;            //之前的中点X值

    int16_t PredictTopMidline;  // 预测中线最大行数

	
	int16_t smd;
    uint8_t Midline_Lost_Count; //中线丢失行数
};

extern struct watch_o watch;

void Straight_recognition(struct watch_o *watch,struct lineinfo_s lineinfo[]);
void Clear_Recognition_Flag(struct watch_o *watch);
void Island_loop_and_curve_recognition(struct watch_o *watch,struct lineinfo_s lineinfo[]);
void Cross_recognition(struct watch_o *watch,struct lineinfo_s lineinfo[]);
/*
void Island_loop_and_curve_recognition(struct watch_o *watch);
void Cross_recognition(struct watch_o *watch);
void Straight_recognition(struct watch_o *watch);
void Clear_Recognition_Flag(struct watch_o *watch);*/

#endif /* CODE_CAMERA_PROCESS_ELEMENT_RECOGNITION_H_ */
