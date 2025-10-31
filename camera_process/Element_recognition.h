#ifndef CODE_CAMERA_PROCESS_ELEMENT_RECOGNITION_H_
#define CODE_CAMERA_PROCESS_ELEMENT_RECOGNITION_H_
#include "main.h"
#include "type_def.h"

//祖传摄像头处理代码的结构体，里面主要是摄像头识别的赛道信息，后续可以在此添加自己的代码
#define image_h 120
//绘制边界线
void draw_edge();
/*
//生长方向序列匹配结构体
typedef struct {
    uint8_t end;              // 若匹配到序列 记录终止行号
    uint16_t matched;       // 是否完整匹配 (0 = false, 1 = true)
    uint16_t total_gap;     // 实际总间隔数 (越小越好)
    float   confidence;    // 置信度：1.0 = 完美连续, 0.0 = 间隔最大
} match_result;

//生长方向序列结构体
typedef struct{
    uint16_t outer_up[6];
    uint16_t inner_up[6];
    uint16_t up_outer[6];
    uint16_t up_inner[6];
    uint16_t up_outerdownarc[8];
    uint16_t outer_uparc[8];
}growth_array;

*/
struct watch_o
{
	/* 大津法使用 */
    uint8_t threshold;  	//图像二值化阈值

	/* 摄像头视野 */
    int watch_line;
    int watch_lost;//摄像头所能看到赛道的最远端

    /* 统计丢线相关变量 */
    int cross;  			//统计丢线算法中，左边线与右边线都丢线的行数
    int left_near_lost;		//统计丢线算法中,左边线开始丢线的行数
    int right_near_lost;	//统计丢线算法中,右边线开始丢线的行数
    uint8_t left_lost_num;	//左边线丢失次数
    uint8_t right_lost_num;	//右边线丢失次数
	/* 斑马线相关 */
    int ZebraInLine;

    /* 赛道类型标志位 */
    uint8_t Straight_flag;	//直道标志位
    uint8_t Curve_right_flag;	//右弯道标志位
    uint8_t Curve_left_flag;	//左弯道标志位
    uint8_t Curve_flag;		//弯道标志位
    uint8_t cross_flag;		//十字路口标志位
	uint8_t zebra_flag;		//斑马线标志位
	uint8_t black_obstacle_flag;	//黑色障碍标志位
    /*圆环状态0:无环
    1：检测到左环第一个角点，此时补左侧第一条线保持直行
    2：正在入环，此时右侧补线入环
    3：从陀螺仪积分一定值，此时完全入环
    4：陀螺仪积分完成，准备出环
    5：出圆环后直行 6：检测到右环第一个角点，之后类推*/
    uint8_t InLoop;
    //出环标记变量 1:出环时进入直道与圆环交界处
    uint8_t OutLoop;
	
	//圆环标志位
    uint8_t InLoopAngleL;  //入左环前直行的第一个角所在行（直道与圆环交接的角点）
    int InLoopAngleR;  //入右环前直行的第一个角所在行（直道与圆环交接的角点）
    int InLoopCirc;   //圆环上凸弧
    int InLoopAngle2; //开始转向入环时前方的角点所在行（直道与圆环交接的角点）
    int InLoopAngle2_x;
    int InLoopAngle2_y;//开始转向入环时前方的角点所在列
    int OutLoopAngle2; //出环后直行时前方的角点所在行（直道与圆环交接的角点）
    int OutLoopAngle1; //出环时边上的角点（出左环时在右侧，出右环时在左侧）
    int OutLoop_turn_point_x;//转向点横坐标，根据该点进行补线
	
    //十字的标志位
    int cross_RD_angle;//十字右下角点
    int cross_LD_angle;//十字左下角点

    int cross_line;
    uint32_t DectectStart;
    int cross_lost;
    int cross_AngleL;
    int cross_AngleR;
    int cross_AngleL_x;
    int cross_AngleR_x;
	//小型黑色路障标记位
    //uint8_t black_obstacle_flag;
    uint8_t left_obstacle_flag;
    uint8_t right_obstacle_flag;
    int black_obstacle_line;
    int left_obstacle_x;
    int right_obstacle_x;
	//补线
    int top_x;//赛道左右两条直线交汇处的横坐标值（此时y=115）用于补线
    uint8_t fl_border[image_h];//补线用的边界数组
    uint8_t fr_border[image_h];//补线用的边界数组

    int16_t CurrentY;           //当前的Y值
	int16_t LastLine;           //如果断线（不论是否接回） 最后有效中点行数
	int16_t CurrentMid;         //当前的的中点X值
	int16_t LastMid;            //之前的中点X值

    int16_t PredictTopMidline;  // 预测中线最大行数

	
	int16_t smd;
    uint8_t Midline_Lost_Count; //中线丢失行数
};

extern struct watch_o watch;

//宏定义
#define image_h	120//图像高度
#define image_w	188//图像宽度


#endif /* CODE_CAMERA_PROCESS_ELEMENT_RECOGNITION_H_ */
