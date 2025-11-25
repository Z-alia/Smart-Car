/*
 * Element_recognition.h
 *
 *  Created on: 2023��6��21��
 *      Author: Admin
 */

#ifndef CODE_CAMERA_PROCESS_ELEMENT_RECOGNITION_H_
#define CODE_CAMERA_PROCESS_ELEMENT_RECOGNITION_H_
#include "global.h"
#include "Binarization.h"
#include "scan_line.h"
#include "patch_line.h"
//#include "act_persp.h"
//#include "garage.h"
#include "circle.h"
#include "cross.h"
#include "slope.h"
//#include "broken_circuit.h"
//#include "black_obstacle.h"

typedef enum
{
    None=0,          //无元素       0
    Left_ring,       //左环         1
    Right_ring,      //右环         2
    Slope,           //坡道         3
    broken_circuit,  //断路         4
    obstacle,        //障碍物       5
    ingarage,        //入库         6
    outgarage,       //出库         7
    crossing,        //十字路口     8
    black_obstacle,  //小型黑色路障 9
    zebra,           //斑马线       10
    red_obstacle     //红色障碍物   11  ← 新增

}Element_range;
struct Element_struct
{
     uint8 Element_count;//Ԫ�ؼ���
     uint8 Element_data[21];
     uint8 loop_count;//Բ������
     uint8 loop_data[4];//Բ������
};
struct watch_o
{
    //int base_line;

    int cam_times;//��򷨼���
    uint8 brightness; //�������ͼ������
    uint8 threshold;  //ͼ���ֵ����ֵ


    int watch_line;
    int watch_lost;//����ͷ���ܿ�����������Զ��
    int loop_flag;

    int cpu0_fps;
    int cpu1_fps;
    short gray;

    //ͳ�ƶ�����ر���  ʹ��λ�ã�void count_lost(void)
    int cross;  //ͳ�ƶ����㷨�У���������ұ��߶����ߵ�����
    int left_lost; //ͳ�ƶ����㷨�У�����߶��ߵ�����
    int right_lost;//ͳ�ƶ����㷨�У��ұ��߶��ߵ�����
    int left_near_lost;//ͳ�ƶ����㷨��,����߿�ʼ���ߵ�����
    int right_near_lost;//ͳ�ƶ����㷨��,�ұ��߿�ʼ���ߵ�����

    //ֱ�߱�־��ر���      ʹ��λ�ã�void Straight_check(void);
    int RLStraight;//����ֱ�� 0����ֱ�� 1�������ֱ�ߣ��ұ��߲���ֱ�� 2���ұ�����ֱ������߲��� 3�����ұ��߶���ֱ��

    /*Բ��״̬0:�޻�
    1����⵽�󻷵�һ���ǵ㣬��ʱ������һ���߱���ֱ��
    2�������뻷����ʱ�Ҳಹ���뻷
    3���������ǻ���һ��ֵ����ʱ��ȫ�뻷
    4�������ǻ�����ɣ�׼������
    5����Բ����ֱ�� 6����⵽�һ���һ���ǵ㣿������
	*/
    int InLoop;
    //������Ǳ��� 1:����ʱ����ֱ����Բ�����紦
    int OutLoop;

    //Բ����־λ
    int InLoopAngleL;  //����ǰֱ�еĵ�һ���������У�ֱ����Բ�����ӵĽǵ㣩
    int InLoopAngleR;  //���һ�ǰֱ�еĵ�һ���������У�ֱ����Բ�����ӵĽǵ㣩
    int InLoopCirc;   //Բ����͹��
    int InLoopAngle2; //��ʼת���뻷ʱǰ���Ľǵ������У�ֱ����Բ�����ӵĽǵ㣩
    int InLoopAngle2_x;
    int InLoopAngle2_y;//��ʼת���뻷ʱǰ���Ľǵ�������
    int OutLoopAngle2; //������ֱ��ʱǰ���Ľǵ������У�ֱ����Բ�����ӵĽǵ㣩
    int OutLoopAngle1; //����ʱ���ϵĽǵ㣨����ʱ���Ҳ࣬���һ�ʱ����ࣩ
    int OutLoop_turn_point_x;//ת�������꣬���ݸõ���в���
    //int OutLoopRight;  //������תʱ��ǰ���ǵ������
    //int OutLoopRightY; //������תʱ��ǰ���ǵ�������
    //int OutLoopLeft;   //���һ���תʱ��ǰ���ǵ������
    //int OutLoopLeftY;  //���һ���תʱ��ǰ���ǵ�������
    int OutLoop2;
    int dis_Loop;
    int top_x;//������������ֱ�߽��㴦�ĺ�����ֵ����ʱy=115�����ڲ���
    float fix_slope;//����б��
    //˵������͸��ͼ����������ƽ���߽�������һ�㣬����ʵ��ó����ۣ�������������ƹ̶�Ϊ115��(������Ϊ��ƽ��)�����ݴ������в���




    //�Ƿ��������·��־����21��
    int Junc_flag; //0��1��2��3���뵫��֪������
    int DeltaR;    //��ʱ������ʾ
    int DeltaL;    //��ʱ������ʾ

    int JuncLineL; //����·�ǵ���������
    int JuncLineR; //����·�ǵ���������
    int JuncLine;  //ȡƽ��
    int JuncAngle;
    int JuncAngleX;  //ȡƽ��
    int JuncTime;  //����·ʱ��
    int junc_linefirstcontinual;//�����µ����жϴ��·���ʼ��������������
    int junc_func_linefirstcontinual;//�����µ����жϴ��·���ʼ�������߼�����
    //����·��־
    int T_flag;//1��2��3���4�ҳ�
    int T_Clear_Flag;//T���ڰ����ߴ������־λ
    int T_Angle;//�Ͻǵ�
    int T_outangleL;
    int T_outangleLX;
    int T_outangleR;
    int T_outangleRX;
    int T_outFlag;

    int zebra_flag;
    int zebra_flag2;
    int ZebraLine;
    int ZebraRowL;
    int ZebraRowR;
    int ZebraInLine;
    int8 zebra_stop;
    int stop_count;
    int Garge_line;
    int Gargeout_Flag;
    int Zebra_Angle;
    int Zebra_Angle2;
    int Zebra_Angle2_x;
    //ʮ�ֵı�־λ

    int cross_flag;
    int cross_RD_angle;//ʮ�����½ǵ�
    int cross_LD_angle;//ʮ�����½ǵ�

    int cross_line;
    uint32 DectectStart;
    int cross_lost;
    int cross_AngleL;
    int cross_AngleR;
    int cross_AngleL_x;
    int cross_AngleR_x;

    //ֱ�߼�����
    int16 StrNumL;      //ֱ�߼���㷨���õ��ĵ����С��ֱ�߶ε���Ŀ     ʹ��λ�ã�void Straight_check(void);
    int16 StrNumR;      //ֱ�߼���㷨���õ��ĵ��Ҳ�С��ֱ�߶ε���Ŀ
    float slopeL_max;   //�����б�����ֵ
    float slopeL_min;   //�����б����Сֵ
    float slopeR_max;   //�ұ���б�����ֵ
    float slopeR_min;   //�����б����Сֵ
    float d_angle_left; //����߽Ƕȱ仯
    float d_angle_right;//�ұ��߽Ƕȱ仯

    int April_flag;
    int AprilLine;
    uint32 AprilStart;

    uint32 FruitStart;
    uint32 LaserStart;
    uint32 AnimalStart;

    char servo_flag;
    uint32 ServoStart;

    int distance;





    int del_dir;
    int outloop7_angleflag;

    //·�̻������
    //int Dis_Flag;//·�̻��ֱ�־λ����1ʱ����·�̻��֣�������ɺ��Զ���2
    int16 distanceThres;     //������ֵ��·�̻��ֵ���ֵʱ����
    char Yaw_flag;   //�ǶȻ��ֱ�־ 0��δ��ʼ���� 1�����ڽ��нǶȻ��� 2���ǶȻ������

    char Beep_flag;
    int  Beep_time;

    int watchX;
    int watchY;
    int watchleft;//����е�������
    int watchright;//�Ҳ��е�������
    float watch_length;//ģ�����мǲ���Ϊ�㣬��Ȼ���bug
    ////��������Ȧ
    int8 turn_count;
    //�����߼���
    int zebra_count;
    //��·�м������
    int broken_circuit_line_count;
    //����·��Ǵ���
    uint8 broken_circuit_time_count;
    //��·���Ϊ  1:��ʼ�����·  0�����ڶ�· 2:��ȫ�����·
    uint8 broken_circuit_flag;
    //б�±��λ
    uint8 slope_flag;
    //С�ͺ�ɫ·�ϱ��λ
    uint8 black_obstacle_flag;
    uint8 left_obstacle_flag;
    uint8 right_obstacle_flag;
    int black_obstacle_line;
    int left_obstacle_x;
    int right_obstacle_x;
    //ǰ���ϰ������
    float forward_distance;
    uint8 obstacle_flag;  //���ϱ��λ
    uint8 obstacle_angle; //�ϰ���ǵ�
    uint8 Line_patrol_mode;//Ѳ��ģʽ  0:���� 1������ 2������
    uint8 out_garage_flag;//������λ 1�������ǰֱ�� 2�������ǳ��� 3���ҳ���ǰֱ�� 4�����Ҵ�ǳ���  0:���������������ʻ
    //ͣ�����λ
    uint8 garage_stop;
    uint8 garage_flag;
    //���������
    uint8 angle_far_line;
    uint8 angle_near_line;
    int track_count;
    int track_count_far;

    //ɨ�����ƣ������ֲַ�ʱ��0�������İ׿�Ϊ���� 1�������Ϊ��׼�������� 2�����Ҳ�Ϊ��׼��������
    uint8 scan_line_advantage;

};

extern struct watch_o watch;
extern Element_range Element;
extern struct Element_struct Element_rem;
void Element_recognition();
void enter_element(Element_range element);
void out_element();
void element_init();
void clear_all_flags();
void enter_task();
void count_lost();
#endif /* CODE_CAMERA_PROCESS_ELEMENT_RECOGNITION_H_ */
