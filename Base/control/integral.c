/*
 * integeral.c
 *
 *  Created on: 2023年6月21日
 *      Author: Admin
 */
#include "integral.h"
integeral_STRUCT angle_integral,//角度积分结构体
                distance_integral;//路程积分结构体
//开始角度积分，入口参数为目标角度
void begin_angle_integeral(int16_t target_angle)
{
    angle_integral.integeral_data=0;
    angle_integral.integeral_Thres=target_angle;
    angle_integral.integeral_flag=1;
}
//清除角度积分变量
void clear_angle_integeral(void)
{
    angle_integral.integeral_data=0;
    angle_integral.integeral_flag=0;
}
//角度积分函数，放在陀螺仪读取数据的定时器中断中
void angal_integeral(float gyro_z)
{
    if(angle_integral.integeral_flag==1)
    {
        angle_integral.integeral_data+=gyro_z;
        if(angle_integral.integeral_Thres>0)
        {
            if(angle_integral.integeral_data>angle_integral.integeral_Thres)
            {
                angle_integral.integeral_flag=2;
                //angle_integral.integeral_data=0;
            }
        }
        else
        {
            if(angle_integral.integeral_data<angle_integral.integeral_Thres)
            {
                angle_integral.integeral_flag=2;
                //indata.YawAngle=0;
            }
        }
    }
}
//开始路程积分函数，入口参数为目标积分距离
void begin_distant_integeral(int target_distant)
{
    distance_integral.integeral_flag=1;
    distance_integral.integeral_data=0;
    distance_integral.integeral_Thres=target_distant;
}
//清除路程积分变量
void clear_distant_integeral(void)
{
    distance_integral.integeral_flag=0;
    distance_integral.integeral_data=0;
}
//路程积分函数，放在读取编码器的中断函数里
void distant_integeral(float present_speed)  //路程积分
{
    if(distance_integral.integeral_flag==1)
    {
        distance_integral.integeral_data+=present_speed;
        if(distance_integral.integeral_data>distance_integral.integeral_Thres)
        {
            //distance_integral.integeral_data=0;
            distance_integral.integeral_flag=2;
        }
    }
}
//获取积分状况
uint8_t get_integeral_state(integeral_STRUCT* integeral)
{
    return integeral->integeral_flag;
}
//获取积分数据
float get_integeral_data(integeral_STRUCT* integeral)
{
    return integeral->integeral_data;
}



