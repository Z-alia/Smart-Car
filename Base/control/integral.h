/*
 * integeral.h
 *
 *  Created on: 2023年6月21日
 *      Author: Admin
 */

#ifndef CODE_CONTROL_INTEGRAL_H_
#define CODE_CONTROL_INTEGRAL_H_
#include "main.h"
typedef struct
{
   uint8_t integeral_flag; //积分标记位 0:未开始积分 1：正在积分 2：积分完成
   float integeral_data; //积分数据
   float integeral_Thres; //积分阈值
   //float last_data;      //上次数据

}integeral_STRUCT;
extern integeral_STRUCT angle_integral,distance_integral;
extern uint8_t get_integeral_state(integeral_STRUCT* integeral);
extern float get_integeral_data(integeral_STRUCT* integeral);
void begin_angle_integeral(int16_t target_angle);
void angal_integeral(float gyro_z);
void begin_distant_integeral(int target_distant);
void distant_integeral(float present_speed);
void clear_angle_integeral(void);
void clear_distant_integeral(void);




#endif /* CODE_CONTROL_INTEGRAL_H_ */
