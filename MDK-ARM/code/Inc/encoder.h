#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"

//编码器数据
typedef struct{
  volatile int16_t encoder_count_left;
  volatile int16_t encoder_count_right;      // 保存当前编码器计数值
  volatile int16_t last_encoder_count_left;
  volatile int16_t last_encoder_count_right; // 保存上一次的计数值
  volatile int16_t motor_speed_left;
  volatile int16_t motor_speed_right; 
 }MotorSpeed;

//溢出修正
void Encoder_Correct(MotorSpeed *speed_data);

#endif /* __ENCODER_H */