#include "control.h"
control_t control;

float get_speed(void)
{
    control.left_speed = 10.0f * (float)(control.lencoder_count - control.lencoder_count_last) * 6.28f * Radius / Encoder_PPR;
    control.right_speed = 10.0f * (float)(control.rencoder_count - control.rencoder_count_last) * 6.28f * Radius / Encoder_PPR;
    return (control.left_speed + control.right_speed) / 2.0f; // 返回平均速度 近似车身速度
}
