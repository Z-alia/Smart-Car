#include "encoder.h"

void Encoder_Correct(MotorSpeed *speed_data)
{
    int32_t diff_left = speed_data->encoder_count_left - speed_data->last_encoder_count_left;
    int32_t diff_right = speed_data->encoder_count_right - speed_data->last_encoder_count_right;

    speed_data->motor_speed_left = (diff_left > 32767) ? (diff_left - 65536) : ((diff_left < -32767) ? (diff_left + 65536) : diff_left);
    speed_data->motor_speed_right = (diff_right > 32767) ? (diff_right - 65536) : ((diff_right < -32767) ? (diff_right + 65536) : diff_right);

    speed_data->last_encoder_count_left = speed_data->encoder_count_left;
    speed_data->last_encoder_count_right = speed_data->encoder_count_right;

}