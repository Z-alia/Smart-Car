#include "main.h"

#define Radius 0.03f //车轮半径，单位m
#define Encoder_PPR 256.0f //编码器每转脉冲数 256线

typedef struct
{
    float left_speed;
    float right_speed;
    int16_t lencoder_count;
    int16_t lencoder_count_last;
    int16_t rencoder_count;
    int16_t rencoder_count_last;
} control_t;

extern control_t control;

float get_speed(void);