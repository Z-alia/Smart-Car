#ifndef ACT_PERSP_H_
#define ACT_PERSP_H_
//#include "global.h"
#include "main.h"
#define uint8 uint8_t
#define int16 int16_t
struct persp_struct
{
      uint8 persp_x[188];
      int16 y;
};
void act_perst_init();
void persp_task(int16 xl,int16 xr,int16 y);
int16 get_persp_data_ox(int16 x,int16 y);
int16 get_persp_data_oy(int16 x,int16 y);



#endif 