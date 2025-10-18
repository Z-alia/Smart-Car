
//本文件留作接口文件，剩下的元素识别相关的需要自己移植
#include "Element_recognition.h"

#include "Binarization.h"
#include "main.h"



//小车状态变量
struct watch_o watch;

//环岛及弯道识别(环岛为二级检测)
void Island_loop_and_curve_recognition()
{    
    
}

//十字路口识别(判断更具特点，优先级最高)
void Cross_recognition()
{
    
}

//直道识别
void Straight_recognition(struct watch_o *watch,struct lineinfo_s lineinfo[])
{
    uint8_t line = 0;
    //if(watch->Straight_flag == 0)
    //{
    //watch->Straight_flag=check_midline_straight(90,110,20);    
    
        for (line = 60; line < 120; line++)
        {
            if(lineinfo[line].left_lost==1||lineinfo[line].right_lost==1)
                break; //有一行线丢失则跳出
        }
        if(line==120||(stable_curve_params.a<=0.002f&&stable_curve_params.a>=-0.002f)) //如果20~119行全部不丢线则认定为直线
        {
            int8_t temp=lineinfo[watch->PredictTopMidline].mid-94;
			if(temp>=-15&&temp<=15)
			{
            watch->Straight_flag=1; //直道标志位 置1，认为项目结束
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET);
			Clear_Recognition_Flag(watch);
			}
        }
		else
			watch->Straight_flag=0;
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET);
	
    if(watch->Straight_flag==1)
	{
		//直道标志位 置1，认为项目结束
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET);
		Clear_Recognition_Flag(watch);
	}
    //}
}

//标志位初始化(完成项目(进入项目后再次检测到直线)后调用)
void Clear_Recognition_Flag(struct watch_o *watch)
{
    watch->Curve_flag=0;
    watch->Cross_flag=0;
    watch->Crossroads_flag_left=0;
	watch->Crossroads_flag_right=0;
	watch->Curve_left_flag=0;
	watch->Curve_right_flag=0;
}
//补线(已有十字路口，可填其他)
void Patch_line(struct watch_o *watch,struct lineinfo_s lineinfo[])
{
	uint8_t line=0;
	if(watch->Cross_flag)
	{
		for(line=90;line>0;line--)
		{
			
		}
	}
		
}
//环岛检测状态机

//图像预处理


