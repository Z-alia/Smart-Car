
//本文件留作接口文件，剩下的元素识别相关的需要自己移植
#include "Element_recognition.h"
#include "scan_line.h"
#include "Binarization.h"

//小车状态变量
struct watch_o watch;

//环岛及弯道识别(环岛为二级检测)
void Island_loop_and_curve_recognition(struct watch_o *watch,struct lineinfo_s lineinfo[])
{    
    uint8_t line=0;
	uint16_t flag_lt=0,flag_rt=0;
    //弯道识别代码
    if(watch->Curve_flag == 0 && watch->Cross_flag == 0 && watch->Crossroads_flag == 0)
    {
        for (line = 10; line < 60; line++)//10 60
        {
            if(lineinfo[line].left_lost==0&&lineinfo[line].right_lost==0)
                break; //有一行线不丢失则跳出
        }
        if(line==60) //如果1~69行全部丢线则认定为弯道
        {
            watch->Curve_flag=1;
            watch->Straight_flag=0; //检测到弯道则直道标志位清零
        }
		if(watch->Curve_flag==1)
		{
			for(line=50;line<60;line++)
			{
				if(lineinfo[line].left_lost==1&&lineinfo[line].right_lost==0)
				{
					flag_lt++;
				}
				if(lineinfo[line].left_lost==0&&lineinfo[line].right_lost==1)
				{
					flag_rt++;
				}
			}
			if(flag_lt>flag_rt)
			{
				watch->Curve_left_flag=1;
			watch->Curve_right_flag=0;
			}
			else
			{
				watch->Curve_right_flag=1;
			watch->Curve_left_flag=0;
			}
			
			
		}		
    }
    line=0;
/*
    //环岛识别代码
    if(watch->Curve_flag == 1 && watch->Cross_flag == 0 && watch->Crossroads_flag == 0)
    {
        uint8_t flag=0;
        for (uint8_t line = 100; line > 80; line--)
        {
            for (uint8_t px = 1; px < 188 ; px++)
            {
                //if ((inputimg[px - 1]-watch.threshold)*(inputimg[px]-watch.threshold) <= 0) //分布在watch.threshold两侧认为是跳边沿。。不稳定
                if(!((imo[line][px-1]>watch->threshold&&imo[line][px]<watch->threshold)||(imo[line][px-1]<watch->threshold&&imo[line][px]>watch->threshold)))    //可变灰度区分值
                    continue;
                if (edge_store_idx >= _EDGE_STORE_SIZE)
                    break;
                edge_store_idx++;
            }
            if( edge_store_idx >= 8)
            {
                // 进行标志位重置
               flag++;
            }
            edge_store_idx=0;
        }
        if(flag>=10) //连续检测到10次则认定为环岛
        {
            watch->Crossroads_flag=1;
            watch->Curve_flag=0; //检测到环岛则弯道标志位清零
            watch->Straight_flag=0; //检测到环岛则直道标志位清零
        }
        else
        {
            watch->Crossroads_flag=0;
        }
    }
        */
}

//十字路口识别(判断更具特点，优先级最高)
void Cross_recognition(struct watch_o *watch,struct lineinfo_s lineinfo[])
{
    uint8_t line = 0;
    if(watch->Cross_flag == 0)
    {
        for (line = 60; line < 80; line++)
        {
            if(lineinfo[line].left_lost==0&&lineinfo[line].right_lost==0)
                break; //有一行线不丢失则跳出
        }
        if(line==80) //如果60~79行全部丢线则认定为十字路口
        {
            watch->Cross_flag=1;
            //watch->Straight_flag=0; //检测到弯道则直道标志位清零
        }
    }
}

//直道识别
void Straight_recognition(struct watch_o *watch,struct lineinfo_s lineinfo[])
{
    uint8_t line = 0;
    //if(watch->Straight_flag == 0)
    //{
        
    
        for (line = 30; line < 120; line++)
        {
            if(lineinfo[line].left_lost==1&&lineinfo[line].right_lost==1)
                break; //有一行线丢失则跳出
        }
        if(line==120) //如果20~119行全部不丢线则认定为直线
        {
            
            watch->Straight_flag=1; //直道标志位 置1，认为项目结束
			Clear_Recognition_Flag(watch);
        }
    
    //}
}

//标志位初始化(完成项目(进入项目后再次检测到直线)后调用)
void Clear_Recognition_Flag(struct watch_o *watch)
{
    watch->Curve_flag=0;
    watch->Cross_flag=0;
    watch->Crossroads_flag=0;
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
