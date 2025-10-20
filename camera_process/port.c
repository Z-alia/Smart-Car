/*
 * port.c
 *
 *  Created on: 2023年6月20日
 *      Author: Admin
 */
#include "port.h"
/*******与flash相关的接口函数**********************************************************/

//----------------------------------------------------------------------
//flash 擦除一页的全部内容的函数
// 参数说明    页码数
//----------------------------------------------------------------------
//如果实在不会移植flash可以把下面这三个函数的内容全注释掉，即不使用flash存储功能（注意不是注释整个函数，只注释其内容）
void GUI_flash_erase_page(u32 page_num)
{
    flash_erase_page(0,page_num);
}

//----------------------------------------------------------------------
//flash 读取一页内容的函数
// 参数说明   page_num：页码数
// 参数说明   buf：读取后数据所存放的地址（数组地址）
// 参数说明   len：数据数量
//----------------------------------------------------------------------
void GUI_flash_read_page( u32 page_num, u32 *buf, u16 len)
{
    flash_read_page(0,page_num,buf,len);
}

//----------------------------------------------------------------------
//flash 写入一页内容的函数
// 参数说明   page_num：页码数
// 参数说明   buf：要写入的数据所存放的地址（数组地址）
// 参数说明   len：数据数量
//----------------------------------------------------------------------
void GUI_flash_write_page(u32 page_num, const u32 *buf, u16 len)
{
    flash_write_page (0,page_num,buf,len);
}

/***********************************************************************************/

/*******显示函数相关的接口函数***********************************************************/

//----------------------------------------------------------------------
// 字符串显示函数
// 参数说明     x               坐标x方向的起点，单位为一像素
// 参数说明     y               坐标y方向的起点，单位为一行
// 参数说明     dat             需要显示的字符串
//----------------------------------------------------------------------
void GUI_display_string(u16 x,u16 y,const char dat[])//x的单位为像素，y的单位为一行，千万不要弄错
{
    ips200_show_string(x,y*12,dat);
}

/***********************************************************************************/
#define long_press_time 750
//按键扫描函数
void key_scan()
{
    static uint8 key_press;
    static uint16 key_press_time;
    static uint16 key_interval_time;
    if(key_press==0&&(KEY1_PIN==0||KEY2_PIN==0||KEY3_PIN==0||PRESS_PIN==0))
   {
       //GUI_delay_ms(1);
       key_press=1;//检测到有按钮压下
       if(KEY1_PIN==0)
           {
               tsui.buttonevent=UP;          //发送指令
               //tsui.buttonevent=CCW;
           }
       else if(KEY2_PIN==0)
           {
               tsui.buttonevent=DOWN;          //发送指令
               //tsui.buttonevent=CW;
           }
       else if(KEY3_PIN==0)
           {
               tsui.buttonevent=OK;          //发送指令
               //tsui.buttonevent=PRESS;
           }
       else if(PRESS_PIN==0)
           {
               tsui.buttonevent=PRESS;   //发送指令
               if(mycar.start_warn_flag==1||mycar.car_running==1)car_stop();
               adjust.take_picture=1;
               if(key_interval_time>0)
               {
                   //TSUI_FlashSave();                    //存储参数
                   tsui.buttonevent=Press2;
                   key_interval_time=0;
               }
               else if(key_interval_time==0)key_interval_time=400;
           }
   }
    else if(key_press==1)
   {
       if(KEY1_PIN==0)//如果key1处于按下状态
       {
           key_press_time++;            //不断累加
           if(key_press_time>long_press_time)//按下超过一定时间
           {
               TSUI_FlashSave();                    //存储参数
               tsui.buttonevent=PRESS;              //发送指令，表示储存成功（这是方便我们看是否存储成功，对单片机来说没什么意义）
           }
       }
       else if(KEY2_PIN==0)//如果key2处于按下状态
       {
           key_press_time++;            //不断累加
           if(key_press_time>long_press_time)//按下超过一定时间
           {
               TSUI_FlashSave();                    //存储参数
               tsui.buttonevent=PRESS;              //发送指令，表示储存成功（这是方便我们看是否存储成功，对单片机来说没什么意义）
           }
       }
       else if(KEY3_PIN==0)//如果key3处于按下状态
       {
           key_press_time++;            //不断累加
           if(key_press_time==long_press_time)//按下超过一定时间
           {
               //TSUI_FlashSave();                    //存储参数
               tsui.buttonevent=PRESS;              //发送指令，表示储存成功（这是方便我们看是否存储成功，对单片机来说没什么意义）
           }
       }
       else if(PRESS_PIN==0)//如果处于按下状态
       {
           key_press_time++;            //不断累加
           if(key_press_time==long_press_time)//按下超过一定时间
           {
               TSUI_FlashSave();
               tsui.buttonevent=PRESS;              //发送指令，表示储存成功（这是方便我们看是否存储成功，对单片机来说没什么意义）
               beep2(5,200);
           }
           if(key_press_time==2*long_press_time)//按下超过一定时间
           {

               tsui.buttonevent=PRESS;              //发送指令，表示储存成功（这是方便我们看是否存储成功，对单片机来说没什么意义）
               car_start_warning();     //先蜂鸣器警告再发车
               //car_begin();         //发车指令
           }
       }
       else                                 //按键松开
       {
           key_press=0;                     //清除检测按键是否按下的变量
           key_press_time=0;                //清除计时器
       }
   }
    if(key_interval_time!=0)key_interval_time--;
    //sprintf(page[0].line14,"interval_time=%d ",key_interval_time);
}
//ec11扫描函数，放在定时器中断里
void ec11_scan()
{
    static uint8 CW_PIN_level,CCW_PIN_level,CW_PIN_level_last,CCW_PIN_level_last;
    CW_PIN_level=CW_PIN;
    CCW_PIN_level=CCW_PIN;
    if(CW_PIN_level==0&&CW_PIN_level_last==1)
    {
        if(CCW_PIN_level==1)tsui.buttonevent=CW;
        else if(CCW_PIN_level==0)tsui.buttonevent=CCW;
    }
//    else if(CW_PIN_level==1&&CW_PIN_level_last==0)
//    {
//        if(CCW_PIN_level==0)tsui.buttonevent=CCW;
//        else if(CCW_PIN_level==1)tsui.buttonevent=CW;
//    }
    CW_PIN_level_last=CW_PIN_level;
    CCW_PIN_level_last=CCW_PIN_level;
}





