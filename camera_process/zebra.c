/*
 * zebra.c
 *
 *  Created on: 2024��3��24��
 *      Author: hwj
 */
#include "zebra.h"
//�����µ�����
void zebra_enter()
{
    if(mycar.RUNTIME<setpara.zebra_begin_time)return;
    int zebra_count=0;
    for(int y=15;y<80;y++)
    {
        if(lineinfo[y].zebra_flag==1)
        {
            zebra_count++;
            //vofa.zebra[0]=zebra_count;

        }
    }
    if(zebra_count>setpara.zebra_line_count)
    {
        clear_all_flags();
        enter_element(zebra);
        watch.zebra_flag=1;

        begin_distant_integeral(setpara.zebra_distance);
    }
}
//�������߻���
void zebra_stop()
{
    if(watch.zebra_flag==1&&get_integeral_state(&distance_integral)==2)
    {
        //set_speed(setpara.zebra_speed);
        // change_pid_para(&Speed_left,&setpara.stop_PID);  // TODO: 需要实现PID参数切换
        // change_pid_para(&Speed_right,&setpara.stop_PID);  // TODO: 需要实现PID参数切换
        watch.zebra_flag=2;
        //change_pid_para(&CAM_Turn,&setpara.loop_turn_PID);//��ת��PID������Ϊ����ת��PID
    }

}
void zebra_out()
{
    if(watch.zebra_flag==2&&mycar.present_speed<=0)
    {
        watch.stop_count++;
        if(watch.stop_count>setpara.stop_over_count)
        {
            watch.stop_count=0;
            out_element();
//            car_stop();
//            zebra_stop_warning();
        }
    }
}


