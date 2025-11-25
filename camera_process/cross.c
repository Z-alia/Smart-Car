/*
 * cross.c
 *
 *  Created on: 2023��7��9��
 *      Author: Admin
 */
#include "cross.h"
//ʮ�ֽ��뺯��
void cross_enter()
{
    uint8 cross_count=0;
    for(int y=40;y<100;y++)
    {
        if(lineinfo[y].left_lost&&
           lineinfo[y].right_lost&&
           Grayscale[119-y][93]==255&&
           Grayscale[119-y][94]==255&&
           Grayscale[119-y][1]==255&&
           Grayscale[119-y][186]==255&&
           y<watch.track_count_far)
        {
            cross_count++;
        }
    }
    if(cross_count>6&&watch.cross_flag==0)
    {

        enter_element(crossing);   //����ʮ�� clear_all_flags();Ԫ��
//        watch.angle_far_line=110;
        //set_speed(setpara.cross_speed);
        watch.cross_flag=3;
        begin_distant_integeral(9000);
    }
    if(setpara.cross_open_flag==1){
    for(int y=20;y<60;y++)
    {
        if(lineinfo[y].left_lost&&
           lineinfo[y+1].left_lost&&
           lineinfo[y+2].left_lost&&
           lineinfo[y+3].left_lost&&
           lineinfo[y-1].left_lost&&
           lineinfo[y-2].left_lost&&
           lineinfo[y-5].left_lost&&
           !lineinfo[y].right_lost&&
           !lineinfo[y-1].right_lost&&
           !lineinfo[y-2].right_lost&&
           lineinfo[y].right<=lineinfo[y+1].right&&
           lineinfo[y+1].right<=lineinfo[y+2].right&&
           lineinfo[y+2].right<=lineinfo[y+3].right&&
           lineinfo[y].right<=lineinfo[y-1].right&&
           lineinfo[y-1].right<=lineinfo[y-2].right&&
           lineinfo[y-2].right<=lineinfo[y-3].right&&
           lineinfo[y-2].right<lineinfo[y-4].right&&
           lineinfo[y-1].right<lineinfo[y-3].right&&
           lineinfo[y].right<lineinfo[y-2].right&&
           lineinfo[y].right<lineinfo[y+2].right&&
           lineinfo[y+1].right<lineinfo[y+3].right&&
           Grayscale[119-y-2][lineinfo[y].right]==255&&//���������Ե�򷴹��γɽǵ�
           Grayscale[119-y-3][lineinfo[y].right]==255&&
           Grayscale[119-y-4][lineinfo[y].right]==255&&
           Grayscale[119-y-5][lineinfo[y].right+5]==255




                )
        {
            //�����������ʮ��
            enter_element(crossing);   //����ʮ�� clear_all_flags();Ԫ��
    //        watch.angle_far_line=110;
            //set_speed(setpara.cross_speed);
            watch.cross_flag=2;
            watch.cross_RD_angle=y;
            begin_distant_integeral(8000);
        }
        else if(lineinfo[y].right_lost&&
           lineinfo[y+1].right_lost&&
           lineinfo[y+2].right_lost&&
           lineinfo[y+3].right_lost&&
           lineinfo[y-1].right_lost&&
           lineinfo[y-2].right_lost&&
           lineinfo[y-5].right_lost&&
           !lineinfo[y].left_lost&&
           !lineinfo[y-1].left_lost&&
           !lineinfo[y-2].left_lost&&
           lineinfo[y].left>=lineinfo[y+1].left&&
           lineinfo[y+1].left>=lineinfo[y+2].left&&
           lineinfo[y+2].left>=lineinfo[y+3].left&&
           lineinfo[y].left>=lineinfo[y-1].left&&
           lineinfo[y-1].left>=lineinfo[y-2].left&&
           lineinfo[y-2].left>=lineinfo[y-3].left&&
           lineinfo[y-2].left>lineinfo[y-4].left&&
           lineinfo[y-1].left>lineinfo[y-3].left&&
           lineinfo[y].left>lineinfo[y-2].left&&
           lineinfo[y].left>lineinfo[y+2].left&&
           lineinfo[y+1].left>lineinfo[y+3].left&&
           Grayscale[119-y-2][lineinfo[y].left]==255&&
           Grayscale[119-y-3][lineinfo[y].left]==255&&
           Grayscale[119-y-4][lineinfo[y].left]==255&&
           Grayscale[119-y-5][lineinfo[y].left-5]==255



                )
        {
            //�����������ʮ��
            enter_element(crossing);   //����ʮ�� clear_all_flags();Ԫ��
    //        watch.angle_far_line=110;
            watch.cross_LD_angle=y;
            //set_speed(setpara.cross_speed);
            watch.cross_flag=1;
            begin_distant_integeral(8000);
        }
    }
    }
}
//��������ʱ�ҽ��㺯��
void cross_running2()
{
    if(watch.cross_flag!=2&&watch.cross_flag!=1)return;
    for(int y=15;y<87;y++)//����ɨ��
    {
        if(watch.cross_flag==2&&
           lineinfo[y].right<=lineinfo[y+1].right&&
           lineinfo[y+1].right<=lineinfo[y+2].right&&
           lineinfo[y+2].right<=lineinfo[y+3].right&&
           lineinfo[y].right<=lineinfo[y-1].right&&
           lineinfo[y-1].right<=lineinfo[y-2].right&&
           lineinfo[y-2].right<=lineinfo[y-3].right&&
           y<watch.cross_RD_angle&&
           Grayscale[119-y-2][lineinfo[y].right]==255&&
           lineinfo[y-2].right<lineinfo[y-4].right&&
           lineinfo[y-1].right<lineinfo[y-3].right&&
           lineinfo[y].right<lineinfo[y-2].right&&
           lineinfo[y].right<lineinfo[y+2].right&&
           lineinfo[y+1].right<lineinfo[y+3].right
            )
        {
            watch.cross_RD_angle=y;
            break;
        }
        if(watch.cross_flag==1&&
           lineinfo[y].left>=lineinfo[y+1].left&&
           lineinfo[y+1].left>=lineinfo[y+2].left&&
           lineinfo[y+2].left>=lineinfo[y+3].left&&
           lineinfo[y].left>=lineinfo[y-1].left&&
           lineinfo[y-1].left>=lineinfo[y-2].left&&
           lineinfo[y-2].left>=lineinfo[y-3].left&&
           y<watch.cross_LD_angle&&
           Grayscale[119-y-2][lineinfo[y].left]==255&&
           lineinfo[y-2].left>lineinfo[y-4].left&&
           lineinfo[y-1].left>lineinfo[y-3].left&&
           lineinfo[y].left>lineinfo[y-2].left&&
           lineinfo[y].left>lineinfo[y+2].left&&
           lineinfo[y+1].left>lineinfo[y+3].left
            )
        {
            watch.cross_LD_angle=y;
            break;
        }
    }
}
//ʮ�����к���
void cross_running()
{
    if(watch.cross_flag!=1&&watch.cross_flag!=2&&watch.cross_flag!=3)return;
    for(int y=25;y<87;y++)//����ɨ��
    {if(
            !lineinfo[y+2].left_lost&&
            !lineinfo[y+1].left_lost&&
            !lineinfo[y].left_lost&&((
            lineinfo[y].left-lineinfo[y-1].left>=3&&
            lineinfo[y].left-lineinfo[y-2].left>=7&&
            lineinfo[y].left-lineinfo[y-3].left>12
//            lineinfo[y-3].left_lost
            )||(lineinfo[y-2].left_lost&&lineinfo[y-3].left_lost&&lineinfo[y-4].left_lost))&&
            lineinfo[y+1].left-lineinfo[y].left<=3&&
            lineinfo[y+1].left-lineinfo[y].left>=0&&
            lineinfo[y+2].left-lineinfo[y+1].left<=3&&
            lineinfo[y+2].left-lineinfo[y+1].left>=0&&
            lineinfo[y+3].left-lineinfo[y+2].left<=3&&
            lineinfo[y+3].left-lineinfo[y+2].left>=0&&
            lineinfo[y+4].left-lineinfo[y+3].left<=3&&
            lineinfo[y+4].left-lineinfo[y+3].left>=0&&
            lineinfo[y+1].left > 30&&
            y>40)
    {
        watch.cross_AngleL=y;
        watch.cross_AngleL_x=lineinfo[watch.cross_AngleL].left;

    }
    }
    for(int y=25;y<87;y++)//����ɨ��
    {if(
            !lineinfo[y+2].right_lost&&
            !lineinfo[y+1].right_lost&&
            !lineinfo[y].right_lost&&((
            lineinfo[y-1].right-lineinfo[y].right>3&&
            lineinfo[y-2].right-lineinfo[y].right>7&&
            lineinfo[y-3].right-lineinfo[y].right>12
//            lineinfo[y-3].right_lost
            )|| (lineinfo[y-1].right_lost&&lineinfo[y-2].right_lost&&lineinfo[y-3].right_lost))&&
            lineinfo[y].right-lineinfo[y+1].right<=3&&
            lineinfo[y].right-lineinfo[y+1].right>=0&&
            lineinfo[y+1].right-lineinfo[y+2].right<=3&&
            lineinfo[y+1].right-lineinfo[y+2].right>=0&&
            lineinfo[y+2].right-lineinfo[y+3].right<=3&&
            lineinfo[y+2].right-lineinfo[y+3].right>=0&&
            lineinfo[y+3].right-lineinfo[y+4].right<=3&&
            lineinfo[y+3].right-lineinfo[y+4].right>=0&&
            lineinfo[y+1].right<158&&

            y>40)
    {
        watch.cross_AngleR=y;
        watch.cross_AngleR_x=lineinfo[watch.cross_AngleR].right;
    }
    if(watch.cross_AngleL<110&&watch.cross_AngleR<110)
    {
        watch.cross_flag=3;
    }
    if(watch.cross_AngleL<50||watch.cross_AngleR<50)
    {
        watch.cross_AngleL=120;
        watch.cross_AngleR=120;
    }
    }

}
//ʮ���뿪����
void cross_out()
{
    if(get_integeral_state(&distance_integral)==2&&watch.cross_flag==3)
        {
            clear_distant_integeral();
            out_element();
            // beep2(3,100);  // TODO: 需要实现beep2函数
        }
        else if(!lineinfo[65].left_lost&&!lineinfo[65].right_lost)
        {
//            watch.cross_AngleL=120;
//            watch.cross_AngleR=120;
            watch.angle_far_line=70;
        }
}


