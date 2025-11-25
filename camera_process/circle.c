/*
 * circle.c
 *
 *  Created on: 2023��6��21��
 *      Author: Admin
 */
#include "circle.h"
#define loop_forward_far 100
#define loop_forward_near 20
//����󻷵�һ���ǵ�(Inloop=1)
void left_ring_first_angle()
{
    if(watch.InLoop!=0&&watch.InLoop!=1)return;//��ѭ��֮ǰ��������ʡʱ��
    for(int y=loop_forward_near;y<loop_forward_far;y++)//����ɨ��
    {
        if(//lineinfo[y + 3].left_lost
            //&&lineinfo[y + 2].left_lost&&
            ((lineinfo[y + 1].left_lost)||((lineinfo[y].left-lineinfo[y+1].left)>=5*(lineinfo[y+1].left-lineinfo[y+2].left)))///////////
            && !lineinfo[y - 3].left_lost
            && !lineinfo[y - 2].left_lost
            && !lineinfo[y - 1].left_lost
            && !lineinfo[y].left_lost
            && !lineinfo[y + 5].right_lost
            && !lineinfo[y + 3].right_lost
            && !lineinfo[y + 2].right_lost
            && !lineinfo[y + 1].right_lost
            && !lineinfo[y].right_lost
            && !lineinfo[y - 1].right_lost
            && !lineinfo[y - 2].right_lost
            && !lineinfo[y - 3].right_lost
            && !lineinfo[y - 4].right_lost
            && !lineinfo[y - 5].right_lost

            &&lineinfo[y].left-lineinfo[y+4].left>10
            &&y<watch.InLoopAngleL
            //&&lineinfo[y].left>=lineinfo[y-2].left
            &&y<75
            )
        {//��Բ���ĵ�һ���ǵ�������
            watch.InLoopAngleL = y;
            if(Element==None)     //�ڵ�ǰ��Ԫ��ʱ�������²���������ʱ��ֻ�ҽǵ�
            {
                   left_ring_confirm();
            }
            break;
        }
    }
}
//�󻷶���ȷ�Ϻ���
void left_ring_confirm()
{
    uint8 zebra_confirm=0,white_count1=0,white_count2=0,white_count3=0,black_count=0,right_lost=0;
    //right_ring_first_angle();//ɨ���Ƿ�����һ��ǵ�
    //left_ring_circular_arc();//ɨ���Ƿ�������ϻ�
    for(int y=loop_forward_near;y<95;y++)//����ɨ��
    {
        if(((lineinfo[y+2].right-lineinfo[y].right)>2)||(lineinfo[y].right-lineinfo[y+2].right)>4)
            right_lost+=1;
//        if(lineinfo[y].right-lineinfo[y+2].right>20
//           &&lineinfo[y-1].right-lineinfo[y+3].right>20
//           &&lineinfo[y-2].right-lineinfo[y+4].right>20)
//            right_lost+=3;
    }
       if(right_lost<3)
       {
//           for(int x=lineinfo[watch.InLoopAngleL-1].left;x>0;x--)
//           {
//               if(Grayscale[119-watch.InLoopAngleL][x]==255)
//                   white_count1++;
//               if(Grayscale[119-(watch.InLoopAngleL-1)][x]==255)
//                   white_count2++;
//               if(Grayscale[119-(watch.InLoopAngleL-2)][x]==255)
//                   white_count3++;
//           }
           //vofa.loop[5]=white_count1;
           //vofa.loop[6]=white_count2;
           //vofa.loop[7]=white_count3;
           for(int y=watch.InLoopAngleL;y>loop_forward_near;y--)
           {
               if(Grayscale[119-y][lineinfo[watch.InLoopAngleL].left]==0)
                  black_count++;
           }
           vofa.loop[6]=lineinfo[watch.InLoopAngleL].left;
           vofa.loop[7]=black_count;
           if(/*(white_count1>=10&&white_count2>=10&&white_count3>=10)&&*/(black_count<10))
           {
               enter_element(Left_ring);    //��ʽ������Բ��Ԫ��
               begin_distant_integeral(6000);
              // if(Element_rem.loop_data[Element_rem.loop_count]==0)//�����С��
               {
                   set_speed(setpara.loop_target_speed);
                   mycar.pid_ctrl=0;
                   // change_pid_para(&CAM_Turn,&setpara.loop_turn_PID);  // TODO: 需要实现PID参数切换
               }
/*               else//����Ǵ�
               {
                   set_speed(setpara.big_loop_speed);
                   //change_pid_para(&CAM_Turn,&setpara.big_loop_PID);
               }*/
               watch.InLoop = 1;
               beep2(1,20);//������
               return;
           }
       }

    Element=None;
    watch.InLoopAngleR=120;
    watch.InLoopAngleL=120;
}


//������뻷��һ���ǵ���Բ��
void left_ring_circular_arc()
{
    if (watch.InLoop != 1&&watch.InLoop != 2)return;//��ѭ��֮ǰ��������ʡʱ��
    //beep(20);
    for(int y=loop_forward_near;y<loop_forward_far;y++)//����ɨ��
    {
        if (//y <watch.InLoopAngle2  &&
               (watch.InLoopAngleL<65||get_integeral_state(&distance_integral)==2)
           &&(y>(watch.InLoopAngleL+20)||get_integeral_state(&distance_integral)==2)
           &&y <watch.InLoopCirc
           &&!lineinfo[y+3].left_lost
           &&!lineinfo[y+2].left_lost
           &&!lineinfo[y+1].left_lost
           &&!lineinfo[y-3].left_lost
           &&!lineinfo[y-2].left_lost
           &&!lineinfo[y-1].left_lost
           &&lineinfo[y+1].left <= lineinfo[y].left
           &&lineinfo[y+2].left <= lineinfo[y].left
           &&lineinfo[y+3].left <= lineinfo[y].left
           &&lineinfo[y-1].left <= lineinfo[y].left
           &&lineinfo[y-2].left <= lineinfo[y].left
           &&lineinfo[y-3].left <= lineinfo[y].left
           //&&(watch.right_lost+watch.cross_lost)<5
            )
       { //�뻷��������
            watch.InLoopCirc = y;
            //beep(20);
            break;
       }
    }
}
//����󻷵ڶ����ǵ�
void left_ring_second_angle()
{
    if(watch.InLoop != 1&&watch.InLoop != 2)return;//��ѭ��֮ǰ��������ʡʱ��
    for(int y=loop_forward_near;y<loop_forward_far;y++)//����ɨ��
    {
        if (watch.InLoopCirc<66
             &&y<watch.InLoopAngle2
             &&watch.InLoopAngle2==120
             &&get_integeral_state(&distance_integral)==2
           &&y > 60
           &&y < (loop_forward_far-2)
           &&y>watch.InLoopCirc
           &&lineinfo[y+1].left > 30
           &&(lineinfo[y+1].left-lineinfo[y].left)<=2
           &&(lineinfo[y].left-lineinfo[y-4].left)>lineinfo[y].left/2
           )
           {
               watch.InLoopAngle2 = y;
               watch.InLoopAngle2_x=lineinfo[watch.InLoopAngle2].left;
               //if()
               //watch.InLoopCirc=0;
               break;
           }
    }
    if(watch.InLoopAngle2!=120
        &&watch.InLoopAngle2>50
        )
    {
        find_angle_left_down(&watch.InLoopAngle2_x,&watch.InLoopAngle2);
    }
}
//�󻷿�ʼת����(watch.InLoop=2)
void left_ring_begin_turn()
{
    if(watch.InLoop!=1)return;//��ѭ��֮ǰ��������ʡʱ��
    if(get_integeral_state(&distance_integral)==2//·�̻������
        &&watch.InLoop==1
        &&watch.InLoopAngle2<=80
    )
    {
        clear_distant_integeral();//���·�̻��ֱ���
        watch.InLoop=2;
        //set_speed(setpara.loop_target_speed);
        //change_pid_para(&CAM_Turn,&setpara.loop_turn_PID);//��ת��PID������Ϊ����ת��PID
        //watch.fix_slope=(float)(lineinfo[watch.InLoopAngle2].left)/(115-watch.InLoopAngle2);
        begin_angle_integeral(260);
        // beep2(2,20);  // TODO: 需要实现beep2函数
    }
}
//����С���Ƿ���ȫ�뻷
void left_ring_in_loop()
{
    if(watch.InLoop != 2)return;
    if( watch.InLoop == 2
        &&get_integeral_state(&angle_integral)==1
        &&get_integeral_data(&angle_integral)>40//ת��һ���Ƕ�
        //&&watch.right_near_lost<40
        //&&lineinfo[watch.right_near_lost].right>180
        //&&(!lineinfo[40].right_lost)
        )
        {
            //watch.InLoopCirc = 0;
            watch.InLoop = 3;
            //change_pid_para(&CAM_Turn,&setpara.loop_turn_PID);//��ת��PID������Ϊ����ת��PID
            set_speed(setpara.loop_target_speed+3);
            beep2(3,20);
        }
}
//С���ǶȻ�����ɣ�׼������
void left_ring_prepare_out()
{
    if(watch.InLoop != 3)return;
    if( watch.InLoop == 3
        &&get_integeral_state(&angle_integral)==1
        &&get_integeral_data(&angle_integral)>160
        &&lineinfo[69].right<152
        &&lineinfo[69].right>82)
   {
       watch.InLoop = 4;
       watch.OutLoop_turn_point_x=lineinfo[69].right;
       // beep2(4,20);  // TODO: 需要实现beep2函数
   }
}
//������ʱ�ҽǵ�λ��
void left_ring_out_angle()
{
    if(watch.InLoop != 4)return;//��ѭ��֮ǰ��������ʡʱ��
    for(int y=loop_forward_near;y<loop_forward_far;y++)//����ɨ��
        {
        if ((watch.InLoop == 4)&&y<80
                 //lineinfo[y].left_lost
                 &&lineinfo[y+1].right >= lineinfo[y].right
                 &&lineinfo[y+2].right >= lineinfo[y+1].right
                 &&lineinfo[y-1].right >= lineinfo[y].right
                 &&lineinfo[y-2].right >= lineinfo[y].right
/*                 &&lineinfo[y - 3].right > lineinfo[y - 1].right
                 &&lineinfo[y + 4].right > lineinfo[y + 2].right
                 &&lineinfo[y - 5].right > lineinfo[y - 3].right*/
                 &&lineinfo[y].right > 30
                 &&Grayscale[119-y-2][lineinfo[y].right]==255
)
             {
                 if(watch.OutLoopAngle1>y)
                 {
                     //watch.OutLoopRight = lineinfo[y].right;
                     watch.OutLoopAngle1 = y; //�����ж���
                     break;
                 }
             }
        }
}
//������ת(����Բ����ֱ�߽��紦)
void left_ring_out_loop_turn()
{
    if(watch.InLoop != 4)return;
    if(watch.InLoop == 4
       //&&(watch.cross+watch.right_lost)>30//�Ҳඪ�߹��࣬˵������Բ����ֱ�߽��紦
       &&watch.OutLoopAngle1<120
       &&get_integeral_state(&angle_integral)==2
       &&lineinfo[watch.OutLoopAngle1].right_lost==1
       &&watch.OutLoop==0
    )
    {
        clear_angle_integeral();
//        if(Element_rem.loop_data[Element_rem.loop_count]==0)//�����С��
//        {
//            begin_angle_integeral(setpara.loop_angle_out);
//        }
//        else//����Ǵ�
//        {
//            begin_angle_integeral(setpara.big_loop_out);
//        }
        begin_distant_integeral(3000);//����·�̻��֣���ʱҪ������ת
        watch.OutLoop=1;
        beep2(5,20);
    }
}
//�Ҳ�Ϊֱ��ʱֱ��
void left_ring_out_loop()
{
    if(watch.InLoop != 4&&watch.OutLoop!=1)return;
    for(int y=loop_forward_near;y<loop_forward_far;y++)//����ɨ��
        {
        if((watch.InLoop == 4
            &&watch.OutLoop==1
            &&y<60
            &&lineinfo[y].right_lost==0
            &&lineinfo[y+10].right_lost==0
            &&lineinfo[y].right>10///////////////////
            &&lineinfo[y+10].right>10/////////////////
            &&(188-(lineinfo[y].right-lineinfo[y+10].right)*(115-y)/10)>40
            &&lineinfo[y].right>lineinfo[y+10].right
            &&get_integeral_state(&distance_integral)==2)
                )
            {
            clear_distant_integeral();
//            if(Element_rem.loop_data[Element_rem.loop_count]==0)//�����С��
//            {
                begin_distant_integeral(setpara.loop_out_distance);
//            }
//            else//����Ǵ�
//            {
//                begin_distant_integeral(setpara.big_loop_out_distance);
//            }
            clear_angle_integeral();
            beep2(6,20);
            watch.InLoop =5;////���������ǣ�������ͷ������ȡ����Ԫ�أ�
            }
        }
}
//�����󻷽���ֱ�ߺ����ǵ�
void left_ring_straight_out_angle()
{
    if(watch.InLoop != 5&&watch.OutLoop!=1)return;
    for(int y=loop_forward_near;y<loop_forward_far;y++)//����ɨ��
        {
        if((watch.InLoop==5)
          &&watch.OutLoop==1
          &&y<(watch.watch_lost-10)
          &&y<80
          &&y<watch.OutLoopAngle2
          &&lineinfo[y].left<120
          &&lineinfo[y].right>60
          && watch.zebra_flag == 0
          &&(lineinfo[y + 2].right-lineinfo[y + 2].left)<(lineinfo[y -1].right-lineinfo[y -1].left)-30
          &&(lineinfo[y + 1].right-lineinfo[y + 1].left)<(lineinfo[y -2].right-lineinfo[y -2].left)-30
          &&(lineinfo[y].right-lineinfo[y].left)<(lineinfo[y -3].right-lineinfo[y -3].left)-30
          &&lineinfo[y+1].right-lineinfo[y+2].right<5
          &&!lineinfo[y].left_lost
          &&!lineinfo[y+1].left_lost
          &&!lineinfo[y+2].left_lost)///////////
            {
                watch.OutLoopAngle2 = y;
            }
        }
}

//�����ȫ����
void left_ring_complete_out()
{
    if(watch.InLoop != 5&&watch.OutLoop!=1)return;
    for(int y=loop_forward_near;y<loop_forward_far;y++)//����ɨ��
    {
    if (watch.InLoop == 5
        &&get_integeral_state(&distance_integral)==2
        &&watch.OutLoopAngle2<70
        )
     {
         clear_all_flags();// �����ɹ�,������б�־
         out_element();
         //mycar.target_speed=setpara.com_target_speed;//�ָ������ٶ�
         //change_pid_para(&CAM_Turn,&setpara.com_turn_PID);//恢复普通转向环PID
         // beep2(7,20);  // TODO: 需要实现beep2函数
     }
    }
}

//����Ϊ�һ�����

//����һ���һ���ǵ�(Inloop=1)
//����ֵ��1���ҵ��ǵ� 0���޽ǵ�
void right_ring_first_angle()
{
    if(watch.InLoop != 0&&watch.InLoop!=6)return;
    for(int y=loop_forward_near;y<loop_forward_far;y++)//����ɨ��
        {
        if (
          y < 75
         &&((lineinfo[y + 1].right_lost)||(lineinfo[y+1].right-lineinfo[y].right>=5*(lineinfo[y+2].right-lineinfo[y+1].right)))
         && !lineinfo[y - 3].right_lost
         && !lineinfo[y - 2].right_lost
         &&!lineinfo[y - 1].right_lost
         &&!lineinfo[y].right_lost
         &&!lineinfo[y + 5].left_lost
         &&!lineinfo[y + 3].left_lost
         &&!lineinfo[y + 2].left_lost
         &&!lineinfo[y + 1].left_lost
         &&!lineinfo[y].left_lost
         &&!lineinfo[y - 5].left_lost
         &&!lineinfo[y - 4].left_lost
         &&!lineinfo[y - 3].left_lost
         &&!lineinfo[y - 2].left_lost
         &&!lineinfo[y - 1].left_lost
         &&y<watch.InLoopAngleR
         &&lineinfo[y+4].right-lineinfo[y].right>10
    //     &&abs_m(lineinfo[y - 5].right,lineinfo[y - 4].right)<8
    //     &&abs_m(lineinfo[y - 6].right,lineinfo[y - 5].right)<8
         )
            { //��Բ���ĵ�һ���ǵ�������
                watch.InLoopAngleR = y;
                   //Element=Rifht_ring_confirm;
                if(Element==None)     //�ڵ�ǰ��Ԫ��ʱ�������²���������ʱ��ֻ�ҽǵ�
                {
                    right_ring_confirm();       //�����һ�����ȷ��
                }
            }
        }
}
//�һ�����ȷ�Ϻ���
void right_ring_confirm()
{
    uint8 zebra_confirm,white_count1=0,white_count2=0,white_count3=0,left_lost=0,black_count=0;
        //right_ring_circular_arc();  //ɨ���Ƿ�����һ��ϻ�
        //left_ring_first_angle();   //ɨ���Ƿ�����󻷽ǵ�
        for(int y=loop_forward_near;y<95;y++)//����ɨ��
        {
            if((lineinfo[y].left-lineinfo[y+2].left>2)||(lineinfo[y+2].left-lineinfo[y].left>4))
                left_lost+=1;
//            if(lineinfo[y+2].left-lineinfo[y].left>20
//              &&lineinfo[y+3].left-lineinfo[y-1].left>20
//              &&lineinfo[y+4].left-lineinfo[y-2].left>20)
//               left_lost+=3;
        }
        if(left_lost<3)
        {
            {
//                for(int x=lineinfo[watch.InLoopAngleR-1].right;x<188;x++)
//                {
//                    if(Grayscale[119-watch.InLoopAngleR][x]==255)
//                        white_count1++;
//                    if(Grayscale[119-watch.InLoopAngleR-1][x]==255)
//                        white_count2++;
//                    if(Grayscale[119-watch.InLoopAngleR-2][x]==255)
//                        white_count3++;
//                }
                for(int y=watch.InLoopAngleR;y>loop_forward_near;y--)
                {
                    if(Grayscale[119-y][lineinfo[watch.InLoopAngleR].right]==0)
                       black_count++;
                }
                if(/*(white_count1>=10&&white_count2>=10&&white_count3>=10)&&*/black_count<10)
                {
                    //Element=Right_ring;        //��ʽ�����һ�Ԫ��
                    enter_element(Right_ring);
                    if(Element_rem.loop_data[Element_rem.loop_count]==0)//�����С��
                    {
                        set_speed(setpara.loop_target_speed);
                        mycar.pid_ctrl=0;
                        // change_pid_para(&CAM_Turn,&setpara.loop_turn_PID);  // TODO: 需要实现PID参数切换
                    }
//                    else//����Ǵ�
//                    {
//                        set_speed(setpara.big_loop_speed);
//                        //change_pid_para(&CAM_Turn,&setpara.big_loop_PID);
//                    }
                    begin_distant_integeral(6000);
                    watch.InLoop = 6;
                    // beep2(1,20);  // TODO: 需要实现beep2函数
                    return;
                }

            }
        }
        Element=None;
        watch.InLoopAngleR=120;
        watch.InLoopAngleL=120;
        return;

}
//�һ���ʼת����(watch.InLoop=2)
void right_ring_begin_turn()
{
    if(watch.InLoop != 6)return;
    if(get_integeral_state(&distance_integral)==2//·�̻������
        &&watch.InLoop==6
        &&(watch.InLoopAngle2<=80)
        )
        {
                clear_distant_integeral();//���·�̻��ֱ���
                //change_pid_para(&CAM_Turn,&setpara.loop_turn_PID);//��ת��PID������Ϊ����ת��PID
                watch.InLoop=7;
                //set_speed(setpara.loop_target_speed);
                //watch.fix_slope=(float)(188-lineinfo[watch.InLoopAngle2].right)/(115-watch.InLoopAngle2);
                begin_angle_integeral(-260);
                // beep2(2,20);  // TODO: 需要实现beep2函数
        }

}
/*
if ((watch.InLoop == 1)
   &&y<watch.watch_lost-5
   &&y <watch.InLoopAngle2 &&y>watch.InLoopAngle
   &&y <watch.InLoopCirc
   &&!lineinfo[y+3].left_lost
   &&!lineinfo[y+2].left_lost
   &&!lineinfo[y+1].left_lost
   &&!lineinfo[y-3].left_lost
   &&!lineinfo[y-2].left_lost
   &&!lineinfo[y-1].left_lost
   &&lineinfo[y+1].left <= lineinfo[y].left
   &&lineinfo[y+2].left <= lineinfo[y+1].left
   &&lineinfo[y-1].left <= lineinfo[y].left
   &&lineinfo[y-2].left <= lineinfo[y-1].left*/
//����һ��뻷��һ���ǵ���Բ��
void right_ring_circular_arc()
{
    if(watch.InLoop != 6&&watch.InLoop != 7)return;
    for(int y=loop_forward_near;y<loop_forward_far;y++)//����ɨ��
        {
        if (//(watch.InLoop == 6)&&y <watch.InLoopAngle2 &&
             (watch.InLoopAngleR<65||get_integeral_state(&distance_integral)==2)
         &&(y>(watch.InLoopAngleR+20)||get_integeral_state(&distance_integral)==2)
           &&y <watch.InLoopCirc
           &&!lineinfo[y+3].right_lost
           &&!lineinfo[y+2].right_lost
           &&!lineinfo[y+1].right_lost
           &&!lineinfo[y-3].right_lost
           &&!lineinfo[y-2].right_lost
           &&!lineinfo[y-1].right_lost
           &&lineinfo[y+1].right >= lineinfo[y].right
           &&lineinfo[y+2].right >= lineinfo[y+1].right
           &&lineinfo[y+3].right >= lineinfo[y].right

           &&lineinfo[y-1].right >= lineinfo[y].right
           &&lineinfo[y-2].right >= lineinfo[y-1].right
           &&lineinfo[y-3].right >= lineinfo[y].right

           )
           { //�뻷��������
                watch.InLoopCirc = y;
                break;
           }
        }
}

/*
if(watch.InLoopCirc<120
       &&y<watch.InLoopAngle2
       &&get_integeral_state(&distance_integral)==2
       &&y>watch.InLoopCirc
       &&lineinfo[y-1].left_lost&&!lineinfo[y].left_lost
       &&lineinfo[y].edge_store[1]>50
       &&lineinfo[y].edge_store[0]==0
       &&lineinfo[y+1].edge_store[1]<=lineinfo[y].edge_store[1]
       &&lineinfo[y+1].edge_store[2]>=lineinfo[y].edge_store[1])*/
//����һ��ڶ����ǵ�
void right_ring_second_angle()
{
    if(watch.InLoop != 6&&watch.InLoop != 7)return;//��ѭ��֮ǰ��������ʡʱ��
    for(int y=loop_forward_near;y<loop_forward_far;y++)//��µĵ㣬������ʱ��Ϊ���ף���һ�оݣ�
    {
        if(watch.InLoopCirc<66
               &&y<watch.InLoopAngle2
               &&watch.InLoopAngle2==120
               &&get_integeral_state(&distance_integral)==2
               &&y > 60
               &&y < (loop_forward_far-2)
               &&y>watch.InLoopCirc
                        &&lineinfo[y+1].right <158
                        &&(lineinfo[y].right-lineinfo[y+1].right)<=2
                        &&(lineinfo[y-4].right-lineinfo[y].right)>(187-lineinfo[y].right)/2
                        )
                        {
                            watch.InLoopAngle2 = y;
                            watch.InLoopAngle2_x=lineinfo[watch.InLoopAngle2].right;
                            //if()
                            //watch.InLoopCirc=0;
                            break;
                        }
                 }
                 if(watch.InLoopAngle2!=120
                     &&watch.InLoopAngle2>50
                     )
                 {
                     find_angle_right_down(&watch.InLoopAngle2_x,&watch.InLoopAngle2);
                 }

//               &&lineinfo[y-1].right_lost&&!lineinfo[y].right_lost
//               &&lineinfo[y].edge_store[lineinfo[y].edge_count-2]<130
//               &&lineinfo[y].edge_store[lineinfo[y].edge_count-1]==187
//               &&lineinfo[y+1].edge_store[lineinfo[y+1].edge_count-2]>=lineinfo[y].edge_store[lineinfo[y].edge_count-2]
//               &&lineinfo[y+1].edge_store[lineinfo[y].edge_count-3]<=lineinfo[y].edge_store[lineinfo[y].edge_count-2])
//           )
//               {
//            watch.InLoopAngle2 = y;
//            watch.InLoopAngle2_x=lineinfo[y].edge_store[lineinfo[y].edge_count-3];
//            return;
//        }
//    }//�������������أ�Զ����ʱ���ף��ڶ��оݣ�
//    for(int y=loop_forward_near;y<loop_forward_far;y++)//����ɨ��
//        {
//        if (watch.InLoopCirc<120
//             &&y<watch.InLoopAngle2
//             &&watch.InLoopAngle2==120
//             &&get_integeral_state(&distance_integral)==2
//             &&y > 60
//             &&lineinfo[watch.InLoopCirc].right_lost
//             &&y<watch.InLoopAngle2
//             &&y>watch.InLoopCirc
//             &&(lineinfo[y].right-lineinfo[y+1].right)<=2
//             &&(lineinfo[y-4].right-lineinfo[y].right)>(188-lineinfo[y].right)/2
//            )
//           {
//               watch.InLoopAngle2 = y;
//               watch.InLoopAngle2_x=lineinfo[y].right;
//               break;
//           }
//        }
//    if(watch.InLoopAngle2<120
//            &&watch.InLoopAngle2>50)
//    {
//        find_angle_right_down(watch.InLoopAngle2_x,watch.InLoopAngle2,&watch.InLoopAngle2_x,&watch.InLoopAngle2);
//    }
}

//����С���Ƿ���ȫ���һ�
void right_ring_in_loop()
{
    if(watch.InLoop != 7)return;

    if( watch.InLoop == 7
        &&get_integeral_state(&angle_integral)==1
        &&get_integeral_data(&angle_integral)<-40//ת��һ���Ƕ�
        //&&watch.left_near_lost<40
        )
        {
            watch.InLoop = 8;
            //change_pid_para(&CAM_Turn,&setpara.loop_turn_PID);//��ת��PID������Ϊ����ת��PID
            set_speed(setpara.loop_target_speed+3);
            beep2(3,20);
        }
}

//С���ǶȻ�����ɣ�׼������
void right_ring_prepare_out()
{
    if(watch.InLoop != 8)return;
    if( watch.InLoop == 8
        &&get_integeral_state(&angle_integral)==1
        &&get_integeral_data(&angle_integral)<-160
        &&lineinfo[69].left>35
        &&lineinfo[69].left<105)///////////////////////
   {
       watch.InLoop = 9;
       watch.OutLoop_turn_point_x=lineinfo[69].left;
       // beep2(4,20);  // TODO: 需要实现beep2函数
   }
}
//

//������ʱ��ǵ�λ��
void right_ring_out_angle()
{
    if(watch.InLoop != 9)return;
    for(int y=loop_forward_near;y<loop_forward_far;y++)//����ɨ��
        {
        if ((watch.InLoop == 9) &&y<80
                 //lineinfo[y].left_lost
                 &&lineinfo[y + 1].left <= lineinfo[y].left
                 &&lineinfo[y+2].left <= lineinfo[y+1].left
                 &&lineinfo[y-1].left <= lineinfo[y].left//
                 &&lineinfo[y-2].left <= lineinfo[y].left
                 &&lineinfo[y].left < 158
                 &&Grayscale[119-y-2][lineinfo[y].left]==255
                 &&y>30
                 )
             {
                 if(watch.OutLoopAngle1>y)
                 {
                 //watch.OutLoopLeft = lineinfo[y].left;
                     watch.OutLoopAngle1 = y; //�����ж���
                 }
             }
        }
}
//���һ���ת
void right_ring_out_loop_turn()
{
    if(watch.InLoop == 9
       //&&(watch.cross+watch.left_lost)>30//�Ҳඪ�߹��࣬˵������Բ����ֱ�߽��紦
       &&watch.OutLoopAngle1<120
       &&get_integeral_state(&angle_integral)==2
       &&lineinfo[watch.OutLoopAngle1].left_lost==1
       &&watch.OutLoop==0)
    {
        if(Element_rem.loop_data[Element_rem.loop_count]==0)//�����С��
        {
            begin_distant_integeral(setpara.loop_out_distance);
        }
        else//����Ǵ�
        {
            begin_distant_integeral(setpara.big_loop_out_distance);
        }
        clear_angle_integeral();
        begin_distant_integeral(3000);//����·�̻��֣���ʱҪ������ת
        watch.OutLoop=1;
        beep2(5,20);
    }
}

//���Ϊֱ��ʱֱ��
void right_ring_out_loop()
{
    if(watch.InLoop != 9&&watch.OutLoop!=1)return;
    for(int y=loop_forward_near;y<loop_forward_far;y++)//����ɨ��
        {
        if((watch.InLoop == 9
            &&watch.OutLoop==1
            &&y<60
            &&lineinfo[y].left_lost==0
            &&lineinfo[y+10].left_lost==0
            &&lineinfo[y].left>10///////////////////
            &&lineinfo[y+10].left>10/////////////////
            &&((lineinfo[y+10].left-lineinfo[y].left)*(115-y)/10)<148
            &&lineinfo[y+10].left-lineinfo[y].left>0////////////////
            &&get_integeral_state(&distance_integral)==2)
                //||get_integeral_state(&angle_integral)==2
                )
            {
            clear_distant_integeral();
//            if(Element_rem.loop_data[Element_rem.loop_count]==0)//�����С��
//            {
                begin_distant_integeral(setpara.loop_out_distance);
//            }
//            else//����Ǵ�
//            {
//                begin_distant_integeral(setpara.big_loop_out_distance);
//            }
            beep2(6,20);
            watch.InLoop =10;////���������ǣ�������ͷ������ȡ����Ԫ�أ�
            }
        }
}

//�����һ�����ֱ�ߺ����ǵ�
void right_ring_straight_out_angle()
{
    if(watch.InLoop != 10&&watch.OutLoop!=1)return;
    for(int y=loop_forward_near;y<loop_forward_far;y++)//����ɨ��
        {
        if((watch.InLoop==10)
          &&watch.OutLoop==1
          &&y<(watch.watch_lost-10)
          &&y<80
          &&y<watch.OutLoopAngle2
          &&lineinfo[y].left<120
          &&lineinfo[y].right>60
          && watch.zebra_flag == 0
          &&(lineinfo[y + 2].right-lineinfo[y + 2].left)<(lineinfo[y -1].right-lineinfo[y -1].left)-30
          &&(lineinfo[y + 1].right-lineinfo[y + 1].left)<(lineinfo[y -2].right-lineinfo[y -2].left)-30
          &&(lineinfo[y].right-lineinfo[y].left)<(lineinfo[y -3].right-lineinfo[y -3].left)-30
          &&lineinfo[y+2].left-lineinfo[y+1].left<5///////////////////
          &&!lineinfo[y].right_lost
          &&!lineinfo[y+1].right_lost
          &&!lineinfo[y+2].right_lost)
            {
                watch.OutLoopAngle2 = y;
            }
        }
}

//�����ȫ����
void right_ring_complete_out()
{
    if (watch.InLoop == 10
        &&get_integeral_state(&distance_integral)==2
        &&watch.OutLoopAngle2<70
        )
     {
         //clear_all_flags();// �����ɹ�,������б�־
         out_element();
         //mycar.target_speed=setpara.com_target_speed;//�ָ������ٶ�
         //change_pid_para(&CAM_Turn,&setpara.com_turn_PID);//恢复普通转向环PID
         // beep2(7,100);  // TODO: 需要实现beep2函数
     }
}
//�������ҽǵ�
void find_angle_left_down(int*angle_x,int*angle_y)
{
    int x=*angle_x, y=*angle_y;
    while(Grayscale[119-y][x]!=0&&y<110)
    {
        y++;
    }
    while(Grayscale[119-y][x+1]!=255&&x<187)
    {
        x++;
    }
    while(y>40)
    {
        if(Grayscale[119-y][x]==0)
        {

        }
        else if(Grayscale[119-y][x-1]==0)
        {
            x--;
        }
        else if(Grayscale[119-y][x+1]==0)
        {
            x++;
        }
        else if(Grayscale[119-y][x-2]==0)
        {
            x=x-2;
        }
        else if(Grayscale[119-y][x+2]==0)
        {
            x=x+2;
        }
        else if(Grayscale[119-y][x-3]==0)
        {
            x=x-3;
        }
        else if(Grayscale[119-y][x+3]==0)
        {
            x=x+3;
        }
        else break;
        y--;
    }
    *angle_x=x;
    *angle_y=y;
}
//�������ҽǵ�
void find_angle_right_down(int*angle_x,int*angle_y)
{
    int x=*angle_x, y=*angle_y;
    while(Grayscale[119-y][x]!=0&&y<110)
    {
        y++;
    }
    while(Grayscale[119-y][x-1]!=255&&x>0)
    {
        x--;
    }
    while(y>40)
    {
        if(Grayscale[119-y][x]==0)
        {

        }
        else if(Grayscale[119-y][x+1]==0)
        {
            x++;
        }
        else if(Grayscale[119-y][x-1]==0)
        {
            x--;
        }
        else if(Grayscale[119-y][x+2]==0)
        {
            x=x+2;
        }
        else if(Grayscale[119-y][x-2]==0)
        {
            x=x-2;
        }
        else if(Grayscale[119-y][x+3]==0)
        {
            x=x+3;
        }
        else if(Grayscale[119-y][x-3]==0)
        {
            x=x-3;
        }
        else break;
        y--;
    }
    *angle_x=x;
    *angle_y=y;
}

