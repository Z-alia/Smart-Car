/*
 * Element_recognition.c
 *
 *  Created on: 2023��6��21��
 *      Author: Admin
 */
#include "Element_recognition.h"
#include "red_obstacle.h"  // ← 新增:红色障碍物模块
#include "global.h"
struct watch_o watch;
Element_range Element=None;
struct Element_struct Element_rem;
//struct watch_o watch;
//�ܵ�Ԫ���жϵĴ���
void Element_recognition()
{
    //Element=broken_circuit;
    switch(Element)
    {
        case None:
            enter_task(); //Ԫ����ں������ڴ�ʶ�𲢽��뺯��
            break;
        case Left_ring:
             left_ring_first_angle();       //Ѱ���󻷵�һ���ǵ�
             left_ring_circular_arc();      //������뻷��һ���ǵ���Բ��,���ڲ���
             left_ring_begin_turn();        //�󻷿�ʼת��
             left_ring_second_angle();      //����󻷵ڶ����ǵ�,���ڲ���
             left_ring_in_loop();           //����С���Ƿ���ȫ�뻷
             left_ring_prepare_out();       //С���ǶȻ�����ɣ�׼������
             left_ring_out_loop_turn();     //������ת
             left_ring_out_angle();         //������ʱ�ҽǵ�λ��
             left_ring_out_loop();          //�Ҳ�Ϊֱ��ʱֱ��
             left_ring_straight_out_angle();//�����󻷽���ֱ�ߺ����ǵ㣬���ڲ���
             left_ring_complete_out();      //�����ȫ����
            break;
        case Right_ring:
             right_ring_first_angle();       //����һ���һ���ǵ�
             right_ring_circular_arc();      //����һ��뻷��һ���ǵ���Բ��
             right_ring_begin_turn();        //�һ���ʼת����
             right_ring_second_angle();      //����һ��ڶ����ǵ�
             right_ring_in_loop();           //����С���Ƿ���ȫ���һ�
             right_ring_prepare_out();       //С���ǶȻ�����ɣ�׼������
             right_ring_out_loop_turn();     //���һ���ת
             right_ring_out_angle();         //������ʱ��ǵ�λ��
             right_ring_out_loop();          //���Ϊֱ��ʱֱ��
             right_ring_straight_out_angle();//�����һ�����ֱ�ߺ����ǵ�
             right_ring_complete_out();      //�����ȫ����
            break;
//        case ingarage:
////             left_garage_first_angle();//�����೵���һ���ǵ�
///*             left_garage_second_turn();
//             right_garage_first_angle();
//             right_garage_second_turn();
//             garage_stop();*/
//            if(setpara.start_mode==1||setpara.start_mode==3)
//            {
//                left_garage_first_angle();
//                left_garage_second_angle();
//                left_garage_begin_turn();
//                left_garage_stop_turn();
//                garage_stop2();
//            }
//            else if(setpara.start_mode==2||setpara.start_mode==4)
//            {
//                right_garage_first_angle();
//                right_garage_second_angle();
//                right_garage_begin_turn();
//                right_garage_stop_turn();
//                garage_stop2();
//            }
//            break;
//        case broken_circuit:
//            broken_circuit_enter();
//            broken_circuit_slow();
//            broken_circuit_complete_enter();
//            broken_circuit_complete_out();
//            break;
//        case obstacle:
//            obstacle_stop();
//            break;
        case crossing:
            cross_running();
            cross_running2();
            cross_out();
            break;
//        case Slope:
//            slope_down();
//            slope_out();
//            break;
//        case outgarage:
//            out_garage_turn();
//            complete_out_garage();
//            break;
//        case black_obstacle:
//            //black_obstacle_enter();
//            black_obstacle_out();
//            break;
        case zebra:
            
            break;
        case red_obstacle:  // ← 新增:红色障碍物处理
            red_obstacle_avoid();
            red_obstacle_out();
            break;

        default:break;
    }
}
//Ԫ��ʶ����ں���
void enter_task()
{
    switch( Element_rem.Element_data[Element_rem.Element_count])
    {
        case 1:
            right_ring_first_angle(); //����󻷵�һ���ǵ�
            break;
        case 2:
            cross_enter();//����һ���һ���ǵ�
            break;
        case 3:
//            cross_enter();
            break;
        case 4:
            //broken_circuit_prepare();
            break;
        case 5:

            break;
        case 6:
            //garage_prepare();
            break;
        case 7:
            //out_garage();
            break;
        case 8:
            //cross_enter();
            break;
        case 0:            //元素标号为0时自动扫描元素
            if(mycar.RUNTIME>=setpara.begin_time||mycar.car_running==0){
            left_ring_first_angle(); //检测左环第一个角点
            right_ring_first_angle();//检测右环第一个角点

            //slope_enter();
            //obstacle_identification();

            if(setpara.cross_open_flag>=1){
            cross_enter();}

            

            //zebra_enter();
            red_obstacle_enter();  // ← 新增:红色障碍物检测
            }
            break;
        default:
            out_element();
            //zebra_out();
            break;
    }
    //zebra_indentification(); //������ʶ��
}
//Ԫ�س�ʼ��
void element_init()
{
    uint8 count=0;
    Element_rem.Element_count=0;
    Element_rem.Element_data[0]=7;    //����
    uint8 Element[21];
    if(setpara.start_mode==1||setpara.start_mode==3)
    {
        for(int i=1;i<21;i++)
        {
            Element_rem.Element_data[i]= setpara.set_element[i];
        }
    }
    else if(setpara.start_mode==2||setpara.start_mode==4)
    {
        for(int i=1;i<21;i++)
        {
            if(setpara.set_element[i]!=6)
            {
                Element[i]=setpara.set_element[i];
                count++;
            }
            else if(setpara.set_element[i]==6)
            {
                count++;
                break;
            }
        }
        for(int i=1;i<count;i++)
        {
            if(Element[i]==1)
                Element_rem.Element_data[count-i]=2;
            else if(Element[i]==2)
                Element_rem.Element_data[count-i]=1;
            else
            Element_rem.Element_data[count-i]=Element[i];
        }
        Element_rem.Element_data[count]=6;
    }
    Element_rem.loop_count=0;
    Element_rem.loop_data[0]=setpara.loop_data[0];
    Element_rem.loop_data[1]=setpara.loop_data[1];
    Element_rem.loop_data[2]=setpara.loop_data[2];
    Element_rem.loop_data[3]=setpara.loop_data[3];
}
//����Ԫ��
void enter_element(Element_range element)
{
    Element=element;
}
//�뿪Ԫ��
void out_element()
{
//    if(Element==Left_ring||Element==Right_ring)
//    Element_rem.loop_count++;
    clear_all_flags();
//    Element_rem.Element_count++;
}

void clear_all_flags()
{
    //���Ԫ�ر��
    Element=None;
    //������ֱ�־λ
    clear_angle_integeral();
    clear_distant_integeral();
    //���Բ����־
    watch.InLoop = 0;
    watch.InLoopAngleL = 120;
    watch.InLoopAngleR = 120;
    watch.InLoopCirc = 120;
    watch.InLoopAngle2 = 120;
    watch.OutLoop=0;

/*    watch.OutLoopRight = 0;
    watch.OutLoopRightY =120;
    watch.OutLoopLeft = 187;
    watch.OutLoopLeftY = 120;*/
    watch.OutLoopAngle2 = 120;
    watch.OutLoopAngle1 = 120;

    //���ʮ��·�ڵı���
    watch.cross_flag=0;
    watch.cross_line=120;
    watch.Garge_line=120;
    watch.cross_RD_angle=120;
    watch.cross_LD_angle=120;

    watch.cross_AngleL=120;
    watch.cross_AngleR=120;
    watch.cross_AngleL_x=0;
    watch.cross_AngleR_x=187;
    //�����·��־λ
    watch.broken_circuit_flag=0;
    //��������߱�־λ


    // flag.stop = 0;
    // indata.YawAngle = 0;
    //gpio_set(C10,0);
    // if(caminfo.apriltag_count<3)
    //     watch.AprilLine = 120 ;
    //���б�±�־λ
    watch.slope_flag=0;

    //�ָ�Ѱ����ģʽ
    watch.Line_patrol_mode=0;
    mycar.speed_ctrl=1;                         //�ָ������ٶ�
    mycar.pid_ctrl=1;                           //�ָ�����PID
    // change_pid_para(&CAM_Turn,&CAM_FUZZY_PID);  // TODO: 需要实现PID参数切换
    // change_pid_para(&Speed_middle,&setpara.com_speed_PID);  // TODO: 需要实现PID参数切换
    // change_pid_para(&Speed_left,&setpara.com_speed_PID);  // TODO: 需要实现PID参数切换
    // change_pid_para(&Speed_right,&setpara.com_speed_PID);  // TODO: 需要实现PID参数切换
    //���������λ
    watch.zebra_flag = 0;
    watch.Zebra_Angle=120;
    watch.Zebra_Angle2=120;
    watch.stop_count=0;
    //��������־λ
    watch.out_garage_flag=0;
    //���ͣ�����λ
    watch.garage_stop=0;
    //���ͣ�����λ
    mycar.car_stop=0;
    watch.cross_flag=0;

    watch.obstacle_flag=0;
    watch.black_obstacle_flag=0;
    watch.black_obstacle_line=120;
    watch.left_obstacle_x=0;
    watch.right_obstacle_x=187;
    mycar.tracking_mode=0;  //�ָ�����ͷѭ��

    watch.angle_near_line=30;
    watch.angle_far_line=setpara.far_line;
    watch.garage_flag=0;
    mycar.target_speed=setpara.speed_min;

}
