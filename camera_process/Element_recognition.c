
//本文件留作接口文件，剩下的元素识别相关的需要自己移植
#include "Element_recognition.h"
#include "image.h"
#include "Binarization.h"
#include "main.h"

#define loop_forward_far 100
#define loop_forward_near 20


//小车状态变量
struct watch_o watch;

//弯道识别
void curve_recognition(struct watch_o *watch)
{    
    
}

//十字路口识别(判断更具特点，优先级最高)
void Cross_recognition()
{
    
}

//直道识别
void Straight_recognition(struct watch_o *watch)
{
    
}

//标志位初始化(完成项目(进入项目后再次检测到直线)后调用)
void Clear_Recognition_Flag(struct watch_o *watch)
{
    
}
//补线

//状态机

//图像预处理

//环岛检测
//向左下找角点
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

//检测左环第一个角点(Inloop=1)
void left_ring_first_angle()
{
    if(watch.InLoop!=0&&watch.InLoop!=1)return;//在循环之前跳出，节省时间
    for(int y=loop_forward_near;y<loop_forward_far;y++)//逐行扫描
    {
        if(//lineinfo[y + 3].left_lost
            //&&lineinfo[y + 2].left_lost&&
            ((leftlost[y + 1]||((l_border[y]-l_border[y+1])>=5*(l_border[y+1]-l_border[y+2])))///////////
            && !leftlost[y - 3]
            && !leftlost[y - 2]
            && !leftlost[y - 1]
            && !leftlost[y]
            && !rightlost[y + 5]
            && !rightlost[y + 3]
            && !rightlost[y + 2]
            && !rightlost[y + 1]
            && !rightlost[y]
            && !rightlost[y - 1]
            && !rightlost[y - 2]
            && !rightlost[y - 3]
            && !rightlost[y - 4]
            && !rightlost[y - 5]

            &&l_border[y]t-l_border[y+4]>10
            &&y<watch.InLoopAngleL
            //&&lineinfo[y].left>=lineinfo[y-2].left
            &&y<75
            )
        {//左圆环的第一个角点所在行
            watch.InLoopAngleL = y;
            if(Element==None)     //在当前无元素时进行以下操作，其他时候只找角点
            {
                   left_ring_confirm();
            }
            break;
        }
    }
}

//左环二次确认函数
void left_ring_confirm()
{
    uint8 zebra_confirm=0,white_count1=0,white_count2=0,white_count3=0,black_count=0,right_lost=0;
    //right_ring_first_angle();//扫描是否存在右环角点
    //left_ring_circular_arc();//扫描是否存在左环上弧
    for(int y=loop_forward_near;y<95;y++)//逐行扫描
    {
        if(((r_border[y+2]-r_border[y])>2)||(r_border[y]-r_border[y+2])>4)
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
               if(Grayscale[119-y][l_border[watch.InLoopAngleL]]==0)
                  black_count++;
           }
		   //老学长上位机
			//           vofa.loop[6]=l_border[watch.InLoopAngleL].left;
		   //           vofa.loop[7]=black_count;   
           if(/*(white_count1>=10&&white_count2>=10&&white_count3>=10)&&*/(black_count<10))
           {
			   //状态机
               //enter_element(Left_ring);    //正式进入左圆环元素
			   //编码器积分
               //begin_distant_integeral(6000);
              // if(Element_rem.loop_data[Element_rem.loop_count]==0)//如果是小环
			   //牢学长的石
//               {
//                   set_speed(setpara.loop_target_speed);
//                   mycar.pid_ctrl=0;
//                   change_pid_para(&CAM_Turn,&setpara.loop_turn_PID);//将转向PID参数调为环内转向PID
//               }
/*               else//如果是大环
               {
                   set_speed(setpara.big_loop_speed);
                   //change_pid_para(&CAM_Turn,&setpara.big_loop_PID);
               }*/
               watch.InLoop = 1;
               //beep2(1,20);//蜂鸣器
               return;
           }
       }
	   //状态机
    //Element=None;
    watch.InLoopAngleR=120;
    watch.InLoopAngleL=120;
}

//检测左环入环第一个角点后的圆弧
void left_ring_circular_arc()
{
    if (watch.InLoop != 1&&watch.InLoop != 2)return;//在循环之前跳出，节省时间
    //beep(20);
    for(int y=loop_forward_near;y<loop_forward_far;y++)//逐行扫描
    {
        if (//y <watch.InLoopAngle2  &&
               (watch.InLoopAngleL<65)//去除了两个积分条件
           &&(y>(watch.InLoopAngleL+20))
           &&y <watch.InLoopCirc
           &&!leftlost[y+3]
           &&!leftlost[y+2]
           &&!leftlost[y+1]
           &&!leftlost[y-3]
           &&!leftlost[y-2]
           &&!leftlost[y-1]
           &&l_border[y+1] <= l_border[y]
           &&l_border[y+2] <= l_border[y]
           &&l_border[y+3] <= l_border[y]
           &&l_border[y-1] <= l_border[y]
           &&l_border[y-2] <= l_border[y]
           &&l_border[y-3] <= l_border[y]
           //&&(watch.right_lost+watch.cross_lost)<5
            )
       { //入环点所在行
            watch.InLoopCirc = y;
            //beep(20);
            break;
       }
    }
}

//检测左环第二个角点
void left_ring_second_angle()
{
    if(watch.InLoop != 1&&watch.InLoop != 2)return;//在循环之前跳出，节省时间
    for(int y=loop_forward_near;y<loop_forward_far;y++)//逐行扫描
    {
        if (watch.InLoopCirc<66
             &&y<watch.InLoopAngle2
             &&watch.InLoopAngle2==120
             //&&get_integeral_state(&distance_integral)==2
           &&y > 60
           &&y < (loop_forward_far-2)
           &&y>watch.InLoopCirc
           &&l_border[y+1] > 30
           &&(l_border[y+1]-l_border[y])<=2
           &&(l_border[y]-l_border[y-4])>l_border[y]/2
           )
           {
               watch.InLoopAngle2 = y;
               watch.InLoopAngle2_x=l_border[watch.InLoopAngle2];
               //if()
               //watch.InLoopCirc=0;
               break;
           }
    }
	//持续抓住第一角点，保证补线完整
    if(watch.InLoopAngle2!=120
        &&watch.InLoopAngle2>50
        )
    {
        find_angle_left_down(&watch.InLoopAngle2_x,&watch.InLoopAngle2);
    }
}

//左环开始转向函数(watch.InLoop=2) 状态机函数
void left_ring_begin_turn()
{
	//去除了路径积分和角度积分
    if(watch.InLoop!=1)return;//在循环之前跳出，节省时间
    if(/*get_integeral_state(&distance_integral)==2 路程积分完成
        &&*/watch.InLoop==1
        &&watch.InLoopAngle2<=80
    )
    {
        //clear_distant_integeral();//清除路程积分变量
        watch.InLoop=2;
        //set_speed(setpara.loop_target_speed);
        //change_pid_para(&CAM_Turn,&setpara.loop_turn_PID);//将转向PID参数调为环内转向PID
        //watch.fix_slope=(float)(lineinfo[watch.InLoopAngle2].left)/(115-watch.InLoopAngle2);
        //begin_angle_integeral(260);
        //beep2(2,20);
    }
}

//检验小车是否完全入环
void left_ring_in_loop()
{
    if(watch.InLoop != 2)return;
    if( watch.InLoop == 2
			//&&get_integeral_state(&angle_integral)==1
			//&&get_integeral_data(&angle_integral)>40//转过一定角度
        //&&watch.right_near_lost<40
        //&&lineinfo[watch.right_near_lost].right>180
        //&&(!lineinfo[40].right_lost)
        )
        {
            //watch.InLoopCirc = 0;
            watch.InLoop = 3;
            //change_pid_para(&CAM_Turn,&setpara.loop_turn_PID);//将转向PID参数调为环内转向PID
            set_speed(setpara.loop_target_speed+3);
            beep2(3,20);
        }
}
//小车角度积分完成，准备出环
void left_ring_prepare_out()
{
    if(watch.InLoop != 3)return;
    if( watch.InLoop == 3
			//&&get_integeral_state(&angle_integral)==1
			//&&get_integeral_data(&angle_integral)>160
        &&r_border[69]<152
        &&r_border[69]>82)
   {
       watch.InLoop = 4;
       watch.OutLoop_turn_point_x=r_border[69];
       //beep2(4,20);
   }
}
//检测出环时右角点位置
void left_ring_out_angle()
{
    if(watch.InLoop != 4)return;//在循环之前跳出，节省时间
    for(int y=loop_forward_near;y<loop_forward_far;y++)//逐行扫描
        {
        if ((watch.InLoop == 4)&&y<80
                 //lineinfo[y].left_lost
                 &&r_border[y+1] >= r_border[y]
                 &&r_border[y+2] >= r_border[y+1]
                 &&r_border[y-1] >= r_border[y]
                 &&r_border[y-2] >= r_border[y]
/*                 &&lineinfo[y - 3].right > lineinfo[y - 1].right
                 &&lineinfo[y + 4].right > lineinfo[y + 2].right
                 &&lineinfo[y - 5].right > lineinfo[y - 3].right*/
                 &&r_border[y] > 30
                 &&Grayscale[119-y-2][r_border[y]]==255
)
             {
                 if(watch.OutLoopAngle1>y)
                 {
                     //watch.OutLoopRight = lineinfo[y].right;
                     watch.OutLoopAngle1 = y; //出环判断列
                     break;
                 }
             }
        }
}
//出左环右转(进入圆环与直线交界处)
void left_ring_out_loop_turn()
{
    if(watch.InLoop != 4)return;
    if(watch.InLoop == 4
       //&&(watch.cross+watch.right_lost)>30//右侧丢线过多，说明进入圆环与直线交界处
       &&watch.OutLoopAngle1<120
		//&&get_integeral_state(&angle_integral)==2
       &&rightlost[watch.OutLoopAngle1]==1
       &&watch.OutLoop==0
    )
    {
		//清除角度积分
			//clear_angle_integeral();
//        if(Element_rem.loop_data[Element_rem.loop_count]==0)//如果是小环
//        {
//            begin_angle_integeral(setpara.loop_angle_out);
//        }
//        else//如果是大环
//        {
//            begin_angle_integeral(setpara.big_loop_out);
//        }
			//begin_distant_integeral(3000);//开启路程积分，此时要保持左转
        watch.OutLoop=1;
        //beep2(5,20);
    }
}
//右侧为直线时直行
void left_ring_out_loop()
{
    if(watch.InLoop != 4&&watch.OutLoop!=1)return;
    for(int y=loop_forward_near;y<loop_forward_far;y++)//逐行扫描
        {
        if((watch.InLoop == 4
            &&watch.OutLoop==1
            &&y<60
            &&rightlost[y]==0
            &&rightlost[y+10]==0
            &&r_border[y]>10///////////////////
            &&r_border[y+10]>10/////////////////
            &&(188-(r_border[y]-r_border[y+10])*(115-y)/10)>40
            &&r_border[y]>r_border[y+10]
				//&&get_integeral_state(&distance_integral)==2)
                )
            {
            clear_distant_integeral();
//            if(Element_rem.loop_data[Element_rem.loop_count]==0)//如果是小环
//            {
                begin_distant_integeral(setpara.loop_out_distance);
//            }
//            else//如果是大环
//            {
//                begin_distant_integeral(setpara.big_loop_out_distance);
//            }
            clear_angle_integeral();
            beep2(6,20);
            watch.InLoop =5;////不用陀螺仪，用摄像头自身提取赛道元素；
            }
        }
}
//检测出左环进入直线后左侧角点
void left_ring_straight_out_angle()
{
    if(watch.InLoop != 5&&watch.OutLoop!=1)return;
    for(int y=loop_forward_near;y<loop_forward_far;y++)//逐行扫描
        {
        if((watch.InLoop==5)
          &&watch.OutLoop==1
			//&&y<(watch.watch_lost-10)
          &&y<80
          &&y<watch.OutLoopAngle2
          &&l_border[y]<120
          &&r_border[y]>60
          && watch.zebra_flag == 0
          &&(r_border[y + 2]-l_border[y + 2])<(r_border[y -1]-l_border[y -1])-30
          &&(r_border[y + 1]-l_border[y + 1])<(r_border[y -2]-l_border[y -2])-30
          &&(r_border[y]-l_border[y])<(r_border[y -3]-l_border[y -3])-30
          &&r_border[y+1]-r_border[y+2]<5
          &&!leftlost[y]
          &&!leftlost[y+1]
          &&!leftlost[y+2])///////////
            {
                watch.OutLoopAngle2 = y;
            }
        }
}

//检测完全出环
void left_ring_complete_out()
{
    if(watch.InLoop != 5&&watch.OutLoop!=1)return;
    for(int y=loop_forward_near;y<loop_forward_far;y++)//逐行扫描
    {
    if (watch.InLoop == 5
			//&&get_integeral_state(&distance_integral)==2
        &&watch.OutLoopAngle2<70
        )
     {
         //clear_all_flags();// 出环成功,清除所有标志
         //状态机
		 //out_element();
         //mycar.target_speed=setpara.com_target_speed;//恢复正常速度
         //change_pid_para(&CAM_Turn,&setpara.com_turn_PID);//恢复正常转向PID
         //beep2(7,20);
     }
    }
}
