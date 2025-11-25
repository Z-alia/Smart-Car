/*
 * patch_line.c
 *
 *  Created on: 2023年6月21日
 *      Author: Admin
 */
#include "patch_line.h"
float slopeL;
float slopeOL;
float slopeR;
float slopeOR;
float slopeJ;
float slopeCL;
float slopeCR;
//进行补线与逆透视变换
void linefix()
{
    switch(Element)
    {
        case Left_ring:
            left_ring_linefix();
            break;
        case Right_ring:
            right_ring_linefix();
            break;
//        case ingarage:
//            left_garage_linefix();
            /*if (watch.zebra_flag==1 && watch.Zebra_Angle < 68) ingarage_linefix_1();
            else if (watch.zebra_flag2==1 && watch.Zebra_Angle2 < 68) ingarage_linefix_2(); */
            break;
        case crossing:
            crossing_linefix();
            break;
//        case obstacle:
//            obstacle_linefix();
//            break;
//        case black_obstacle:
//            black_obstacle_linefix();
//            break;
        default:
            common_linefix();
            break;
    }
}

//正常情况下,只逆透视变换，不补线
void common_linefix()
{
    for (int y =forward_near; y <=watch.watch_lost; y++)
    {
        int16 xl,xr;
        xl = lineinfo[y].left;
        xr = lineinfo[y].right;
        //记录补线后的结果
        lineinfo[y].left_adjust=xl;
        lineinfo[y].right_adjust=xr;
        //对补线后的结果进行逆透视变换
        //persp_task(xl,xr,y);
    }
}

void left_ring_linefix()
{
    int16 xl,xr;
    //vofa.loop[2]=watch.watch_lost;
    for (int y =forward_near; y <=watch.watch_lost; y++)
    {
        xl = lineinfo[y].left;
        xr = lineinfo[y].right;
        if (watch.InLoop == 1 && watch.InLoopAngleL < watch.InLoopCirc
              && watch.zebra_flag == 0
              && y < 81 && watch.InLoopAngle2 == 120)
           {// 先拉一道实现封住出口,由于左边丢线右边不丢线,故以右边为参考补左边线
              slopeL=(float)(lineinfo[0].right-lineinfo[80].right)/80;//x=k*y
              watch.top_x=lineinfo[0].right-118*slopeL;
              slopeL=(float)(watch.top_x-lineinfo[0].left)/118;
               xl = watch.top_x-slopeL*(118-y);
           }

           // 开始入左环
           // 入环点为了解决从右边沿拉线导致，打角不稳问题
          else if(watch.InLoop == 2)
          {
              //slopeR=(float)(lineinfo[40].right-watch.InLoopAngle2_x)/(watch.InLoopAngle2-40);
              slopeR=(float)watch.InLoopAngle2_x/(115-watch.InLoopAngle2);//115是左顶点纵坐标
              xr=slopeR*(watch.InLoopAngle2-y)+watch.InLoopAngle2_x;
              if(y>watch.InLoopAngle2||watch.InLoopAngle2<70)xl=0;
          }
          else if(watch.InLoop == 3)
          {
              if(y>50)xl=0;
          }
           // 开始出左环
           else if (watch.InLoop == 4 )
           {
               if(y>50)xl=0;
               if(lineinfo[watch.OutLoopAngle1].right > 60 && y > watch.OutLoopAngle1)
               {
               // 一元一次方程,参考图片/出左环.png
               xr=watch.OutLoop_turn_point_x+(69-y);
               }
               //begin_angal_integeral(50);
           }
           // 出左环直行
           else if (watch.InLoop == 5
                   &&watch.OutLoopAngle2==120
                   &&watch.zebra_flag == 0)
           {// 封住入环口,补线思路是从角点向下拉线到near右边沿减145的地方
               // xl = lineinfo[y].right - 132 + y;
               slopeL=(float)(lineinfo[45].right-lineinfo[75].right)/30;
               watch.top_x=lineinfo[45].right-73*slopeL;
               slopeL=(float)(watch.top_x-20)/118;
                xl = watch.top_x-slopeL*(118-y);
    //            slopeL=(lineinfo[watch.watch_lost].left-lineinfo[100].left)/(watch.watch_lost-100);
    //            xl = lineinfo[100].left+(y-100)*slopeL;
           }
           else if (watch.InLoop == 5
                   &&watch.OutLoopAngle2!=120
                   &&y < watch.OutLoopAngle2
                   && watch.zebra_flag == 0)
           {// 封住入环口，基本跟上面一样，为了鲁棒大圆环
               // xl = lineinfo[y].right - 132 + y;
               slopeL=(float)(lineinfo[20].right-lineinfo[80].right)/60;
               watch.top_x=lineinfo[20].right-98*slopeL;
               slopeL=(float)(watch.top_x-lineinfo[watch.OutLoopAngle2+1].left)/(117-watch.OutLoopAngle2);
               xl=watch.top_x-slopeL*(118-y);
    //            slopeL=(lineinfo[watch.watch_lost].left-lineinfo[100].left)/(watch.watch_lost-100);
    //            xl = lineinfo[100].left+(y-100)*slopeL;
           }
        //对补线后的结果进行逆透视变换
        //persp_task(xl,xr,y);
		   lineinfo[y].left_adjust=xl;
        lineinfo[y].right_adjust=xr;
    }
}

void right_ring_linefix()
{
    int16 xl,xr;
    for (int y =forward_near; y <=watch.watch_lost; y++)
    {
        xl = lineinfo[y].left;
        xr = lineinfo[y].right;
        if (watch.InLoop == 6 && watch.InLoopAngleR < watch.InLoopCirc
              && watch.zebra_flag == 0
              && y < 81 && watch.InLoopAngle2 == 120)
           {// 先拉一道实现封住出口,由于左边丢线右边不丢线,故以右边为参考补左边线
              slopeR=(float)(lineinfo[60].left-lineinfo[0].left)/60;
              watch.top_x=lineinfo[0].left+118*slopeR;
              slopeR=(float)(lineinfo[0].right-watch.top_x)/118;
               xr = watch.top_x+slopeR*(118-y);
           }

           // 开始入右环
           // 入环点为了解决从右边沿拉线导致，打角不稳问题
          else if(watch.InLoop == 7)
          {
              //slopeL=(float)(watch.InLoopAngle2_x-lineinfo[40].left)/(watch.InLoopAngle2-40);
              slopeL=(float)(188-watch.InLoopAngle2_x)/(118-watch.InLoopAngle2);
              //xl=188-watch.fix_slope*(115-y);
              xl=slopeL*(y-watch.InLoopAngle2)+watch.InLoopAngle2_x;
              if(y>watch.InLoopAngle2||watch.InLoopAngle2<70)xr=187;
              //xr=187;
          }
          else if(watch.InLoop == 8)
          {
              if(y>50)xr=187;
          }
           // 开始出右环
           else if (watch.InLoop == 9 )
           {// 一元一次方程
               if(lineinfo[watch.OutLoopAngle1].left < 128 && y > watch.OutLoopAngle1)
               {
               //watch.OutLoop=1;
               //slopeOR = (float)(188-lineinfo[watch.OutLoopAngle1].left) / (watch.watch_lost - watch.OutLoopAngle1);
               //xl = 188-slopeOR * (watch.watch_lost - y);
               xl=watch.OutLoop_turn_point_x+(y-69);
               }
               if(y>50)xr=187;
               //begin_angal_integeral(50);
           }
           // 出右左环直行
           else if (watch.InLoop == 10
                   &&watch.OutLoopAngle2==120
                   &&watch.zebra_flag == 0)
           {// 封住入环口,补线思路是从角点向下拉线到near右边沿减145的地方
               // xl = lineinfo[y].right - 132 + y;
               slopeR=(float)(lineinfo[75].left-lineinfo[45].left)/30;
               watch.top_x=lineinfo[45].left+73*slopeR;
               slopeR=(float)(168-watch.top_x)/118;
                xr = watch.top_x+slopeR*(118-y);
    //            slopeL=(lineinfo[watch.watch_lost].left-lineinfo[100].left)/(watch.watch_lost-100);
    //            xl = lineinfo[100].left+(y-100)*slopeL;
           }
           else if (watch.InLoop == 10
                   &&watch.OutLoopAngle2!=120
                   &&y < watch.OutLoopAngle2
                   && watch.zebra_flag == 0)
           {// 封住入环口，基本跟上面一样，为了鲁棒大圆环
               // xl = lineinfo[y].right - 132 + y;
//               slopeR=(float)(lineinfo[80].left-lineinfo[20].left)/60;
               slopeR=(float)(lineinfo[watch.angle_far_line].left-lineinfo[20].left)/(watch.angle_far_line-20);

               watch.top_x=lineinfo[20].left+98*slopeR;
               slopeR=(float)(lineinfo[watch.OutLoopAngle2+1].right-watch.top_x)/(117-watch.OutLoopAngle2);
               xr=watch.top_x+slopeR*(117-y);
    //            slopeL=(lineinfo[watch.watch_lost].left-lineinfo[100].left)/(watch.watch_lost-100);
    //            xl = lineinfo[100].left+(y-100)*slopeL;
           }
        //对补线后的结果进行逆透视变换
        //persp_task(xl,xr,y);
		   lineinfo[y].left_adjust=xl;
        lineinfo[y].right_adjust=xr;
    }
}

#define OutLeft  1
#define OutRight 2
//void garage_linefix()
//{
//    int16 xl,xr;
//    //flag.CircCount=0;
//    float slopeZL,slopeZR;
//    for (int y =forward_near; y <=watch.watch_lost; y++)
//    {
//        // 左车库 补第一段
//        if(watch.zebra_flag && setpara.mode == OutLeft && y<=watch.ZebraLine && mycar.CircCount==3)
//        {
//            slopeZL = (187.0 - lineinfo[watch.ZebraLine + 2].left * 1.0) / (watch.ZebraLine) * 0.8;
//            if( (watch.ZebraLine - y) * slopeZL + lineinfo[watch.ZebraLine + 2].left < xr)
//                xr = (watch.ZebraLine - y) * slopeZL + lineinfo[watch.ZebraLine + 2].left;
//                //if( y > watch.ZebraLine - 20 ||  !caminfo.zebra_count)
//                xl = 0;
//        }
//        // 右车库 补第一段
//        else if(watch.zebra_flag && setpara.mode == OutRight && y<=watch.ZebraLine && mycar.CircCount==3)
//        {
//            slopeZR = (lineinfo[watch.ZebraLine + 2].right * 1.0) / (watch.ZebraLine) * 0.8;
//            if( lineinfo[watch.ZebraLine + 2].right - (watch.ZebraLine - y) * slopeZR > xl)
//                xl = lineinfo[watch.ZebraLine + 2].right - (watch.ZebraLine - y) * slopeZR;
//    //            if( y > watch.ZebraLine - 20 ||  !caminfo.zebra_count)
//                xr = 187;
//        }
//        // 左车库 补第二段
//        if(watch.zebra_flag && setpara.mode == OutLeft && y>watch.ZebraLine && mycar.CircCount==3)
//        {
//            slopeZL = (lineinfo[watch.ZebraLine + 2].left * 1.0) / (119 - watch.ZebraLine);
//    //            if( lineinfo[watch.ZebraLine + 2].left - slopeZL * (y - watch.ZebraLine) < xr)
//                xr = lineinfo[watch.ZebraLine + 2].left - slopeZL * (y - watch.ZebraLine);
//            xl = 0;
//        }
//        // 右车库 补第二段
//        else if(watch.zebra_flag && setpara.mode == OutRight && y>watch.ZebraLine && mycar.CircCount==3)
//        {
//            slopeZR = (187.0 - lineinfo[watch.ZebraLine + 2].right * 1.0) / (119 - watch.ZebraLine );
//    //            if( lineinfo[watch.ZebraLine + 2].right + slopeZR * (y - watch.ZebraLine) > xl)
//                xl = lineinfo[watch.ZebraLine + 2].right + slopeZR * (y - watch.ZebraLine);
//            xr = 187;
//        }
//        //记录补线后的结果
//        lineinfo[y].left_adjust=xl;
//        lineinfo[y].right_adjust=xr;
//        //对补线后的结果进行逆透视变换
//        //persp_task(xl,xr,y);

//    }
//}
void crossing_linefix()
{
    int bottom_l,bottom_r;
    static float top_x;
    static float  SlopeR,SlopeL;
    for (int y =forward_near; y <=watch.watch_lost; y++)
    {
        int16 xl,xr;
//        xl = lineinfo[y].left;
//        xr = lineinfo[y].right;
        if(watch.cross_flag==3)
        {
            if(watch.cross_AngleL<110&&watch.cross_AngleR<110&&watch.cross_AngleR_x-watch.cross_AngleL_x>15)
            {
                if(lineinfo[0].left_lost)bottom_l=20;
                else bottom_l=lineinfo[0].left;
                if(lineinfo[0].right_lost)bottom_r=168;
                else bottom_r=lineinfo[0].right;
                slopeCL=(float)(watch.cross_AngleL_x-bottom_l)/watch.cross_AngleL;
                xl=slopeCL*y+bottom_l;
                slopeCR=(float)(bottom_r-watch.cross_AngleR_x)/watch.cross_AngleR;
                xr=bottom_r-slopeCR*y;
            }
            else
            {
                xl = lineinfo[y].left;
                xr = lineinfo[y].right;
            }

        }
        else if(watch.cross_flag==2)
        {
            if(watch.cross_RD_angle>21)
            {
                float x1=lineinfo[watch.cross_RD_angle].right,x2=lineinfo[watch.cross_RD_angle-5].right,x3=lineinfo[watch.cross_RD_angle-10].right,x4=lineinfo[watch.cross_RD_angle-15].right;
                float y1=watch.cross_RD_angle;
                float x_avg=(x1+x2+x3+x4)/4;
                float y_avg=(float)watch.cross_RD_angle-7.5;
                SlopeR=-(x1*x1+x2*x2+x3*x3+x4*x4-4*x_avg*x_avg)/(y1*x1+(y1-5)*x2+(y1-10)*x3+(y1-15)*x4-4*y_avg*x_avg);//用线性回归计算斜率
                top_x=lineinfo[0].right-SlopeR*115;
                xr = lineinfo[0].right-SlopeR*y;
                SlopeL=(float)(top_x-lineinfo[0].left)/115;
                xl=top_x-(115-y)*SlopeL;
            }
            else
            {
                xr=top_x+SlopeR*(115-y);
                xl=top_x-(115-y)*SlopeL;
            }
        }
        else if(watch.cross_flag==1)
        {
            if(watch.cross_LD_angle>21)
            {
                float x1=lineinfo[watch.cross_LD_angle].left,x2=lineinfo[watch.cross_LD_angle-5].left,x3=lineinfo[watch.cross_LD_angle-10].left,x4=lineinfo[watch.cross_LD_angle-15].left;
                float y1=watch.cross_LD_angle;
                float x_avg=(x1+x2+x3+x4)/4;
                float y_avg=(float)watch.cross_LD_angle-7.5;
                SlopeL=(x1*x1+x2*x2+x3*x3+x4*x4-4*x_avg*x_avg)/(y1*x1+(y1-5)*x2+(y1-10)*x3+(y1-15)*x4-4*y_avg*x_avg);//用线性回归计算斜率
                top_x=lineinfo[0].left+SlopeL*115;
                xl = lineinfo[0].left+SlopeL*y;
                SlopeR=(float)(lineinfo[0].right-top_x)/115;
                xr=top_x+(115-y)*SlopeR;
            }
            else
            {
                xr=top_x+SlopeR*(115-y);
                xl=top_x-(115-y)*SlopeL;
            }
        }
        else
        {
            xl = lineinfo[y].left;
            xr = lineinfo[y].right;
//                    //记录补线后的结果
//                    lineinfo[y].left_adjust=xl;
//                    lineinfo[y].right_adjust=xr;
//                    //对补线后的结果进行逆透视变换
//                    persp_task(xl,xr,y);
        }

       if(lineinfo[y].left>94)
        {
            xl=0;
        }
        if(lineinfo[y].right<86)
        {
            xr=187;
        }
        //记录补线后的结果
        lineinfo[y].left_adjust=xl;
        lineinfo[y].right_adjust=xr;
        //对补线后的结果进行逆透视变换
        //persp_task(xl,xr,y);
    }
}
//void left_garage_linefix()
//{
//    for (int y =forward_near; y <=watch.watch_lost; y++)
//    {
//        int16 xl,xr;
//        xl = lineinfo[y].left;
//        xr = lineinfo[y].right;
//        if (watch.zebra_flag==1&& y < 91 && watch.Zebra_Angle2 ==120)
//           {// 先拉一道实现封住出口,由于左边丢线右边不丢线,故以右边为参考补左边线
//              slopeL=(float)(lineinfo[50].right-lineinfo[90].right)/40;
//              watch.top_x=lineinfo[50].right-65*slopeL;
//              slopeL=(float)(watch.top_x-lineinfo[25].left)/90;
//               xl = watch.top_x-slopeL*(115-y);
//           }
//        else if(watch.zebra_flag==2&&watch.Zebra_Angle2<90)
//        {
//            xl=0;
//            slopeR=(float)watch.Zebra_Angle2_x/(115-watch.Zebra_Angle2);
//            xr=slopeR*(watch.Zebra_Angle2-y)+watch.Zebra_Angle2_x;
//        }
//        else if(watch.zebra_flag==4)
//        {
//            slopeR=(float)(lineinfo[90].left-lineinfo[50].left)/40;
//            watch.top_x=lineinfo[50].left+65*slopeR;
//            slopeR=(float)(watch.top_x-lineinfo[35].left)/80;
//             xr = watch.top_x+slopeR*(115-y);
//        }
//        else if(watch.zebra_flag==5&&watch.Zebra_Angle2<90)
//        {
//            //slopeL=(float)(watch.InLoopAngle2_x-lineinfo[40].left)/(watch.InLoopAngle2-40);
//            slopeL=(float)(188-watch.Zebra_Angle2_x)/(115-watch.Zebra_Angle2);
//            //xl=188-watch.fix_slope*(115-y);
//            xl=slopeL*(y-watch.Zebra_Angle2)+watch.Zebra_Angle2_x;
//            xr=187;
//        }

//        //记录补线后的结果
//        lineinfo[y].left_adjust=xl;
//        lineinfo[y].right_adjust=xr;
//        //对补线后的结果进行逆透视变换
//        persp_task(xl,xr,y);
//    }
//}

//void ingarage_linefix_1()
//{
//    for (uint8 y = forward_near+10; y < watch.watch_lost-2; y += 1)
//    {
//        int16 xl,xr;
//        float kl,kr;
//        {
//              kl = 8/125;
//              kr = -188/120;
//              xl = 0.08*y-1.2;
//              xr = kr*y+188;
//              lineinfo[y].left_adjust=xl;
//              lineinfo[y].right_adjust=xr;
//              persp_task(xl,xr,y);//对补线后的结果进行逆透视变换
//        }
//    }
//}

//void ingarage_linefix_2()
//{
//    for (uint8 y = forward_near+10; y < watch.watch_lost-2; y += 1)
//    {
//        int16 xl,xr;
//        float kl,kr;
//        {
//             /* kl = 8/125;
//              kr = 8/125;
//              xr = -0.08*y+189.6;
//              xl = kl*y;*/
//              kl=188/120;
//              kr=-0.08;
//              xl=kl*y;
//              xr=kr*y+189.6;
//              lineinfo[y].left_adjust=xl;
//              lineinfo[y].right_adjust=xr;
//              persp_task(xl,xr,y);//对补线后的结果进行逆透视变换
//        }
//    }
//}
//void obstacle_linefix()
//{
//    for (int y =forward_near; y <=watch.watch_lost; y++)
//    {
//        int16 xl,xr;
//        xl = lineinfo[y].left;
//        xr = lineinfo[y].right;
//        if(watch.obstacle_flag==5&&setpara.obstacle_dir==0)
//        {
//            xl=watch.top_x-(115-y)*watch.fix_slope;
//        }
//        else if(watch.obstacle_flag==5&&setpara.obstacle_dir==1)
//        {
//            xr=watch.top_x+(115-y)*watch.fix_slope;
//        }

//        //记录补线后的结果
//        lineinfo[y].left_adjust=xl;
//        lineinfo[y].right_adjust=xr;
//        //对补线后的结果进行逆透视变换
//        persp_task(xl,xr,y);
//    }
//}

//void black_obstacle_linefix()
//{
//    for (int y =forward_near; y <=watch.watch_lost; y++)
//    {
//        int16 xl,xr;
//        if((!watch.left_obstacle_flag&&!watch.right_obstacle_flag)||watch.black_obstacle_line<41)
//        {
//            xl = lineinfo[y].left;
//            xr = lineinfo[y].right;
//        }
//        else if(watch.left_obstacle_flag)
//        {
//            slopeOL=(float)(watch.left_obstacle_x-lineinfo[0].left)/watch.black_obstacle_line;
//            xl=slopeOL*y+lineinfo[0].left;
//            xr=lineinfo[y].right;
//        }
//        else if(watch.right_obstacle_flag)
//        {
//            slopeOR=-(float)(watch.right_obstacle_x-lineinfo[0].right)/watch.black_obstacle_line;
//            xl=lineinfo[y].left;
//            xr=-slopeOR*y+lineinfo[0].right;
//        }
//            //记录补线后的结果
//            lineinfo[y].left_adjust=xl;
//            lineinfo[y].right_adjust=xr;
//            //对补线后的结果进行逆透视变换
//            persp_task(xl,xr,y);
//    }
//}

