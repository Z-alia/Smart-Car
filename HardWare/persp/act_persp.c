#include "act_persp.h"
#include "image.h"
#include "Binarization.h"

#define img_showmode   2 //显示模式选择




//逆透视变换矩阵
//float a1=0.488,a2=-0.000,a3=-0.000,b1=-0.776,b2=0.371,b3=-0.008,c1=47.835,c2=3.883;
//float a1=0.4571,a2=0.0000,a3=0.0000,b1=-0.7694,b2=0.3314,b3=-0.0086,c1=48.1971,c2=4.9143;
double a1=0.457143,a2=0.000000,a3=0.000000,b1=-0.801429,b2=0.331429,b3=-0.008571,c1=50.757143,c2=4.914286;
//struct persp_struct persp_line[50];
//uint8 persp_x_left[50][94];
//uint8 persp_x_right[50][94];
int16 persp_x[80][188];
uint8 persp_y[80];
//静态逆透视初始化（先计算一遍，存入数组，后面就不用再算一遍了，以空间换时间）
void act_perst_init()
{
    int ox,oy;
    for(int y=0;y<80;y++)//x轴只存放30行到79行的数据，节省空间
    {
        for(int x=0;x<188;x++)
        {
            ox=(int)((a1*x+b1*y+c1)/(a3*x+b3*y+1));
            persp_x[y][x]=(int16)(ox);
        }
    }
    for(int y=0;y<80;y++)//y轴全部存放
    {
        oy=(int)((a2*0+b2*y+c2)/(a3*0+b3*y+1));
        if(oy>255)ox=255;
        else if(oy<0)ox=0;
        persp_y[y]=(uint8)oy;
    }
}

//逆透视变换函数
int16 get_persp_data_ox(int16 x,int16 y)
{
    if(y>=0&&y<80)
        return (persp_x[y][x]);
    else                            //如果不在范围内就计算
        return 0;//(a1*x+b1*y+c1)/(a3*x+b3*y+1);
    //return (a1*x+b1*y+c1)/(a3*x+b3*y+1);
}
int16 get_persp_data_oy(int16 x,int16 y)
{
    if(y>=0&&y<80)
        return persp_y[y];              //如果在存储范围内就直接取出
    else                                //如果不在范围内就计算
    return 0;//(a2*x+b2*y+c2)/(a3*x+b3*y+1);
}


//进行逆透视变换并计算斜率（以下为祖传算法）
void persp_task(int16 xl,int16 xr,int16 y)
{
    int16 oxl,oxr,oyl,oyr;
    //限幅
    if(xl<0)xl=0;
    else if(xl>187)xl=187;
    if(xr<0) xr=0;
    else if(xr>187)xr=187;
    //记录补线后的结果
    //lineinfo[y].left_adjust=xl;
    //lineinfo[y].right_adjust=xr;
    //记录扫线值
    //edge_line(y,xl,xr);
    //if(y<80)
    {
        oxl=get_persp_data_ox(xl,y);
        oyl=get_persp_data_oy(xl,y);
        oxr=get_persp_data_ox(xr,y);
        oyr=get_persp_data_oy(xr,y);

        persp_lx[y] = oxl;
        persp_ly[y] = oyl;
        persp_rx[y] = oxr;
        persp_ry[y] = oyr;
/*
    lineinfo[y].angel_left  = 125 * (lineinfo[y].persp_lx - setpara.camwl ) / (lineinfo[y].persp_ly - setpara.camwf );//persp_lx=67
    //计算此行右边界点到右轮的斜率的倒数的1000倍
    lineinfo[y].angel_right = 125 * (lineinfo[y].persp_rx - setpara.camwr ) / (lineinfo[y].persp_ry - setpara.camwf );//persp_lf=97
*/
    }

//imo是对lineinfo[y]->left和lineinfo[y]—>right进行逆透视变换后左以1存储，右以2存储
    if (img_showmode == 2) //188*120 显示模式2（显示正常扫线图线）
        {

                imo[y][l_border[y]] = 1;

                imo[y][r_border[y]] = 2;
        }
    else if (img_showmode == 3) //188*120 显示模式3（显示补线图）
    {
       if (oxl < 188&&oxl>=0&&oyl<120&&oyl>=0)
           imo[y][xl] = 1;
       if (oxr < 188&&oxr>=0&&oyr<120&&oyr>=0)
           imo[y][xr] = 2;
    }
    else if (img_showmode == 4) //128*128 显示模式4 （显示逆透视图像）
    {
       if (oxl < 188&&oxl>=0&&oyl<120&&oyl>=0)
       imo[oyl ][oxl ] = 1;
       if (oxr < 188&&oxr>=0&&oyr<120&&oyr>=0)
       imo[oyr ][oxr ] = 2;
    }
}
