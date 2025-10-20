
//本文件留作接口文件，剩下的元素识别相关的需要自己移植
#include "Element_recognition.h"
#include "image.h"
#include "Binarization.h"
#include "main.h"



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

//环岛检测状态机

//图像预处理


