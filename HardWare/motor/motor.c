#include "motor.h"
#include "main.h"
#include "tim.h"
#include <math.h>   // fabsf
#include "Element_recognition.h"
#include "Binarization.h"
//#include "image.h"
//本工程的PWM分辨率为1000
#define tgtspd 100
#define TL_tgtspd 100
#define TR_tgtspd 100
Motor leftmotor={0, 1, tgtspd, 0};
Motor rightmotor={0, 1, tgtspd, 1};

// 初始化电机驱动
void motor_init(void)
{
    // 初始化电机控制引脚
    HAL_GPIO_WritePin(PH_LMOTOR_GPIO_Port, PH_LMOTOR_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PH_RMOTOR_GPIO_Port, PH_RMOTOR_Pin, GPIO_PIN_RESET);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    motor_stop();
}

// 控制单个电机运行 (speed: -1000 to 1000)
void motor_run(Motor *motor_ptr, int16_t speed,uint8_t v)
{
    if ((speed<-1000)||(speed>1000))
        return;
    motor_ptr->speed = speed;
    if(motor_ptr->lor == 0) // 左电机
    {
        if(speed >= 0)
        {
            motor_ptr->dir = 1; // 正转
            HAL_GPIO_WritePin(PH_LMOTOR_GPIO_Port, PH_LMOTOR_Pin, GPIO_PIN_SET);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed+v);
        }
        else if(speed < 0)
        {
            motor_ptr->dir = 0; // 反转
            speed = -speed;
            HAL_GPIO_WritePin(PH_LMOTOR_GPIO_Port, PH_LMOTOR_Pin, GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed+v);
        }
    }
    else // 右电机
    {
        if(speed >= 0)
        {
            motor_ptr->dir = 1; // 正转
            HAL_GPIO_WritePin(PH_RMOTOR_GPIO_Port, PH_RMOTOR_Pin, GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed+v);
        }
        else if(speed < 0)
        {
            motor_ptr->dir = 0; // 反转
            speed = -speed;
            HAL_GPIO_WritePin(PH_RMOTOR_GPIO_Port, PH_RMOTOR_Pin, GPIO_PIN_SET);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed+v);
        }
    }
}

// 刹车
void motor_stop(void)
{
    motor_run(&leftmotor, 0,0);
    motor_run(&rightmotor, 0,0);
}
/*
void straight_error_get(PIDController *PID)
{
    uint8_t edge_store[10]; // 存储边沿位置的数组，大小根据实际情况调整
    uint8_t left=0, right=0;
    for(uint8_t i=0;i<10;i++)
    {
       if(edge_store[i]>left&&edge_store[i]<94)
           left=edge_store[i];
        else if(edge_store[i]<right&&edge_store[i]>94)
            right=edge_store[i];
    }
    get_orign_edges(imo[5],edge_store);
    PID->error = (left + right) / 2 - 94;
}
*/
//void pid_init(PIDController* pid, float kp, float ki, float kd, float kff) {
//    pid->kp = kp;
//    pid->ki = ki;
//    pid->kd = kd;
//	pid->kff = kff;
//	pid->ahead=0;
//    pid->error = 0.0;
//    pid->last_error = 0.0;
//    pid->integral = 0.0;
//    pid->derivative = 0.0;
//    pid->output = 0.0;
//    pid->integral_limit = 300.0;
//    pid->output_limit = 300.0;
//}
////pid计算 setpoint希望是0 feedback时摄像头输出的偏差值
//float pid_calculate(PIDController* pid) {
//    // 计算误差
//    //pid->error = setpoint - feedback;
//    
//    // 计算微分项
//    pid->derivative = pid->error - pid->last_error;
//    
//    //error平滑
////    if(pid->derivative>5.0)
////    	pid->error=pid->last_error+5.0;
////    else if(pid->derivative<-5.0)
////    	pid->error=pid->last_error-5.0;

//    // 计算积分项
//    pid->integral += pid->error;
//    pid->integral = (pid->integral > pid->integral_limit) ? pid->integral_limit : ((pid->integral < -pid->integral_limit) ? -pid->integral_limit : pid->integral);



//    // 计算输出
//    float output = pid->kp * pid->error + 
//                  pid->ki * pid->integral + 
//                  pid->kd * pid->derivative;
//    pid->output = (output > pid->output_limit) ? pid->output_limit : ((output < -pid->output_limit) ? -pid->output_limit : output);
//    // 保存上次误差
//    pid->last_error = pid->error;
//    
//    return pid->output;
//}
////内环pid计算
//float pid_speed_calculate(PIDController* pid,Motor *motor)
//{
//	// 计算误差
//    pid->error = motor->target_speed - motor->speed;
//    
//    // 计算微分项
//    pid->derivative = pid->error - pid->last_error;
//    
//    //error平滑
////    if(pid->derivative>5.0)
////    	pid->error=pid->last_error+5.0;
////    else if(pid->derivative<-5.0)
////    	pid->error=pid->last_error-5.0;

//    // 计算积分项
//    pid->integral += pid->error;
//    pid->integral = (pid->integral > pid->integral_limit) ? pid->integral_limit : ((pid->integral < -pid->integral_limit) ? -pid->integral_limit : pid->integral);



//    // 计算输出
//    float output = pid->kp * pid->error + 
//                  pid->ki * pid->integral + 
//                  pid->kd * pid->derivative;
//    pid->output = (output > pid->output_limit) ? pid->output_limit : ((output < -pid->output_limit) ? -pid->output_limit : output);
//    // 保存上次误差
//    pid->last_error = pid->error;
//    
//    return pid->output;
//}
////前馈函数
//float PID_pre_calculate(PIDController *pid)
//{
//	return pid->kff*pid->ahead;
//}
////循迹
//void run_follow(PIDController* pid,Motor *motor_left,Motor *motor_right)//串级循迹
//{
//	motor_run(&leftmotor,tgtspd+pid_speed_calculate(pid,motor_left));
//	motor_run(&rightmotor,tgtspd+pid_speed_calculate(pid,motor_right));
//}

//void run_follow_v0(PIDController* pid)//开环循迹
//{
//	motor_run(&leftmotor,tgtspd + pid_calculate(pid) - (PID_pre_calculate(pid)/2));
//	motor_run(&rightmotor,tgtspd + pid_calculate(pid) + (PID_pre_calculate(pid)/2));
//}
//////图像到误差转换
////float straight_error_get(void
//////	PIDController *PID,struct lineinfo_s lineinfo[],struct watch_o *watch,float derta
////		)
////{
////    float sum=0.0,temp=0.0; // 平均值，可后加加权
////	
////	
//////	//直道
//////	if(watch->Straight_flag==1)
//////	{
//////		uint16_t length=0;
//////	
//////    for(uint8_t i=1;i<30;i++)
//////    {
//////		if(lineinfo[i].mid!=0)
//////			length++;
//////       temp+=((float)lineinfo[i].mid);
//////    }
//////	sum+=(temp/length)*weight_dw;
//////	length=0;
//////	temp=0.0;
//////	for(uint8_t i=30;i<60;i++)
//////    {
//////	   if(lineinfo[i].mid!=0)
//////	   length++;
//////       temp+=((float)lineinfo[i].mid);
//////    }
//////	sum+=(temp/length)*weight_md;
//////	temp=0.0;
//////	length=0;
//////	for(uint8_t i=60;i<120;i++)
//////    {
//////	   if(lineinfo[i].mid!=0)
//////	   length++;
//////       temp+=((float)lineinfo[i].mid);
//////    }
//////	sum+=(temp/(float)length)*weight_up;
//////    PID->error = sum- (94.0+derta);
//////	
//////    }
////	//弯道
////	
//////	else
//////	{
////    //用原始中线计算误差
//////	int16_t top=watch->LastLine;
//////	int16_t middle=top-5;
//////	int16_t bottom=0;
//////	    if(top==0||middle==0)
//////    {
//////        PID->error=0;
//////        return;
//////    }
//////    for(int16_t i=bottom;i<middle;i++)
//////    {
//////       temp+=((float)lineinfo[i].mid);
//////    }
//////	sum+=((temp/middle)-94.0f)*weight_curve_up;
//////	temp=0.0;
//////	for(int16_t i=middle;i<=top;i++)
//////    {
//////       temp+=((float)lineinfo[i].mid);
//////    }
//////	sum+=(temp/5.0-94.0f)*weight_curve_down;
//////	sum=sum*110.0f/(float)top;
//////    PID->error = sum- derta;
//////	}
////    
////	/*
////    //用预测中线计算误差
////    int16_t top=watch->PredictTopMidline;
////	int16_t middle=2*(top/3);
////	int16_t bottom=0;
////	    if(top==0||middle==0)
////    {
////        PID->error=0;
////        return;
////    }

////    for(int16_t i=bottom;i<middle;i++)
////    {
////       temp+=((float)lineinfo[i].midpredict);
////    }
////	sum+=(temp/middle)*weight_curve_down;
////	temp=0.0;
////	for(int16_t i=middle;i<=top;i++)
////    {
////       temp+=((float)lineinfo[i].midpredict);
////    }
////	sum+=(temp/(top-middle))*weight_curve_up;
////	sum=sum*(110.0/(float)top);
////    PID->error = sum- (94.0+derta);
////	}
////	*/
////	uint8_t length=0;
////	for(uint8_t i=1;i<30;i++)
////    {
////		if(center_line[i]!=0)
////		length++;
////       temp+=center_line[i];
////		
////    }
////	sum+=(temp/length)*weight_dw;
////	length=0;
////	temp=0.0;
////	for(uint8_t i=30;i<60;i++)
////    {
////	   if(center_line[i]!=0)
////	   length++;
////       temp=center_line[i];
////    }
////	sum+=(temp/length)*weight_md;
////	temp=0.0;
////	length=0;
////	for(uint8_t i=60;i<120;i++)
////    {
////	    if(center_line[i]!=0)
////	   length++;
////       temp=center_line[i];
////    }
////	sum+=(temp/(float)length)*weight_up;
////    return sum- (94.0);
////}
