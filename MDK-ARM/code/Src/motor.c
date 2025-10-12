#include "motor.h"
#include "main.h"
#include "tim.h"
#include "scan_line.h"
#include "Binarization.h"
#include "Element_recognition.h"
//本工程的PWM分辨率为1000
#define tgtspd 230 //700改70
#define tgtspd_curve 225
#define TL_tgtspd 250
#define TR_tgtspd 250
#define weight_up 0.15//下部为0-30 中部为30-60 上部为60-120
#define weight_md 0.45
#define weight_dw 0.40
#define weight_curve_up 0.75
#define weight_curve_down 0.25

PIDController PID;
PIDController PID_curve;
Motor leftmotor={0, 1, 0, 1};
Motor rightmotor={0, 0, 0, 0};

//pid控制器初始化
void pid_init(PIDController* pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->error = 0.0;
    pid->last_error = 0.0;
    pid->integral = 0.0;
    pid->derivative = 0.0;
    pid->output = 0.0;
    pid->integral_limit = 300.0;
    pid->output_limit = 300.0;
}

//pid计算 setpoint希望是0 feedback时摄像头输出的偏差值
float pid_calculate(PIDController* pid) {
    // 计算误差
    //pid->error = setpoint - feedback;
    
    // 计算积分项
    pid->integral += pid->error;
    pid->integral = (pid->integral > pid->integral_limit) ? pid->integral_limit : ((pid->integral < -pid->integral_limit) ? -pid->integral_limit : pid->integral);
    // 计算微分项
    pid->derivative = pid->error - pid->last_error;
    
    // 计算输出
    float output = pid->kp * pid->error + 
                  pid->ki * pid->integral + 
                  pid->kd * pid->derivative;
    pid->output = (output > pid->output_limit) ? pid->output_limit : ((output < -pid->output_limit) ? -pid->output_limit : output);
    // 保存上次误差
    pid->last_error = pid->error;
    
    return pid->output;
}

// 初始化电机驱动
void motor_init(void)
{
    // 初始化电机控制引脚
    HAL_GPIO_WritePin(GPIOB, MOTOR_LEFT_DIRE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, MOTOR_RIGHT_DIRE_Pin, GPIO_PIN_SET);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    motor_stop();
}

// 控制单个电机运行 (speed: -1000 to 1000)
// 通道：ch1左 ch2右
void motor_run(Motor *motor_ptr, int32_t speed)
{
    if ((speed<-1000)||(speed>1000))
        return;
    motor_ptr->speed = speed;
    if(motor_ptr->lor == 0) // 右电机
    {
        if(speed >= 0)
        {
            motor_ptr->dir = 1; // 正转
            HAL_GPIO_WritePin(GPIOB, MOTOR_RIGHT_DIRE_Pin, GPIO_PIN_SET);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, speed);
        }
        else if(speed < 0)
        {
            motor_ptr->dir = 0; // 反转
            HAL_GPIO_WritePin(GPIOB, MOTOR_RIGHT_DIRE_Pin, GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, -speed);
        }
    }
    else if(motor_ptr->lor == 1) // 左电机
    {
        if(speed >= 0)
        {
            motor_ptr->dir = 1; // 正转
            HAL_GPIO_WritePin(GPIOB, MOTOR_LEFT_DIRE_Pin, GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
        }
        else if(speed < 0)
        {
            motor_ptr->dir = 0; // 反转
            HAL_GPIO_WritePin(GPIOB, MOTOR_LEFT_DIRE_Pin, GPIO_PIN_SET);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, -speed);
        }
    }
}

void run_follow(PIDController* pid)
{
	motor_run(&leftmotor,tgtspd+pid_calculate(pid));
	motor_run(&rightmotor,tgtspd-pid_calculate(pid));
}

//左转
void motor_turnleft(void)
{
    motor_run(&leftmotor, tgtspd);
    motor_run(&rightmotor, TR_tgtspd);
}

// 右转
void motor_turnright(void)
{
    motor_run(&leftmotor, TL_tgtspd);
    motor_run(&rightmotor, tgtspd);
}

//滑行
void motor_coast(void)
{
    HAL_GPIO_WritePin(GPIOB, MOTOR_LEFT_DIRE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, MOTOR_RIGHT_DIRE_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
}

// 刹车
void motor_stop(void)
{
    motor_run(&leftmotor, 0);
    motor_run(&rightmotor, 0);
}
//图像到误差转换
void straight_error_get(PIDController *PID,struct lineinfo_s lineinfo[],struct watch_o *watch,float derta)
{
    float sum=0.0,temp=0.0; // 平均值，可后加加权
	
	
	//直道
	if(watch->Straight_flag==1)
	{
	uint16_t length=0;
    for(uint8_t i=1;i<30;i++)
    {
		if(lineinfo[i].mid!=0)
			length++;
       temp+=((uint16_t)lineinfo[i].mid);
    }
	sum+=(temp/length)*weight_dw;
	length=0;
	temp=0.0;
	for(uint8_t i=30;i<60;i++)
    {
	   if(lineinfo[i].mid!=0)
	   length++;
       temp+=((uint16_t)lineinfo[i].mid);
    }
	sum+=(temp/length)*weight_md;
	temp=0.0;
	length=0;
	for(uint8_t i=60;i<120;i++)
    {
	   if(lineinfo[i].mid!=0)
	   length++;
       temp+=((uint16_t)lineinfo[i].mid);
    }
	sum+=(temp/length)*weight_up;
    PID->error = sum- (94.0+derta);
    }
	//弯道
	
	else
	{
	int16_t top=watch->LastLine;
	int16_t middle=watch->LastLine/2;
	int16_t bottom=0;
	    if(middle==0)
    {
        PID->error=0;
        return;
    }
    for(int16_t i=bottom;i<middle;i++)
    {
       temp+=((uint16_t)lineinfo[i].mid);
    }
	sum+=(temp/middle)*weight_curve_up;
	temp=0.0;
	for(int16_t i=middle;i<=top;i++)
    {
       temp+=((uint16_t)lineinfo[i].mid);
    }
	sum+=(temp/middle)*weight_curve_down;
	sum=sum*110/watch->LastLine;
    PID->error = sum- (94.0+derta);
	}
}

void motor_follow_line_curve(PIDController* pid)
{
	motor_stop();
	motor_run(&leftmotor,tgtspd_curve+pid_calculate(pid));
	motor_run(&rightmotor,tgtspd_curve-pid_calculate(pid));
}