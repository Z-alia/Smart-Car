/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dcmi.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd_spi_200.h"
#include "dcmi_ov2640.h"
#include "Binarization.h"
#include "morph_binary_bitpacked.h"
#include "element_recognition.h"
#include "scan_line.h"
#include "encoder.h"
#include "ec11.h"
#include "motor.h"
#include "mpu6050.h"
#include "line_straight_check.h"
#include "Kalman.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
MotorSpeed motor_speed;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void take_image(struct watch_o *watch,PIDController* pid);
uint8_t flag=0,oldflag[5]={0};
uint8_t pre_flag=0;
float mpu=0.0f,p=2.0f,i=0.0f,d=0.0f;
uint32_t smd=0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  LCD_SetAsciiFont(&ASCII_Font20);      // 设置字体
  LCD_ShowNumMode(Fill_Space);           // 设置多余位补0（可选，Fill_Space 为补空格）

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI4_Init();
  MX_DCMI_Init();
  MX_TIM6_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM7_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
	LCD_Init();
	OV2640_Init();	
	OV2640_DMA_Transmit_Continuous(Camera_Buffer,OV2640_BufferSize);	
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
	HAL_TIM_Base_Start_IT(&htim6);
	pid_init(&PID,1.5,0.0,0);//直线pid
	pid_init(&PID_curve,p,i,d);//弯道pid
	motor_init();
	Clear_Recognition_Flag(&watch);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET);
	mpu6050_init();
	//调参阶段while
	
	while(1)
	{
		LCD_DisplayDecimals( 175, 300, p, 5,1);
		LCD_DisplayDecimals( 175, 320, i, 5,1);
		LCD_DisplayDecimals( 175, 340, d, 5,1);
		
		LCD_DisplayNumber( 0, 20, oldflag[4], 5);
		LCD_DisplayNumber( 0, 40, oldflag[3], 5);
		LCD_DisplayNumber( 0, 60, oldflag[2], 5);
		LCD_DisplayNumber( 0, 80, oldflag[1], 5);
		
		
		if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4)==GPIO_PIN_RESET)
			break;
		
	}
	HAL_TIM_Base_Start_IT(&htim7);
	//HAL_Delay(5000);
	HAL_TIM_Base_Start_IT(&htim7);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //pid_calculate(&PID);
	  	if (DCMI_FrameState == 1)	// 采集到了一帧图像
		{
			DCMI_FrameState = 0;		// 清零标志位
			
			/* 大津法计算二值化阈值 */
			watch.threshold = img_otsu((uint16_t *)mt9v03x_image[30], 60, Display_Width, 10); 
			
			/* 二值化阈值限幅 */
			if(watch.threshold>180)
			{
				watch.threshold=180;
			}
			else if(watch.threshold<150)//将80改为130
			{
				watch.threshold=150;
			}
			
			/* 二值化 */
			Binarization();
			
      //洗图、提取边缘
      morph_clean_u8_binary_adapter(Grayscale[0], Display_Width, Display_Height, imo[0]);

			/* 扫描赛道边线 */
			scan_line();
			
			/* 卡尔曼滤波*/
      stable_curve_params = ProcessLineWithKalman(lineinfo, watch.LastLine);
			PopulatePredictedLine(&stable_curve_params, lineinfo, Display_Width, Display_Height);

			/* 在图像上绘制出赛道边线 */
			draw_edge();
			
			/* 显示摄像头图像 */
			//显示原图像
			//show_ov2640_image(0, 0, mt9v03x_image[0], Display_Width, Display_Height, Display_Width, Display_Height, 0);		
			//显示二值化扫线图;
			show_ov2640_image_int8(0, 0, imo[0], Display_Width, Display_Height, Display_Width, Display_Height);		
            //图像到误差转换
            if(watch.Straight_flag==1&&watch.Crossroads_flag_left==0&&watch.Crossroads_flag_right==0)
		    {straight_error_get(&PID,lineinfo,&watch,0);
			PID_curve.output=PID.output;
			}
	        else if(watch.Curve_flag==1/*&&watch.Crossroads_flag_left==0&&watch.Crossroads_flag_right==0*/)
		    {straight_error_get(&PID_curve,lineinfo,&watch,0);
             PID.output=PID_curve.output;	
			}				
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    LCD_DisplayNumber( 250, 250, watch.Curve_flag, 5);
	LCD_DisplayNumber( 250, 270, watch.Curve_left_flag, 5);
	LCD_DisplayNumber( 250, 290, watch.Curve_right_flag, 5);
	LCD_DisplayNumber( 250, 310, watch.Straight_flag, 5);
	//LCD_DisplayNumber( 250, 330, (int16_t)PID.error, 5);
	LCD_DisplayNumber( 250, 350, watch.threshold, 5);
	
	LCD_DisplayNumber( 0, 430, watch.Left_Break_flag, 5);
	LCD_DisplayNumber( 0, 450, watch.Right_Break_flag, 5);
		
	LCD_DisplayNumber( 250, 370, watch.Crossroads_flag_left, 5);
	LCD_DisplayNumber( 250, 390, watch.Cross_flag, 5);
		
	LCD_DisplayNumber( 100, 400, leftmotor.speed, 5);	
	LCD_DisplayNumber( 100, 420, rightmotor.speed, 5);	
	LCD_DisplayNumber( 100, 440, (int16_t)PID_curve.error, 5);
	LCD_DisplayNumber( 100, 460, (int16_t)PID.error, 5);
		
	LCD_DisplayNumber( 175, 420, watch.PredictTopMidline, 5);	
	LCD_DisplayNumber( 175, 440, watch.LastLine, 5);	
    LCD_DisplayNumber( 175, 460, lineinfo[watch.PredictTopMidline-10].midpredict, 5);
	LCD_DisplayDecimals( 175, 480, p, 5,1);
	
	
	LCD_DisplayDecimals( 250, 410, stable_curve_params.a, 5,3);
		
	//LCD_DisplayNumber( 250, 410, mpu/500, 5);
	
	
	//straight_error_get(&PID_curve,lineinfo,&watch,0);
	//motor_follow_line_curve(&PID_curve);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
 if (htim->Instance == TIM6)
    {
        //每 10ms 执行

        // 1. 读取编码器当前计数值
        //motor_speed.encoder_count_left = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
        //motor_speed.encoder_count_right = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
        // 2. 计算速度 修正溢出 更新上一次的计数值
        //Encoder_Correct(&motor_speed);
        // 3. 调用电机PID控制函数
		
//		if(watch.Straight_flag==1&&watch.Crossroads_flag_left==0&&watch.Crossroads_flag_right==0)
//		straight_error_get(&PID,lineinfo,&watch,0);
//	    else if(watch.Curve_flag==1/*&&watch.Crossroads_flag_left==0&&watch.Crossroads_flag_right==0*/)
//		straight_error_get(&PID_curve,lineinfo,&watch,0);
//		else if(watch.Crossroads_flag_left==1)
//			straight_error_get(&PID_curve,lineinfo,&watch,10);
//		else if(watch.Crossroads_flag_left==2&&(mpu/500.0)<=3)
//			straight_error_get(&PID_curve,lineinfo,&watch,15);
//		else if(watch.Crossroads_flag_left==2&&(mpu/500.0)>3)
//		{
//			watch.Crossroads_flag_left=0;
//			watch.Crossroads_flag_right=0;
//			mpu=0;
//		}
		
		
		//4.赛道识别
		Island_loop_and_curve_recognition(&watch,lineinfo);
		
		
		//Cross_recognition(&watch,lineinfo);
		
		
		
    }
if (htim->Instance == TIM7)
{
	
	if(watch.Straight_flag==1)
		run_follow(&PID);//电机注释，调试图像
	else if(watch.Curve_flag==1)
		motor_follow_line_curve(&PID_curve);
	else if(watch.Crossroads_flag_left==1)
		motor_follow_line_curve(&PID_curve);
	
	Straight_recognition(&watch,lineinfo);	
	//motor_run(&rightmotor,100);//电机测试
	
	//take_image(&watch,&PID_curve);
	mpu6050_get_gyro();
		mpu+=mpu6050_gyro_transition(mpu6050_gyro_z);
}

}

void take_image(struct watch_o *watch,PIDController* pid)
{
	if(watch->Curve_flag==1&&pid->error>=15)
	{
		
		if(watch->Curve_left_flag==1)
			motor_turnright();
		else if(watch->Curve_right_flag==1)
			motor_turnleft();
	 
	}
}
void ssxxzzyyybaba_cw()
{
//4 3 2 1 0
			oldflag[4]=oldflag[3];
			oldflag[3]=oldflag[2];
			oldflag[2]=oldflag[1];
			oldflag[1]=1;
			if(oldflag[4]==0&&oldflag[3]==1&&oldflag[2]==0&&oldflag[1]==1)
			{
				flag=(flag+1)%3;
				memset(oldflag,0,sizeof(oldflag));
			}
}
void ssxxzzyyybaba_ccw()
{
//4 3 2 1 0
			oldflag[4]=oldflag[3];
			oldflag[3]=oldflag[2];
			oldflag[2]=oldflag[1];
			oldflag[1]=0;
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	uint32_t smd1=HAL_GetTick();
	switch(flag)
	{
		case 0:
	{
	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0)==1)
	{
		if(smd1-smd>=600)
		{
			//4是时间最远的一次操作
			//1正转0反
			smd=HAL_GetTick();
			p+=0.1f;
			ssxxzzyyybaba_cw();
		}
	}
	else
		if(smd1-smd>=600)
		{
			smd=HAL_GetTick();
			p-=0.1f;
			ssxxzzyyybaba_ccw();
		}
		break;
	}
		
		case 1:
	{
	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0)==1)
	{
		if(smd1-smd>=600)
		{
			smd=HAL_GetTick();
			i+=0.1f;
			ssxxzzyyybaba_cw();
		}
	}
	else
		if(smd1-smd>=600)
		{
			smd=HAL_GetTick();
			i-=0.1f;
			ssxxzzyyybaba_ccw();
		}
		break;
	}
	case 2:
	{
	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0)==1)
	{
		if(smd1-smd>=600)
		{
			smd=HAL_GetTick();
			d+=0.1f;
			ssxxzzyyybaba_cw();
		}
	}
	else
		if(smd1-smd>=600)
		{
			smd=HAL_GetTick();
			d-=0.1f;
			ssxxzzyyybaba_ccw();
		}
		break;
	}
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
