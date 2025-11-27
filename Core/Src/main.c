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
#include "quadspi.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd_spi_200.h"
#include "dcmi_ov2640.h"
#include "Binarization.h"
#include "Element_recognition.h"
#include "image.h"
#include "LQ_Transfer_Image.h"
#include "integral.h"
#include "control.h"
#include "ICM-42688P.h"
#include "motor.h"
#include "control_pid.h"

//#include ""
//#include "ICM42688P_Simple.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
PIDController PID_image;
PIDController PID_speed;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t receive_flag,v=5;
float speed =5.5f; //speed 7 p 7m/s

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//extern IMU_Data imu_data;  // 声明外部 IMU 数据结构
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void MPU_Config(void);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	MPU_Config();
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

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
  MX_SPI2_Init();
  MX_QUADSPI_Init();
  MX_SPI1_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_UART4_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  /*----------------------------外设初始化--------------------------------------*/
	LCD_Init();
	OV2640_Init();
	OV2640_DMA_Transmit_Continuous(Camera_Buffer, OV2640_BufferSize);
	ICM42688P_Init();
	motor_init();
	TR_driver_init();
    //TOF_UART_Driver_Init();
//	TOF_Init();
//	TOF_SetOutputMode(1);
//	TOF_SetTriggerMode(0);
  /*----------------------------控制初始化--------------------------------------*/
	cascade_pid_init(0.008f,
                                  0.08f, 0.0f, 0.0f,   // 图像PID参数
                                 75.0f, 0.0f, 0.0f); // 右轮PID参数 // PID参数可根据需要调整
//	pid_init(&PID_image,1.0,0,0,0);
//	pid_init(&PID_speed,1.0,0,0,0);

	// 启动编码器模式（不使用中断，仅计数）
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);

    // 启动 TIM15 中断
    HAL_TIM_Base_Start_IT(&htim15);
	HAL_TIM_Base_Start_IT(&htim16);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  /*-----------------------------状态机-----------------------------------------*/
	  if(watch.InLoop==2)
	  {
//		  speed =2.0f;
//		  v=5;
//		  cascade_pid_init(0.008f,
//                                  0.25f, 0.0f, 0.0f,   // 图像PID参数
//                                 20.0f, 0.0f, 0.0f);
		  distance_integral.integeral_flag=1;
		  if(distance_integral.integeral_data>300)//150  
		  {
			  watch.InLoop=10;
			  clear_distant_integeral();
			  //distance_integral.integeral_flag=0;
		  }
	}
	  if(watch.InLoop==10)
	  {
		 distance_integral.integeral_flag=1;
		  if(distance_integral.integeral_data>400)//150  
		  {
			  watch.InLoop=4;
			  clear_distant_integeral();
			  //distance_integral.integeral_flag=0;
		  } 
	  }
	  if(watch.InLoop==4)
	  {
		 distance_integral.integeral_flag=1;
		  if(distance_integral.integeral_data>300)//150  
		  {
			  watch.InLoop=11;
//			  speed=7.0f;
//			  v=5;
//			  cascade_pid_init(0.08f,
//                                  0.75f, 0.0f, 0.0f,   // 图像PID参数
//                                 50.0f, 0.0f, 0.0f);
			  clear_distant_integeral();
			  //distance_integral.integeral_flag=0;
		  } 
	  }
//	  if(watch.InLoop==5)
//	  {
//		 distance_integral.integeral_flag=1;
//		  if(distance_integral.integeral_data>100)//150  
//		  {
//			  watch.InLoop=13;
//			  clear_distant_integeral();
//			  //distance_integral.integeral_flag=0;
//		  } 
//	  }
		/*---------------------------以下为图像区域--------------------------------*/
	if (DCMI_FrameState == 1)	// 采集到了一帧图像
		{
			DCMI_FrameState = 0;		// 清零标志位
			
			// 大津法全局二值化
			Global_Binarization();
			// 自适应阈值二值化
			//Adaptive_Binarization(119, 5); 
			// Sauvola自适应二值化
			//Sauvola_Binarization(99, 0.5f, 32767.0f);

			// 显示摄像头图像
			//显示原图像
			show_ov2640_image_from_ptr_array(0, 120, mt9v03x_image, Display_Width, Display_Height, Display_Width, Display_Height, 0);
			//显示二值化扫线图
			show_ov2640_image_int8(0, 0, imo[0], Display_Width, Display_Height, Display_Width, Display_Height);
			
			
			image_process();
			
			// wifi图传
			//TR_Write_Image(120, 188, (unsigned char *)imo);
			
		}
		/*
		LCD_DisplayDecimals(220, 190, control.left_speed, 3,5); 
		LCD_DisplayDecimals(220, 170, control.right_speed, 3,5); 
		LCD_DisplayDecimals(220,150,straight_error_get(),3,1);
		LCD_DisplayDecimals(220,130,receive_flag,2,0);
		
		//LCD_DisplayDecimals(200,20,watch.InLoop,1,0);
		LCD_DisplayDecimals(200,40,watch.InLoopAngleL,1,0);
		LCD_DisplayDecimals(200,60,watch.InLoopCirc,1,0);
		LCD_DisplayDecimals(200,80,watch.InLoopAngle2_x,1,0);
		LCD_DisplayDecimals(200,100,watch.InLoopAngle2_y,1,0);
		*/
        LCD_DisplayDecimals(200, 220, watch.zebra_flag, 1, 0);
    
    
    
    
		
		
//		//日志回传
//		TR_Log_AddByte(watch.InLoop);
//		TR_Log_AddByte(watch.InLoopAngle2);
//		TR_Log_AddByte(watch.InLoopAngle2_x);
//		TR_Log_AddByte(watch.InLoopAngle2_y);
//		TR_Log_AddByte(watch.InLoopAngleL);
//		TR_Log_AddByte(watch.InLoopCirc);
//		TR_Send_Log();
		TR_Log_Clear();
	/*----------------------------图像测试-----------------------------*/
	if(watch.InLoop!=5&&watch.InLoop!=11)
	{
		left_ring_first_angle();
		left_ring_circular_arc();
		left_ring_second_angle();
		left_ring_begin_turn();
		left_ring_prepare_out();
		left_ring_out_angle();
		left_ring_out_loop_turn();
//		left_ring_out_loop();
//		left_ring_straight_out_angle();
		//left_ring_complete_out();
	}
		left_ring_linefix();
//	if(imu_data.gyro_y>5)
//	{
//		speed=25.0f;
//		distant_integeral(50);
//	}
//	if(imu_data.gyro_y<-10)
//	{
//		speed=3.0f;
//		clear_angle_integeral();
//	}
	
    /*---------------------------以下为控制区域--------------------------------*/
		
//		//速度决策
//	if(control.error>15.0f||control.error<-15.0f)
//	{
//		speed=5.0f;
//		cascade_pid_init(0.08f,
//                                  0.5f, 0.0f, 0.0f,   // 图像PID参数
//                                 50.0f, 0.0f, 0.0f);
//	}
		//电机控制调用
//		motor_run(&leftmotor,control.left_target_speed);
//		motor_run(&rightmotor,control.right_target_speed);
//	/*---------------imutest-----------------*/
//	  //ICM42688P_ReadIMUData(&imu_data);
//	  LCD_DisplayDecimals(50,200,imu_data.gyro_z,3,2);
//	  LCD_DisplayDecimals(50,175,imu_data.accel_z,3,2);
//	  LCD_DisplayDecimals(50,150,imu_data.temperature,3,1);
//	//3.编码器
//		LCD_DisplayDecimals(200, 150, get_speed(), 3,1);
//		LCD_DisplayDecimals(250, 175, control.lencoder_count, 5,0);
//		LCD_DisplayDecimals(250, 200, control.rencoder_count, 5,0);
//		LCD_DisplayDecimals(150, 175, control.left_speed, 3,1);
//		LCD_DisplayDecimals(150, 200, control.right_speed, 3,1);
		
//	//4.电机
//		motor_run(&leftmotor,100);
//		motor_run(&rightmotor,100);
//	//5.wifi
//		TR_Write_Image_Pixle(120, 188, (unsigned char *)Grayscale);
    //tof test
		//LCD_DisplayDecimals(200, 200, watch.Red_obstacle_flag, 10,0);

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
  RCC_OscInitStruct.PLL.PLLQ = 20;
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

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI2|RCC_PERIPHCLK_SPI1;
  PeriphClkInitStruct.PLL2.PLL2M = 15;
  PeriphClkInitStruct.PLL2.PLL2N = 144;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_0;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


//	配置MPU
//
void MPU_Config(void)
{
	MPU_Region_InitTypeDef MPU_InitStruct;

	HAL_MPU_Disable();		// 先禁止MPU

	MPU_InitStruct.Enable 				= MPU_REGION_ENABLE;
	MPU_InitStruct.BaseAddress 		= 0x24000000;
	MPU_InitStruct.Size 					= MPU_REGION_SIZE_512KB;
	MPU_InitStruct.AccessPermission 	= MPU_REGION_FULL_ACCESS;
	MPU_InitStruct.IsBufferable 		= MPU_ACCESS_BUFFERABLE;
	MPU_InitStruct.IsCacheable 		= MPU_ACCESS_CACHEABLE;
	MPU_InitStruct.IsShareable 		= MPU_ACCESS_SHAREABLE;
	MPU_InitStruct.Number 				= MPU_REGION_NUMBER0;
	MPU_InitStruct.TypeExtField 		= MPU_TEX_LEVEL0;
	MPU_InitStruct.SubRegionDisable 	= 0x00;
	MPU_InitStruct.DisableExec 		= MPU_INSTRUCTION_ACCESS_ENABLE;

	HAL_MPU_ConfigRegion(&MPU_InitStruct);	

	HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);	// 使能MPU
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // 检查是否是TIM15的更新事件中断
  if (htim->Instance == TIM15)
  {
	//100ms
	  
	//TR_Receive_Packet(&receive_flag,16,1);
    //进行陀螺仪数据收集和积分
    ICM42688P_ReadIMUData(&imu_data);
    angal_integeral(imu_data.gyro_z);
	//watch.Red_obstacle_flag=TOF_ReadDistanceFiltered();
	 //图像环计算
	 cascade_pid_outer_loop(speed); //目标速度设定
	  
//	 //test
//	  g_cascade_pid.vL_target=7.0f;
//	  g_cascade_pid.vR_target=7.0f;
	  
//	 TR_Log_AddFloat(control.left_target_speed);
//	 TR_Log_AddFloat(control.right_target_speed);
	  
	  
  }
  if (htim->Instance == TIM16)
  {
	  //速度决策
	if(control.error>10.0f||control.error<-10.0f)
	{
		speed=5.0f;
		cascade_pid_set_image_params(0.5f, 0.0f, 0.0f);
		cascade_pid_set_speed_params(50.0f, 0.0f, 0.0f);
		
//		cascade_pid_init(0.08f,
//                                  0.5f, 0.0f, 0.0f,   // 图像PID参数
//                                 50.0f, 0.0f, 0.0f);
	}
	else
	{
		speed=5.5f;
		cascade_pid_set_image_params(0.08f, 0.0f, 0.0f);
		cascade_pid_set_speed_params(75.0f, 0.0f, 0.0f);
	}
	  //10ms
	  //进行编码器积分
  control.lencoder_count = (int32_t)__HAL_TIM_GET_COUNTER(&htim5);//左编码器计数
  control.rencoder_count = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);//右编码器计数
    distant_integeral(get_speed());
	  
	 //日志
	TR_Log_AddFloat(control.left_speed);
	TR_Log_AddFloat(control.right_speed);
	  
	  
    control.lencoder_count_last = control.lencoder_count;
    control.rencoder_count_last = control.rencoder_count;
	  //速度环计算
	  float pwm_L, pwm_R;
    //cascade_pid_control(speed,control.left_speed,control.right_speed,&pwm_L, &pwm_R);
//    // 使用上次图像环计算的速度目标（只执行速度环PID）
    cascade_pid_inner_loop(control.left_speed,
                          control.right_speed,
                          &pwm_L, &pwm_R);
	  //日志

	TR_Log_AddFloat(g_cascade_pid.vL_target);
	TR_Log_AddFloat(g_cascade_pid.vR_target);
	
	TR_Log_AddUint8(straight);
    // 输出到电机
    control.left_target_speed = (int16_t)(pwm_L);
    
    control.right_target_speed = (int16_t)(pwm_R);
    
	motor_run(&leftmotor,control.left_target_speed,v);
	motor_run(&rightmotor,control.right_target_speed,v);
		
	TR_Send_Log();
	TR_Log_Clear();
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
#ifdef USE_FULL_ASSERT
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
