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
#include "scan_line.h"
#include "Binarization.h"
#include "Element_recognition.h"
#include "system_config.h"        // 系统配置
#include "open_loop_pid.h"        // 开环PID控制
#include "integral.h"             // 积分器
#include "ICM-42688P.h"           // ICM42688陀螺仪
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// ==================== 全局结构体变量 ====================
//setpara_struct setpara;           // 参数配置
//mycar_struct mycar;               // 小车状态
//imu_struct imu;                   // IMU数据
//watch_o watch;               // 视觉监控
//vofa_struct vofa;                 // VOFA调试数据
//uint16_t dl1b_distance_mm = 9999; // 激光测距数据

// ==================== 控制相关变量 ====================
float v_forward = 100.0f;           // 前进速度(m/s)
float left_target_speed = 0;   // 左轮目标速度
float right_target_speed = 0.0f;  // 右轮目标速度

// ==================== 编码器相关变量 ====================
int32_t encoder_left_count = 0;   // 左编码器计数
int32_t encoder_right_count = 0;  // 右编码器计数
int32_t encoder_left_last = 0;    // 上次左编码器计数
int32_t encoder_right_last = 0;   // 上次右编码器计数

// ==================== 定时器标志 ====================
uint8_t control_timer_flag = 0;   // 控制定时器标志(20ms)
uint32_t system_tick = 0;         // 系统时间计数(ms)

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
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
	// ==================== 硬件初始化 ====================
	OV2640_Init();	// 初始化OV2640摄像头
	OV2640_DMA_Transmit_Continuous(Camera_Buffer,OV2640_BufferSize);	// 启动DMA传输(连续模式)
	LCD_Init();  // 显示屏初始化
	motor_init();
	// 初始化ICM42688陀螺仪
	if(ICM42688P_Init() == 0)
	{
		// 初始化成功，启动传感器
		ICM42688P_Start();
	}
	else
	{
		// 初始化失败，可以在这里添加错误处理
		// 例如LED指示、串口输出等
	}
	
	// ==================== 系统配置初始化 ====================
	system_config_init();    // 初始化系统参数(setpara, mycar, imu等)
	element_init();          // 初始化元素识别系统
	
	// ==================== 开环PID控制器初始化 ====================
	open_loop_pid_init(
		1.0f,    // kp: 比例增益
		0.1f,    // ki: 积分增益
		0.05f,   // kd: 微分增益
		50.0f,   // integral_max: 积分限幅
		2.0f     // output_max: 输出限幅(差速,m/s)
	);
	
	// ==================== 定时器启动 ====================
	/* 
	 * 定时器配置说明:
	 * TIM2: 左编码器(Encoder模式) - Period=4294967295 (32位计数器)
	 * TIM3: 右编码器(Encoder模式) - Period=65535 (16位计数器)
	 * TIM6: 控制周期定时器(20ms) - 用于速度计算和PID控制
	 * TIM7: 系统滴答定时器(1ms) - 用于RUNTIME计数
	 */
	
	// 启动编码器定时器
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
	
	// 启动控制周期定时器(20ms) - 注意:不要修改定时器配置,只启动中断
	// TIM6配置: Prescaler和Period需要在CubeMX中配置为20ms中断
	HAL_TIM_Base_Start_IT(&htim15);
	HAL_TIM_Base_Start_IT(&htim16);
	HAL_TIM_Base_Start_IT(&htim17);  // 启动TIM6中断(20ms周期)
	
	// 启动系统滴答定时器(1ms) - 注意:不要修改定时器配置
	// TIM7配置: Prescaler和Period需要在CubeMX中配置为1ms中断
	//HAL_TIM_Base_Start_IT(&htim7);  // 启动TIM7中断(1ms系统时钟)
	
	/* 
	 * 定时器配置警告:
	 * 如果定时器没有配置正确,请在CubeMX中检查:
	 * TIM6: 计算公式 Period = (APB1_Timer_Clock / (Prescaler+1) / 50Hz) - 1
	 * TIM7: 计算公式 Period = (APB1_Timer_Clock / (Prescaler+1) / 1000Hz) - 1
	 * APB1_Timer_Clock通常为240MHz
	 */
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		LCD_DisplayDecimals(220, 190, g_open_loop_pid_state.left_target, 3,5); 
		LCD_DisplayDecimals(220, 170, g_open_loop_pid_state.left_target, 3,5); 
		LCD_DisplayDecimals(220,150,g_open_loop_pid_state.error,3,1);
	  	if (DCMI_FrameState == 1)	// 采集到新一帧图像
		{
			DCMI_FrameState = 0;		// 清除标志位
			
			/* ==================== 图像处理流程 ==================== */
			
			/* OTSU算法计算阈值 */
			watch.threshold = img_otsu((uint16_t *)mt9v03x_image[30], 60, Display_Width, 10); 
			
			/* 阈值上下限幅 */
			if(watch.threshold > 120)
			{
				watch.threshold = 120;
			}
			else if(watch.threshold < 80)
			{
				watch.threshold = 80;
			}
			
			/* 二值化处理 */
			Binarization();    // mt9v03x_image → Grayscale
			
			/* 扫线算法提取边线 */
			scan_line();       // Grayscale → lineinfo[]
			
			/* ==================== 元素识别与状态机 ==================== */
			Element_recognition();  // 状态机处理各种赛道元素
			
			/* ==================== 开环PID控制 ==================== */
			// 调用开环PID控制器计算左右轮目标速度
			open_loop_pid_calculate(v_forward, &left_target_speed, &right_target_speed);
			
			/* TODO: 输出到电机驱动 */
			motor_run(&leftmotor, left_target_speed,0);
			motor_run(&rightmotor, right_target_speed,0);
			
			/* ==================== 图像显示 ==================== */
			
			/* 在图像上绘制出扫线结果 */
			draw_edge();
			
			/* 显示摄像头图像 */
			// 显示原图像:
			//show_ov2640_image(0, 0, mt9v03x_image[0], Display_Width, Display_Height, Display_Width, Display_Height, 0);		
			
			// 显示二值化+扫线图:
			show_ov2640_image_int8(0, 0, imo[0], Display_Width, Display_Height, Display_Width, Display_Height);			
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
// ==================== 定时器中断回调函数 ====================

/**
 * @brief 定时器中断回调函数
 * @param htim 定时器句柄
 * @note TIM6: 20ms控制周期(50Hz) - 速度计算、PID控制、积分器更新
 *       TIM7: 1ms系统滴答(1000Hz) - 系统时间计数
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// ==================== TIM6: 20ms控制周期 ====================
	if (htim->Instance == TIM17)
	{
		control_timer_flag = 1;  // 设置控制标志
		
		// 读取编码器计数值
		encoder_left_count = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);   // TIM2左编码器
		encoder_right_count = (int32_t)__HAL_TIM_GET_COUNTER(&htim3);  // TIM3右编码器
		
		// 计算编码器增量
		int32_t delta_left = encoder_left_count - encoder_left_last;
		int32_t delta_right = encoder_right_count - encoder_right_last;
		
		// 保存本次计数值供下次使用
		encoder_left_last = encoder_left_count;
		encoder_right_last = encoder_right_count;
		
		// 计算速度 (优化版)
		// 编码器参数:
		//   - 256线四倍频: 256 × 4 = 1024脉冲/编码器转
		//   - 编码器转4圈 = 轮子转1圈: 1024 × 4 = 4096脉冲/轮转
		//   - 轮子半径: R = 4cm
		//   - 轮子周长: C = 2πR = 2 × 3.14159 × 4 ≈ 25.13cm
		//   - 控制周期: Δt = 20ms = 0.02s
		// 速度计算: v(cm/s) = (delta_count / 4096) × 25.13 / 0.02
		//                  = delta_count × (25.13 / 4096 / 0.02)
		//                  = delta_count × 0.30664
		const float PULSES_PER_WHEEL_REV = 4096.0f;      // 脉冲数/轮转
		const float WHEEL_CIRCUMFERENCE = 25.13274f;     // 轮子周长(cm): 2π×4
		const float CONTROL_PERIOD = 0.02f;              // 控制周期(s): 20ms
		const float SPEED_COEFF = WHEEL_CIRCUMFERENCE / PULSES_PER_WHEEL_REV / CONTROL_PERIOD;  // ≈0.30664
		
		float left_speed = (float)delta_left * SPEED_COEFF;   // cm/s
		float right_speed = (float)delta_right * SPEED_COEFF; // cm/s
		float average_speed = (left_speed + right_speed) / 2.0f; // cm/s
		
		// 更新小车状态
		mycar.present_speed = average_speed;
		
		// 路径积分更新(用于元素识别)
		distant_integeral(average_speed);
		
		// 读取IMU数据并更新陀螺仪数据
		ICM42688P_ReadIMUData(&imu_data);
		
		// 角度积分更新(使用实际陀螺仪Z轴数据)
		angal_integeral(imu_data.gyro_z);
	}
	
	// ==================== TIM7: 1ms系统滴答 ====================
//	else if (htim->Instance == TIM7)
//	{
//		system_tick++;
//		mycar.RUNTIME = system_tick;  // 更新系统运行时间(ms)
//	}
}

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
