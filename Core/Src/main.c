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
#include "sdmmc.h"
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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
//=======================CubeMX官方注释不要删！！！！！！！=======================================
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

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
  MX_SDMMC1_SD_Init();
  MX_SPI1_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_UART4_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
  TR_driver_init(); 
	OV2640_Init();	//配置OV2640
	OV2640_DMA_Transmit_Continuous(Camera_Buffer,OV2640_BufferSize);	// 启动DMA传输，连续模式
	LCD_Init();//显示屏初始化
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  	if (DCMI_FrameState == 1)	// 采集到了一帧图像
		{
			DCMI_FrameState = 0;		// 清零标志位
			
			// 大津法全局二值化
			//Global_Binarization();
			// 自适应阈值二值化
			//Adaptive_Binarization(119, 5); 
      // Sauvola自适应二值化
      Sauvola_Binarization(119, 0.5f, 32767.0f);

      // wifi图传
      TR_Write_Image_Pixle(120, 188, (unsigned char *)Grayscale);

			/* 显示摄像头图像 */
			//显示原图像
			show_ov2640_image(0, 0, mt9v03x_image[0], Display_Width, Display_Height, Display_Width, Display_Height, 0);		
			//显示二值化扫线图
			//show_ov2640_image_int8(0, 0, imo[0], Display_Width, Display_Height, Display_Width, Display_Height);
      LCD_DisplayNumber(250, 250, OV2640_FPS, 3); // 显示当前帧率
      
			image_process();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CKPER;
  PeriphClkInitStruct.CkperClockSelection = RCC_CLKPSOURCE_HSI;
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
