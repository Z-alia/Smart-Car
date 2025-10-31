/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_signal_Pin GPIO_PIN_2
#define LED_signal_GPIO_Port GPIOA
#define Press_Pin GPIO_PIN_3
#define Press_GPIO_Port GPIOA
#define wifi_io1_Pin GPIO_PIN_7
#define wifi_io1_GPIO_Port GPIOE
#define wifi_io2_Pin GPIO_PIN_8
#define wifi_io2_GPIO_Port GPIOE
#define LCD_CS_Pin GPIO_PIN_11
#define LCD_CS_GPIO_Port GPIOE
#define LCD_SCL_Pin GPIO_PIN_12
#define LCD_SCL_GPIO_Port GPIOE
#define LCD_SDA_Pin GPIO_PIN_14
#define LCD_SDA_GPIO_Port GPIOE
#define LCD_DC_Pin GPIO_PIN_15
#define LCD_DC_GPIO_Port GPIOE
#define WIFI_CS_Pin GPIO_PIN_12
#define WIFI_CS_GPIO_Port GPIOB
#define PH_LMOTOR_Pin GPIO_PIN_8
#define PH_LMOTOR_GPIO_Port GPIOD
#define PH_RMOTOR_Pin GPIO_PIN_9
#define PH_RMOTOR_GPIO_Port GPIOD
#define LCD_BL_Pin GPIO_PIN_15
#define LCD_BL_GPIO_Port GPIOD
#define MOTOR_PWM_LEFT_Pin GPIO_PIN_8
#define MOTOR_PWM_LEFT_GPIO_Port GPIOA
#define MOTOR_PWM_RIGHT_Pin GPIO_PIN_9
#define MOTOR_PWM_RIGHT_GPIO_Port GPIOA
#define sccb_scl_Pin GPIO_PIN_6
#define sccb_scl_GPIO_Port GPIOD
#define sccb_sda_Pin GPIO_PIN_7
#define sccb_sda_GPIO_Port GPIOD
#define SCCB_SCL_Pin GPIO_PIN_8
#define SCCB_SCL_GPIO_Port GPIOB
#define SCCB_SDA_Pin GPIO_PIN_9
#define SCCB_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
