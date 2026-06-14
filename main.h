/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
#define NTC_Pin GPIO_PIN_0
#define NTC_GPIO_Port GPIOA
#define DC_Volt_Pin GPIO_PIN_1
#define DC_Volt_GPIO_Port GPIOA
#define U_Current_Pin GPIO_PIN_2
#define U_Current_GPIO_Port GPIOA
#define V_Current_Pin GPIO_PIN_3
#define V_Current_GPIO_Port GPIOA
#define W_Current_Pin GPIO_PIN_4
#define W_Current_GPIO_Port GPIOA
#define U_Voltage_Pin GPIO_PIN_5
#define U_Voltage_GPIO_Port GPIOA
#define V_Voltage_Pin GPIO_PIN_6
#define V_Voltage_GPIO_Port GPIOA
#define W_Voltage_Pin GPIO_PIN_7
#define W_Voltage_GPIO_Port GPIOA
#define DC_Current_Pin GPIO_PIN_0
#define DC_Current_GPIO_Port GPIOB
#define FAN_12V_Pin GPIO_PIN_1
#define FAN_12V_GPIO_Port GPIOB
#define SOFT_START_Relay_Pin GPIO_PIN_2
#define SOFT_START_Relay_GPIO_Port GPIOB
#define STOP_Pin GPIO_PIN_11
#define STOP_GPIO_Port GPIOA
#define STOP_EXTI_IRQn EXTI15_10_IRQn
#define ERORR_Pin GPIO_PIN_12
#define ERORR_GPIO_Port GPIOA
#define START_Pin GPIO_PIN_8
#define START_GPIO_Port GPIOB
#define START_EXTI_IRQn EXTI9_5_IRQn
#define CW_CCW_Pin GPIO_PIN_9
#define CW_CCW_GPIO_Port GPIOB
#define CW_CCW_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
