/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f0xx_hal.h"

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
#define IR_Sensor_1_Pin GPIO_PIN_2
#define IR_Sensor_1_GPIO_Port GPIOC
#define IR_Sensor_1_EXTI_IRQn EXTI2_3_IRQn
#define IR_Sensor_2_Pin GPIO_PIN_3
#define IR_Sensor_2_GPIO_Port GPIOC
#define IR_Sensor_2_EXTI_IRQn EXTI2_3_IRQn
#define Blue_PushButton_Pin GPIO_PIN_0
#define Blue_PushButton_GPIO_Port GPIOA
#define LD4_Pin GPIO_PIN_8
#define LD4_GPIO_Port GPIOC
#define LD3_Pin GPIO_PIN_9
#define LD3_GPIO_Port GPIOC
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
#define BUFFER_SIZE 100
#define SENSOR_MESSAGE_SIZE 20
#define LED_BLINK_DELAY 500
#define SENSOR_STATE_CHECK 100
#define TRANSMISSION_TIMEOUT 1000
#define WALKTHROUGH_INTERVAL 2000

#define USER_DATA_START_PAGE_ADDRESS ((uint32_t)0x0800FC00)
#define FLASH_MEMORY_WORD_SIZE 4
#define atoa(x) #x
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
