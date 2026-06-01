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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */


extern DFSDM_Filter_HandleTypeDef hdfsdm1_filter0;

extern DFSDM_Filter_HandleTypeDef hdfsdm1_filter1;

extern SPI_HandleTypeDef hspi3;

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Vin_Sens_Pin GPIO_PIN_0
#define Vin_Sens_GPIO_Port GPIOC
#define OC_Pin GPIO_PIN_4
#define OC_GPIO_Port GPIOA
#define OC_EXTI_IRQn EXTI4_IRQn
#define I_PRIM_Pin GPIO_PIN_6
#define I_PRIM_GPIO_Port GPIOA
#define DS_VCAP_Pin GPIO_PIN_5
#define DS_VCAP_GPIO_Port GPIOC
#define DS_CLKOUT_Pin GPIO_PIN_0
#define DS_CLKOUT_GPIO_Port GPIOB
#define DS_IOUT_Pin GPIO_PIN_1
#define DS_IOUT_GPIO_Port GPIOB
#define LED_R_Pin GPIO_PIN_9
#define LED_R_GPIO_Port GPIOC
#define WD_SNS_Pin GPIO_PIN_11
#define WD_SNS_GPIO_Port GPIOA
#define nWD_RST_Pin GPIO_PIN_12
#define nWD_RST_GPIO_Port GPIOA
#define LED_G_Pin GPIO_PIN_4
#define LED_G_GPIO_Port GPIOB
#define LED_B_Pin GPIO_PIN_5
#define LED_B_GPIO_Port GPIOB
#define WD_TIM_Pin GPIO_PIN_6
#define WD_TIM_GPIO_Port GPIOB
#define nCLR_OC_Pin GPIO_PIN_9
#define nCLR_OC_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
