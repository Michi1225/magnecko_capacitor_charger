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
#include <string.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;

extern SPI_HandleTypeDef hspi1;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define VIN_MEAS_Pin GPIO_PIN_0
#define VIN_MEAS_GPIO_Port GPIOC
#define WD_nRST_Pin GPIO_PIN_1
#define WD_nRST_GPIO_Port GPIOC
#define WD_SNS_Pin GPIO_PIN_0
#define WD_SNS_GPIO_Port GPIOA
#define NCS_Pin GPIO_PIN_1
#define NCS_GPIO_Port GPIOA
#define NCS_EXTI_IRQn EXTI1_IRQn
#define IMON_Pin GPIO_PIN_4
#define IMON_GPIO_Port GPIOA
#define VOUT_MEAS_P_Pin GPIO_PIN_4
#define VOUT_MEAS_P_GPIO_Port GPIOC
#define VOUT_MEAS_N_Pin GPIO_PIN_5
#define VOUT_MEAS_N_GPIO_Port GPIOC
#define I_MEAS_Pin GPIO_PIN_1
#define I_MEAS_GPIO_Port GPIOB
#define CLR_Pin GPIO_PIN_6
#define CLR_GPIO_Port GPIOC
#define DRV_TIM_Pin GPIO_PIN_9
#define DRV_TIM_GPIO_Port GPIOC
#define WD_TIM_Pin GPIO_PIN_8
#define WD_TIM_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define LED_G_Pin GPIO_PIN_10
#define LED_G_GPIO_Port GPIOC
#define LED_R_Pin GPIO_PIN_11
#define LED_R_GPIO_Port GPIOC
#define OC_Pin GPIO_PIN_2
#define OC_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
