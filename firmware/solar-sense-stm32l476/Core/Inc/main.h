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
#include "stm32l4xx_hal.h"

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
#define ADC_UV_Pin GPIO_PIN_0
#define ADC_UV_GPIO_Port GPIOA
#define ADC_HX94C_RH_Pin GPIO_PIN_1
#define ADC_HX94C_RH_GPIO_Port GPIOA
#define TC1_CS_Pin GPIO_PIN_4
#define TC1_CS_GPIO_Port GPIOA
#define ADC_HX94C_TEMP_Pin GPIO_PIN_4
#define ADC_HX94C_TEMP_GPIO_Port GPIOC
#define RAIN_AO_Pin GPIO_PIN_5
#define RAIN_AO_GPIO_Port GPIOC
#define TC2_CS_Pin GPIO_PIN_0
#define TC2_CS_GPIO_Port GPIOB
#define TC3_CS_Pin GPIO_PIN_1
#define TC3_CS_GPIO_Port GPIOB
#define EN_12V_Pin GPIO_PIN_2
#define EN_12V_GPIO_Port GPIOB
#define LORA_RESET_Pin GPIO_PIN_10
#define LORA_RESET_GPIO_Port GPIOB
#define LORA_DIO1_Pin GPIO_PIN_11
#define LORA_DIO1_GPIO_Port GPIOB
#define LORA_BUSY_Pin GPIO_PIN_12
#define LORA_BUSY_GPIO_Port GPIOB
#define BAT_ALRT_Pin GPIO_PIN_13
#define BAT_ALRT_GPIO_Port GPIOB
#define CN3791_CHRG_Pin GPIO_PIN_14
#define CN3791_CHRG_GPIO_Port GPIOB
#define CN3791_DONE_Pin GPIO_PIN_15
#define CN3791_DONE_GPIO_Port GPIOB
#define SDMMC1_CD_Pin GPIO_PIN_6
#define SDMMC1_CD_GPIO_Port GPIOC
#define LORA_CS_Pin GPIO_PIN_8
#define LORA_CS_GPIO_Port GPIOA
#define LED_STATUS_Pin GPIO_PIN_3
#define LED_STATUS_GPIO_Port GPIOB
#define LED_ERR_Pin GPIO_PIN_4
#define LED_ERR_GPIO_Port GPIOB
#define RAIN_DET_Pin GPIO_PIN_5
#define RAIN_DET_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
