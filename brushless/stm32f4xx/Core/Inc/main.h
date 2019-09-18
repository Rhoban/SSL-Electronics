/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void main_error_handler(uint32_t);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define I_U_Pin GPIO_PIN_0
#define I_U_GPIO_Port GPIOC
#define I_W_Pin GPIO_PIN_1
#define I_W_GPIO_Port GPIOC
#define I_REF_Pin GPIO_PIN_2
#define I_REF_GPIO_Port GPIOC
#define I_V_Pin GPIO_PIN_3
#define I_V_GPIO_Port GPIOC
#define LED_Pin GPIO_PIN_1
#define LED_GPIO_Port GPIOB
#define ENC_INT_CS_Pin GPIO_PIN_12
#define ENC_INT_CS_GPIO_Port GPIOB
#define ENC_SCK_Pin GPIO_PIN_13
#define ENC_SCK_GPIO_Port GPIOB
#define ENC_MISO_Pin GPIO_PIN_14
#define ENC_MISO_GPIO_Port GPIOB
#define ENC_MOSI_Pin GPIO_PIN_15
#define ENC_MOSI_GPIO_Port GPIOB
#define UEN_Pin GPIO_PIN_6
#define UEN_GPIO_Port GPIOC
#define VEN_Pin GPIO_PIN_7
#define VEN_GPIO_Port GPIOC
#define WEN_Pin GPIO_PIN_8
#define WEN_GPIO_Port GPIOC
#define ENC_EXT_CS_Pin GPIO_PIN_9
#define ENC_EXT_CS_GPIO_Port GPIOC
#define UPWM_Pin GPIO_PIN_8
#define UPWM_GPIO_Port GPIOA
#define VPWM_Pin GPIO_PIN_9
#define VPWM_GPIO_Port GPIOA
#define WPWM_Pin GPIO_PIN_10
#define WPWM_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */
#define ADC_DATA_SIZE 3 //3
#define ADC_CHANNEL_I_U ADC_CHANNEL_10
#define ADC_CHANNEL_I_W ADC_CHANNEL_11
#define ADC_CHANNEL_I_REF ADC_CHANNEL_12
#define ADC_CHANNEL_I_V ADC_CHANNEL_13
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
