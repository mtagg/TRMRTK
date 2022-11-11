/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LEDR_Out_Pin GPIO_PIN_13
#define LEDR_Out_GPIO_Port GPIOC
#define LEDG_Out_Pin GPIO_PIN_14
#define LEDG_Out_GPIO_Port GPIOC
#define LEDB_Out_Pin GPIO_PIN_15
#define LEDB_Out_GPIO_Port GPIOC
#define LED_Power_OSC_IN_Pin GPIO_PIN_0
#define LED_Power_OSC_IN_GPIO_Port GPIOD
#define LED_Button_In_OSC_OUT_Pin GPIO_PIN_1
#define LED_Button_In_OSC_OUT_GPIO_Port GPIOD
#define ADC1_IN1_BATT_Pin GPIO_PIN_0
#define ADC1_IN1_BATT_GPIO_Port GPIOA
#define nTILT_BUTTON_Pin GPIO_PIN_1
#define nTILT_BUTTON_GPIO_Port GPIOA
#define MP6543B_SOA_X_Pin GPIO_PIN_4
#define MP6543B_SOA_X_GPIO_Port GPIOA
#define MP6543B_SOB_X_Pin GPIO_PIN_5
#define MP6543B_SOB_X_GPIO_Port GPIOA
#define MP6543B_SOC_X_Pin GPIO_PIN_6
#define MP6543B_SOC_X_GPIO_Port GPIOA
#define MP6543B_SOA_Y_Pin GPIO_PIN_7
#define MP6543B_SOA_Y_GPIO_Port GPIOA
#define MP6543B_SOB_Y_Pin GPIO_PIN_0
#define MP6543B_SOB_Y_GPIO_Port GPIOB
#define MP6543B_SOC_Y_Pin GPIO_PIN_1
#define MP6543B_SOC_Y_GPIO_Port GPIOB
#define MP6543B_nFAULT_Y_Pin GPIO_PIN_2
#define MP6543B_nFAULT_Y_GPIO_Port GPIOB
#define MP6543B_nFAULT_X_Pin GPIO_PIN_12
#define MP6543B_nFAULT_X_GPIO_Port GPIOB
#define MP6543B_DIR_X_Pin GPIO_PIN_13
#define MP6543B_DIR_X_GPIO_Port GPIOB
#define MP6543B_nSLEEP_X_Pin GPIO_PIN_14
#define MP6543B_nSLEEP_X_GPIO_Port GPIOB
#define MP6543B_nBRAKE_X_Pin GPIO_PIN_15
#define MP6543B_nBRAKE_X_GPIO_Port GPIOB
#define MP6543B_PWM_X_Pin GPIO_PIN_8
#define MP6543B_PWM_X_GPIO_Port GPIOA
#define MP6543B_PWM_Y_Pin GPIO_PIN_9
#define MP6543B_PWM_Y_GPIO_Port GPIOA
#define MP6543B_DIR_Y_Pin GPIO_PIN_10
#define MP6543B_DIR_Y_GPIO_Port GPIOA
#define MP6543B_nSLEEP_Y_Pin GPIO_PIN_11
#define MP6543B_nSLEEP_Y_GPIO_Port GPIOA
#define MP6543B_nBRAKE_Y_Pin GPIO_PIN_12
#define MP6543B_nBRAKE_Y_GPIO_Port GPIOA
#define SYS_SWDIO_Pin GPIO_PIN_13
#define SYS_SWDIO_GPIO_Port GPIOA
#define SYS_SWCLK_Pin GPIO_PIN_14
#define SYS_SWCLK_GPIO_Port GPIOA
#define SPI1_CSn_Pin GPIO_PIN_15
#define SPI1_CSn_GPIO_Port GPIOA
#define MC3479_INTN1_Pin GPIO_PIN_6
#define MC3479_INTN1_GPIO_Port GPIOB
#define MC3479_INTN2_Pin GPIO_PIN_7
#define MC3479_INTN2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */