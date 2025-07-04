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
#define LED_ERR_Pin GPIO_PIN_2
#define LED_ERR_GPIO_Port GPIOE
#define LED_ARM_Pin GPIO_PIN_3
#define LED_ARM_GPIO_Port GPIOE
#define LED_HEARTBEAT_Pin GPIO_PIN_4
#define LED_HEARTBEAT_GPIO_Port GPIOE
#define FIRE1_P_Pin GPIO_PIN_0
#define FIRE1_P_GPIO_Port GPIOH
#define FIRE1_N_Pin GPIO_PIN_1
#define FIRE1_N_GPIO_Port GPIOH
#define ISENSE_GPS_Pin GPIO_PIN_0
#define ISENSE_GPS_GPIO_Port GPIOC
#define ISENSE_CHG_Pin GPIO_PIN_1
#define ISENSE_CHG_GPIO_Port GPIOC
#define ISENSE_3V3_Pin GPIO_PIN_2
#define ISENSE_3V3_GPIO_Port GPIOC
#define ISENSE_BATT_Pin GPIO_PIN_0
#define ISENSE_BATT_GPIO_Port GPIOA
#define PYRO1_CURR_Pin GPIO_PIN_1
#define PYRO1_CURR_GPIO_Port GPIOA
#define PYRO1_DIAG_Pin GPIO_PIN_2
#define PYRO1_DIAG_GPIO_Port GPIOA
#define PYRO1_VOLT_Pin GPIO_PIN_3
#define PYRO1_VOLT_GPIO_Port GPIOA
#define PYRO2_CURR_Pin GPIO_PIN_4
#define PYRO2_CURR_GPIO_Port GPIOA
#define PYRO2_DIAG_Pin GPIO_PIN_5
#define PYRO2_DIAG_GPIO_Port GPIOA
#define PYRO2_VOLT_Pin GPIO_PIN_6
#define PYRO2_VOLT_GPIO_Port GPIOA
#define VSENSE_MAG_Pin GPIO_PIN_0
#define VSENSE_MAG_GPIO_Port GPIOB
#define VSENSE_BATT_Pin GPIO_PIN_1
#define VSENSE_BATT_GPIO_Port GPIOB
#define FIRE2_P_Pin GPIO_PIN_9
#define FIRE2_P_GPIO_Port GPIOE
#define FIRE2_N_Pin GPIO_PIN_10
#define FIRE2_N_GPIO_Port GPIOE
#define GPS_RESET_Pin GPIO_PIN_3
#define GPS_RESET_GPIO_Port GPIOD
#define GPS_POS_VALID_Pin GPIO_PIN_4
#define GPS_POS_VALID_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
