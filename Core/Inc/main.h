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
#define ADC_MEG_Pin GPIO_PIN_0
#define ADC_MEG_GPIO_Port GPIOC
#define IOUT_1_Pin GPIO_PIN_1
#define IOUT_1_GPIO_Port GPIOC
#define IOUT_2_Pin GPIO_PIN_2
#define IOUT_2_GPIO_Port GPIOC
#define PWM2_A_Pin GPIO_PIN_0
#define PWM2_A_GPIO_Port GPIOA
#define STOP_Pin GPIO_PIN_1
#define STOP_GPIO_Port GPIOA
#define ICM_INT_Pin GPIO_PIN_4
#define ICM_INT_GPIO_Port GPIOA
#define ICM_INT_EXTI_IRQn EXTI4_IRQn
#define ICM_SCLK_Pin GPIO_PIN_5
#define ICM_SCLK_GPIO_Port GPIOA
#define ICM_MISO_Pin GPIO_PIN_6
#define ICM_MISO_GPIO_Port GPIOA
#define ICM_MOSI_Pin GPIO_PIN_7
#define ICM_MOSI_GPIO_Port GPIOA
#define ICM_CS_Pin GPIO_PIN_4
#define ICM_CS_GPIO_Port GPIOC
#define TOF_DRDY_Pin GPIO_PIN_5
#define TOF_DRDY_GPIO_Port GPIOC
#define TOF_DRDY_EXTI_IRQn EXTI9_5_IRQn
#define LED1_Pin GPIO_PIN_0
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_1
#define LED2_GPIO_Port GPIOB
#define KEY3_Pin GPIO_PIN_13
#define KEY3_GPIO_Port GPIOB
#define KEY2_Pin GPIO_PIN_14
#define KEY2_GPIO_Port GPIOB
#define KEY1_Pin GPIO_PIN_15
#define KEY1_GPIO_Port GPIOB
#define ENCODER2_B_Pin GPIO_PIN_6
#define ENCODER2_B_GPIO_Port GPIOC
#define ENCODER2_A_Pin GPIO_PIN_7
#define ENCODER2_A_GPIO_Port GPIOC
#define OLED_SDA_Pin GPIO_PIN_8
#define OLED_SDA_GPIO_Port GPIOC
#define OLED_SCL_Pin GPIO_PIN_9
#define OLED_SCL_GPIO_Port GPIOC
#define ENCODER1_B_Pin GPIO_PIN_8
#define ENCODER1_B_GPIO_Port GPIOA
#define ENCODER1_A_Pin GPIO_PIN_9
#define ENCODER1_A_GPIO_Port GPIOA
#define ADC_DRDY_Pin GPIO_PIN_15
#define ADC_DRDY_GPIO_Port GPIOA
#define ADC_DRDY_EXTI_IRQn EXTI15_10_IRQn
#define ADC_SCLK_Pin GPIO_PIN_10
#define ADC_SCLK_GPIO_Port GPIOC
#define ADC_MISO_Pin GPIO_PIN_11
#define ADC_MISO_GPIO_Port GPIOC
#define ADC_MOSI_Pin GPIO_PIN_12
#define ADC_MOSI_GPIO_Port GPIOC
#define ADC_CS_Pin GPIO_PIN_2
#define ADC_CS_GPIO_Port GPIOD
#define PWM2_B_Pin GPIO_PIN_3
#define PWM2_B_GPIO_Port GPIOB
#define PWM1_B_Pin GPIO_PIN_6
#define PWM1_B_GPIO_Port GPIOB
#define PWM1_A_Pin GPIO_PIN_7
#define PWM1_A_GPIO_Port GPIOB
#define TOF_SCL_Pin GPIO_PIN_8
#define TOF_SCL_GPIO_Port GPIOB
#define TOF_SDA_Pin GPIO_PIN_9
#define TOF_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
