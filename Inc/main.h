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
#define E3_Pin GPIO_PIN_3
#define E3_GPIO_Port GPIOE
#define IR_R_EN_Pin GPIO_PIN_4
#define IR_R_EN_GPIO_Port GPIOE
#define FAN_Pin GPIO_PIN_5
#define FAN_GPIO_Port GPIOE
#define LED_R_Pin GPIO_PIN_6
#define LED_R_GPIO_Port GPIOE
#define KEY_Pin GPIO_PIN_13
#define KEY_GPIO_Port GPIOC
#define IR_R_Pin GPIO_PIN_0
#define IR_R_GPIO_Port GPIOC
#define SOC_R_Pin GPIO_PIN_1
#define SOC_R_GPIO_Port GPIOC
#define SOB_R_Pin GPIO_PIN_2
#define SOB_R_GPIO_Port GPIOC
#define SOA_R_Pin GPIO_PIN_3
#define SOA_R_GPIO_Port GPIOC
#define IR_0_Pin GPIO_PIN_0
#define IR_0_GPIO_Port GPIOA
#define IR_1_Pin GPIO_PIN_1
#define IR_1_GPIO_Port GPIOA
#define IR_2_Pin GPIO_PIN_4
#define IR_2_GPIO_Port GPIOA
#define IR_3_Pin GPIO_PIN_5
#define IR_3_GPIO_Port GPIOA
#define IR_4_Pin GPIO_PIN_6
#define IR_4_GPIO_Port GPIOA
#define ADC_BAT_Pin GPIO_PIN_7
#define ADC_BAT_GPIO_Port GPIOA
#define SOC_L_Pin GPIO_PIN_4
#define SOC_L_GPIO_Port GPIOC
#define SOB_L_Pin GPIO_PIN_5
#define SOB_L_GPIO_Port GPIOC
#define SOA_L_Pin GPIO_PIN_0
#define SOA_L_GPIO_Port GPIOB
#define IR_L_Pin GPIO_PIN_1
#define IR_L_GPIO_Port GPIOB
#define LED_L_Pin GPIO_PIN_2
#define LED_L_GPIO_Port GPIOB
#define IR_L_EN_Pin GPIO_PIN_7
#define IR_L_EN_GPIO_Port GPIOE
#define IR_CEN_EN_Pin GPIO_PIN_8
#define IR_CEN_EN_GPIO_Port GPIOE
#define LCD_CS_Pin GPIO_PIN_11
#define LCD_CS_GPIO_Port GPIOE
#define LCD_WR_RS_Pin GPIO_PIN_13
#define LCD_WR_RS_GPIO_Port GPIOE
#define ENC_L_IN1_Pin GPIO_PIN_10
#define ENC_L_IN1_GPIO_Port GPIOB
#define IMU_CS_Pin GPIO_PIN_12
#define IMU_CS_GPIO_Port GPIOB
#define IMU_SCK_Pin GPIO_PIN_13
#define IMU_SCK_GPIO_Port GPIOB
#define IMU_MISO_Pin GPIO_PIN_14
#define IMU_MISO_GPIO_Port GPIOB
#define IMU_MOSI_Pin GPIO_PIN_15
#define IMU_MOSI_GPIO_Port GPIOB
#define MTR_L_DRVOFF_Pin GPIO_PIN_8
#define MTR_L_DRVOFF_GPIO_Port GPIOD
#define MTR_L_NSLEEP_Pin GPIO_PIN_9
#define MTR_L_NSLEEP_GPIO_Port GPIOD
#define MTR_L_NFAULT_Pin GPIO_PIN_10
#define MTR_L_NFAULT_GPIO_Port GPIOD
#define ENC_L_IN2_Pin GPIO_PIN_11
#define ENC_L_IN2_GPIO_Port GPIOD
#define ENC_R_IN1_Pin GPIO_PIN_12
#define ENC_R_IN1_GPIO_Port GPIOD
#define MTR_L_INH1_Pin GPIO_PIN_13
#define MTR_L_INH1_GPIO_Port GPIOD
#define MTR_L_INH2_Pin GPIO_PIN_14
#define MTR_L_INH2_GPIO_Port GPIOD
#define MTR_L_INH3_Pin GPIO_PIN_15
#define MTR_L_INH3_GPIO_Port GPIOD
#define MTR_R_INH1_Pin GPIO_PIN_6
#define MTR_R_INH1_GPIO_Port GPIOC
#define MTR_R_INH2_Pin GPIO_PIN_7
#define MTR_R_INH2_GPIO_Port GPIOC
#define MTR_R_INH3_Pin GPIO_PIN_8
#define MTR_R_INH3_GPIO_Port GPIOC
#define MTR_R_NFAULT_Pin GPIO_PIN_8
#define MTR_R_NFAULT_GPIO_Port GPIOA
#define MTR_R_NSLEEP_Pin GPIO_PIN_9
#define MTR_R_NSLEEP_GPIO_Port GPIOA
#define MTR_R_DRVOFF_Pin GPIO_PIN_10
#define MTR_R_DRVOFF_GPIO_Port GPIOA
#define MTR_INLx_Pin GPIO_PIN_15
#define MTR_INLx_GPIO_Port GPIOA
#define ENC_SCK_Pin GPIO_PIN_10
#define ENC_SCK_GPIO_Port GPIOC
#define ENC_MISO_Pin GPIO_PIN_11
#define ENC_MISO_GPIO_Port GPIOC
#define ENC_L_CS_Pin GPIO_PIN_12
#define ENC_L_CS_GPIO_Port GPIOC
#define ENC_R_CS_Pin GPIO_PIN_0
#define ENC_R_CS_GPIO_Port GPIOD
#define SWL_Pin GPIO_PIN_1
#define SWL_GPIO_Port GPIOD
#define SWR_Pin GPIO_PIN_3
#define SWR_GPIO_Port GPIOD
#define SWU_Pin GPIO_PIN_4
#define SWU_GPIO_Port GPIOD
#define SWD_Pin GPIO_PIN_5
#define SWD_GPIO_Port GPIOD
#define MTR_L_CS_Pin GPIO_PIN_6
#define MTR_L_CS_GPIO_Port GPIOD
#define MTR_MOSI_Pin GPIO_PIN_7
#define MTR_MOSI_GPIO_Port GPIOD
#define MTR_SCK_Pin GPIO_PIN_3
#define MTR_SCK_GPIO_Port GPIOB
#define MTR_MISO_Pin GPIO_PIN_4
#define MTR_MISO_GPIO_Port GPIOB
#define MTR_R_CS_Pin GPIO_PIN_5
#define MTR_R_CS_GPIO_Port GPIOB
#define XSHUT_Pin GPIO_PIN_8
#define XSHUT_GPIO_Port GPIOB
#define BUZZER_Pin GPIO_PIN_9
#define BUZZER_GPIO_Port GPIOB
#define SWC_Pin GPIO_PIN_0
#define SWC_GPIO_Port GPIOE
#define ENC_R_IN2_Pin GPIO_PIN_1
#define ENC_R_IN2_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
