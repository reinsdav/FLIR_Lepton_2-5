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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define JOY_UP_Pin GPIO_PIN_0
#define JOY_UP_GPIO_Port GPIOC
#define DISP_TE_Pin GPIO_PIN_0
#define DISP_TE_GPIO_Port GPIOA
#define DISP_NRESET_Pin GPIO_PIN_1
#define DISP_NRESET_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define JOY_RIGHT_Pin GPIO_PIN_0
#define JOY_RIGHT_GPIO_Port GPIOB
#define SPI1_DCX_Pin GPIO_PIN_10
#define SPI1_DCX_GPIO_Port GPIOB
#define JOY_CENTER_Pin GPIO_PIN_7
#define JOY_CENTER_GPIO_Port GPIOC
#define SPI2_NCS_Pin GPIO_PIN_8
#define SPI2_NCS_GPIO_Port GPIOA
#define SPI1_NCS_Pin GPIO_PIN_9
#define SPI1_NCS_GPIO_Port GPIOA
#define PW_DOWN_L_Pin GPIO_PIN_11
#define PW_DOWN_L_GPIO_Port GPIOA
#define RESET_L_Pin GPIO_PIN_12
#define RESET_L_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define SPI3_CS_Pin GPIO_PIN_12
#define SPI3_CS_GPIO_Port GPIOC
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define JOY_DOWN_Pin GPIO_PIN_4
#define JOY_DOWN_GPIO_Port GPIOB
#define JOY_LEFT_Pin GPIO_PIN_6
#define JOY_LEFT_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;
extern SPI_HandleTypeDef hspi3;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
