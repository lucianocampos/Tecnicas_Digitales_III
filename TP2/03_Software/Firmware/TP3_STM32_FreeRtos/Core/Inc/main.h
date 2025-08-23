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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
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
#define led_Pin GPIO_PIN_13
#define led_GPIO_Port GPIOC
#define DOUT_01_Pin GPIO_PIN_3
#define DOUT_01_GPIO_Port GPIOA
#define DOUT_02_Pin GPIO_PIN_4
#define DOUT_02_GPIO_Port GPIOA
#define DOUT_03_Pin GPIO_PIN_5
#define DOUT_03_GPIO_Port GPIOA
#define PWM_01_Pin GPIO_PIN_6
#define PWM_01_GPIO_Port GPIOA
#define PWM_02_Pin GPIO_PIN_7
#define PWM_02_GPIO_Port GPIOA
#define RS485_DE_Pin GPIO_PIN_0
#define RS485_DE_GPIO_Port GPIOB
#define RS485_RE_Pin GPIO_PIN_1
#define RS485_RE_GPIO_Port GPIOB
#define RS485_TX_Pin GPIO_PIN_10
#define RS485_TX_GPIO_Port GPIOB
#define RS485_RX_Pin GPIO_PIN_11
#define RS485_RX_GPIO_Port GPIOB
#define SPI_NSS_Pin GPIO_PIN_12
#define SPI_NSS_GPIO_Port GPIOB
#define SPI_SCK_Pin GPIO_PIN_13
#define SPI_SCK_GPIO_Port GPIOB
#define SPI_MISO_Pin GPIO_PIN_14
#define SPI_MISO_GPIO_Port GPIOB
#define SPI_MOSI_Pin GPIO_PIN_15
#define SPI_MOSI_GPIO_Port GPIOB
#define TTL_MODBUS_Pin GPIO_PIN_12
#define TTL_MODBUS_GPIO_Port GPIOA
#define DIN_01_Pin GPIO_PIN_3
#define DIN_01_GPIO_Port GPIOB
#define DIN_02_Pin GPIO_PIN_4
#define DIN_02_GPIO_Port GPIOB
#define DIN_03_Pin GPIO_PIN_6
#define DIN_03_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

extern volatile uint16_t adc_valores[3];	// variable para guardar valores de los canales analógicos

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
