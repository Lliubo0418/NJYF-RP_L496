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
#define IO1_Pin GPIO_PIN_0
#define IO1_GPIO_Port GPIOC
#define IO2_Pin GPIO_PIN_1
#define IO2_GPIO_Port GPIOC
#define IO3_Pin GPIO_PIN_2
#define IO3_GPIO_Port GPIOC
#define S5_EN_Pin GPIO_PIN_3
#define S5_EN_GPIO_Port GPIOC
#define HART_TX_Pin GPIO_PIN_2
#define HART_TX_GPIO_Port GPIOA
#define HART_RX_Pin GPIO_PIN_3
#define HART_RX_GPIO_Port GPIOA
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define ADC_CS_X_Pin GPIO_PIN_4
#define ADC_CS_X_GPIO_Port GPIOC
#define HART_RESET_Pin GPIO_PIN_5
#define HART_RESET_GPIO_Port GPIOC
#define AD5241_FAULT_Pin GPIO_PIN_2
#define AD5241_FAULT_GPIO_Port GPIOB
#define AD5241_FAULT_EXTI_IRQn EXTI2_IRQn
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define RS485_DE_Pin GPIO_PIN_8
#define RS485_DE_GPIO_Port GPIOA
#define DISP_TX_Pin GPIO_PIN_9
#define DISP_TX_GPIO_Port GPIOA
#define DISP_RX_Pin GPIO_PIN_10
#define DISP_RX_GPIO_Port GPIOA
#define HART_CD_Pin GPIO_PIN_11
#define HART_CD_GPIO_Port GPIOA
#define HART_CD_EXTI_IRQn EXTI15_10_IRQn
#define HART_RTS_Pin GPIO_PIN_12
#define HART_RTS_GPIO_Port GPIOA
#define RS485_TX_Pin GPIO_PIN_10
#define RS485_TX_GPIO_Port GPIOC
#define RS485_RX_Pin GPIO_PIN_11
#define RS485_RX_GPIO_Port GPIOC
#define TEMP_INT_Pin GPIO_PIN_12
#define TEMP_INT_GPIO_Port GPIOC
#define TEMP_INT_EXTI_IRQn EXTI15_10_IRQn
#define EEPROM_WP_Pin GPIO_PIN_2
#define EEPROM_WP_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* 串口调试总开关：DEBUG_PRINT_ENABLE 置 0 即全工程关闭 printf（见 bsp_debug.h）。
 * 所有业务模块经 main.h 包含链自动生效，协议帧（USART1）不受影响。 */
#include "bsp_debug.h"

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
