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

// #define I2C_MASTER
#define I2C_SLAVE
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <stdio.h>
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
/* -------------------- Debug Macros -------------------- */
#define RA_POWEREX_DEBUG_ENABLE 1
#if RA_POWEREX_DEBUG_ENABLE
    #define RA_POWEREX_PRINTF(fmt, ...)  printf("[RA_POWEREX] " fmt, ##__VA_ARGS__)
    #define RA_POWEREX_INFO(fmt, ...)    printf("[RA_POWEREX][INFO] " fmt, ##__VA_ARGS__)
    #define RA_POWEREX_WARN(fmt, ...)    printf("[RA_POWEREX][WARN] " fmt, ##__VA_ARGS__)
    #define RA_POWEREX_ERROR(fmt, ...)   printf("[RA_POWEREX][ERROR] " fmt, ##__VA_ARGS__)
    #define RA_POWEREX_DEBUG(fmt, ...)   printf("[RA_POWEREX][DEBUG] " fmt, ##__VA_ARGS__)
#else
    #define RA_POWEREX_PRINTF(fmt, ...)  do {} while(0)
    #define RA_POWEREX_INFO(fmt, ...)    do {} while(0)
    #define RA_POWEREX_WARN(fmt, ...)    do {} while(0)
    #define RA_POWEREX_ERROR(fmt, ...)   do {} while(0)
    #define RA_POWEREX_DEBUG(fmt, ...)   do {} while(0)
#endif
#define I2C_DEBUG_ENABLE
#ifdef I2C_DEBUG_ENABLE
    #define I2C_DEBUG(fmt, ...)          printf("[I2C DEBUG] " fmt, ##__VA_ARGS__)
    #define I2C_INFO(fmt, ...)          printf("[I2C INFO] " fmt, ##__VA_ARGS__)
#else
    #define I2C_DEBUG(fmt, ...)          do {} while(0)
    #define I2C_INFO(fmt, ...)          do {} while(0)
#endif
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* -------------------- UART Pin Definitions -------------------- */
#define MASTER_TX_Pin GPIO_PIN_8
#define MASTER_TX_GPIO_Port GPIOD
#define MASTER_RX_Pin GPIO_PIN_9
#define MASTER_RX_GPIO_Port GPIOD
/* -------------------- Power Control Pin Definitions -------------------- */
#define VCC_EN_Pin GPIO_PIN_3
#define VCC_EN_GPIO_Port GPIOE
#define IOVCC_EN_Pin GPIO_PIN_2
#define IOVCC_EN_GPIO_Port GPIOE
#define ELVDD_EN_Pin GPIO_PIN_4
#define ELVDD_EN_GPIO_Port GPIOA
#define ELVSS_EN_Pin GPIO_PIN_4
#define ELVSS_EN_GPIO_Port GPIOE
#define SHUTDOWN GPIO_PIN_1
#define SHUTDOWN_EN_GPIO_Port GPIOE
/* -------------------- Level Shifter Pin Definitions -------------------- */
// LEVEL_SHIFT_OE_Pin, LEVEL_SHIFT_OE_GPIO_Port: Level shifter output enable pin
#define LEVEL_SHIFT_OE_Pin GPIO_PIN_14
#define LEVEL_SHIFT_OE_GPIO_Port GPIOE
/* -------------------- ADC Pin Definitions -------------------- */
#define ADC_RESET_Pin GPIO_PIN_0
#define ADC_RESET_GPIO_Port GPIOA
#define ADC_SPI_CS_Pin GPIO_PIN_2
#define ADC_SPI_CS_GPIO_Port GPIOA
#define ADC_DRDY_Pin GPIO_PIN_3
#define ADC_DRDY_GPIO_Port GPIOA
#define ADC_SPI_MISO_Pin GPIO_PIN_6
#define ADC_SPI_MISO_GPIO_Port GPIOA
#define ADC_SPI_MOSI_Pin GPIO_PIN_7
#define ADC_SPI_CLK_Pin GPIO_PIN_5
#define ADC_SPI_MOSI_GPIO_Port GPIOA
/* -------------------- FLASH SPI Pin Definitions -------------------- */
#define FLASH_CS_Pin GPIO_PIN_9
#define FLASH_CS_GPIO_Port GPIOC
#define FLASH_CLK_Pin GPIO_PIN_10
#define FLASH_CLK_GPIO_Port GPIOC
#define FLASH_MISO_Pin GPIO_PIN_11
#define FLASH_MISO_GPIO_Port GPIOC
#define FLASH_MOSI_Pin GPIO_PIN_12
#define FLASH_MOSI_GPIO_Port GPIOC
#define ADC_SPI_CLK_GPIO_Port GPIOB
/* -------------------- DAC Pin Definitions -------------------- */
#define LDAC_Pin GPIO_PIN_11
#define LDAC_Port GPIOA

/*for power enhance board*/
#define LEVEL_SHIFT_DAC_Pin GPIO_PIN_4
#define LEVEL_SHIFT_DAC_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
