/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    i2c.h
 * @brief   Function prototypes and definitions for i2c.c
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

#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "cmsis_os2.h"      // For osMutexId_t
#include "main.h"
#include <stdbool.h>

/* I2C configuration macros */
#define SLAVE_ADDR            0x80
#define SLAVE_ADDR1           0x20
#define DATA_SIZE             256
#define I2C_TIMEOUT           100

/* Uncomment to enable I2C1 IRQ RX as slave */
#define I2C_SLAVE_I2C1_IRQ_RX
/* Uncomment to enable I2C2 IRQ RX as slave */
// #define I2C_SLAVE_I2C2_IRQ_RX

/* Frame header definitions */
#define FRAME_CMD_HEADER_TX   0xF8
#define FRAME_CMD_HEADER_RX   0x8F
#define FRAME_DATA_HEADER_TX  0xE7
#define FRAME_DATA_HEADER_RX  0x7E

/* RTOS handles */
extern osSemaphoreId_t i2cSemaphore;
extern osThreadId_t slaveTxTaskHandle;
extern osThreadId_t slaveRxTaskHandle;

/* I2C buffers */
extern uint8_t tx_buf[33];  // Transmit buffer
extern uint8_t rx_buf[33];  // Receive buffer

/* Status flags */
extern volatile uint8_t exti_notify_from_peer; // External interrupt notification flag
extern volatile uint8_t master_data_ready;     // Master has data to send flag
extern volatile uint8_t slave_data_ready;      // Slave has data to send flag
extern volatile uint8_t i2c_bus_busy;          // I2C bus busy flag

/* I2C handles */
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

/* Function prototypes */

/**
 * @brief Send data to master device.
 * @param tx_buf Pointer to transmit buffer.
 * @param len Length of data to send.
 */
void send_data_to_master(uint8_t *tx_buf, uint16_t len);

/**
 * @brief Initialize I2C1 peripheral.
 */
void MX_I2C1_Init(void);

/**
 * @brief Initialize I2C2 peripheral.
 */
void MX_I2C2_Init(void);

/**
 * @brief Recover SDA line if stuck low.
 * @param hi2c I2C handle pointer.
 * @param port GPIO port for SDA/SCL.
 * @param scl SCL pin.
 * @param sda SDA pin.
 */
void I2C_RecoverSDA(I2C_HandleTypeDef *hi2c, GPIO_TypeDef *port, uint16_t scl, uint16_t sda);

/**
 * @brief Check if SDA line is low.
 * @param hi2c I2C handle pointer.
 * @param port GPIO port for SDA/SCL.
 * @param scl SCL pin.
 * @param sda SDA pin.
 * @return true if SDA is low, false otherwise.
 */
bool I2C_IsSDALow(I2C_HandleTypeDef *hi2c, GPIO_TypeDef *port, uint16_t scl, uint16_t sda);

/**
 * @brief Check if SCL line is low.
 * @param hi2c I2C handle pointer.
 * @param port GPIO port for SDA/SCL.
 * @param scl SCL pin.
 * @param sda SDA pin.
 * @return true if SCL is low, false otherwise.
 */
bool I2C_IsSCLLow(I2C_HandleTypeDef *hi2c, GPIO_TypeDef *port, uint16_t scl, uint16_t sda);

/**
 * @brief Recover SCL line if stuck low.
 * @param hi2c I2C handle pointer.
 * @param port GPIO port for SDA/SCL.
 * @param scl SCL pin.
 * @param sda SDA pin.
 */
void I2C_RecoverSCL(I2C_HandleTypeDef *hi2c, GPIO_TypeDef *port, uint16_t scl, uint16_t sda);

/* USER CODE BEGIN Prototypes */
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */