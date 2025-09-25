/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32f4xx_it.c
 * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
#include "i2c.h"
#include "bsp_ads1256.h"
#include "usart.h"
#include "delay.h"
#include "bsp_power.h"

static uint8_t first_byte_state = 1; // 是否收到第1个字节,也就是偏移地址（0：已收到，1：没有收到）                              
uint8_t offset = 0;         // 偏移地址

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DAC_HandleTypeDef hdac;
extern DMA_HandleTypeDef hdma_i2c1_rx;
extern DMA_HandleTypeDef hdma_i2c1_tx;
extern DMA_HandleTypeDef hdma_i2c2_rx;
extern DMA_HandleTypeDef hdma_i2c2_tx;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern DMA_HandleTypeDef hdma_spi3_rx;
extern DMA_HandleTypeDef hdma_spi3_tx;
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi3;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim6;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  RA_POWEREX_DEBUG("HardFault_Handler\r\n");
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
 * @brief This function handles Memory management fault.
 */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
 * @brief This function handles Pre-fetch fault, memory access fault.
 */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles EXTI line1 interrupt.
 */
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */

  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(ADC_DRDY_Pin);
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  /* USER CODE END EXTI1_IRQn 1 */
}

/**
 * @brief This function handles DMA1 stream0 global interrupt.
 */
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

  /* USER CODE END DMA1_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_i2c1_rx);
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
 * @brief This function handles DMA1 stream2 global interrupt.
 */
void DMA1_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream2_IRQn 0 */

  /* USER CODE END DMA1_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi3_rx);
  /* USER CODE BEGIN DMA1_Stream2_IRQn 1 */

  /* USER CODE END DMA1_Stream2_IRQn 1 */
}

/**
 * @brief This function handles DMA1 stream3 global interrupt.
 */
void DMA1_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */

  /* USER CODE END DMA1_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_i2c2_rx);
  /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

  /* USER CODE END DMA1_Stream3_IRQn 1 */
}

/**
 * @brief This function handles DMA1 stream5 global interrupt.
 */
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */

  /* USER CODE END DMA1_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi3_tx);
  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

  /* USER CODE END DMA1_Stream5_IRQn 1 */
}

/**
 * @brief This function handles DMA1 stream6 global interrupt.
 */
void DMA1_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream6_IRQn 0 */

  /* USER CODE END DMA1_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_i2c1_tx);
  /* USER CODE BEGIN DMA1_Stream6_IRQn 1 */

  /* USER CODE END DMA1_Stream6_IRQn 1 */
}

/**
 * @brief This function handles I2C1 event interrupt.
 */
void I2C1_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_EV_IRQn 0 */

  /* USER CODE END I2C1_EV_IRQn 0 */
  HAL_I2C_EV_IRQHandler(&hi2c1);
  /* USER CODE BEGIN I2C1_EV_IRQn 1 */
  /* USER CODE END I2C1_EV_IRQn 1 */
}

/**
 * @brief This function handles I2C2 event interrupt.
 */
void I2C2_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C2_EV_IRQn 0 */

  /* USER CODE END I2C2_EV_IRQn 0 */
  //I2C_DEBUG("I2C2_EV_IRQHandler\r\n");
  HAL_I2C_EV_IRQHandler(&hi2c2);
  /* USER CODE BEGIN I2C2_EV_IRQn 1 */

  /* USER CODE END I2C2_EV_IRQn 1 */
}

/**
 * @brief This function handles SPI1 global interrupt.
 */
void SPI1_IRQHandler(void)
{
  /* USER CODE BEGIN SPI1_IRQn 0 */

  /* USER CODE END SPI1_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi1);
  /* USER CODE BEGIN SPI1_IRQn 1 */

  /* USER CODE END SPI1_IRQn 1 */
}

/**
 * @brief This function handles USART1 global interrupt.
 */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
 * @brief This function handles DMA1 stream7 global interrupt.
 */
void DMA1_Stream7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream7_IRQn 0 */

  /* USER CODE END DMA1_Stream7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_i2c2_tx);
  /* USER CODE BEGIN DMA1_Stream7_IRQn 1 */

  /* USER CODE END DMA1_Stream7_IRQn 1 */
}

/**
 * @brief This function handles SPI3 global interrupt.
 */
void SPI3_IRQHandler(void)
{
  /* USER CODE BEGIN SPI3_IRQn 0 */

  /* USER CODE END SPI3_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi3);
  /* USER CODE BEGIN SPI3_IRQn 1 */

  /* USER CODE END SPI3_IRQn 1 */
}

/**
 * @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
 */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  //  if (hdac.State != HAL_DAC_STATE_RESET) {
  //    HAL_DAC_IRQHandler(&hdac);
  //  }
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
 * @brief This function handles DMA2 stream0 global interrupt.
 */
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_rx);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/**
 * @brief This function handles DMA2 stream2 global interrupt.
 */
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */

  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

  /* USER CODE END DMA2_Stream2_IRQn 1 */
}

/**
 * @brief This function handles DMA2 stream3 global interrupt.
 */
void DMA2_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream3_IRQn 0 */

  /* USER CODE END DMA2_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_tx);
  /* USER CODE BEGIN DMA2_Stream3_IRQn 1 */

  /* USER CODE END DMA2_Stream3_IRQn 1 */
}

/**
 * @brief This function handles DMA2 stream7 global interrupt.
 */
void DMA2_Stream7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream7_IRQn 0 */

  /* USER CODE END DMA2_Stream7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA2_Stream7_IRQn 1 */

  /* USER CODE END DMA2_Stream7_IRQn 1 */
}

#ifdef I2C_SLAVE
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
  // 完成一次通信，清除状态
  first_byte_state = 1;
  offset = 0;
  //I2C_DEBUG("HAL_I2C_ListenCpltCallback\r\n");
  HAL_I2C_EnableListen_IT(hi2c);
}


void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
  I2C_DEBUG("I2C Slave AddrCallback: AddrMatchCode=0x%02X, TransferDirection=%s\r\n", 
              AddrMatchCode, (TransferDirection == I2C_DIRECTION_TRANSMIT) ? "Transmit" : "Receive");
  if(TransferDirection == I2C_DIRECTION_TRANSMIT) 
  {
    
      //I2C_DEBUG("I2C_DIRECTION_TRANSMIT\r\n");
      HAL_I2C_Slave_Seq_Receive_IT(hi2c, rx_buf, 1, I2C_LAST_FRAME);
      //HAL_I2C_Slave_Receive(&hi2c2, rx_buf, 1, 1000);
  }
  else
  {
    //I2C_DEBUG("I2C_DIRECTION_RECEIVE\r\n");
    //HAL_I2C_Slave_Transmit(&hi2c2, tx_buf, 2, 1000);
    // 主机接收，从机发送
    tx_buf[0]=11;
    tx_buf[1]=22;
    HAL_I2C_Slave_Seq_Transmit_IT(hi2c, tx_buf, 2, I2C_LAST_FRAME);
  }
  HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{

  if (hi2c->Instance == hi2c1.Instance)
  {
#if 0
    // 打印错误码
    I2C_DEBUG("I2C Error: Instance=0x%p, ErrorCode=0x%08lX\r\n", hi2c->Instance, hi2c->ErrorCode);

    // 根据错误类型做细致处理
    if (hi2c->ErrorCode & HAL_I2C_ERROR_AF) {
        I2C_DEBUG("I2C Error: NACK received (AF)\r\n");
        // 可做重试或通知主机
    }
    if (hi2c->ErrorCode & HAL_I2C_ERROR_ARLO) {
        I2C_DEBUG("I2C Error: Arbitration lost (ARLO)\r\n");
        // 多主机场景可做仲裁恢复
    }
    if (hi2c->ErrorCode & HAL_I2C_ERROR_BERR) {
        I2C_DEBUG("I2C Error: Bus error (BERR)\r\n");
        // 可做总线复位
    }
    if (hi2c->ErrorCode & HAL_I2C_ERROR_OVR) {
        I2C_DEBUG("I2C Error: Overrun/Underrun (OVR)\r\n");
        // 可清理缓冲区
    }
    if (hi2c->ErrorCode & HAL_I2C_ERROR_TIMEOUT) {
        I2C_DEBUG("I2C Error: Timeout\r\n");
        // 可做超时重试
    }
    // 清理自定义状态变量
    first_byte_state = 1;
    offset = 0;
#endif
      if (I2C_IsSDALow(hi2c,GPIOB,GPIO_PIN_6,GPIO_PIN_7)) {
        // 如果 SDA 被拉低，调用恢复函数
        I2C_RecoverSDA(hi2c,GPIOB,GPIO_PIN_6,GPIO_PIN_7);
      }
      if (I2C_IsSCLLow(hi2c,GPIOB,GPIO_PIN_6,GPIO_PIN_7)) {
        // 如果 SCL 被拉低，调用恢复函数
        I2C_RecoverSCL(hi2c,GPIOB,GPIO_PIN_6,GPIO_PIN_7);
      }
      MX_I2C1_Init();
    // 重新使能侦听模式，保证从机持续响应主机
      HAL_I2C_EnableListen_IT(&hi2c1);
  }
  // 处理其他I2C实例的错误
  else if (hi2c->Instance == hi2c2.Instance)
  {
#if 0
        // 打印错误码
    I2C_DEBUG("I2C Error: Instance=0x%p, ErrorCode=0x%08lX\r\n", hi2c->Instance, hi2c->ErrorCode);

    // 根据错误类型做细致处理
    if (hi2c->ErrorCode & HAL_I2C_ERROR_AF) {
        I2C_DEBUG("I2C Error: NACK received (AF)\r\n");
        // 可做重试或通知主机
    }
    if (hi2c->ErrorCode & HAL_I2C_ERROR_ARLO) {
        I2C_DEBUG("I2C Error: Arbitration lost (ARLO)\r\n");
        // 多主机场景可做仲裁恢复
    }
    if (hi2c->ErrorCode & HAL_I2C_ERROR_BERR) {
        I2C_DEBUG("I2C Error: Bus error (BERR)\r\n");
        // 可做总线复位
    }
    if (hi2c->ErrorCode & HAL_I2C_ERROR_OVR) {
        I2C_DEBUG("I2C Error: Overrun/Underrun (OVR)\r\n");
        // 可清理缓冲区
    }
    if (hi2c->ErrorCode & HAL_I2C_ERROR_TIMEOUT) {
        I2C_DEBUG("I2C Error: Timeout\r\n");
        // 可做超时重试
    }

    // 清理自定义状态变量
    first_byte_state = 1;
    offset = 0;
#endif
    if (I2C_IsSDALow(&hi2c2,GPIOB,GPIO_PIN_10,GPIO_PIN_11)) {
      // 如果 SDA 被拉低，调用恢复函数
      I2C_RecoverSDA(&hi2c2,GPIOB,GPIO_PIN_10,GPIO_PIN_11);
    }
    if (I2C_IsSCLLow(&hi2c2,GPIOB,GPIO_PIN_10,GPIO_PIN_11)) {
      // 如果 SCL 被拉低，调用恢复函数
      I2C_RecoverSCL(&hi2c2,GPIOB,GPIO_PIN_10,GPIO_PIN_11);
    }
    MX_I2C2_Init();
    // 重新使能侦听模式，保证从机持续响应主机
    HAL_I2C_EnableListen_IT(&hi2c2);

  }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  //offset++;
  //HAL_StatusTypeDef ret = HAL_I2C_Slave_Seq_Receive_IT(hi2c, &rx_buf[0], 1, I2C_NEXT_FRAME);
  //I2C_DEBUG("I2C Slave Receive 1 byte rx_buf[%d]: %d\r\n", offset-1 , rx_buf[offset-1]);

  //HAL_I2C_Slave_Seq_Transmit_IT(hi2c, tx_buf, 2, I2C_NEXT_FRAME);
  //I2C_DEBUG("I2C Slave Transmit 2 bytes: %d, %d\r\n", tx_buf[0], tx_buf[1]);
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{                                                                            
  //HAL_I2C_Slave_Seq_Transmit_IT(hi2c, tx_buf, 2, I2C_NEXT_FRAME);
  //I2C_DEBUG("I2C Slave Transmit 2 bytes: %d, %d\r\n", tx_buf[0], tx_buf[1]);

  //HAL_StatusTypeDef ret = HAL_I2C_Slave_Seq_Receive_IT(hi2c, &rx_buf[0], 1, I2C_NEXT_FRAME);
  //I2C_DEBUG("I2C Slave Receive 1 byte rx_buf[%d]: %d\r\n", offset-1 , rx_buf[offset-1]);

}
#endif

#ifdef I2C_MASTER
// I2C中断回调
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{

  printf("Master received ...\r\n");
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  HAL_I2C_DeInit(&hi2c1);
  HAL_I2C_Init(&hi2c1);
}
#endif
void I2C1_ER_IRQHandler(void)
{
  HAL_I2C_ErrorCallback(&hi2c1);
}

void I2C2_ER_IRQHandler(void)
{
  HAL_I2C_ErrorCallback(&hi2c2);
}

// 电压电流采样回调函数
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == ADC_DRDY_Pin)
  {

#if BSP_VOL_DEBUG
    if (cnt == 0)
    {
      printf("-----%d----\r\n", channel);

      bsp_ads1256_set_single_channel(&dev_vol, channel);
      bsp_delay_us(5);
    }
    else if (cnt == 1)
    {
      bsp_ads1256_sync_wakeup(&dev_vol);
    }
    else if (cnt == 2)
    {
      bsp_ads1256_read_data(&dev_vol, &buf[channel]);
      if (channel == 5)
      {
        channel = 1;
      }
      else
      {
        channel = 5;
      }
      cnt = 0;
      return;
    }

    cnt += 1;
#elif BSP_VOL_WORK
    bsp_ads1256_irq_handle(&dev_vol);
#endif

#if BSP_CUR_DEBUG
    if (cnt == 0)
    {
      printf("cur -----%d----\r\n", channel);

      bsp_ads1256_set_single_channel(&dev_cur, channel);
      bsp_delay_us(5);
    }
    else if (cnt == 1)
    {
      bsp_ads1256_sync_wakeup(&dev_cur);
    }
    else if (cnt == 2)
    {
      bsp_ads1256_read_data(&dev_cur, &buf[channel]);
      if (channel == 2)
      {
        channel = 1;
      }
      else
      {
        channel = 2;
      }
    }
    else if (cnt == 3)
    {
      cnt = 0;
      return;
    }
    cnt += 1;
#elif BSP_CUR_WORK
    // bsp_ads1256_irq_handle(&dev_cur);
#endif
  }
#ifdef I2C_MASTER
  if (GPIO_Pin == I2C_EXTI0_Pin) // 主机收到从机通知
  {
    printf("I2C Master received notify from Slave\r\n");
    TIM2_Start();
    if (HAL_I2C_Master_Receive(&hi2c1, SLAVE_ADDR, rx_buf, DATA_SIZE, 1000) != HAL_OK)
    {
      printf("I2C Master not received...\r\n");
    }
    TIM2_Stop();
  }
#endif
}
/* USER CODE END 1 */
