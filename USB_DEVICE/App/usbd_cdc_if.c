/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v1.0_Cube
  * @brief          : Usb device for Virtual Com Port.
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
#include "usbd_cdc_if.h"
#include "main.h"//Io defination and debug macro
#include "bsp.h" //import version
#include "cdc_task.h" //CDC private type define
#include "stm32f4xx_hal_gpio.h"//addr recon
#include "bsp_mcp4728.h"
#include "bsp_ads1256.h"
#include "bsp_power.h"//powen en or disen
#include "bsp_calibration.h" //calibration data
#include "calibration_utils.h"
/* USER CODE BEGIN INCLUDE */

/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_CDC_IF
  * @{
  */

/** @defgroup USBD_CDC_IF_Private_TypesDefinitions USBD_CDC_IF_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Defines USBD_CDC_IF_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */
/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Macros USBD_CDC_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Variables USBD_CDC_IF_Private_Variables
  * @brief Private variables.
  * @{
  */

/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
uint8_t UserRxBufferHS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferHS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceHS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_FunctionPrototypes USBD_CDC_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CDC_Init_HS(void);
static int8_t CDC_DeInit_HS(void);
static int8_t CDC_Control_HS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_HS(uint8_t* pbuf, uint32_t *Len);
static int8_t CDC_TransmitCplt_HS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_CDC_ItfTypeDef USBD_Interface_fops_HS =
{
  CDC_Init_HS,
  CDC_DeInit_HS,
  CDC_Control_HS,
  CDC_Receive_HS,
  CDC_TransmitCplt_HS
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes the CDC media low layer over the USB HS IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_HS(void)
{
  /* USER CODE BEGIN 8 */
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceHS, UserTxBufferHS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceHS, UserRxBufferHS);
  return (USBD_OK);
  /* USER CODE END 8 */
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @param  None
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_HS(void)
{
  /* USER CODE BEGIN 9 */
  return (USBD_OK);
  /* USER CODE END 9 */
}

/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_HS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* USER CODE BEGIN 10 */
  switch(cmd)
  {
  case CDC_SEND_ENCAPSULATED_COMMAND:

    break;

  case CDC_GET_ENCAPSULATED_RESPONSE:

    break;

  case CDC_SET_COMM_FEATURE:

    break;

  case CDC_GET_COMM_FEATURE:

    break;

  case CDC_CLEAR_COMM_FEATURE:

    break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
  case CDC_SET_LINE_CODING:

    break;

  case CDC_GET_LINE_CODING:

    break;

  case CDC_SET_CONTROL_LINE_STATE:

    break;

  case CDC_SEND_BREAK:

    break;

  default:
    break;
  }

  return (USBD_OK);
  /* USER CODE END 10 */
}

/**
  * @brief Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will issue a NAK packet on any OUT packet received on
  *         USB endpoint until exiting this function. If you exit this function
  *         before transfer is complete on CDC interface (ie. using DMA controller)
  *         it will result in receiving more data while previous ones are still
  *         not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAILL
  */
static int8_t CDC_Receive_HS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 11 */
    // 定义变量
    uint8_t cmd_header = 0;
    uint8_t cmd = 0;
    uint8_t cdc_tx_buf[64] = {0};
    set_power_data_frame set_power_frame;
    BSP_STATUS status = BSP_ERROR;



    //打印收到的数据
    CDC_DEBUG("receive data len: %d\r\n", *Len);
#ifdef CDC_DEBUG_ENABLE
    for (uint32_t i = 0; i < *Len; i++) {
        printf("%02X ", Buf[i]);
        if ((i + 1) % 16 == 0) {
            printf("\r\n");
        }
    }
#endif
    //解析收到的数据
    cmd_header = Buf[0];
    cmd = Buf[1];

    //tx和rx头一样
    cdc_tx_buf[0] = cmd_header;
    cdc_tx_buf[1] = cmd;

    // 检查缓冲区长度是否足够
    if (*Len < 4) {
        *Len = 0;
        CDC_ERROR("receive data len: %d\r\n",*Len);
        return BSP_ERROR; // 错误：数据长度不足
    }

    // 解析header
    if (cmd_header != 0xA0) {
        *Len = 0;
        CDC_ERROR("receive data header: %x\r\n",cmd_header);
        return BSP_ERROR;// 错误：无效的header
    }
    switch(cmd){
      case GET_ID:
        cdc_tx_buf[2] = id;//1,2,3,4:bit1~4
        break;
      case GET_SW_VERSION:
        cdc_tx_buf[2] = id;

        memcpy(&cdc_tx_buf[3],sw_version,4);
        break;
      case VOL_SET:
        memcpy(&set_power_frame, Buf, sizeof(set_power_frame));
        CDC_DEBUG("set_power_frame.power_name:%x\r\n",set_power_frame.power_name);
        switch (set_power_frame.power_name)
        {
        case MCP4728_CHANNEL_A:
          ELVDD_DISABLE();
          CDC_DEBUG("Target Vol:%f\r\n",set_power_frame.value.float_value[0]);
          //校准数据
          set_power_frame.value.float_value[0] = (set_power_frame.value.float_value[0] - da_calibration_data.elvdd_set_offset)/(da_calibration_data.elvdd_set_gain); 
          dac_dev.val[0] = float_to_uint16_round(set_power_frame.value.float_value[0]);
          CDC_DEBUG("Vi:%f\r\n",set_power_frame.value.float_value[0]);
          status = bsp_dac_single_voltage_set(&dac_dev, 0, dac_dev.val[0], 0);
          HAL_GPIO_WritePin(LDAC_Port, LDAC_Pin, GPIO_PIN_RESET);
          ELVDD_ENABLE();
          break;
        case MCP4728_CHANNEL_B:
          ELVSS_DISABLE();
          CDC_DEBUG("Target Vol:%f\r\n",set_power_frame.value.float_value[0]);
          //校准数据
          set_power_frame.value.float_value[0] = (set_power_frame.value.float_value[0] - da_calibration_data.elvss_set_offset)/(da_calibration_data.elvss_set_gain); 
          dac_dev.val[1] = float_to_uint16_round(set_power_frame.value.float_value[0]);
          CDC_DEBUG("Vi:%f\r\n",set_power_frame.value.float_value[0]);
          status = bsp_dac_single_voltage_set(&dac_dev, 1, dac_dev.val[1], 0);
          HAL_GPIO_WritePin(LDAC_Port, LDAC_Pin, GPIO_PIN_RESET);
          ELVSS_ENABLE();
          break;
        case MCP4728_CHANNEL_C:
          VCC_DISABLE();
          CDC_DEBUG("Target Vol:%f\r\n",set_power_frame.value.float_value[0]);
          //校准数据
          set_power_frame.value.float_value[0] = (set_power_frame.value.float_value[0] - da_calibration_data.vcc_set_offset)/(da_calibration_data.vcc_set_gain); 
          dac_dev.val[2] = float_to_uint16_round(set_power_frame.value.float_value[0]);
          status = bsp_dac_single_voltage_set(&dac_dev, 2, dac_dev.val[2], 0);
          CDC_DEBUG("Vi:%f\r\n",set_power_frame.value.float_value[0]);
          HAL_GPIO_WritePin(LDAC_Port, LDAC_Pin, GPIO_PIN_RESET);
          VCC_ENABLE();
          break;
        case MCP4728_CHANNEL_D:
          IOVCC_DISABLE();
          CDC_DEBUG("Target Vol:%f\r\n",set_power_frame.value.float_value[0]);
          //校准数据
          set_power_frame.value.float_value[0] = (set_power_frame.value.float_value[0] - da_calibration_data.vcc_set_offset)/(da_calibration_data.vcc_set_gain); 
          dac_dev.val[3] = float_to_uint16_round(set_power_frame.value.float_value[0]);
          status = bsp_dac_single_voltage_set(&dac_dev, 3, dac_dev.val[3], 0);
          CDC_DEBUG("Vi:%f\r\n",set_power_frame.value.float_value[0]);
          HAL_GPIO_WritePin(LDAC_Port, LDAC_Pin, GPIO_PIN_RESET);
          IOVCC_ENABLE();
          break;
        default:
          CDC_ERROR("unknow power name\r\n");
          break;
        }
        cdc_tx_buf[2] = id;
        cdc_tx_buf[3] = status;
        break;
      case LIM_SET:
        break;
      case ALL_POWER_SET:
        if(Buf[2] == 0x01)  ELVDD_ENABLE();
        if(Buf[3] == 0x01)  ELVSS_ENABLE();
        if(Buf[4] == 0x01)  VCC_ENABLE();
        if(Buf[5] == 0x01)  IOVCC_ENABLE();
        if(Buf[2] == 0x0)  ELVDD_DISABLE();
        if(Buf[3] == 0x0)  ELVSS_DISABLE();
        if(Buf[4] == 0x0)  VCC_DISABLE();
        if(Buf[5] == 0x0)  IOVCC_DISABLE();
        cdc_tx_buf[2] = id;
        cdc_tx_buf[3] = BSP_OK;
        break;
      case SINGLE_POWER_SET:
        if(Buf[2]==MCP4728_CHANNEL_A)
        {
          if(Buf[3]) ELVDD_ENABLE();
          else ELVDD_DISABLE();
        }
        if(Buf[2]==MCP4728_CHANNEL_B)
        {
          if(Buf[3]) ELVSS_ENABLE();
          else ELVSS_DISABLE();
        }
        if(Buf[2]==MCP4728_CHANNEL_C)
        {
          if(Buf[3]) VCC_ENABLE();
          else VCC_DISABLE();
        }
        if(Buf[2]==MCP4728_CHANNEL_D)
        {
          if(Buf[3]) IOVCC_ENABLE();
          else IOVCC_DISABLE();
        }
        cdc_tx_buf[2] = id;
        cdc_tx_buf[3] = BSP_OK;
        break;
      case SINGLE_VOL_GET:
        if(Buf[2]==MCP4728_CHANNEL_A)
        {
          memcpy(&cdc_tx_buf[3],&latest_sample_data[AD_V_ELVDD],4);
        }
        if(Buf[2]==MCP4728_CHANNEL_B)
        {
          memcpy(&cdc_tx_buf[3],&latest_sample_data[AD_V_ELVSS],4);
        }
        if(Buf[2]==MCP4728_CHANNEL_C)
        {
          memcpy(&cdc_tx_buf[3],&latest_sample_data[AD_V_VCC],4);
        }
        if(Buf[2]==MCP4728_CHANNEL_D)
        {
          memcpy(&cdc_tx_buf[3],&latest_sample_data[AD_V_IOVCC],4);
        }
        cdc_tx_buf[2] = id;
        break;
      case SINGLE_CUR_GET:
        if(Buf[2]==MCP4728_CHANNEL_A)
        {
          memcpy(&cdc_tx_buf[3],&latest_sample_data[AD_I_ELVDD],4);
        }
        if(Buf[2]==MCP4728_CHANNEL_B)
        {
          memcpy(&cdc_tx_buf[3],&latest_sample_data[AD_I_ELVSS],4);
        }
        if(Buf[2]==MCP4728_CHANNEL_C)
        {
          memcpy(&cdc_tx_buf[3],&latest_sample_data[AD_I_VCC],4);
        }
        if(Buf[2]==MCP4728_CHANNEL_D)
        {
          memcpy(&cdc_tx_buf[3],&latest_sample_data[AD_I_IOVCC],4);
        }
        cdc_tx_buf[2] = id;
        break;
      default:
        CDC_DEBUG("unknow command\r\n");
        break;
  }
    //打印发送的数据
    CDC_DEBUG("send data len: %d\r\n", *Len);
#ifdef CDC_DEBUG_ENABLE
    for (uint32_t i = 0; i < *Len; i++) {
        printf("%02X ", cdc_tx_buf[i]);
        if ((i + 1) % 16 == 0) {
            printf("\r\n");
        }
    }
#endif
  //CDC_Transmit_HS(Buf, *Len);
  CDC_Transmit_HS(cdc_tx_buf, sizeof(cdc_tx_buf));
  /*作用：设置 USB CDC 接收缓冲区地址，为下一次接收数据做准备。
  说明：每次接收完成后都要重新设置接收缓冲区，否则无法继续接收新的数据。*/
  USBD_CDC_SetRxBuffer(&hUsbDeviceHS, &Buf[0]);
  /*作用：使能 USB CDC 接收功能，准备接收主机发来的下一包数据。
  说明：这一步是 CDC 协议的标准流程，必须调用，否则不会继续接收数据。*/
  USBD_CDC_ReceivePacket(&hUsbDeviceHS);
  return (USBD_OK);
  /* USER CODE END 11 */
}

/**
  * @brief  Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_HS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 12 */
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceHS.pClassData;
  if (hcdc->TxState != 0){
    return USBD_BUSY;
  }

  USBD_CDC_SetTxBuffer(&hUsbDeviceHS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceHS);
  /* USER CODE END 12 */
  return result;
}

/**
  * @brief  CDC_TransmitCplt_HS
  *         Data transmitted callback
  *
  *         @note
  *         This function is IN transfer complete callback used to inform user that
  *         the submitted Data is successfully sent over USB.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_TransmitCplt_HS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 14 */
  UNUSED(Buf);
  UNUSED(Len);
  UNUSED(epnum);
  /* USER CODE END 14 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */
