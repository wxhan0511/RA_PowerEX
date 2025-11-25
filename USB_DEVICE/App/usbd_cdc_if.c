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
#include "main.h"               //Io defination and debug macro
#include "bsp.h"                //import version
#include "cdc_task.h"           //CDC private type define
#include "stm32f4xx_hal_gpio.h" //addr recon
#include "bsp_mcp4728.h"
#include "bsp_ads1256.h"
#include "bsp_power.h"       //powen en or disen
#include "bsp_calibration.h" //calibration data
#include "calibration_utils.h"
#include "usbd_composite.h"
/*ra xb*/
#include "drv_ra_device.h"
/* USER CODE BEGIN INCLUDE */

/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t cmd_header = 0;
uint8_t cmd = 0;
uint8_t cdc_tx_buf[64] = {0};
set_power_data_frame set_power_frame;
BSP_STATUS status = BSP_ERROR;
uint8_t state = 0;
uint8_t temp = 0;
int16_t temp_data;
double temp_double_data;
float temp_float_data;
float_bytes_t float_bytes;
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
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
 * @}
 */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
 * @brief Public variables.
 * @{
 */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
 * @}
 */

/** @defgroup USBD_CDC_IF_Private_FunctionPrototypes USBD_CDC_IF_Private_FunctionPrototypes
 * @brief Private functions declaration.
 * @{
 */

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t *pbuf, uint32_t *Len);
static int8_t CDC_TransmitCplt_FS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
 * @}
 */

USBD_CDC_ItfTypeDef USBD_CDC_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS,
  CDC_TransmitCplt_FS
};

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Initializes the CDC media low layer over the USB FS IP
 * @retval USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t CDC_Init_FS(void)
{
  /* USER CODE BEGIN 8 */
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return (USBD_OK);
  /* USER CODE END 8 */
}

/**
 * @brief  DeInitializes the CDC media low layer
 * @param  None
 * @retval USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t CDC_DeInit_FS(void)
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
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length)
{
  /* USER CODE BEGIN 10 */
  switch (cmd)
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
static int8_t CDC_Receive_FS(uint8_t *Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 11 */
  // 定义变量

  memset(cdc_tx_buf, 0, sizeof(cdc_tx_buf));
  // 打印收到的数据
  CDC_DEBUG("receive data len: %d\r\n", *Len);
#ifdef CDC_DEBUG_ENABLE
  for (uint32_t i = 0; i < *Len; i++)
  {
    printf("%02X ", Buf[i]);
    if ((i + 1) % 16 == 0)
    {
      printf("\r\n");
    }
  }
#endif
  // 解析收到的数据
  cmd_header = Buf[0];
  cmd = Buf[1];

  // tx和rx头一样
  cdc_tx_buf[0] = cmd_header;
  cdc_tx_buf[1] = cmd;

  // 检查缓冲区长度是否足够
  if (*Len < 4)
  {
    *Len = 0;
    CDC_ERROR("receive data len: %d\r\n", *Len);
    return BSP_ERROR; // 错误：数据长度不足
  }

  // 解析header
  if (cmd_header != 0xA0)
  {
    *Len = 0;
    CDC_ERROR("receive data header: %x\r\n", cmd_header);
    return BSP_ERROR; // 错误：无效的header
  }

  state = BSP_OK;

  switch (cmd)
  {
  case GET_ID:
    cdc_tx_buf[2] = id; // 1,2,3,4:bit1~4
    break;
  case GET_SW_VERSION:
    cdc_tx_buf[2] = id;
    memcpy(&cdc_tx_buf[3], sw_version, 4);
    break;
  case VOL_SET:

    memcpy(&set_power_frame, Buf, sizeof(set_power_frame));
    CDC_DEBUG("set_power_frame.power_name:%x\r\n", set_power_frame.power_name);
    switch (set_power_frame.power_name)
    {
    case MCP4728_CHANNEL_A:
      CDC_DEBUG("Setting ELVDD Voltage...\r\n");
      ELVDD_DISABLE();
      CDC_DEBUG("Target Vol:%f\r\n", set_power_frame.value.float_value[0]);
      //保存最新电压值vi
      g_calibration_manager.data.elvdd_last_voltage = set_power_frame.value.float_value[0];
      calibration_save();
      // 校准数据
      set_power_frame.value.float_value[0] = (set_power_frame.value.float_value[0] - da_calibration_data.elvdd_set_offset) / (da_calibration_data.elvdd_set_gain);
      dac_dev.val[0] = float_to_uint16_round(set_power_frame.value.float_value[0]);
      CDC_DEBUG("Vi:%f\r\n", set_power_frame.value.float_value[0]);
      status = bsp_dac_single_voltage_set(&dac_dev, 0, dac_dev.val[0], 0);
      if (status != BSP_OK)
      {
        CDC_DEBUG("ELVDD set voltage failed\r\n");
        state = 1;
      }
      HAL_GPIO_WritePin(LDAC_Port, LDAC_Pin, GPIO_PIN_RESET);
      ELVDD_ENABLE();
      CDC_DEBUG("ELVDD set vol done\r\n");
      break;
    case MCP4728_CHANNEL_B:
      CDC_DEBUG("Setting ELVSS Voltage...\r\n");
      ELVSS_DISABLE();
      CDC_DEBUG("Target Vol:%f\r\n", set_power_frame.value.float_value[0]);
      //保存最新电压值
      g_calibration_manager.data.elvss_last_voltage = set_power_frame.value.float_value[0];
      calibration_save();
      // 校准数据    g_calibration_manager.data.elvss_last_voltage = (-g_calibration_manager.data.elvss_last_voltage + da_calibration_data.elvss_set_offset) / (da_calibration_data.elvss_set_gain);

      set_power_frame.value.float_value[0] = (-set_power_frame.value.float_value[0] + da_calibration_data.elvss_set_offset) / (da_calibration_data.elvss_set_gain);
      dac_dev.val[1] = float_to_uint16_round(set_power_frame.value.float_value[0]);
      CDC_DEBUG("Vi:%f\r\n", set_power_frame.value.float_value[0]);
      status = bsp_dac_single_voltage_set(&dac_dev, 1, dac_dev.val[1], 0);

      if (status != BSP_OK)
      {
        CDC_DEBUG("ELVSS set voltage failed\r\n");
        state = 1;
      }
      HAL_GPIO_WritePin(LDAC_Port, LDAC_Pin, GPIO_PIN_RESET);
      CDC_DEBUG("ELVSS set vol done\r\n");
      ELVSS_ENABLE();
      break;
    case MCP4728_CHANNEL_C:
      CDC_DEBUG("Setting VCC Voltage...\r\n");
      VCC_DISABLE();
      CDC_DEBUG("Target Vol:%f\r\n", set_power_frame.value.float_value[0]);
      //保存最新电压值
      g_calibration_manager.data.vcc_last_voltage = set_power_frame.value.float_value[0];
      calibration_save();
      // 校准数据
      set_power_frame.value.float_value[0] = (set_power_frame.value.float_value[0] - da_calibration_data.vcc_set_offset) / (da_calibration_data.vcc_set_gain);
      dac_dev.val[2] = float_to_uint16_round(set_power_frame.value.float_value[0]);
      status = bsp_dac_single_voltage_set(&dac_dev, 2, dac_dev.val[2], 0);
      if (status != BSP_OK)
      {
        CDC_DEBUG("VCC set voltage failed\r\n");
        state = 1;
      }
      CDC_DEBUG("Vi:%f\r\n", set_power_frame.value.float_value[0]);
      HAL_GPIO_WritePin(LDAC_Port, LDAC_Pin, GPIO_PIN_RESET);
      VCC_ENABLE();
      CDC_DEBUG("VCC set vol done\r\n");
      break;
    case MCP4728_CHANNEL_D:
      CDC_DEBUG("Setting IOVCC Voltage...\r\n");
      IOVCC_DISABLE();
      CDC_DEBUG("Target Vol:%f\r\n", set_power_frame.value.float_value[0]);

      //保存最新电压值
      g_calibration_manager.data.iovcc_last_voltage = set_power_frame.value.float_value[0];
      calibration_save();
      // 校准数据
      set_power_frame.value.float_value[0] = (set_power_frame.value.float_value[0] - da_calibration_data.vcc_set_offset) / (da_calibration_data.vcc_set_gain);
      dac_dev.val[3] = float_to_uint16_round(set_power_frame.value.float_value[0]);
      status = bsp_dac_single_voltage_set(&dac_dev, 3, dac_dev.val[3], 0);

      if (status != BSP_OK)
      {
        CDC_DEBUG("IOVCC set voltage failed\r\n");
        state = 1;
      }
      CDC_DEBUG("Vi:%f\r\n", set_power_frame.value.float_value[0]);
      HAL_GPIO_WritePin(LDAC_Port, LDAC_Pin, GPIO_PIN_RESET);
      IOVCC_ENABLE();
      CDC_DEBUG("IOVCC set vol done\r\n");
      break;
    case 4:
      CDC_DEBUG("Setting IOVCCxb Voltage...\r\n");
      CDC_DEBUG("Target Vol:%f\r\n", set_power_frame.value.float_value[0]);
      temp = float_to_uint8_round(set_power_frame.value.float_value[0] / 100);
      status = ra_dev_main_0.ops->set_power_vol(ra_dev_main_0.dev, RA_POWER_IOVCC, temp);
      //保存最新电压值
      g_calibration_manager.data.xb_iovcc_last_voltage = set_power_frame.value.float_value[0];
      calibration_save();
      if (status != BSP_OK)
      {
        CDC_DEBUG("ra xb IOVCC set power failed\r\n");
        state = 1;
      }
      ra_dev_main_0.dev->lp3907->read(RA_LP3907_1_ADDRESS, 0x10, &temp);
      CDC_DEBUG("raxb IOVCC read reg done:%x\r\n", temp);
      ra_dev_main_0.dev->lp3907->write(RA_LP3907_1_ADDRESS, 0x10, temp | (1 << 4));
      CDC_DEBUG("ra xb IOVCC write reg done:%x\r\n", temp);
      CDC_DEBUG("ra xb IOVCC set vol done\r\n");
      break;
    case 5:
      CDC_DEBUG("Setting VCIxb Voltage...\r\n");
      CDC_DEBUG("Target Vol:%f\r\n", set_power_frame.value.float_value[0]);
      temp = float_to_uint8_round(set_power_frame.value.float_value[0] / 100);
      // osDelay(10);
      status = ra_dev_main_0.ops->set_power_vol(ra_dev_main_0.dev, RA_POWER_VCI, temp);
      //保存最新电压值
      g_calibration_manager.data.vci_last_voltage = set_power_frame.value.float_value[0];
      calibration_save();
      if (status != BSP_OK)
      {
        CDC_DEBUG("VCI set power failed\r\n");
        state = 1;
      }
      ra_dev_main_0.dev->lp3907->read(RA_LP3907_1_ADDRESS, 0x10, &temp);
      CDC_DEBUG("VCI read reg done:%x\r\n", temp);
      ra_dev_main_0.dev->lp3907->write(RA_LP3907_1_ADDRESS, 0x10, temp | (1 << 6));
      CDC_DEBUG("VCI write reg done:%x\r\n", temp);
      CDC_DEBUG("VCI set vol done\r\n");
      break;
    case 6:
      CDC_DEBUG("Setting VSPxb Voltage...\r\n");
      CDC_DEBUG("Target Vol:%f\r\n", set_power_frame.value.float_value[0]);
      temp = float_to_uint8_round(set_power_frame.value.float_value[0] / 100);
      // osDelay(10);
      status = ra_dev_main_0.ops->set_power_vol(ra_dev_main_0.dev, RA_POWER_VSP, temp);
      //保存最新电压值
      g_calibration_manager.data.vsp_last_voltage = set_power_frame.value.float_value[0];
      calibration_save();
      if (status != BSP_OK)
      {
        CDC_DEBUG("VSP set power failed\r\n");
        state = 1;
      }
      ra_dev_main_0.ops->set_power_en(ra_dev_main_0.dev, RA_POWER_VSP, 1);
      CDC_DEBUG("VSP set vol done\r\n");
      break;
    case 7:
      CDC_DEBUG("Setting VSNxb Voltage...\r\n");
      CDC_DEBUG("Target Vol:%f\r\n", set_power_frame.value.float_value[0]);
      temp = float_to_uint8_round(set_power_frame.value.float_value[0] / 100);
      // osDelay(10);
      status = ra_dev_main_0.ops->set_power_vol(ra_dev_main_0.dev, RA_POWER_VSN, temp);
      //保存最新电压值
      g_calibration_manager.data.vsn_last_voltage = set_power_frame.value.float_value[0];
      calibration_save();
      if (status != BSP_OK)
      {
        CDC_DEBUG("VSN set power failed\r\n");
        state = 1;
      }
      ra_dev_main_0.ops->set_power_en(ra_dev_main_0.dev, RA_POWER_VSN, 1);
      CDC_DEBUG("VSN set vol done\r\n");
      break;
    default:
      CDC_ERROR("unknow power name\r\n");
      break;
    }
    cdc_tx_buf[2] = id;
    cdc_tx_buf[3] = state;
    break;
  case LIM_SET:
    if (Buf[2] == MCP4728_CHANNEL_A)
    {
      memcpy(&float_bytes, &Buf[3], sizeof(float_bytes));
      CDC_DEBUG("Setting ELVDD Current Limit...\r\n");
      CDC_DEBUG("Target Lim:%f\r\n", float_bytes.f);
      CDC_DEBUG("Latest ELVDD Current:%f\r\n", latest_sample_data[AD_I_ELVDD]);
      if (latest_sample_data[AD_I_ELVDD] > float_bytes.f)
      {
        ELVDD_DISABLE();
        CDC_DEBUG("ELVDD DISABLE\r\n");
      }
    }
    if (Buf[2] == MCP4728_CHANNEL_B)
    {
      memcpy(&float_bytes, &Buf[3], sizeof(float_bytes));
      CDC_DEBUG("Setting ELVSS Current Limit...\r\n");
      CDC_DEBUG("Target Lim:%f\r\n", float_bytes.f);
      CDC_DEBUG("Latest ELVSS Current:%f\r\n", latest_sample_data[AD_I_ELVSS]);
      if (latest_sample_data[AD_I_ELVSS] > float_bytes.f)
      {
        ELVSS_DISABLE();
        CDC_DEBUG("ELVSS DISABLE\r\n");
      }
    }
    if (Buf[2] == MCP4728_CHANNEL_C)
    {
      memcpy(&float_bytes, &Buf[3], sizeof(float_bytes));
      CDC_DEBUG("Setting VCC Current Limit...\r\n");
      CDC_DEBUG("Target Lim:%f\r\n", float_bytes.f);
      CDC_DEBUG("Latest VCC Current:%f\r\n", latest_sample_data[AD_I_VCC]);
      if (latest_sample_data[AD_I_VCC] > float_bytes.f)
      {
        VCC_DISABLE();
        CDC_DEBUG("VCC DISABLE\r\n");
      }
    }
    if (Buf[2] == MCP4728_CHANNEL_D)
    {
      memcpy(&float_bytes, &Buf[3], sizeof(float_bytes));
      CDC_DEBUG("Setting IOVCC Current Limit...\r\n");
      CDC_DEBUG("Target Lim:%f\r\n", float_bytes.f);
      CDC_DEBUG("Latest IOVCC Current:%f\r\n", latest_sample_data[AD_I_IOVCC]);
      if (latest_sample_data[AD_I_IOVCC] > float_bytes.f)
      {
        IOVCC_DISABLE();
        CDC_DEBUG("IOVCC DISABLE\r\n");
      }
    }
    if (Buf[2] == 4) // IOVCC
    {
      memcpy(&float_bytes, &Buf[3], sizeof(float_bytes));
      CDC_DEBUG("Setting RA XB IOVCC Current Limit...\r\n");
      CDC_DEBUG("Target Lim:%f\r\n", float_bytes.f);
      status = ra_dev_main_0.dev->ina3221->read(RA_INA3221_ADDRESS, 0x03, &temp_data);
      if (status == BSP_ERROR)
      {
        CDC_DEBUG("ra xb iovcc cur read error\r\n");
        state = BSP_ERROR;
      }
      else
      {
        temp_double_data = (temp_data >> 3) * 0.040 / (RA_INA3221_resistor);
        temp_float_data = abs((float)temp_double_data);
        CDC_DEBUG("Latest RA XB IOVCC Current:%f\r\n", temp_float_data);
        if (temp_float_data > float_bytes.f)
        {
          ra_dev_main_0.dev->lp3907->write(RA_LP3907_1_ADDRESS, 0x10, 0x25);
          CDC_DEBUG("RA XB IOVCC DISABLE\r\n");
        }
      }
    }
    if (Buf[2] == 5) // VCi
    {
      memcpy(&float_bytes, &Buf[3], sizeof(float_bytes));
      CDC_DEBUG("Setting RA XB VCI Current Limit...\r\n");
      CDC_DEBUG("Target Lim:%f\r\n", float_bytes.f);
      status = ra_dev_main_0.dev->ina3221->read(RA_INA3221_ADDRESS, 0x01, &temp_data);
      if (status == BSP_ERROR)
      {
        CDC_DEBUG("ra xb vci cur read error\r\n");
        state = BSP_ERROR;
      }
      else
      {
        temp_double_data = (temp_data >> 3) * 0.040 / (RA_INA3221_resistor);
        temp_float_data = (float)temp_double_data;
        CDC_DEBUG("Latest RA XB VCI Current:%f\r\n", temp_float_data);
        if (temp_float_data > float_bytes.f)
        {
          ra_dev_main_0.dev->lp3907->write(RA_LP3907_1_ADDRESS, 0x10, 0x25);
          CDC_DEBUG("RA XB VCI DISABLE\r\n");
        }
      }
    }
    if (Buf[2] == 6) // VSP
    {
      memcpy(&float_bytes, &Buf[3], sizeof(float_bytes));
      CDC_DEBUG("Setting RA XB VSP Current Limit...\r\n");
      CDC_DEBUG("Target Lim:%f\r\n", float_bytes.f);
      status = ra_dev_main_0.dev->ina3221->read(RA_INA3221_ADDRESS, 0x05, &temp_data);
      if (status == BSP_ERROR)
      {
        CDC_DEBUG("ra xb vsp cur read error\r\n");
        state = BSP_ERROR;

      }
      else
      {
        temp_double_data = (temp_data >> 3) * 0.040 / (RA_INA3221_resistor);
        temp_float_data = (float)temp_double_data;
        CDC_DEBUG("Latest RA XB VSP Current:%f\r\n", temp_float_data);
        if (temp_float_data > float_bytes.f)
        {
          ra_dev_main_0.ops->set_power_en(ra_dev_main_0.dev, RA_POWER_VSP, 0);
          CDC_DEBUG("RA XB VSP DISABLE\r\n");
        }
      }
    }
    if (Buf[2] == 7) // VSN
    {
      memcpy(&float_bytes, &Buf[3], sizeof(float_bytes));
      CDC_DEBUG("Setting RA XB VSN Current Limit...\r\n");
      CDC_DEBUG("Target Lim:%f\r\n", float_bytes.f);
      status = ra_dev_main_0.dev->adc121c027->read(RA_ADC121C027_ADDRESS, 0x00, &temp_data);
      if (status == BSP_ERROR)
      {
        CDC_DEBUG("ra xb vsn cur read error\r\n");
        state = BSP_ERROR;
      }
      else
      {
        temp_double_data = (((uint16_t)temp_data) & 0x0fff) * 8.05664 / (RA_ADC121_resistor) / 100;
        temp_float_data = (float)temp_double_data;
        CDC_DEBUG("Latest RA XB VSN Current:%f\r\n", temp_float_data);
        if (temp_float_data > float_bytes.f)
        {
          ra_dev_main_0.ops->set_power_en(ra_dev_main_0.dev, RA_POWER_VSN, 0);
          CDC_DEBUG("RA XB VSN DISABLE\r\n");
        }
      }
    }
    cdc_tx_buf[2] = id;
    cdc_tx_buf[3] = state;
    break;
  case ALL_POWER_EN:
    // 01使能，0不使能

    if (Buf[2] == 0x01)
      ELVDD_ENABLE();
    else if (Buf[2] == 0x0)
      ELVDD_DISABLE();
    if (Buf[3] == 0x01)
      ELVSS_ENABLE();
    else if (Buf[3] == 0x0)
      ELVSS_DISABLE();
    if (Buf[4] == 0x01)
      VCC_ENABLE();
    else if (Buf[4] == 0x0)
      VCC_DISABLE();
    if (Buf[5] == 0x01)
      IOVCC_ENABLE();
    else if (Buf[5] == 0x0)
      IOVCC_DISABLE();
    if (Buf[6] == 0x1) // IOVCC
    {
      HAL_I2C_Mem_Read(&hi2c1, RA_LP3907_1_ADDRESS, 0x10, I2C_MEMADD_SIZE_8BIT, &temp, 1, 100);
      CDC_DEBUG("IOVCC temp:%x\r\n", temp);
      ra_dev_main_0.dev->lp3907->write(RA_LP3907_1_ADDRESS, 0x10, temp | (1 << 4));
      CDC_DEBUG("IOVCC temp:%x\r\n", temp);
      // ra_dev_main_0.dev->lp3907->write(RA_LP3907_2_ADDRESS,0x10,25);
    }
    else if (Buf[6] == 0x0)
    {
      HAL_I2C_Mem_Read(&hi2c1, RA_LP3907_1_ADDRESS, 0x10, I2C_MEMADD_SIZE_8BIT, &temp, 1, 100);
      CDC_DEBUG("temp:%x\r\n", temp);
      ra_dev_main_0.dev->lp3907->write(RA_LP3907_1_ADDRESS, 0x10, temp & (~(1 << 4)));
      // 打印寄存器值
      CDC_DEBUG("temp:%x\r\n", temp);
    }
    if (Buf[7] == 0x1) // VCI
    {
      HAL_I2C_Mem_Read(&hi2c1, RA_LP3907_1_ADDRESS, 0x10, I2C_MEMADD_SIZE_8BIT, &temp, 1, 100);
      CDC_DEBUG("temp:%x\r\n", temp);
      ra_dev_main_0.dev->lp3907->write(RA_LP3907_1_ADDRESS, 0x10, temp | (1 << 6));
      CDC_DEBUG("temp:%x\r\n", temp);
    }
    else if (Buf[7] == 0x0)
    {
      HAL_I2C_Mem_Read(&hi2c1, RA_LP3907_1_ADDRESS, 0x10, I2C_MEMADD_SIZE_8BIT, &temp, 1, 100);
      printf("temp:%x\r\n", temp);
      ra_dev_main_0.dev->lp3907->write(RA_LP3907_1_ADDRESS, 0x10, temp & (~(1 << 6)));
      printf("temp:%x\r\n", temp);
    }
    if (Buf[8] == 0x1) // VSP
    {
      ra_dev_main_0.ops->set_power_en(ra_dev_main_0.dev, RA_POWER_VSP, 1);
    }
    else if (Buf[8] == 0x0)
      ra_dev_main_0.ops->set_power_en(ra_dev_main_0.dev, RA_POWER_VSP, 0);
    // VSN
    if (Buf[9] == 0x1)
      ra_dev_main_0.ops->set_power_en(ra_dev_main_0.dev, RA_POWER_VSN, 1);
    else if (Buf[9] == 0x0)
      ra_dev_main_0.ops->set_power_en(ra_dev_main_0.dev, RA_POWER_VSN, 0);
    cdc_tx_buf[2] = id;
    cdc_tx_buf[3] = BSP_OK;
    break;
  case SINGLE_POWER_EN:
    if (Buf[2] == MCP4728_CHANNEL_A)
    {
      if (Buf[3])
      {
        ELVDD_ENABLE();
        CDC_DEBUG("ELVDD ENABLE\r\n");
      }
      else
      {
        ELVDD_DISABLE();
        CDC_DEBUG("ELVDD DISABLE\r\n");
      }
    }
    if (Buf[2] == MCP4728_CHANNEL_B)
    {
      if (Buf[3])
      {
        ELVSS_ENABLE();
        CDC_DEBUG("ELVSS ENABLE\r\n");
      }
      else
      {
        ELVSS_DISABLE();
        CDC_DEBUG("ELVSS DISABLE\r\n");
      }
    }
    if (Buf[2] == MCP4728_CHANNEL_C)
    {
      if (Buf[3])
        VCC_ENABLE();
      else
        VCC_DISABLE();
    }
    if (Buf[2] == MCP4728_CHANNEL_D)
    {
      if (Buf[3])
        IOVCC_ENABLE();
      else
        IOVCC_DISABLE();
    }
    if (Buf[2] == 4) // RA XB IOVCC
    {
      if (Buf[3])
      {
        // HAL_I2C_Mem_Read(&hi2c1, RA_LP3907_1_ADDRESS, 0x10, I2C_MEMADD_SIZE_8BIT, &temp, 1, 100);
        // printf("temp:%x\r\n",temp);
        // ra_dev_main_0.dev->lp3907->write(RA_LP3907_1_ADDRESS,0x10,temp|(1<<4));
        // printf("temp:%x\r\n",temp);
        ra_dev_main_0.dev->lp3907->write(RA_LP3907_1_ADDRESS, 0x10, 0x75);
      }
      else
      {
        // HAL_I2C_Mem_Read(&hi2c1, RA_LP3907_1_ADDRESS, 0x10, I2C_MEMADD_SIZE_8BIT, &temp, 1, 100);
        // printf("temp:%x\r\n",temp);
        // ra_dev_main_0.dev->lp3907->write(RA_LP3907_1_ADDRESS,0x10,temp&(~(1<<4)));
        // printf("temp:%x\r\n",temp);
        ra_dev_main_0.dev->lp3907->write(RA_LP3907_1_ADDRESS, 0x10, 0x25);
      }
    }
    if (Buf[2] == 5) // VCI
    {
      if (Buf[3])
      {
        // ra_dev_main_0.dev->lp3907->read(RA_LP3907_1_ADDRESS,0x10,&temp);
        // printf("temp:%x\r\n",temp);
        // ra_dev_main_0.dev->lp3907->write(RA_LP3907_1_ADDRESS,0x10,temp|(1<<6));
        // printf("temp:%x\r\n",temp);
        ra_dev_main_0.dev->lp3907->write(RA_LP3907_1_ADDRESS, 0x10, 0x75);
      }
      else
      {
        // ra_dev_main_0.dev->lp3907->read(RA_LP3907_1_ADDRESS,0x10,&temp);
        // printf("temp:%x\r\n",temp);
        // ra_dev_main_0.dev->lp3907->write(RA_LP3907_1_ADDRESS,0x10,temp&(~(1<<6)));
        // printf("temp:%x\r\n",temp);
        ra_dev_main_0.dev->lp3907->write(RA_LP3907_1_ADDRESS, 0x10, 0x25);
      }
    }
    if (Buf[2] == 6) // VSP
    {
      if (Buf[3])
        ra_dev_main_0.ops->set_power_en(ra_dev_main_0.dev, RA_POWER_VSP, 1);
      else
        ra_dev_main_0.ops->set_power_en(ra_dev_main_0.dev, RA_POWER_VSP, 0);
    }
    if (Buf[2] == 7) // VSN
    {
      if (Buf[3])
        ra_dev_main_0.ops->set_power_en(ra_dev_main_0.dev, RA_POWER_VSN, 1);
      else
        ra_dev_main_0.ops->set_power_en(ra_dev_main_0.dev, RA_POWER_VSN, 0);
    }
    cdc_tx_buf[2] = id;
    cdc_tx_buf[3] = BSP_OK;
    break;
  case SINGLE_VOL_GET:
    if (Buf[2] == MCP4728_CHANNEL_A)
    {
      memcpy(&cdc_tx_buf[3], &latest_sample_data[AD_V_ELVDD], 4);
    }
    if (Buf[2] == MCP4728_CHANNEL_B)
    {
      memcpy(&cdc_tx_buf[3], &latest_sample_data[AD_V_ELVSS], 4);
    }
    if (Buf[2] == MCP4728_CHANNEL_C)
    {
      memcpy(&cdc_tx_buf[3], &latest_sample_data[AD_V_VCC], 4);
    }
    if (Buf[2] == MCP4728_CHANNEL_D)
    {
      memcpy(&cdc_tx_buf[3], &latest_sample_data[AD_V_IOVCC], 4);
    }
    if (Buf[2] == 4) // IOVCC
    {
      status = ra_dev_main_0.dev->ina3221->read(RA_INA3221_ADDRESS, 0x04, &temp_data);
      if (status == BSP_ERROR)
      {
        CDC_DEBUG("ina3221 read error\r\n");
      }
      else
      {
        temp_float_data = (float)temp_data;
        memcpy(&cdc_tx_buf[3], &temp_float_data, 4);
      }
    }
    if (Buf[2] == 5) // VCI
    {
      status = ra_dev_main_0.dev->ina3221->read(RA_INA3221_ADDRESS, 0x02, &temp_data);
      if (status == BSP_ERROR)
      {
        state = 1;
      }
      else
      {
        temp_float_data = (float)temp_data;
        memcpy(&cdc_tx_buf[3], &temp_float_data, 4);
      }
    }
    if (Buf[2] == 6) // VSP
    {
      status = ra_dev_main_0.dev->ina3221->read(RA_INA3221_ADDRESS, 0x06, &temp_data);
      if (status == BSP_ERROR)
      {
        state = 1;
      }
      else
      {
        temp_float_data = (float)temp_data;
        memcpy(&cdc_tx_buf[3], &temp_float_data, 4);
      }
    }
    if (Buf[2] == 7) // VSN
    {
    }
    cdc_tx_buf[2] = id;
    break;
  case SINGLE_CUR_GET:
    if (Buf[2] == MCP4728_CHANNEL_A)
    {
      memcpy(&cdc_tx_buf[3], &latest_sample_data[AD_I_ELVDD], 4);
      if(latest_sample_data[AD_I_ELVDD]==0.0f)
      {
        printf("AD_I_ELVDD cur too low,%f\r\n",latest_sample_data[AD_I_ELVDD]);
      }
    }
    if (Buf[2] == MCP4728_CHANNEL_B)
    {
      memcpy(&cdc_tx_buf[3], &latest_sample_data[AD_I_ELVSS], 4);
      if(latest_sample_data[AD_I_ELVSS]==0.0f)
      {
        printf("AD_I_ELVSS cur too low,%f\r\n",latest_sample_data[AD_I_ELVSS]);
      }
    }
    if (Buf[2] == MCP4728_CHANNEL_C)
    {
      memcpy(&cdc_tx_buf[3], &latest_sample_data[AD_I_VCC], 4);
      if(latest_sample_data[AD_I_VCC]==0.0f)
      {
        printf("AD_I_VCC cur too low,%f\r\n",latest_sample_data[AD_I_VCC]);
      }
    }
    if (Buf[2] == MCP4728_CHANNEL_D)
    {
      memcpy(&cdc_tx_buf[3], &latest_sample_data[AD_I_IOVCC], 4);
      if(latest_sample_data[AD_I_IOVCC]==0.0f)
      {
        printf("AD_I_IOVCC cur too low,%f\r\n",latest_sample_data[AD_I_IOVCC]);
      }
    }
    if (Buf[2] == 4) // IOVCC
    {
      status = ra_dev_main_0.dev->ina3221->read(RA_INA3221_ADDRESS, 0x03, &temp_data);
      // printf("ra vsp read status %d \r\n",status);
      if (status == BSP_ERROR)
      {
        state = BSP_ERROR;
      }
      else
      {
        temp_double_data = (temp_data >> 3) * 0.040 / (RA_INA3221_resistor);
        temp_float_data = abs((float)temp_double_data);
        memcpy(&cdc_tx_buf[3], &temp_float_data, 4);
        if(temp_float_data<2.0f)
        {
          //printf("RA XB IOVCC cur too low,%f\r\n",temp_float_data);
        }
      }
    }
    if (Buf[2] == 5) // VCi
    {
      status = ra_dev_main_0.dev->ina3221->read(RA_INA3221_ADDRESS, 0x01, &temp_data);
      // printf("ra vsp read status %d \r\n",status);
      if (status == BSP_ERROR)
      {
        state = BSP_ERROR;
      }
      else
      {
        temp_double_data = (temp_data >> 3) * 0.040 / (RA_INA3221_resistor);
        temp_float_data = (float)temp_double_data;
        memcpy(&cdc_tx_buf[3], &temp_float_data, 4);
        if(temp_float_data<2.0f)
        {
          //printf("RA XB VCI cur too low,%f\r\n",temp_float_data);
        }
      }
    }
    if (Buf[2] == 6) // VSP
    {
      status = ra_dev_main_0.dev->ina3221->read(RA_INA3221_ADDRESS, 0x05, &temp_data);
      // printf("ra vsp read status %d \r\n",status);
      if (status == BSP_ERROR)
      {
        state = BSP_ERROR;
      }
      else
      {
        temp_double_data = (temp_data >> 3) * 0.040 / (RA_INA3221_resistor);
        temp_float_data = (float)temp_double_data;
        memcpy(&cdc_tx_buf[3], &temp_float_data, 4);
        if(temp_float_data<2.0f)
        {
          //printf("RA XB VSP cur too low,%f\r\n",temp_float_data);
        }
      }
    }
    if (Buf[2] == 7) // VSN
    {
      status = ra_dev_main_0.dev->adc121c027->read(RA_ADC121C027_ADDRESS, 0x00, &temp_data);
      if (status == BSP_ERROR)
      {
        state = BSP_ERROR;
      }
      else
      {
        temp_double_data = (((uint16_t)temp_data) & 0x0fff) * 8.05664 / (RA_ADC121_resistor) / 100;
        temp_float_data = (float)temp_double_data;
        memcpy(&cdc_tx_buf[3], &temp_float_data, 4);
        if(temp_float_data<2.0f)
        {
          //printf("RA XB VSN cur too low,%f\r\n",temp_float_data);
        }
      }
    }
    cdc_tx_buf[2] = id;
    break;
  default:
    CDC_DEBUG("unknow command\r\n");
    break;
  }
  // 打印发送的数据
  CDC_DEBUG("send data len: %d\r\n", *Len);
#ifdef CDC_DEBUG_ENABLE
  for (uint32_t i = 0; i < *Len; i++)
  {
    printf("%02X ", cdc_tx_buf[i]);
    if ((i + 1) % 16 == 0)
    {
      printf("\r\n");
    }
  }
#endif
  // CDC_Transmit_HS(Buf, *Len);
  CDC_Transmit_FS(cdc_tx_buf, sizeof(cdc_tx_buf));
  /*作用：设置 USB CDC 接收缓冲区地址，为下一次接收数据做准备。
  说明：每次接收完成后都要重新设置接收缓冲区，否则无法继续接收新的数据。*/
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  /*作用：使能 USB CDC 接收功能，准备接收主机发来的下一包数据。
  说明：这一步是 CDC 协议的标准流程，必须调用，否则不会继续接收数据。*/
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
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
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)pCDCData;//(USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc->TxState != 0){
    return USBD_BUSY;
  }
  hUsbDeviceFS.pClassData=pCDCData;
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  /* USER CODE END 7 */
  return result;
}

/**
 * @brief  CDC_TransmitCplt_FS
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
static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
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
