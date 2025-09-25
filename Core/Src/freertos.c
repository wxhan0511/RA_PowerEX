/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "semphr.h"
#include "timers.h"

/* BSP includes */
#include "bsp.h"
#include "bsp_led.h"
#include "bsp_ads1256.h"
#include "bsp_mcp4728.h"
#include "bsp_calibration.h"
#include "bsp_power.h"
#include "oled.h"

/* HAL includes */
#include "i2c.h"
#include "usart.h"
#include "stm32f4xx_it.h"

/* Utility includes */
#include "delay.h"
#include "cmsis_gcc.h"

power_data_frame power_frame;
set_power_data_frame set_power_frame;

osSemaphoreId_t i2cSemaphore;

#ifdef I2C_SLAVE
osThreadId_t slaveTxTaskHandle = NULL;
osThreadId_t slaveRxTaskHandle = NULL;
#endif

#ifdef I2C_MASTER
osThreadId_t masterTxTaskHandle = NULL;
osThreadId_t masterRxTaskHandle = NULL;
#endif

/* Task attributes */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "DefaultTask",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

osTimerId_t led_timerHandle;
const osTimerAttr_t led_timer_attributes = {
    .name = "LEDTimer",
};

osMutexId_t show_mutexHandle;
osStaticMutexDef_t show_mutex_control_block;
const osMutexAttr_t show_mutex_attributes = {
    .name = "ShowMutex",
    .cb_mem = &show_mutex_control_block,
    .cb_size = sizeof(show_mutex_control_block),
};

osMutexId_t uart_mutex;
const osMutexAttr_t uart_mutex_attr = {
    .name = "UARTMutex"};

#ifdef I2C_SLAVE
const osThreadAttr_t slaveTxTask_attributes = {
    .name = "SlaveTxTask",
    .priority = osPriorityNormal,
    .stack_size = 128 * 4};

const osThreadAttr_t slaveRxTask_attributes = {
    .name = "SlaveRxTask",
    .priority = osPriorityNormal,
    .stack_size = 128 * 4};
#endif

/* Private function prototypes -----------------------------------------------*/
void led_timer_callback(void *argument);
void StartDefaultTask(void *argument);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

#ifdef I2C_SLAVE
void SlaveRxTask(void *argument);
void SlaveTxTask(void *argument);
#endif

#ifdef I2C_MASTER
void MasterRxTask(void *argument);
void MasterTxTask(void *argument);
#endif

/**
 * @brief Convert float to int16 with rounding
 * @param value Float value to convert
 * @return Rounded int16 value
 */
int16_t float_to_int16_round(float value) {
    if (value >= 0) {
        return (int16_t)(value + 0.5f);
    } else {
        return (int16_t)(value - 0.5f);
    }
}

/*
  * @brief Select calibration parameters based on channel and type
  * @param ch Channel number (0-7)
  * @param type 0 for set parameters, 1 for read parameters
  * @param power 0 for voltage, 1 for current
  * @param offset Pointer to store the selected offset
  * @param gain Pointer to store the selected gain  
*/
void sel_cali_param(uint8_t ch, uint8_t type, uint8_t power, float *offset, float *gain)
{
  if (ch > 7 || type > 1 || power > 1)
  {
    *offset = 0.0f;
    *gain = 1.0f;
    RA_POWEREX_DEBUG("Invalid channel (%d), type (%d) or power(%d) for calibration selection\r\n", ch, type, power);
    return;
  }

  calibration_data_t *cal = &g_calibration_manager.data;

  if (type == 0)
  { // Set parameters
    switch (ch)
    {
    case 0:
      *offset = cal->ch0_set_v_offset;
      *gain = cal->ch0_set_v_gain;
      break;
    case 1:
      *offset = cal->ch1_set_v_offset;
      *gain = cal->ch1_set_v_gain;
      break;
    case 2:
      *offset = cal->ch2_set_v_offset;
      *gain = cal->ch2_set_v_gain;
      break;
    case 3:
      *offset = cal->ch3_set_v_offset;
      *gain = cal->ch3_set_v_gain;
      break;
    case 4:
      *offset = cal->ch4_set_v_offset;
      *gain = cal->ch4_set_v_gain;
      break;
    case 5:
      *offset = cal->ch5_set_v_offset;
      *gain = cal->ch5_set_v_gain;
      break;
    case 6:
      *offset = cal->ch6_set_v_offset;
      *gain = cal->ch6_set_v_gain;
      break;
    case 7:
      *offset = cal->ch7_set_v_offset;
      *gain = cal->ch7_set_v_gain;
      break;
    default:
      *offset = 0.0f;
      *gain = 1.0f;
      break;
    }
  }
  else
  { // Read parameters
    switch (ch)
    {
    case 0:
      if (power == 0)
      {
        *offset = cal->ch0_read_v_offset;
        *gain = cal->ch0_read_v_gain;
      }
      else
      {
        *offset = cal->ch0_read_c_offset;
        *gain = cal->ch0_read_c_gain;
      }
      break;
    case 1:
      if (power == 0)
      {
        *offset = cal->ch1_read_v_offset;
        *gain = cal->ch1_read_v_gain;
      }
      else
      {
        *offset = cal->ch1_read_c_offset;
        *gain = cal->ch1_read_c_gain;
      }
      break;
    case 2:
      if (power == 0)
      {
        *offset = cal->ch2_read_v_offset;
        *gain = cal->ch2_read_v_gain;
      }
      else
      {
        *offset = cal->ch2_read_c_offset;
        *gain = cal->ch2_read_c_gain;
      }
      break;
    case 3:
      if (power == 0)
      {
        *offset = cal->ch3_read_v_offset;
        *gain = cal->ch3_read_v_gain;
      }
      else
      {
        *offset = cal->ch3_read_c_offset;
        *gain = cal->ch3_read_c_gain;
      }
      break;
    case 4:
      if (power == 0)
      {
        *offset = cal->ch4_read_v_offset;
        *gain = cal->ch4_read_v_gain;
      }
      else
      {
        *offset = cal->ch4_read_c_offset;
        *gain = cal->ch4_read_c_gain;
      }
      break;
    case 5:
      if (power == 0)
      {
        *offset = cal->ch5_read_v_offset;
        *gain = cal->ch5_read_v_gain;
    }
      else
      {
        *offset = cal->ch5_read_c_offset;
        *gain = cal->ch5_read_c_gain;
      }
      break;
    case 6:
      if (power == 0)
      {
        *offset = cal->ch6_read_v_offset;
        *gain = cal->ch6_read_v_gain;
      }
      else
      {
        *offset = cal->ch6_read_c_offset;
        *gain = cal->ch6_read_c_gain;
      }
      break;
    case 7:
      if (power == 0)
      {
        *offset = cal->ch7_read_v_offset;
        *gain = cal->ch7_read_v_gain;
      }
      else
      {
        *offset = cal->ch7_read_c_offset;
        *gain = cal->ch7_read_c_gain;
      }
      break;
    default:
      *offset = 0.0f;
      *gain = 1.0f;
      RA_POWEREX_DEBUG("Invalid channel (%d) for calibration selection\r\n", ch);
      break;
    }
  }
}

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{

  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  if (defaultTaskHandle == NULL)  RA_POWEREX_ERROR("defaultTaskHandle create failed\r\n");

#ifdef I2C_SLAVE
  // slaveTxTaskHandle = osThreadNew(SlaveTxTask, NULL, &slaveTxTask_attributes);
  slaveRxTaskHandle = osThreadNew(SlaveRxTask, NULL, &slaveRxTask_attributes);
  if (slaveRxTaskHandle == NULL)  RA_POWEREX_ERROR("slaveRxTaskHandle create failed\r\n");
#endif

#ifdef I2C_MASTER
  const osThreadAttr_t masterTxTask_attributes = {
      .name = "MasterTxTask",
      .priority = osPriorityHigh7,
      .stack_size = 128 * 4};
  masterTxTaskHandle = osThreadNew(MasterTxTask, NULL, &masterTxTask_attributes);
#endif
}



//for verify , use gc4.0 send command

#ifdef I2C_SLAVE
void SlaveRxTask(void *argument)
{
  volatile uint8_t cmd = 0;  // Command from host computer
  int16_t power_data = 0; // Voltage or current data, in mV or mA, is sent to the host computer
  float offset, gain;
  HAL_StatusTypeDef ret = HAL_ERROR;
  float latest_sample_raw_data[8] = {0}; // Store the latest sampled raw data for 8 channel
  uint16_t latest_sample_data[8] = {0}; // Store the latest sampled data for 8 channel after conversion and calibration
  uint8_t latest_sample_index[8] = {0}; // Store the corresponding index of the latest sampled data for 8 channel
  float IV_data = 0.0f; // Voltage or current data before calibration
  uint8_t r[2]={};
  uint8_t t[2]={};
  for (;;)
  {


#if 0
    //主机
    ret = HAL_I2C_Master_Transmit(&hi2c1,0x80, r, 1,1000); // Get the CMD instruction, 1BYTE
    if (ret == HAL_OK) printf("T ok\r\n");
    else {
      printf("T fail\r\n");
      HAL_I2C_DeInit(&hi2c1);
      HAL_I2C_Init(&hi2c1);
      if (I2C_IsSDALow(&hi2c1,GPIOB,GPIO_PIN_6,GPIO_PIN_7)) {
        // 如果 SDA 被拉低，调用恢复函数
        I2C_RecoverSDA(&hi2c1,GPIOB,GPIO_PIN_6,GPIO_PIN_7);
      }
      if (I2C_IsSCLLow(&hi2c1,GPIOB,GPIO_PIN_6,GPIO_PIN_7)) {
        // 如果 SCL 被拉低，调用恢复函数
        I2C_RecoverSCL(&hi2c1,GPIOB,GPIO_PIN_6,GPIO_PIN_7);
      }
      __disable_irq();
      MX_I2C1_Init();
      __enable_irq();
       
    }
    osDelay(10);
    ret = HAL_I2C_Master_Receive(&hi2c1, 0x80,t, 2, 1000);
    if (ret == HAL_OK) 
    {
      printf("R ok\r\n");
      printf("tx_buf[0]:%x\r\n",t[0]);
      printf("tx_buf[1]:%x\r\n",t[1]);
      t[0]=0;
      t[1]=0;

    }
    else 
    {
      printf("R fail\r\n");
      HAL_I2C_DeInit(&hi2c1);
      HAL_I2C_Init(&hi2c1);
      if (I2C_IsSDALow(&hi2c1,GPIOB,GPIO_PIN_6,GPIO_PIN_7)) {
        // 如果 SDA 被拉低，调用恢复函数
        I2C_RecoverSDA(&hi2c1,GPIOB,GPIO_PIN_6,GPIO_PIN_7);
      }
      if (I2C_IsSCLLow(&hi2c1,GPIOB,GPIO_PIN_6,GPIO_PIN_7)) {
        // 如果 SCL 被拉低，调用恢复函数
        I2C_RecoverSCL(&hi2c1,GPIOB,GPIO_PIN_6,GPIO_PIN_7);
      }
      __disable_irq();
      MX_I2C1_Init();
      __enable_irq();
    }
    osDelay(10);


#endif
    //HAL_I2C_Master_Receive(&hi2c1, 0x40>>1,tx_buf, 2, 1000);
    // RA_POWEREX_DEBUG("111111111\r\n");

    //HAL_I2C_Slave_Receive(&hi2c2,rx_buf, 1, 1000);
    //HAL_I2C_Slave_Transmit(&hi2c2,tx_buf, 2, 10000);


#if 0
    if (ret != HAL_OK)
    {
      continue;
    }
    for (uint8_t i = 0; i < 8; i++)
    {
      latest_sample_raw_data[i] = raw_data_queue_get_data(raw_data_queue_head - 1 - i);
      RA_POWEREX_DEBUG("latest raw data: %f\r\n", latest_sample_raw_data[i]);
      latest_sample_index[i] = raw_data_queue_get_index(raw_data_queue_head - 1 - i);
      RA_POWEREX_DEBUG("latest data index: %d\r\n", latest_sample_index[i]);
    }
    //calibration
    if(cmd == AD_I_ELVDD || cmd == AD_I_VCC || cmd == AD_I_IOVCC || cmd == AD_I_ELVSS){
        IV_data = IV_data * 50 *1e4;
        if(cmd == AD_I_ELVSS){
          IV_data = -IV_data; // ELVSS电流为负值
        }
        IV_data = gain*IV_data + offset; 
    }
    else if(cmd == AD_V_ELVSS || cmd == AD_V_ELVDD || cmd == AD_V_VCC || cmd == AD_V_IOVCC){
      IV_data = IV_data * 1e6 * 2.5;
      if(cmd == AD_V_ELVSS){
        IV_data = -IV_data; // ELVSS电压为负值
      }
      IV_data = gain*IV_data + offset; 
    }
    else{
      RA_POWEREX_DEBUG("Unknown command: 0x%02X\r\n", cmd);
    }
    RA_POWEREX_DEBUG("IV_data after calibration: %f\r\n", IV_data);

    power_data = float_to_int16_round(IV_data);
    RA_POWEREX_DEBUG("Power data to send: %d\r\n", power_data);
    
    //send data to host computer
    memcpy(tx_buf, &power_data, 2); // Copy the voltage or current data to the transmit buffer
    ret = HAL_I2C_Slave_Transmit(&hi2c2, tx_buf, 2, 1000);
#endif
#if 0
    if (ret != HAL_OK)
    {
      RA_POWEREX_DEBUG("I2C not receive command!\r\n");
    }
    else
    {
      cmd = rx_buf[0];
      RA_POWEREX_DEBUG("I2C Slave received power command: 0x%X\r\n", cmd);

      // disable interrupts to ensure atomic operation
      __disable_irq();

      //select channel for ADS1256
      dev_vol.work_channel = cmd; // The command directly corresponds to the ADC channel
      
      //sample data from ADS1256
      dev_vol.step_cnt = 6;      //  initial step_cnt need set 6     
      for (uint8_t i = 0; i < 7; i++)
      {
        while (HAL_GPIO_ReadPin(dev_vol.drdy_group, dev_vol.drdy_pin) == GPIO_PIN_RESET)
          ;
        {
          bsp_ads1256_irq_handle(&dev_vol);
        }
      }
      float IV_data = raw_data_queue_get_data(raw_data_queue_head - 1);
      RA_POWEREX_DEBUG("IV_data before calibration: %f\r\n", IV_data);
      
      //select calibration parameters
      sel_cali_param((cmd & 0x07), 1, 0, &offset, &gain);
      RA_POWEREX_DEBUG("Calibration offset: %f, gain: %f\r\n", offset, gain);

      //calibration
      if(cmd == AD_I_ELVDD || cmd == AD_I_VCC || cmd == AD_I_IOVCC || cmd == AD_I_ELVSS){
         IV_data = IV_data * 50 *1e4;
         if(cmd == AD_I_ELVSS){
           IV_data = -IV_data; // ELVSS电流为负值
         }
         IV_data = gain*IV_data + offset; 
      }
      else if(cmd == AD_V_ELVSS || cmd == AD_V_ELVDD || cmd == AD_V_VCC || cmd == AD_V_IOVCC){
        IV_data = IV_data * 1e6 * 2.5;
        if(cmd == AD_V_ELVSS){
          IV_data = -IV_data; // ELVSS电压为负值
        }
        IV_data = gain*IV_data + offset; 
      }
      else{
        RA_POWEREX_DEBUG("Unknown command: 0x%02X\r\n", cmd);
      }
      RA_POWEREX_DEBUG("IV_data after calibration: %f\r\n", IV_data);

      power_data = float_to_int16_round(IV_data);
      RA_POWEREX_DEBUG("Power data to send: %d\r\n", power_data);
      
      //send data to host computer
      memcpy(tx_buf, &power_data, 2); // Copy the voltage or current data to the transmit buffer
      ret = HAL_I2C_Slave_Transmit(&hi2c1, tx_buf, 2, 1000);
      if (ret != HAL_OK)
      {
        RA_POWEREX_DEBUG("I2C transmit data error!\r\n");
      }

      //enable interrupts after atomic operation
      __enable_irq();
    }
#endif
   //printf("111111\r\n"); 
   osDelay(100);

  }

}

void SlaveTxTask(void *argument)
{
  for (;;)
  {
    TIM2_Start();
    if (HAL_I2C_Slave_Transmit(&hi2c2, tx_buf, 33, 1000) != HAL_OK)
    {
      TIM2_Stop();
      HAL_I2C_DeInit(&hi2c2);
      HAL_I2C_Init(&hi2c2);
      TIM2_Start();
      HAL_I2C_Slave_Transmit(&hi2c2, tx_buf, 33, 1000);
      TIM2_Stop();
    }
    TIM2_Stop();
    osDelay(500);
  }
}

#endif

#ifdef I2C_MASTER
// 主机发送任务
void MasterTxTask(void *argument)
{
  for (;;)
  {
    if (osSemaphoreAcquire(i2cSemaphore, I2C_TIMEOUT) == osOK)
    {
    //   TIM2_Start();
    //   // 填充发送数据（可根据需求动态生成）
    //   tx_buf[1]++; // 示例：数据递增
    //   if (HAL_I2C_Master_Transmit(&hi2c2, SLAVE_ADDR, tx_buf, DATA_SIZE, 1000) != HAL_OK)
    //   {
    //     // 错误处理
    //     printf("I2C transmit error!\r\n");
    //     HAL_I2C_DeInit(&hi2c2);
    //     HAL_I2C_Init(&hi2c2);
    //   }
    //   TIM2_Stop();
    //   osSemaphoreRelease(i2cSemaphore);
    // }
    osDelay(100); // 每100ms发送一次
    OLED_Clear();                         //清屏
    //上面的初始化以及清屏的代码在一开始处一定要写
    OLED_ShowString(0,0,"UNICORN_LI",16, 1);    //反相显示8X16字符串
    OLED_ShowString(0,2,"unicorn_li_123",12,0);//正相显示6X8字符串
    
    OLED_ShowCHinese(0,4,0,1); //反相显示汉字“独”
    OLED_ShowCHinese(16,4,1,1);//反相显示汉字“角”
    OLED_ShowCHinese(32,4,2,1);//反相显示汉字“兽”
    OLED_ShowCHinese(0,6,0,0); //正相显示汉字“独”
    OLED_ShowCHinese(16,6,1,0);//正相显示汉字“角”
    OLED_ShowCHinese(32,6,2,0);//正相显示汉字“兽”

    
    OLED_HorizontalShift(0x26);//全屏水平向右滚动播放

    }

  }
}

#endif

/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
void StartDefaultTask(void *argument)
{
  osTimerStart(led_timerHandle, 1000);
  for (;;)
  {
    osDelay(1000);
  }
}

/* led_timer_callback function */
/**
 * @brief  Function implementing the led_timer.
 * @param  argument: Not used
 * @retval None
 */
void led_timer_callback(void *argument)
{
  for (;;)
  {
    osDelay(100);
    osDelay(100);
  }
}
/* USER CODE END Application */


void I2C_RecoverSDA(I2C_HandleTypeDef *hi2c , GPIO_TypeDef* port , uint16_t scl , uint16_t sda) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 1. 禁用 I2C 外设
    __HAL_I2C_DISABLE(hi2c);

    // 2. 将 SDA 和 SCL 引脚配置为 GPIO 输出模式
    GPIO_InitStruct.Pin = sda | scl; // 假设 SDA: PB8, SCL: PB9，需根据实际引脚调整
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;    // 开漏输出
    GPIO_InitStruct.Pull = GPIO_PULLUP;            // 上拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(port, &GPIO_InitStruct);

    // 3. 发送若干 SCL 时钟脉冲以释放 SDA
    for (uint8_t i = 0; i < 9; i++) {
        HAL_GPIO_WritePin(port, scl, GPIO_PIN_SET);   // SCL 高                                        // 短暂延时
        HAL_GPIO_WritePin(port, scl, GPIO_PIN_RESET); // SCL 低
    }

    // 4. 检查 SDA 是否被释放（变为高电平）
    if (HAL_GPIO_ReadPin(port, sda) == GPIO_PIN_SET) {
        // SDA 已恢复
        printf("SDA line recovered successfully.\n");
    } else {
        // SDA 未恢复，可能是硬件问题
        printf("Failed to recover SDA line.\n");
    }

    // 5. 重新配置 SDA 和 SCL 为 I2C 功能
    GPIO_InitStruct.Pin = sda | scl;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;        // 复用开漏模式
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    if(hi2c == &hi2c1) {
        GPIO_InitStruct.Alternate = GPIO_AF4_I2C1; // 根据实际 I2C 外设调整
    } else if(hi2c == &hi2c2) {
        GPIO_InitStruct.Alternate = GPIO_AF4_I2C2; // 根据实际 I2C 外设调整
    }
    HAL_GPIO_Init(port, &GPIO_InitStruct);

    // 6. 重新启用 I2C 外设
    __HAL_I2C_ENABLE(hi2c);

    // 7. 可选：复位 I2C 外设
    HAL_I2C_DeInit(hi2c);
    HAL_I2C_Init(hi2c);
}

bool I2C_IsSDALow(I2C_HandleTypeDef *hi2c , GPIO_TypeDef* port , uint16_t scl , uint16_t sda) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    bool isLow = false;

    // 1. 禁用 I2C 外设以避免干扰
    __HAL_I2C_DISABLE(hi2c);

    // 2. 将 SDA 引脚配置为输入模式
    GPIO_InitStruct.Pin = sda; // 假设 SDA 为 PB8，需根据实际引脚调整
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT; // 输入模式
    GPIO_InitStruct.Pull = GPIO_PULLUP;     // 上拉
    HAL_GPIO_Init(port, &GPIO_InitStruct);

    // 3. 读取 SDA 引脚状态
    if (HAL_GPIO_ReadPin(port, sda) == GPIO_PIN_RESET) {
        isLow = true; // SDA 被拉低
        //printf("SDA is low.\n");
    } else {
        //printf("SDA is high.\n");
    }

    // 4. 恢复 SDA 引脚为 I2C 功能
    GPIO_InitStruct.Pin = sda;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;    // 复用开漏模式
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    if(hi2c == &hi2c1) {
        GPIO_InitStruct.Alternate = GPIO_AF4_I2C1; // 根据实际 I2C 外设调整
    } else if(hi2c == &hi2c2) {
        GPIO_InitStruct.Alternate = GPIO_AF4_I2C2; // 根据实际 I2C 外设调整
    }
    HAL_GPIO_Init(port, &GPIO_InitStruct);

    // 5. 重新启用 I2C 外设
    __HAL_I2C_ENABLE(hi2c);

    return isLow;
}


bool I2C_IsSCLLow(I2C_HandleTypeDef *hi2c , GPIO_TypeDef* port , uint16_t scl , uint16_t sda) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    bool isLow = false;

    // 1. 禁用 I2C 外设以避免干扰
    __HAL_I2C_DISABLE(hi2c);

    // 2. 将 SCL 引脚配置为输入模式
    GPIO_InitStruct.Pin = scl; // 假设 SCL 为 PB9，需根据实际引脚调整
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT; // 输入模式
    GPIO_InitStruct.Pull = GPIO_PULLUP;     // 上拉
    HAL_GPIO_Init(port, &GPIO_InitStruct);

    // 3. 读取 SCL 引脚状态
    if (HAL_GPIO_ReadPin(port, scl) == GPIO_PIN_RESET) {
        isLow = true; // SCL 被拉低
        //printf("SCL is low.\n");
    } else {
        //printf("SCL is high.\n");
    }

    // 4. 恢复 SCL 引脚为 I2C 功能
    GPIO_InitStruct.Pin = scl;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;    // 复用开漏模式
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    if(hi2c == &hi2c1) {
        GPIO_InitStruct.Alternate = GPIO_AF4_I2C1; // 根据实际 I2C 外设调整
    } else if(hi2c == &hi2c2) {
        GPIO_InitStruct.Alternate = GPIO_AF4_I2C2; // 根据实际 I2C 外设调整
    }
    HAL_GPIO_Init(port, &GPIO_InitStruct);

    // 5. 重新启用 I2C 外设
    __HAL_I2C_ENABLE(hi2c);

    return isLow;
}

void I2C_RecoverSCL(I2C_HandleTypeDef *hi2c , GPIO_TypeDef* port , uint16_t scl , uint16_t sda) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 1. 禁用 I2C 外设
    __HAL_I2C_DISABLE(hi2c);

    // 2. 将 SCL 和 SDA 引脚配置为 GPIO 输出模式
    GPIO_InitStruct.Pin = scl | sda; // 假设 SCL: PB9, SDA: PB8，需根据实际引脚调整
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;    // 开漏输出
    GPIO_InitStruct.Pull = GPIO_PULLUP;            // 上拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(port, &GPIO_InitStruct);

    // 3. 尝试释放 SCL（通过切换电平）
    for (uint8_t i = 0; i < 9; i++) {
        HAL_GPIO_WritePin(port, scl, GPIO_PIN_SET);   // SCL 高                                       // 短暂延时
        HAL_GPIO_WritePin(port, scl, GPIO_PIN_RESET); // SCL 低
    }

    // 4. 检查 SCL 是否被释放（变为高电平）
    GPIO_InitStruct.Pin = scl;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT; // 切换为输入模式以检测
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(port, &GPIO_InitStruct);

    if (HAL_GPIO_ReadPin(port, scl) == GPIO_PIN_SET) {
        //printf("SCL line recovered successfully.\n");
    } else {
        //printf("Failed to recover SCL line.\n");
    }

    // 5. 恢复 SCL 和 SDA 为 I2C 功能
    GPIO_InitStruct.Pin = scl | sda;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;    // 复用开漏模式
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    if(hi2c == &hi2c1) {
        GPIO_InitStruct.Alternate = GPIO_AF4_I2C1; // 根据实际 I2C 外设调整
    } else if(hi2c == &hi2c2) {
        GPIO_InitStruct.Alternate = GPIO_AF4_I2C2; // 根据实际 I2C 外设调整
    }
    HAL_GPIO_Init(port, &GPIO_InitStruct);

    // 6. 重新启用 I2C 外设
    __HAL_I2C_ENABLE(hi2c);

    // 7. 可选：复位 I2C 外设
    HAL_I2C_DeInit(hi2c);
    HAL_I2C_Init(hi2c);
}


