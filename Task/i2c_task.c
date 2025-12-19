#include "i2c_task.h"
#include "main.h"
#include "i2c.h"
#include "oled.h"
#include "bsp_ads1256.h"
#include "bsp_mcp4728.h"
#include "bsp_power.h"
float latest_sample_data[8] = {0}; // Store the latest sampled data for 8 channel after conversion and calibration
float latest_sample_raw_data[8] = {0}; // Store the latest sampled raw data for 8 channel
uint8_t latest_sample_index[8] = {0}; // Store the corresponding index of the latest sampled data for 8 channel
uint8_t channel_num = 0;
float offset, gain, IV_data = 0.0f;
osSemaphoreId_t i2c1Semaphore = NULL;

osThreadId_t masterTxTaskHandle = NULL;
osThreadId_t masterRxTaskHandle = NULL;
osThreadId_t slaveTxTaskHandle = NULL;
osThreadId_t slaveRxTaskHandle = NULL;

const osThreadAttr_t masterTxTask_attributes = {
    .name = "MasterTxTask",
    .priority = osPriorityHigh7,
    .stack_size = 128 * 4};

const osThreadAttr_t masterRxTask_attributes = {
    .name = "MasterRxTask",
    .priority = osPriorityHigh7,
    .stack_size = 128 * 4};
const osThreadAttr_t slaveTxTask_attributes = {
    .name = "SlaveTxTask",
    .priority = osPriorityNormal,
    .stack_size = 128 * 4
};
const osThreadAttr_t slaveRxTask_attributes = {
    .name = "SlaveRxTask",
    .priority = osPriorityNormal,
    .stack_size = 128 * 4
};

void I2C_Semaphore_Init(void)
{
    i2c1Semaphore = osSemaphoreNew(1, 1, NULL); // 二值信号量，初始可用
    if (i2c1Semaphore == NULL)
    {
        RA_POWEREX_ERROR("i2c1Semaphore create failed\r\n");
    }
}

void slave_rx_task_init()
{
  slaveRxTaskHandle = osThreadNew(SlaveRxTask, NULL, &slaveRxTask_attributes);
  if (slaveRxTaskHandle == NULL)  RA_POWEREX_ERROR("slaveRxTaskHandle create failed\r\n");
}

void slave_tx_task_init()
{
  slaveTxTaskHandle = osThreadNew(SlaveTxTask, NULL, &slaveTxTask_attributes);
  if (slaveTxTaskHandle == NULL)  RA_POWEREX_ERROR("slaveTxTaskHandle create failed\r\n");
}

void master_tx_task_init()
{
  masterTxTaskHandle = osThreadNew(MasterTxTask, NULL, &masterTxTask_attributes);
  if (masterTxTaskHandle == NULL)  RA_POWEREX_ERROR("masterTxTaskHandle create failed\r\n");
}

void master_rx_task_init()
{
  masterRxTaskHandle = osThreadNew(MasterRxTask, NULL, &masterRxTask_attributes);
  if (masterRxTaskHandle == NULL)  RA_POWEREX_ERROR("masterRxTaskHandle create failed\r\n");
}

void SlaveRxTask(void *argument)
{
  for (;;)
  {
   osDelay(100);

  }

}

void SlaveTxTask(void *argument)
{
  for (;;)
  {
   osDelay(100);

  }
}

void MasterRxTask(void *argument)
{
  for (;;)
  {
   osDelay(100);

  }

}

// 主机发送任务
void MasterTxTask(void *argument)
{
  OLED_Init();//OLED初始
  OLED_Clear();//清屏


  for (;;)
  {
    if (osSemaphoreAcquire(i2c1Semaphore, I2C_TIMEOUT) == osOK)
    {
      __disable_irq();
      for (uint8_t i = 0; i < 8; i++)
      {
        
        latest_sample_raw_data[i] = raw_data_queue_get_data(raw_data_queue_head - 1 - i);
        latest_sample_index[i] = raw_data_queue_get_index(raw_data_queue_head - 1 - i);
      }
      __enable_irq();
      
      //从latest_sample_index[i]获取通道号
      for(uint8_t i = 0; i < 8; i++)
      {
        channel_num = latest_sample_index[i];
        if (channel_num < 4)
        {
          sel_cali_param((channel_num & 0x07), 1, 1, &offset, &gain);
          IV_data = latest_sample_raw_data[i] * 500;
          if(channel_num == AD_I_ELVSS){
            IV_data = -IV_data; // ELVSS电流为负值
          }
          IV_data = gain*IV_data + offset; 

        }
        else if(3 < channel_num && channel_num < 8)
        {
          //sel_cali_param((channel_num & 0x07), 1, 0, &offset, &gain);
          IV_data = latest_sample_raw_data[i] * 1e3;
          if(channel_num == AD_V_ELVSS){
            IV_data = -IV_data; // ELVSS电压为负值
          }
          if(channel_num == AD_V_ELVDD){
            IV_data = IV_data*2.5; // ELVDD电压为2.5倍
          }
          //IV_data = gain*IV_data + offset;

        }
        latest_sample_data[channel_num] = IV_data;
      }
      AD_DATA_INFO("Latest sampled data (after calibration):\r\n");
      for (uint8_t i = 0; i < 8; i++)
      {
        AD_DATA_DEBUG("-----Channel %d: %f\r\n", latest_sample_index[i], latest_sample_data[i]);
      }
    

      OLED_ShowString(0,0,"ELVSS",12,0);
      OLED_Showdecimal(32,0,latest_sample_data[AD_I_ELVSS],3,3,12, 0);
      OLED_Showdecimal(78,0,latest_sample_data[AD_V_ELVSS],4,2,12, 0);
      OLED_ShowString(0,10,"IOVCC",12,0);
      OLED_Showdecimal(32,10,latest_sample_data[AD_I_IOVCC],3,3,12, 0);
      OLED_Showdecimal(78,10,latest_sample_data[AD_V_IOVCC],4,2,12, 0);
//IOVCC 200
//AVDD 
      // if(latest_sample_data[AD_I_IOVCC]>190)
      // {
      //   IOVCC_DISABLE();
      //   ELVSS_DISABLE();
      //   ELVDD_DISABLE();
      //   VCC_DISABLE();
      //   while(1);
      // }

      OLED_ShowString(0,12,"ELVDD",12,0);
      if (HAL_GPIO_ReadPin(ELVDD_EN_GPIO_Port , ELVDD_EN_Pin) == GPIO_PIN_RESET)
      {
        OLED_Showdecimal(32,12,0,3,3,12, 0);
        OLED_Showdecimal(78,12,0,4,2,12, 0);
      }
      else
      {
        OLED_Showdecimal(32,12,latest_sample_data[AD_I_ELVDD],3,3,12, 0);
        OLED_Showdecimal(78,12,latest_sample_data[AD_V_ELVDD],4,2,12, 0);
      }

      OLED_ShowString(0,14,"VCC",12,0);
      OLED_Showdecimal(32,14,latest_sample_data[AD_I_VCC],3,3,12, 0);
      OLED_Showdecimal(78,14,latest_sample_data[AD_V_VCC],4,2,12, 0);
      //OLED_Showdecimal(0,6,num2,2,3,12, 0);
      //OLED_HorizontalShift(0x26);//全屏水平向右滚动播放
      osSemaphoreRelease(i2c1Semaphore);
      __enable_irq();
    }
    osDelay(1000);

  }
}