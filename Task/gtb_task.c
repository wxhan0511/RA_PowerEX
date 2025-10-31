/**
 * @file       gtb.c
 * @brief      GTB  module implementation.
 * @author     wxhan
 * @version    1.0.0
 * @date       2025-10-09
 * @copyright  Copyright (c) 2025 gcoreinc
 * @license    MIT License
 */

/* Includes ------------------------------------------------------------------*/
#include "gtb_task.h"
#include <stdio.h>
#include <string.h>
#include "usbd_customhid.h"
#include "main.h"
#include "cmsis_os.h"
#include "config.h"
#include "gtb_com.h"
#include "gtb_op.h"
#include "usbd_cdc_if.h"
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
tp_config_t tp_config_hid;
extern uint8_t hid_state_fs;
extern uint8_t send_data_fs[64];
// Declare the external USB device handle
extern USBD_HandleTypeDef hUsbDeviceHS;

const osThreadAttr_t server_gtb_attr = {
    .name = "ServerGtbTask",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
osThreadId_t thread_id_gtb;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
void server_gtb_init(){
    thread_id_gtb = osThreadNew(server_gtb,NULL,&server_gtb_attr);
    if(thread_id_gtb == NULL){
        printf("[error] server gtb \r\n");
    }
}

void server_gtb_suspend(){
    if(thread_id_gtb != NULL){
        osThreadSuspend(thread_id_gtb);

    }
    else{
        printf("[gtb log] gtb server is not active\r\n");
    }
}

void server_gtb(void *argument)
{
#ifdef USE_OLED
  osTimerStart(led_timerHandle, 1000);
#endif
  GTB_INFO("[gtb task] active \r\n");
  uint8_t test_buf[] = "hello , CDC Virtual COM\r\n";
  uint32_t send_cnt =0;
  for (;;)
  {
    // if (send_cnt % 10 == 0)
	// {
	// 	CDC_Transmit_HS(test_buf,sizeof(test_buf) - 1);
	// }
	// send_cnt++;
#if 0
    hid_state_fs = 0;
    bsp_gtb_init(3);
    gtb_global_var_init(&tp_config_hid);
    //ex_ti_initial(&tp_config_hid,DISABLE);
    //meter_get_tp_int_voltage();

    __IO uint8_t com_mode = GTB_HID;
//    cdc_tx_buf[0] = 0xf8;
//    cdc_tx_buf[1] = 0x80;


        //检测USB连接状态
    if(hUsbDeviceHS.dev_state != USBD_STATE_CONFIGURED)
    {
        //USB未连接
        RA_POWEREX_DEBUG("USB is not connected.");
        osDelay(100);
        continue;
    }
    else
    {
        //send_data_fs
        //发送
#ifdef USE_USBD_COMPOSITE
        USBD_CUSTOM_HID_SendReport(&hUsbDeviceHS, send_data_fs, 64, 0);
        
#else
        //USBD_CUSTOM_HID_SendReport(&hUsbDeviceHS, send_data_fs, 64);
       
#endif

    }
#endif
    osDelay(100);
  }

}
/* Exported functions --------------------------------------------------------*/
