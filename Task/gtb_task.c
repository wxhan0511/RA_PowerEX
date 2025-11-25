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
#include "main.h"
#include "cmsis_os.h"
#include "config.h"
#include "gtb_com.h"
#include "gtb_op.h"
#include "usbd_cdc_if.h"
#include "usbd_hid.h"
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
tp_config_t tp_config_hid;

const osThreadAttr_t server_gtb_attr = {
    .name = "ServerGtbTask",
    .stack_size = 1024 * 8,
    .priority = (osPriority_t)osPriorityNormal,
};
osThreadId_t thread_id_gtb;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

void server_gtb_init()
{
    thread_id_gtb = osThreadNew(server_gtb, NULL, &server_gtb_attr);
    if (thread_id_gtb == NULL)
    {
        printf("[error] server gtb \r\n");
    }
}

void server_gtb_suspend()
{
    if (thread_id_gtb != NULL)
    {
        osThreadSuspend(thread_id_gtb);
    }
    else
    {
        printf("[gtb log] gtb server is not active\r\n");
    }
}

void server_gtb(void *argument)
{
#ifdef USE_OLED
    osTimerStart(led_timerHandle, 1000);
#endif
    GTB_INFO("[gtb task] active \r\n");
    uint32_t send_cnt = 0;
    uint8_t test_data[64] = {1};
    for (;;)
    {
#if 1
        hid_state_fs = 0;
        bsp_gtb_init(3);
        gtb_global_var_init(&tp_config_hid);
        ex_ti_initial(&tp_config_hid, DISABLE);
        // meter_get_tp_int_voltage();

        __IO uint8_t com_mode = GTB_HID;
        //    cdc_tx_buf[0] = 0xf8;
        //    cdc_tx_buf[1] = 0x80;

        // 检测USB连接状态
        if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
        {
            // USB未连接
            RA_POWEREX_DEBUG("USB is not connected.");
            osDelay(100);
            continue;
        }
        else
        {
            // send_data_fs
            // 发送
#ifdef USE_USBD_COMPOSITE
            USBD_CUSTOM_HID_SendReport(&hUsbDeviceHS, send_data_fs, 64, 0);

#else
            //RA_POWEREX_DEBUG("GTB HID send report %d", send_cnt++);
            //USBD_HID_SendReport(&hUsbDeviceFS, test_data, sizeof(test_data));
            HAL_Delay(10);
            //gtb_fw_mode_com(&tp_config_hid, send_data_fs, get_data_fs, GTB_HID);
            // USBD_HID_SendReport(&hUsbDeviceFS, send_data_fs, 64);
            if (0x40 == send_data_fs[0])
            {
                // if (0xfa == send_data_fs[1] && 0x38 != send_data_fs[7])
                // {
                //     printf("[master]");
                //     for (uint8_t i=0;i<16;i++)
                //         printf("0x%x ",send_data_fs[i]);
                //     printf("\r\n");
                // }

                //gtb_generic_com(&tp_config_hid, send_data_fs, get_data_fs, GTB_HID);
            }
            // else if(0x50 == send_data_fs[0]){
            //     switch (send_data_fs[1]) {
            //         case GTB_HID:
            //             com_mode = GTB_HID;
            //             break;
            //         case GTB_CDC:
            //             com_mode = GTB_CDC;
            //             break;
            //         case GTB_MIX:
            //             com_mode = GTB_MIX;
            //             break;
            //     }
            // }
            // else if(0x60 == send_data_fs[0]){
            //     ex_ti_initial(&tp_config_hid,DISABLE);
            // }
            // else if(0xc0 == send_data_fs[0]){
            //     master_state.cmd_ret_id = USB_FS;
            //     command_c0_handle(&master_state,send_data_fs);
            // }
            hid_state_fs = 0;
        }
#endif
        }
#endif
        osDelay(100);
    }
    /* Exported functions --------------------------------------------------------*/
