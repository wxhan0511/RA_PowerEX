/**
 * @file       cdc_task.h
 * @brief      RA_PowerEX
 * @author     wxhan
 * @version    1.0.0
 * @date       2025-10-25
 * @copyright  Copyright (c) 2025 gcoreinc
 * @license    MIT License
 */

#ifndef __CDC_TASK_H
#define __CDC_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/
typedef enum{
    GET_ID = 0x10,
    GET_SW_VERSION = 0x11,
    VOL_SET = 0x12,
    LIM_SET = 0x13,
    ALL_POWER_EN = 0x14,
    SINGLE_POWER_EN = 0x15,
    SINGLE_VOL_GET = 0x16,
    SINGLE_CUR_GET = 0x17
}cdc_cmd_type;

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __CDC_TASK_H */
