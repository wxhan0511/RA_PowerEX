/**
 * @file       config.h
 * @brief      RA_PowerEX
 * @author     wxhan
 * @version    1.0.0
 * @date       2025-10-10
 * @copyright  Copyright (c) 2025 gcoreinc
 * @license    MIT License
 */

#ifndef __CONFIG_H
#define __CONFIG_H

/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/


/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

#ifdef __cplusplus
}
#endif

// #define SUB_BOARD1
// #define SUB_BOARD2
// #define SUB_BOARD3
// #define SUB_BOARD4
// #define SUB_BOARD5
// #define SUB_BOARD6
// #define SUB_BOARD7
// #define SUB_BOARD8


/* I2C configuration macros */
#ifdef SUB_BOARD1
  #define SLAVE_ADDR            0x10
#elif defined(SUB_BOARD2)
  #define SLAVE_ADDR            0x20
#elif defined(SUB_BOARD3)
  #define SLAVE_ADDR            0x30
#elif defined(SUB_BOARD4)
  #define SLAVE_ADDR            0x40
#elif defined(SUB_BOARD5)
  #define SLAVE_ADDR            0x50
#elif defined(SUB_BOARD6)
  #define SLAVE_ADDR            0x60
#elif defined(SUB_BOARD7)
    #define SLAVE_ADDR           0x70
#elif defined(SUB_BOARD8)
    #define SLAVE_ADDR           0x80
#else
  #define SLAVE_ADDR            0x80 // Default slave address

#endif

#define SLAVE_ADDR1           0x20
#define DATA_SIZE             256
#define I2C_TIMEOUT           100
#define I2C_MASTER            //屏幕显示和AVDD使能控制
//#define I2C_SLAVE              //I2C一拖八
// #define I2C2_IRQ
//#define I2C1_IRQ              //I2C一拖八
//#define I2C_SLAVE_I2C1_LISTEN //I2C一拖八
//#define I2C_SLAVE_I2C2_LISTEN

#define GTB
//#define USE_OLED

//only one can be defined
#define CDC
// #define CHID
//#define RA_POWERSUPPLY_FOR_IC

#endif /* __CONFIG_H */
