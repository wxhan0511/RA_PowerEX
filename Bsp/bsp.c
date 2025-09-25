/**
 * @file       bsp.c
 * @brief      RA_PowerEX
 * @author     wxhan
 * @version    1.0.0
 * @date       2025-09-03
 * @copyright  Copyright (c) 2025 gcoreinc
 * @license    MIT License
 */

/* System header files */
#include <stdio.h>
#include <string.h>

/* HAL library and main configuration */
#include "bsp.h"
#include "main.h"
#include "usart.h"

/* BSP function modules */
#include "bsp_ads1256.h"
#include "bsp_channel_sel.h"
#include "bsp_dwt.h"
#include "bsp_mcp4728.h"
#include "bsp_spi_flash.h"
#include "bsp_calibration.h"
#include "bsp_power.h"
#include "oled.h"

/* Tool modules */
#include "retarget.h"
#include "delay.h"

/* Private function prototypes */
static void bsp_print_version_info(void);
static HAL_StatusTypeDef bsp_init_adc_system(void);
static void bsp_test_spi_flash(void);
static void bsp_init_power_control(void);
static void bsp_init_interrupts(void);
static void bsp_level_shift_direction_set(uint8_t dir);


/**
 * @brief Firmware name stored in specific section
 * @note Used for firmware identification and version management
 */
__attribute__((section(".fw_name"))) const char fw_name[40] = "RA PowerEX Board v1.0";

/**
 * @brief Software version [Major.Minor.Patch.Build]
 */
__attribute__((section(".fw_version"))) uint8_t sw_version[4] = {1, 0, 0, 1};

/**
 * @brief Hardware name
 */
__attribute__((section(".hw_name"))) const char hw_name[40] = "RA PowerEX Board";

/**
 * @brief Hardware version [Major.Minor.Patch.Build]
 */
__attribute__((section(".hw_version"))) uint8_t hw_version[4] = {1, 0, 0, 1};

/**
 * @brief Magic number for firmware verification
 */
uint8_t magic_number[4] = {0x12, 0x34, 0xf8, 0x8f};



/**
 * @brief BSP Board Support Package Initialization
 * @details Initialize all peripherals and function modules in sequence
 * @retval None
 */
void bsp_init()
{

    bsp_retarget_init(&huart1);

    bsp_init_dwt();

    /* Print system version information */
    bsp_print_version_info();

    MX_CRC_Init();

    // bsp_test_spi_flash();

    //calibration_set_defaults();
    //calibration_save();
    calibration_load();
    bsp_init_power_control();
    bsp_init_adc_system();
    bsp_level_shift_direction_set(1);
    bsp_dac_init(&dac_dev);
    RA_POWEREX_INFO("------------- bsp init finish -------------\r\n");
}
/**
 * @brief Initialize power control GPIOs and enable power rails
 * @retval None
 */
static void bsp_init_power_control(void)
{
    ELVSS_ENABLE();
    ELVDD_ENABLE();
    VCC_ENABLE();
    IOVCC_ENABLE();
    //On by default
    SHUTDOWN_ENABLE();
    RA_POWEREX_INFO("------------- bsp init power control finish -------------\r\n");

}
/**
 * @brief Print system version information
 * @retval None
 */
static void bsp_print_version_info(void)
{
    RA_POWEREX_INFO("================================================\r\n");
    RA_POWEREX_INFO("RA PowerEX Board System Information\r\n");
    RA_POWEREX_INFO("================================================\r\n");
    RA_POWEREX_INFO("Firmware Name: %s\r\n", fw_name);
    RA_POWEREX_INFO("Software Version: %d.%d.%d.%d\r\n", 
                       sw_version[0], sw_version[1], sw_version[2], sw_version[3]);
    RA_POWEREX_INFO("Hardware Name: %s\r\n", hw_name);
    RA_POWEREX_INFO("Hardware Version: %d.%d.%d.%d\r\n", 
                       hw_version[0], hw_version[1], hw_version[2], hw_version[3]);
    RA_POWEREX_INFO("================================================\r\n");
}

/**
 * @brief Initialize ADC system (ADS1256)
 * @retval HAL_StatusTypeDef
 */
static HAL_StatusTypeDef bsp_init_adc_system(void)
{
    /* Configure ADS1256 device parameters */
    dev_vol.work_channel = 0;
    dev_vol.channel_en = 0b11111111; // sample all 8 channels
    dev_vol.auto_change_gear = 0;   /* Disable auto gear change */
#ifdef GEAR_FUNCTION
    /* Configure ADS1256 gear settings for 8 channels */
    for (uint8_t i = 0; i < 8; i++)
    {
        if (i == 3 || i == 4 || i == 6)
            dev_vol.sample_res_gear[i] = GEAR_mA;
    }
#endif
    /* Initialize ADS1256 */
    //bsp_ads1256_init(&dev_vol);
#ifdef GEAR_FUNCTION
    /* Set sampling gear */
    bsp_select_sample_gear(dev_vol.sample_res_gear);
#endif
    //ADC_DRDY_Pin:PA3--EXTI3
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);
    
    RA_POWEREX_INFO("------------- bsp init ads1256 finish -------------\r\n");
}

static void bsp_level_shift_direction_set(uint8_t dir)
{
    if (dir)
    {
        HAL_GPIO_WritePin(LEVEL_SHIFT_OE_GPIO_Port, LEVEL_SHIFT_OE_Pin, GPIO_PIN_SET);  // Set direction to high
    }
    else
    {
        HAL_GPIO_WritePin(LEVEL_SHIFT_OE_GPIO_Port, LEVEL_SHIFT_OE_Pin, GPIO_PIN_RESET); // Set direction to low
    }
}
/* Optional: Flash test function (currently disabled) */
/**
 * @brief Test SPI Flash read/write/erase operations
 * @retval None
 */
static void bsp_test_spi_flash(void)
{
    RA_POWEREX_INFO("Starting SPI Flash test...\r\n");
    
    uint8_t write_data[256] = {0};
    uint8_t read_data[256] = {0};
    
    /* Initialize test data */
    for (uint32_t i = 0; i < 256; i++) {
        write_data[i] = i;
    }
    
    /* Erase chip */
    bsp_flash_erase_chip();
    printf("Flash chip erase completed\r\n");
    
    /* Read 256 bytes from SPI Flash */
    bsp_flash_read(read_data, 0x00000000, 256);
    printf("Flash read completed\r\n");
    
    /* Print read data */
    for (uint32_t i = 0; i < 256; i++) {
        printf("%02X ", read_data[i]);
        if ((i + 1) % 16 == 0) {
            printf("\r\n");
        }
    }
    
    /* Write 256 bytes to SPI Flash */
    printf("Flash write start\r\n");
    BSP_STATUS status = bsp_flash_write(write_data, 0x00000000, 256);
    if (status != BSP_OK) {
        printf("Flash write failed\r\n");
    } else {
        printf("Flash write succeeded\r\n");
    }
    
    /* Read back and verify */
    bsp_flash_read(read_data, 0x00000000, 256);
    printf("Flash read verification completed\r\n");
    
    /* Print verification data */
    for (uint32_t i = 0; i < 256; i++) {
        printf("%02X ", read_data[i]);
        if ((i + 1) % 16 == 0) {
            printf("\r\n");
        }
    }
}