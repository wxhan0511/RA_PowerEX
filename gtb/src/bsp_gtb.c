#include "bsp_gtb.h"
#include "bsp_i2c_bus.h"
#include "main.h"

SPI_HandleTypeDef hspi_tp;

void  tp_spi_set_mode(uint8_t mode){
    switch (mode) {
        case 0:
            hspi_tp.Init.CLKPhase          = SPI_PHASE_1EDGE;
            hspi_tp.Init.CLKPolarity       = SPI_POLARITY_LOW;
            break;
        case 1:
            hspi_tp.Init.CLKPhase          = SPI_PHASE_2EDGE;
            hspi_tp.Init.CLKPolarity       = SPI_POLARITY_LOW;
            break;
        case 2:
            hspi_tp.Init.CLKPhase          = SPI_PHASE_1EDGE;
            hspi_tp.Init.CLKPolarity       = SPI_POLARITY_HIGH;
            break;
        default:
            hspi_tp.Init.CLKPhase          = SPI_PHASE_2EDGE;
            hspi_tp.Init.CLKPolarity       = SPI_POLARITY_HIGH;
            break;
    }
    if (HAL_SPI_Init(&hspi_tp) != HAL_OK)
    {
        /* Initialization Error */
        GTB_DEBUG("HAL_SPI_Init failed!\r\n");
    }
}

void tp_spi_cs_enable(bool state)
{
    if(state == true){
        HAL_GPIO_WritePin(TSPI_CS_GPIO_Port, TSPI_CS_Pin, 0);
    }
    else{
        HAL_GPIO_WritePin(TSPI_CS_GPIO_Port, TSPI_CS_Pin, 1);
    }
}
void bsp_gtb_init(uint8_t mode){

    /* Set the SPI parameters */
    hspi_tp.Instance               = SPI2;
    hspi_tp.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;//SPI_BAUDRATEPRESCALER_2 :SPI 21MHz
    hspi_tp.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi_tp.Init.CLKPhase          = SPI_PHASE_1EDGE;
    hspi_tp.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hspi_tp.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi_tp.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi_tp.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi_tp.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi_tp.Init.CRCPolynomial     = 7;
    hspi_tp.Init.NSS               = SPI_NSS_SOFT;
    hspi_tp.Init.Mode 			 = SPI_MODE_MASTER;
    tp_spi_set_mode(mode);

    tp_spi_cs_enable(false);
   
}