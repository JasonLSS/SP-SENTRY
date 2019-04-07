/**
  ******************************************************************************
  * @file       sp_spi.h
  * @author     YTom
  * @version    v0.1
  * @date       2019.Jan.21
  * @brief      CAN-bus module controller
  @verbatim      
        BaudRate:   1Mbps
        PinMap:     PB13(tx) PB12(rx)
      
        PE4  SPI4_MOSI
        PE5  SPI4_MISO
        PE6  SPI4_NSS
        PE12 SPI4_SCK
  @endverbatim
  ******************************************************************************
  * @license
  *
  ******************************************************************************
  */

#ifndef __SP_SPI_H
#define __SP_SPI_H



#include "sp_conf.h"

#ifdef __cplusplus
 extern "C" {
#endif

/** @addtogroup SP
  * @brief      SuperPower
  * @{
  */

/** @defgroup SPI
  * @brief    SPI Module
  * @{
  */

/** @defgroup SPI_Definations
  * @brief    SPI Exported Macros And Definations
  * @ingroup  SPI
  * @{
  */
#define SPI5_NSS_Select     GPIO_ResetBits(GPIOF, GPIO_Pin_6)
#define SPI5_NSS_Release    GPIO_SetBits(GPIOF, GPIO_Pin_6)


typedef struct {
    SPI_TypeDef*        this;
    spPinSet            sck;
    spPinSet            miso;
    spPinSet            mosi;
    spPinSet            nss;
    SPI_InitTypeDef     spi_initer;
} SPI_PinsType;


static SPI_PinsType SPI1_Pins = {
    SPI1,
};

static SPI_PinsType SPI2_Pins = {
    SPI2,
};


static SPI_PinsType SPI3_Pins = {
    SPI3,
};

static SPI_PinsType SPI4_Pins = {
    SPI4,
    .sck = {GPIOE, GPIO_PinSource12},
    .miso = {GPIOE, GPIO_PinSource5},
    .mosi = {GPIOE, GPIO_PinSource6},   
    .nss = {GPIOE, GPIO_PinSource4},
};

static SPI_PinsType SPI5_Pins = {
    SPI5,
    .sck = {GPIOF, GPIO_PinSource7},
    .miso = {GPIOF, GPIO_PinSource8},
    .mosi = {GPIOF, GPIO_PinSource9},   
    .nss = {GPIOF, GPIO_PinSource6},
};

static SPI_PinsType SPI6_Pins = {
    SPI6,
};
/** @} */


/** @defgroup SPI_APIs
  * @brief    SPI user operations
  * @ingroup  SPI
  * @{
  */
extern struct __SPI_Manager_Type {
    uint16_t (*read_write_b)(SPI_TypeDef* spi, uint16_t data);
    uint16_t (*write_b)(SPI_TypeDef* spi, uint16_t data);
    void     (*select)(SPI_PinsType* spi_pins);
    void     (*release)(SPI_PinsType* spi_pins);
} spSPI;
/** @} */


/**
  * @}
  */
  
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /*__SP_SPI_H */

/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/
