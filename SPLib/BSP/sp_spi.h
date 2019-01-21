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

/** @defgroup Definations
  * @brief    Exported Macros And Definations
  * @ingroup  SPI
  * @{
  */
#define SPI5_NSS_Select     GPIO_ResetBits(GPIOF, GPIO_Pin_6)
#define SPI5_NSS_Release    GPIO_SetBits(GPIOF, GPIO_Pin_6)
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
