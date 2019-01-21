/**
  ******************************************************************************
  * @file       sp_usart.h
  * @author     YTom
  * @version    v0.0-alpha
  * @date       2018.Nov.24
  * @brief      USART module driver
  * @usage      
  ******************************************************************************
  * @license
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SP_USART_H
#define __SP_USART_H

#ifdef __cplusplus
 extern "C" {
#endif

/** @addtogroup SP
  * @brief      SuperPower
  * @{
  */

/** @defgroup USART
  * @brief    USART Module
  * @{
  */

#include <stdio.h>
#include "stm32f4xx_dma.h"
#include "stm32f4xx_usart.h"
#include "sp_type.h"
#include "sp_gpio.h"
#include "sp_dma.h"

#include "sp_conf.h"

/** @defgroup ResourceConfiguration
  * @brief    USART Resource Configuration
  * @ingroup  USART
  * @{
  */

/**
  * @brief  USART RCC config
  */
#define spUSART1_RCC_GPIO()     spRCC_Set_GPIOB()
#define spUSART2_RCC_GPIO()     spRCC_Set_GPIO
#define spUSART3_RCC_GPIO()     spRCC_Set_GPIOG()
#define spUART4_RCC_GPIO()      spRCC_Set_GPIO
#define spUART5_RCC_GPIO()      spRCC_Set_GPIO
#define spUSART6_RCC_GPIO()     spRCC_Set_GPIOG()
#define spUART7_RCC_GPIO()      spRCC_Set_GPIOE()
#define spUART8_RCC_GPIO()      spRCC_Set_GPIOE()

/**
  * @brief  USART pin resource config
  */
#ifdef USING_SP_USART1_TX
#define spUSART1_GPIO_TX        GPIOB
#define spUSART1_TX_Source      GPIO_PinSource6
#endif
#ifdef USING_SP_USART1_RX
#define spUSART1_GPIO_RX        GPIOB
#define spUSART1_RX_Source      GPIO_PinSource7
#endif

#ifdef USING_SP_USART2_TX
#define spUSART2_GPIO_TX        GPIO
#define spUSART2_TX_Source      GPIO_PinSource
#endif
#ifdef USING_SP_USART2_RX
#define spUSART2_GPIO_RX        GPIO
#define spUSART2_RX_Source      GPIO_PinSource
#endif

#ifdef USING_SP_USART3_TX
#define spUSART3_GPIO_TX        GPIOD
#define spUSART3_TX_Source      GPIO_PinSource8
#endif
#ifdef USING_SP_USART3_RX
#define spUSART3_GPIO_RX        GPIOD
#define spUSART3_RX_Source      GPIO_PinSource9
#endif

#ifdef USING_SP_UART4_TX
#define spUART4_GPIO_TX         GPIO
#define spUART4_TX_Source       GPIO_PinSource
#endif
#ifdef USING_SP_UART4_RX
#define spUART4_GPIO_RX         GPIO
#define spUART4_RX_Source       GPIO_PinSource
#endif

#ifdef USING_SP_UART5_TX
#define spUART5_GPIO_TX         GPIO
#define spUART5_TX_Source       GPIO_PinSource
#endif
#ifdef USING_SP_UART5_RX
#define spUART5_GPIO_RX         GPIO
#define spUART5_RX_Source       GPIO_PinSource
#endif

#ifdef USING_SP_USART6_TX
#define spUSART6_GPIO_TX        GPIOG
#define spUSART6_TX_Source      GPIO_PinSource14
#endif
#ifdef USING_SP_USART6_RX
#define spUSART6_GPIO_RX        GPIOG
#define spUSART6_RX_Source      GPIO_PinSource9
#endif

#ifdef USING_SP_UART7_TX
#define spUART7_GPIO_TX         GPIOE
#define spUART7_TX_Source       GPIO_PinSource8
#endif
#ifdef USING_SP_UART7_RX
#define spUART7_GPIO_RX         GPIOE
#define spUART7_RX_Source       GPIO_PinSource7
#endif

#ifdef USING_SP_UART8_TX
#define spUART8_GPIO_TX         GPIOE
#define spUART8_TX_Source       GPIO_PinSource1
#endif
#ifdef USING_SP_UART8_RX
#define spUART8_GPIO_RX         GPIOE
#define spUART8_RX_Source       GPIO_PinSource0
#endif
/** @} */



/** @defgroup Initialization
  * @brief    General USART Initialization and Configuration
  * @ingroup  USART
  * @{
  */

/** 
  * @brief  Config specific UASRT port with given baudrate
  * @param  usart: USARTx where x can be 1, 2, 3, 4, 5, 6, 7 or 8 to select the USART or 
  *         UART peripheral.
  *	@param  baudrate    Baud rate
  * @retval If configuration succeed
  * @note   Please use the same baudrate for a USART's RX and TX mode,
  *         or the baud rate will be the LAST given one.
  */
bool USART_RX_Config(USART_TypeDef* usart, uint32_t baudrate);
bool USART_TX_Config(USART_TypeDef* usart, uint32_t baudrate);

/** 
  * @brief  Config USARTx receive DMA
  * @param  usart   USARTx where x can be 1, 2, 3, 4, 5, 6, 7 or 8 to select the USART or 
  *         UART peripheral.
  * @param  size    Supposed data length
  * @param  isstart If start DMA on init finishing
  * @retval If configuration succeed
  * @note   Default to use CIRCULAR mode and DISABLE FIFO
  *         NO interruput configuration
  */
bool DMA_USART_RX_Config(USART_TypeDef* usart, uint32_t address, uint16_t buffsize, bool isstart);

/** 
  * @brief  Config USARTx transmission DMA
  * @param  usart   USARTx where x can be 1, 2, 3, 4, 5, 6, 7 or 8 to select the USART or 
  *         UART peripheral.
  * @param  size    Supposed data length
  * @param  isstart If start DMA on init finishing
  * @retval If configuration succeed
  * @note   Default to use NON-CIRCULAR mode and DISABLE FIFO
  *         NO interruput configuration
  *         Not start immediately
  */
bool DMA_USART_TX_Config(USART_TypeDef* usart);

/** 
  * @brief  USART baudrate config
  * @param  usart: USARTx where x can be 1, 2, 3, 4, 5, 6, 7 or 8 to select the USART or 
  *         UART peripheral.
  * @param  baudrate    Baud rate
  * @note   Code from @file stm32f4xx_usart.c
  */
void USART_SetBaudrate(USART_TypeDef* usart, uint32_t baudrate);
/** @} */
  
  
  
/** @defgroup User APIs
  * @brief    USART User Interfaces
  * @ingroup  USART
  * @{
  */
/** 
  * @brief  Block send message via USARTx
  * @param  usart: USARTx where x can be 1, 2, 3, 4, 5, 6, 7 or 8 to select the USART or 
  *         UART peripheral.
  *	@param  buffer Data array of @type uint8_t
  *	@param  len    Length of buffer
  */
void USART_Send(USART_TypeDef* usart, uint8_t *buffer, uint8_t len);
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

#endif /*__SP_USART_H */

/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/
