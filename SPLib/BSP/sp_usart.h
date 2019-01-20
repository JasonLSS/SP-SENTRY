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

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "stm32f4xx_dma.h"
#include "stm32f4xx_usart.h"
#include "sp_type.h"
#include "sp_gpio.h"
#include "sp_dma.h"

/* Exported types ------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/** @defgroup   General USART Initialization and Configuration
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

/**
  * @}
  */
  
  
  
/** @defgroup   USART User Interfaces
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

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /*__SP_USART_H */

/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/
