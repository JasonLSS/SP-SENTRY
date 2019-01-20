/**
  ******************************************************************************
  * @file       sp_dma.c
  * @author     YTom
  * @version    v0.0-alpha
  * @date       2018.Nov.24
  * @brief      DMA module driver
  * @note       DMA2 only supports transmission between peripherals.
  ******************************************************************************
  * @license
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SP_DMA_H
#define __SP_DMA_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_dma.h"
#include "sp_type.h"
#include "sp_rcc.h"

/* Exported types ------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/** @defgroup DMA_Stream_FlagBits
  * @{
  */
/** 
  * @brief  Basic flag mask for a DMA stream, 
  *         should use @func DMA_ClearStreamFlagBit to make it useful
  */
#define DMA_CR_CTCIFx           ((uint32_t)0x20)
#define DMA_CR_CHTIFx           ((uint32_t)0x10)
#define DMA_CR_CTEIFx           ((uint32_t)0x08)
#define DMA_CR_CDMEIFx          ((uint32_t)0x04)
#define DMA_CR_CFEIFx           ((uint32_t)0x01)
#define DMA_CRx                 (DMA_CR_CTCIFx|DMA_CR_CHTIFx|DMA_CR_CTEIFx|DMA_CR_CDMEIFx|DMA_CR_CFEIFx)
/**
  * @}
  */



/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
///** 
//  * @brief  Enable DMA with given data size
//  * @param  stream  DMA stream type @ref DMAy_Streamx: where y can be 1 or 2 to select the DMA and x can be 0
//  *                 to 7 to select the DMA Stream, or use @ref spDMA_xxx_stream, like @arg spDMA_USART1_tx_stream.
//  * @param  size    Number of data items for transfering, which depends only on the Peripheral data format.
//  * @retval
//  */
//bool DMA_Start(DMA_Stream_TypeDef * stream, uint16_t size);
//  
///** 
//  * @brief  Disable DMA and interrupt
//  * @param  stream  DMA stream type @ref DMAy_Streamx: where y can be 1 or 2 to select the DMA and x can be 0
//  *                 to 7 to select the DMA Stream, or use @ref spDMA_xxx_stream, like @arg spDMA_USART1_tx_stream.
//  * @retval
//  */
//void DMA_Stop(DMA_Stream_TypeDef * stream);

/** 
  * @brief  Clear/Set Stream Flag Bits
  * @param  stream  DMA stream type @ref DMAy_Streamx: where y can be 1 or 2 to select the DMA and x can be 0
  *                 to 7 to select the DMA Stream, or use @ref spDMA_xxx_stream, like @arg spDMA_USART1_tx_stream.
  * @param  flag    DMA stream flag mask @ref DMA_Stream_FlagBits
  *                 @arg DMA_CR_CTCIFx      Transfer complete flag
  *                 @arg DMA_CR_CHTIFx     Half-Transfer Complete flag
  *                 @arg DMA_CR_CTEIFx     Transfer Error flag
  *                 @arg DMA_CR_CDMEIFx    Direct Mode Transfer Error flag
  *                 @arg DMA_CR_CFEIFx     FIFO Mode Transfer Error flag
  *                 @arg DMA_CR_CRx        Mask for all flags 
  * @retval
  */
void DMA_ClearStreamFlagBit(DMA_Stream_TypeDef* stream, uint32_t flag);
bool DMA_GetStreamFlagBit(DMA_Stream_TypeDef* stream, uint32_t flag);

/** 
  * @brief  Start a DMA stream with no channel config for memory-to-memory transfer
  * @param  addr_from   Address of source data, can be @arg NULL
  * @param  addr_to     Address of destination data, can be @arg NULL
  * @param  buffsize    Size of data for transfering, can be @arg NULL/0/0x00 etc.
  * @note   Use @func DMA_SendOnce() to start new transfer.
  */
bool DMA_InitNull(uint8_t* addr_from, uint8_t* addr_to, uint16_t buffsize);

/** 
  * @brief  Start once DMA transmission
  * @param  stream  DMA stream type @ref DMAy_Streamx: where y can be 1 or 2 to select the DMA and x can be 0
  *                 to 7 to select the DMA Stream, or use @ref spDMA_xxx_stream, like @arg spDMA_USART1_tx_stream.
  * @param  target  DMA target address
  * @param  buffer  Sending data via DMA
  * @param  len    Size of the sending data
  * @retval If DMA transfer started.
  * @note   Default sending from @arg target(MEMORY) to @arg buffer(PHERIPHERAL)
  *         Actually from where to where depends on DMA stream setting.
  * @note   ATTENTION!!!  the DMA mode will be changed to NON-CIRCULAR mode
  */
void DMA_SendOnce(DMA_Stream_TypeDef * stream, uint32_t target, uint32_t buffer, uint16_t len);


/** 
  * @brief  Start once DMA transmission between memory and memory
  * @param  target  DMA target address
  * @param  buffer  Sending data via DMA
  * @param  len    Size of the sending data
  * @retval If success return @ref DMA_Stream_TypeDef* , else NULL
  * @note   Auto-select DAM stream
  * @note   ATTENTION!!!  the DMA mode will be changed to NON-CIRCULAR mode
  */
DMA_Stream_TypeDef* DMA_CopyMem2Mem(uint32_t target, uint32_t buffer, uint16_t len);


#ifdef __cplusplus
}
#endif

#endif /*__SP_USART_H */

/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/
