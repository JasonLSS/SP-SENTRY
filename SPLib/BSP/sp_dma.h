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

/** @addtogroup SP
  * @brief      SuperPower
  * @{
  */

/** @defgroup DMA
  * @brief    DMA Module
  * @{
  */


#include "stm32f4xx_dma.h"
#include "sp_type.h"
#include "sp_rcc.h"

#include "sp_conf.h"


/** @defgroup Definations
  * @brief    Exported Macros And Definations
  * @ingroup  DMA
  * @{
  */
/** 
  * @brief  DMA stream and channel selectin pair type
  */
 typedef struct {
    DMA_Stream_TypeDef*             stream;         /*<! DMA stream selection as DMA[0,1]_Stream[0~7] */
    uint32_t                        channel;        /*<! DMA channel selection as DMA_Channel_[0~7] */
} spDMA_SelectorType;

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
/** @} */


/** @defgroup ChannelConfiguration
  * @brief    DMA Stream and Channel Configuration
  * @ingroup  DMA
  * @{
  */

/**
  * @brief    DMA for USARTx
  */
#define spDMA_USART1_tx_stream      DMA2_Stream7
#define spDMA_USART1_tx_chnl        DMA_Channel_4
// 2,2,4 2,5,4
#define spDMA_USART1_rx_stream      DMA2_Stream5
#define spDMA_USART1_rx_chnl        DMA_Channel_4


#define spDMA_USART2_tx_stream      DMA1_Stream6
#define spDMA_USART2_tx_chnl        DMA_Channel_4
#define spDMA_USART2_rx_stream      DMA1_Stream5
#define spDMA_USART2_rx_chnl        DMA_Channel_4


#define spDMA_USART3_tx_stream      DMA1_Stream3
#define spDMA_USART3_tx_chnl        DMA_Channel_4
#define spDMA_USART3_rx_stream      DMA1_Stream1
#define spDMA_USART3_rx_chnl        DMA_Channel_4


#define spDMA_UAR_4_tx_stream       DMA1_Stream4
#define spDMA_UART4_tx_chnl         DMA_Channel_4
#define spDMA_UART4_rx_stream       DMA1_Stream2
#define spDMA_UART4_rx_chnl         DMA_Channel_4


#define spDMA_UART5_tx_stream       DMA1_Stream7
#define spDMA_UART5_tx_chnl         DMA_Channel_4
#define spDMA_UART5_rx_stream       DMA1_Stream0
#define spDMA_UART5_rx_chnl         DMA_Channel_4


// 2,6,5 2,7,5
#define spDMA_USART6_tx_stream      DMA2_Stream6
#define spDMA_USART6_tx_chnl        DMA_Channel_5
// 2,1,5 2,2,5
#define spDMA_USART6_rx_stream      DMA2_Stream1
#define spDMA_USART6_rx_chnl        DMA_Channel_5


#define spDMA_UART7_tx_stream       DMA1_Stream1
#define spDMA_UART7_tx_chnl         DMA_Channel_5
#define spDMA_UART7_rx_stream       DMA1_Stream3
#define spDMA_UART7_rx_chnl         DMA_Channel_5


#define spDMA_UART8_tx_stream       DMA1_Stream0
#define spDMA_UART8_tx_chnl         DMA_Channel_5
#define spDMA_UART8_rx_stream       DMA1_Stream6
#define spDMA_UART8_rx_chnl         DMA_Channel_5

/**
  * @brief    DMA for ADCx
  */
#define spDMA_ACD1_stream_1         DMA2_Stream0
#define spDMA_ACD1_chnl_1           DMA_Channel_0
#define spDMA_ACD1_stream_2         DMA2_Stream4
#define spDMA_ACD1_chnl_2           DMA_Channel_0
#define spDMA_ACD2_stream_1         DMA2_Stream2
#define spDMA_ACD2_chnl_1           DMA_Channel_1
#define spDMA_ACD2_stream_2         DMA2_Stream3
#define spDMA_ACD2_chnl_2           DMA_Channel_1
#define spDMA_ACD3_stream_1         DMA2_Stream0
#define spDMA_ACD3_chnl_1           DMA_Channel_2
#define spDMA_ACD3_stream_2         DMA2_Stream1
#define spDMA_ACD3_chnl_2           DMA_Channel_2

/**
  * @brief    DMA for TIMx
  */
#define spDMA_TIM1_UP_stream        DMA2_Stream5
#define spDMA_TIM1_UP_chnl          DMA_Channel_6
#define spDMA_TIM1_CH1_stream       DMA2_Stream6
#define spDMA_TIM1_CH1_chnl         DMA_Channel_0 // DMA2_Stream1 DMA_Channel_6 DMA2_Stream3 DMA_Channel_6
#define spDMA_TIM1_CH2_stream       DMA2_Stream6
#define spDMA_TIM1_CH2_chnl         DMA_Channel_0 // DMA2_Stream2 DMA_Channel_6
#define spDMA_TIM1_CH3_stream       DMA2_Stream6
#define spDMA_TIM1_CH3_chnl         DMA_Channel_0 // DMA2_Stream4 DMA_Channel_6
#define spDMA_TIM1_CH4_stream       DMA2_Stream4
#define spDMA_TIM1_CH4_chnl         DMA_Channel_6
#define spDMA_TIM1_RXIG_stream      DMA2_Stream0
#define spDMA_TIM1_RXIG_chnl        DMA_Channel_6 // DMA2_Stream4 DMA_Channel_6
#define spDMA_TIM1_COM_stream       DMA2_Stream4
#define spDMA_TIM1_COM_chnl         DMA_Channel_6

#define spDMA_TIM2_UP_stream        DMA1_Stream1
#define spDMA_TIM2_UP_chnl          DMA_Channel_3 // DMA1_Stream7  DMA_Chaneel_3
#define spDMA_TIM2_CH3_stream       DMA1_Stream1
#define spDMA_TIM2_CH3_chnl         DMA_Channel_3
#define spDMA_TIM2_CH1_stream       DMA1_Stream5
#define spDMA_TIM2_CH1_chnl         DMA_Channel_3
#define spDMA_TIM2_CH2_stream       DMA1_Stream6
#define spDMA_TIM2_CH2_chnl         DMA_Channel_3
#define spDMA_TIM2_CH4_stream       DMA1_Stream6
#define spDMA_TIM2_CH4_chnl         DMA_Channel_3 // DMA1_Stream7 DMA_Chaneel_3
//#define spDMA_TIM2_UP_stream        DMA1_Stream7
//#define spDMA_TIM2_UP_chnl          DMA_Channel_3

#define spDMA_TIM3_UP_stream        DMA1_Stream2
#define spDMA_TIM3_UP_chnl          DMA_Channel_5
#define spDMA_TIM3_CH1_stream       DMA1_Stream4
#define spDMA_TIM3_CH1_chnl         DMA_Channel_5
#define spDMA_TIM3_CH2_stream       DMA1_Stream5
#define spDMA_TIM3_CH2_chnl         DMA_Channel_5
#define spDMA_TIM3_CH3_stream       DMA1_Stream7
#define spDMA_TIM3_CH3_chnl         DMA_Channel_5
#define spDMA_TIM3_CH4_stream       DMA1_Stream2
#define spDMA_TIM3_CH4_chnl         DMA_Channel_5
#define spDMA_TIM3_RXIG_stream      DMA1_Stream4
#define spDMA_TIM3_RXIG_chnl        DMA_Channel_5

#define spDMA_TIM4_UP_stream        DMA1_Stream6
#define spDMA_TIM4_UP_chnl          DMA_Channel_2
#define spDMA_TIM4_CH1_stream       DMA1_Stream0
#define spDMA_TIM4_CH1_chnl         DMA_Channel_2
#define spDMA_TIM4_CH2_stream       DMA1_Stream3
#define spDMA_TIM4_CH2_chnl         DMA_Channel_2
#define spDMA_TIM4_CH3_stream       DMA1_Stream7
#define spDMA_TIM4_CH3_chnl         DMA_Channel_2

#define spDMA_TIM5_UP_stream        DMA1_Stream0
#define spDMA_TIM5_UP_chnl          DMA_Channel_6 // DMA1_Stream6 DMA_Channel_6
#define spDMA_TIM5_CH1_stream       DMA1_Stream2
#define spDMA_TIM5_CH1_chnl         DMA_Channel_6
#define spDMA_TIM5_CH2_stream       DMA1_Stream4
#define spDMA_TIM5_CH2_chnl         DMA_Channel_6
#define spDMA_TIM5_CH3_stream       DMA1_Stream0
#define spDMA_TIM5_CH3_chnl         DMA_Channel_6
#define spDMA_TIM5_CH4_stream       DMA1_Stream1
#define spDMA_TIM5_CH4_chnl         DMA_Channel_6 // DMA1_Stream3 DMA_Channel_6
#define spDMA_TIM5_RXIG_stream      DMA1_Stream1
#define spDMA_TIM5_RXIG_chnl        DMA_Channel_6 // DMA1_Stream3 DMA_Channel_6

#define spDMA_TIM6_UP_stream        DMA1_Stream1
#define spDMA_TIM6_UP_chnl          DMA_Channel_7

#define spDMA_TIM7_UP_stream        DMA1_Stream2
#define spDMA_TIM7_UP_chnl          DMA_Channel_1 // DMA1_Stream4 DMA_Chaneel_1

#define spDMA_TIM8_UP_stream        DMA2_Stream1
#define spDMA_TIM8_UP_chnl          DMA_Channel_7
#define spDMA_TIM8_CH1_stream       DMA2_Stream2
#define spDMA_TIM8_CH1_chnl         DMA_Channel_0 // DMA2_Stream2 DMA_Channel_7
#define spDMA_TIM8_CH2_stream       DMA2_Stream2
#define spDMA_TIM8_CH2_chnl         DMA_Channel_0 // DMA2_Stream3 DMA_Channel_7
#define spDMA_TIM8_CH3_stream       DMA2_Stream2
#define spDMA_TIM8_CH3_chnl         DMA_Channel_0 // DMA2_Stream4 DMA_Channel_7
#define spDMA_TIM8_CH4_stream       DMA2_Stream7
#define spDMA_TIM8_CH4_chnl         DMA_Channel_7
#define spDMA_TIM8_RXIG_stream      DMA2_Stream7
#define spDMA_TIM8_RXIG_chnl        DMA_Channel_7
#define spDMA_TIM8_COM_stream       DMA2_Stream7
#define spDMA_TIM8_COM_chnl         DMA_Channel_7

/**
  * @brief    DMA for SPIx
  */
#define spDMA_SPI1_rx_stream        DMA1_Stream0 // DMA1_Stream2 DMA_Channel_3
#define spDMA_SPI1_rx_chnl          DMA_Channel_3
#define spDMA_SPI1_tx_stream        DMA1_Stream3 // DMA1_Stream5 DMA_Channel_3
#define spDMA_SPI1_tx_chnl          DMA_Channel_3

#define spDMA_SPI2_rx_stream        DMA1_Stream3
#define spDMA_SPI2_rx_chnl          DMA_Channel_0
#define spDMA_SPI2_tx_stream        DMA1_Stream4
#define spDMA_SPI2_tx_chnl          DMA_Channel_0

#define spDMA_SPI3_rx_stream        DMA1_Stream0 // DMA1_Stream2 DMA_Channel_0
#define spDMA_SPI3_rx_chnl          DMA_Channel_0
#define spDMA_SPI3_tx_stream        DMA1_Stream5 // DMA1_Stream7 DMA_Channel_0
#define spDMA_SPI3_tx_chnl          DMA_Channel_0

#define spDMA_SPI4_rx_stream        DMA1_Stream0 // DMA1_Stream3 DMA_Channel_4
#define spDMA_SPI4_rx_chnl          DMA_Channel_4
#define spDMA_SPI4_tx_stream        DMA1_Stream1 // DMA1_Stream4 DMA_Channel_4
#define spDMA_SPI4_tx_chnl          DMA_Channel_4

#define spDMA_SPI5_rx_stream        DMA2_Stream3 // DMA2_Stream5 DMA_Channel_7
#define spDMA_SPI5_rx_chnl          DMA_Channel_2
#define spDMA_SPI5_tx_stream        DMA2_Stream4 // DMA2_Stream6 DMA_Channel_7
#define spDMA_SPI5_tx_chnl          DMA_Channel_2

#define spDMA_SPI6_rx_stream        DMA1_Stream3
#define spDMA_SPI6_rx_chnl          DMA_Channel_2
#define spDMA_SPI6_tx_stream        DMA1_Stream4
#define spDMA_SPI6_tx_chnl          DMA_Channel_2

/**
  * @brief    DMA for Non-specific Usage
  * @note     List: 110 111 112 131 160 107 156 176
  *                 201 207 213 222 243 244 255 270 276
  */
//extern spDMA_SelectorType spDMA_Mem2Mem[];
//#define spDMA_NULL_stream           DMA2_Stream0
//#define spDMA_NULL_chnl             DMA_Channel_0

/** @} */


/** @defgroup Decalarations
  * @brief    Exported Function Decalarations
  * @ingroup  DMA
  * @{
  */
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
