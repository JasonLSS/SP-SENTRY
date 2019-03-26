/**
  ******************************************************************************
  * @file       sp_usart.c
  * @author     YTom
  * @version    v0.0-alpha
  * @date       2018.Nov.24
  * @brief      USART module driver
  @verbatim      
 ===============================================================================
                   #####  How to use this driver #####
 ===============================================================================
      (#) Start USART and DMA
        (++) USART_TX_Config(USART3, 115200);
        (++) USART_RX_Config(USART3, 115200);
        (++) DMA_USART_RX_Config(USART3, (uint32_t)uart7_buff, sizeof(uart7_buff), true);
        (++) DMA_USART_TX_Config(USART3);
      (#) Enable interrupt
        (++) DMA_ITConfig(spDMA_USART3_rx_stream, DMA_IT_TC, ENABLE);
        (++) DMA_Cmd(spDMA_USART3_rx_stream, ENABLE);
        (++) NVIC_IRQEnable(DMA1_Stream1_IRQn, 2, 1);
        (++) USART speed can reach to 7372800(bps) on USART6 with OverSampling 8bit.
        (++) But frame error occours offen.
  @endverbatim
  ******************************************************************************
  * @license
  *
  ******************************************************************************
  */

#include "sp_usart.h"
#include "sp_conf.h"

/** @defgroup Override_MicroLIBs
  * @brief    Override inner @ref printf() to suppoer UASRT printf without MicroLIB
  * @{
  */
#if 1
#if defined ( __CC_ARM )
#pragma import(__use_no_semihosting_swi)
#pragma import(_main_redirection)

#elif defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
#pragma asm(" .global __use_no_semihosting\n");
#pragma asm(" .global _main_redirection\n");

#elif defined ( __GNUC__ )


#elif defined ( __ICCARM__ )

#elif defined ( __TI_ARM__ )


#elif defined ( __TASKING__ )


#elif defined ( __CSMC__ )

#endif


char __stdin_name[150];
char __stdout_name[150];
char __stderr_name[150];
typedef int FILEHANDLE;
/**
  * @brief  Using for avoid semi-hosting mode.
  * @note   Semi-hosting means send debug message to host with SWI interrput.
  */ 
void _sys_exit(int status)
{
    while(1);
}
FILEHANDLE _sys_open(const char *name, int openmode)
{
    return 0;
}
 
int _sys_close(FILEHANDLE fh)
{
    return 0;
}
 
int _sys_write(FILEHANDLE fh, const unsigned char *buf, unsigned len, int mode)
{
    return 0;
}
 
int _sys_read(FILEHANDLE fh, unsigned char*buf, unsigned len, int mode)
{
    return 0;
}
 
int _sys_istty(FILEHANDLE fh)
{
    return 0;
}
 
int _sys_seek(FILEHANDLE fh, long pos)
{
    return 0;
}
 
int _sys_ensure(FILEHANDLE fh)
{
    return 0;
}
 
long _sys_flen(FILEHANDLE fh)
{
    return 0;
}
 
int _sys_tmpnam(char *name, int fileno, unsigned maxlength)
{
    return 0;
}
 
int _ttywrch(int ch)
{
    return 0;
}
int remove(const char *filename)
{
    return 0;
}
 
char *_sys_command_string(char *cmd, int len)
{
    return 0;
}
 
int clock(void)
{
    return 0;
}

/**
  * @brief  Standard library support type
  */
//struct __FILE {
//    int handle; 
//}; 
/**
  * @brief  Override printf function
  */
int fputc(int ch, FILE *f) {
    /* Waiting for transmission complete */
    while( (spUSART_PRINTF_USARTx->SR & USART_CR1_TCIE)==RESET );
    spUSART_PRINTF_USARTx->DR = (uint8_t) ch;
    return ch;
}
#endif

/** @} */



void USART_SetBaudrate(USART_TypeDef* usart, uint32_t baudrate) {
    uint32_t tmpreg = 0x00, apbclock = 0x00;
    uint32_t integerdivider = 0x00;
    uint32_t fractionaldivider = 0x00;
    RCC_ClocksTypeDef RCC_ClocksStatus;
    
    /* Configure the USART Baud Rate */
    RCC_GetClocksFreq(&RCC_ClocksStatus);
    if ((usart == USART1) || (usart == USART6)) {
        apbclock = RCC_ClocksStatus.PCLK2_Frequency;
    } else {
        apbclock = RCC_ClocksStatus.PCLK1_Frequency;
    }
    /* Determine the integer part */
    if ((usart->CR1 & USART_CR1_OVER8) != 0) {
        /* Integer part computing in case Oversampling mode is 8 Samples */
        integerdivider = ((25 * apbclock) / (2 * baudrate));    
    } else /* if ((usart->CR1 & USART_CR1_OVER8) == 0) */ {
        /* Integer part computing in case Oversampling mode is 16 Samples */
        integerdivider = ((25 * apbclock) / (4 * baudrate));    
    }
    tmpreg = (integerdivider / 100) << 4;
    /* Determine the fractional part */
    fractionaldivider = integerdivider - (100 * (tmpreg >> 4));
    /* Implement the fractional part in the register */
    if ((usart->CR1 & USART_CR1_OVER8) != 0) {
        tmpreg |= ((((fractionaldivider * 8) + 50) / 100)) & ((uint8_t)0x07);
    } else /* if ((usart->CR1 & USART_CR1_OVER8) == 0) */ {
        tmpreg |= ((((fractionaldivider * 16) + 50) / 100)) & ((uint8_t)0x0F);
    }
    /* Write to USART BRR register */
    usart->BRR = (uint16_t)tmpreg;
}

bool USART_TX_Config(USART_TypeDef* usart, uint32_t baudrate) {
    USART_InitTypeDef       USART_InitStructure;
  #ifdef USING_SP_USART1_TX
    if(usart == USART1) {
        spRCC_Set_USART1();
        spUSART1_RCC_GPIO();
        GPIO_PinAFConfig(spUSART1_GPIO_TX, spUSART1_TX_Source, GPIO_AF_USART1);
        spGPIO_Controllers.alternal_config(spUSART1_GPIO_TX, 
            spGPIO_PinFromPinSource(spUSART1_TX_Source),
            GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_50MHz);
        goto uart_configuration;
    }
  #endif
  #ifdef USING_SP_USART2_TX
    if(usart == USART2) {
        spRCC_Set_USART2();
        spUSART2_RCC_GPIO();
        GPIO_PinAFConfig(spUSART2_GPIO_TX, spUSART2_TX_Source, GPIO_AF_USART2);
        spGPIO_Controllers.alternal_config(spUSART2_GPIO_TX, 
            spGPIO_PinFromPinSource(spUSART2_TX_Source),
            GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_50MHz);
        goto uart_configuration;
    }
  #endif
  #ifdef USING_SP_USART3_TX
    if(usart == USART3) {
        spRCC_Set_USART3();
        spUSART3_RCC_GPIO();
        GPIO_PinAFConfig(spUSART3_GPIO_TX, spUSART3_TX_Source, GPIO_AF_USART3);
        spGPIO_Controllers.alternal_config(spUSART3_GPIO_TX, 
            spGPIO_PinFromPinSource(spUSART3_TX_Source),
            GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_50MHz);
        goto uart_configuration;
    }
  #endif
  #ifdef USING_SP_UART4_TX
    if(usart == UART4) {
        spRCC_Set_UART4();
        spUART4_RCC_GPIO();
        GPIO_PinAFConfig(spUART4_GPIO_TX, spUART4_TX_Source, GPIO_AF_UART4);
        spGPIO_Controllers.alternal_config(spUART4_GPIO_TX, 
            spGPIO_PinFromPinSource(spUART4_TX_Source),
            GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_50MHz);
        goto uart_configuration;
    }
  #endif
  #ifdef USING_SP_UART5_TX
    if(usart == UART5) {
        spRCC_Set_UART5();
        spUART5_RCC_GPIO();
        GPIO_PinAFConfig(spUART5_GPIO_TX, spUART5_TX_Source, GPIO_AF_UART5);
        spGPIO_Controllers.alternal_config(spUART5_GPIO_TX, 
            spGPIO_PinFromPinSource(spUART5_TX_Source),
            GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_50MHz);
        goto uart_configuration;
    }
  #endif
  #ifdef USING_SP_USART6_TX
    if(usart == USART6) {
        spRCC_Set_USART6();
        spUSART6_RCC_GPIO();
        GPIO_PinAFConfig(spUSART6_GPIO_TX, spUSART6_TX_Source, GPIO_AF_USART6);
        spGPIO_Controllers.alternal_config(spUSART6_GPIO_TX, 
            spGPIO_PinFromPinSource(spUSART6_TX_Source),
            GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_50MHz);
        goto uart_configuration;
    }
  #endif
  #ifdef USING_SP_UART7_TX
    if(usart == UART7) {
        spRCC_Set_UART7();
        spUART7_RCC_GPIO();
        GPIO_PinAFConfig(spUART7_GPIO_TX, spUART7_TX_Source, GPIO_AF_UART7);
        spGPIO_Controllers.alternal_config(spUART7_GPIO_TX, 
            spGPIO_PinFromPinSource(spUART7_TX_Source),
            GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_50MHz);
        goto uart_configuration;
    }
  #endif
  #ifdef USING_SP_UART8_TX
    if(usart == UART8) {
        spRCC_Set_UART8();
        spUART8_RCC_GPIO();
        GPIO_PinAFConfig(spUART8_GPIO_TX, spUART8_TX_Source, GPIO_AF_UART8);
        spGPIO_Controllers.alternal_config(spUART8_GPIO_TX, 
            spGPIO_PinFromPinSource(spUART8_TX_Source),
            GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_50MHz);
        goto uart_configuration;
    }
  #endif
    return false;
uart_configuration:
    /* Only USART6 and USART1 can use oversampling-8bit, but use USART6 here. */
    if(usart==USART6) {
        USART_OverSampling8Cmd(usart, ENABLE);
    }
    if(usart->CR1 & USART_CR1_RE) {
        /* If USARTx has been initialized as RX mode */
        usart->CR1 |= USART_CR1_TE;
        USART_SetBaudrate(usart, baudrate);
    } else {
        /* Make configurations */
        USART_InitStructure.USART_BaudRate                  =     baudrate;                         /* Set baudrate */
        USART_InitStructure.USART_Mode                      =     USART_Mode_Tx;                    /* Set Tx mode */
        USART_InitStructure.USART_WordLength                =     USART_WordLength_8b;
        USART_InitStructure.USART_StopBits                  =     USART_StopBits_1;
        USART_InitStructure.USART_Parity                    =     USART_Parity_No;
        USART_InitStructure.USART_HardwareFlowControl       =     USART_HardwareFlowControl_None;
        
        USART_ClearFlag(usart, USART_FLAG_TC);
        
        USART_Init(usart, &USART_InitStructure);
//        USART_Cmd(usart, ENABLE);
    }
    
    return true;
}

bool USART_RX_Config(USART_TypeDef* usart, uint32_t baudrate) {
    USART_InitTypeDef       USART_InitStructure;
  #ifdef USING_SP_USART1_RX
    if(usart == USART1) {
        spRCC_Set_USART1();
        spUSART1_RCC_GPIO();
        GPIO_PinAFConfig(spUSART1_GPIO_RX, spUSART1_RX_Source, GPIO_AF_USART1);
        spGPIO_Controllers.alternal_config(spUSART1_GPIO_RX, 
            spGPIO_PinFromPinSource(spUSART1_RX_Source),
            GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_50MHz);
        goto uart_configuration;
    }
  #endif
  #ifdef USING_SP_USART2_RX
    if(usart == USART2) {
        spRCC_Set_USART2();
        spUSART2_RCC_GPIO();
        GPIO_PinAFConfig(spUSART2_GPIO_RX, spUSART2_RX_Source, GPIO_AF_USART2);
        spGPIO_Controllers.alternal_config(spUSART2_GPIO_RX, 
            spGPIO_PinFromPinSource(spUSART2_RX_Source),
            GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_50MHz);
        goto uart_configuration;
    }
  #endif
  #ifdef USING_SP_USART3_RX
    if(usart == USART3) {
        spRCC_Set_USART3();
        spUSART3_RCC_GPIO();
        GPIO_PinAFConfig(spUSART3_GPIO_RX, spUSART3_RX_Source, GPIO_AF_USART3);
        spGPIO_Controllers.alternal_config(spUSART3_GPIO_RX, 
            spGPIO_PinFromPinSource(spUSART3_RX_Source),
            GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_50MHz);
        goto uart_configuration;
    }
  #endif
  #ifdef USING_SP_UART4_RX
    if(usart == UART4) {
        spRCC_Set_UART4();
        spUART4_RCC_GPIO();
        GPIO_PinAFConfig(spUART4_GPIO_RX, spUART4_RX_Source, GPIO_AF_UART4);
        spGPIO_Controllers.alternal_config(spUART4_GPIO_RX, 
            spGPIO_PinFromPinSource(spUART4_RX_Source),
            GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_50MHz);
        goto uart_configuration;
    }
  #endif
  #ifdef USING_SP_UART5_RX
    if(usart == UART5) {
        spRCC_Set_UART5();
        spUART5_RCC_GPIO();
        GPIO_PinAFConfig(spUART5_GPIO_RX, spUART5_RX_Source, GPIO_AF_UART5);
        spGPIO_Controllers.alternal_config(spUART5_GPIO_RX, 
            spGPIO_PinFromPinSource(spUART5_RX_Source),
            GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_50MHz);
        goto uart_configuration;
    }
  #endif
  #ifdef USING_SP_USART6_RX
    if(usart == USART6) {
        spRCC_Set_USART6();
        spUSART6_RCC_GPIO();
        GPIO_PinAFConfig(spUSART6_GPIO_RX, spUSART6_RX_Source, GPIO_AF_USART6);
        spGPIO_Controllers.alternal_config(spUSART6_GPIO_RX, 
            spGPIO_PinFromPinSource(spUSART6_RX_Source),
            GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_50MHz);
        goto uart_configuration;
    }
  #endif
  #ifdef USING_SP_UART7_RX
    if(usart == UART7) {
        spRCC_Set_UART7();
        spUART7_RCC_GPIO();
        GPIO_PinAFConfig(spUART7_GPIO_RX, spUART7_RX_Source, GPIO_AF_UART7);
        spGPIO_Controllers.alternal_config(spUART7_GPIO_RX, 
            spGPIO_PinFromPinSource(spUART7_RX_Source),
            GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_50MHz);
        goto uart_configuration;
    }
  #endif
  #ifdef USING_SP_UART8_RX
    if(usart == UART8) {
        spRCC_Set_UART8();
        spUART8_RCC_GPIO();
        GPIO_PinAFConfig(spUART8_GPIO_RX, spUART8_RX_Source, GPIO_AF_UART8);
        spGPIO_Controllers.alternal_config(spUART8_GPIO_RX, 
            spGPIO_PinFromPinSource(spUART8_RX_Source),
            GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_50MHz);
        goto uart_configuration;
    }
  #endif
    return false;
uart_configuration:
    /* Only USART6 and USART1 can use oversampling-8bit, but use USART6 here. */
    if(usart==USART6) {
        USART_OverSampling8Cmd(usart, ENABLE);
    }
    if(usart->CR1 & USART_CR1_TE) {
        /* If USARTx has been initialized as TX mode */
        usart->CR1 |= USART_CR1_RE;
        USART_SetBaudrate(usart, baudrate);
    } else {
        /* Make configurations */
        USART_InitStructure.USART_BaudRate                  =     baudrate;                         /* Set baudrate */
        USART_InitStructure.USART_Mode                      =     USART_Mode_Rx;                    /* Set Rx mode */
        USART_InitStructure.USART_WordLength                =     USART_WordLength_8b;
        USART_InitStructure.USART_StopBits                  =     USART_StopBits_1;
        USART_InitStructure.USART_Parity                    =     USART_Parity_No;
        USART_InitStructure.USART_HardwareFlowControl       =     USART_HardwareFlowControl_None;
        
        USART_ClearFlag(usart, USART_FLAG_RXNE);
        
        USART_Init(usart, &USART_InitStructure);
//        USART_Cmd(usart, ENABLE);
    }
    
    return true;
}

void USART_Send(USART_TypeDef* usart, uint8_t *buffer, uint8_t len) {
    uint16_t sent_len = 0;
    while (sent_len < len) {
        /* Waiting for transmission complete */
        while( (usart->SR & USART_CR1_TCIE)==RESET );
        usart->DR = (uint8_t) buffer[sent_len++];
    }
}

bool DMA_USART_TX_Config(USART_TypeDef* usart) {
    DMA_Stream_TypeDef* dma_tx_stream;
    uint32_t            dma_tx_channel;
    DMA_InitTypeDef     DMA_InitStructure;
    
  #ifdef USING_SP_USART1_TX
    if(usart == USART1) {
        dma_tx_stream = spDMA_USART1_tx_stream;
        dma_tx_channel = spDMA_USART1_tx_chnl;
        goto uart_configuration;
    }
  #endif
  #ifdef USING_SP_USART2_TX
    if(usart == USART2) {
        dma_tx_stream = spDMA_USART2_tx_stream;
        dma_tx_channel = spDMA_USART2_tx_chnl;
        goto uart_configuration;
    }
  #endif
  #ifdef USING_SP_USART3_TX
    if(usart == USART3) {
        dma_tx_stream = spDMA_USART3_tx_stream;
        dma_tx_channel = spDMA_USART3_tx_chnl;
        goto uart_configuration;
    }
  #endif
  #ifdef USING_SP_UART4_TX
    if(usart == UART4) {
        dma_tx_stream = spDMA_UART4_tx_stream;
        dma_tx_channel = spDMA_UART4_tx_chnl;
        goto uart_configuration;
    }
  #endif
  #ifdef USING_SP_UART5_TX
    if(usart == UART5) {
        dma_tx_stream = spDMA_UART5_tx_stream;
        dma_tx_channel = spDMA_UART5_tx_chnl;
        goto uart_configuration;
  #endif
  #ifdef USING_SP_USART6_TX
    if(usart == USART6) {
        dma_tx_stream = spDMA_USART6_tx_stream;
        dma_tx_channel = spDMA_USART6_tx_chnl;
        goto uart_configuration;
    }
  #endif
  #ifdef USING_SP_UART7_TX
    if(usart == UART7) {
        dma_tx_stream = spDMA_UART7_tx_stream;
        dma_tx_channel = spDMA_UART7_tx_chnl;
        goto uart_configuration;
    }
  #endif
  #ifdef USING_SP_UART8_TX
    if(usart == UART8) {
        dma_tx_stream = spDMA_UART8_tx_stream;
        dma_tx_channel = spDMA_UART8_tx_chnl;
        goto uart_configuration;
    }
  #endif
    return false;
uart_configuration:
    /* Return false if DAM is occupied and not occupied by given object */
    if(dma_tx_stream->CR & DMA_SxCR_EN) {
        return false;
    }
    
    if((uint32_t)dma_tx_stream < DMA2_BASE) {
        spRCC_Set_DMA1();
    } else {
        spRCC_Set_DMA2();
    }
    // DMA_DeInit(dma_tx_stream);
    // while(DMA_GetCmdStatus(dma_tx_stream)!=DISABLE); 
    
    DMA_InitStructure.DMA_Channel               = dma_tx_channel;
    DMA_InitStructure.DMA_DIR                   = DMA_DIR_MemoryToPeripheral;
//    DMA_InitStructure.DMA_PeripheralBaseAddr    = (uint32_t)&(usart->DR);
//    DMA_InitStructure.DMA_Memory0BaseAddr       = address;
//    DMA_InitStructure.DMA_BufferSize            = buffsize;
    DMA_InitStructure.DMA_PeripheralInc         = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_PeripheralDataSize    = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryInc             = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_MemoryDataSize        = DMA_PeripheralDataSize_Byte;  
    DMA_InitStructure.DMA_Mode                  = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority              = DMA_Priority_High;
    /* Disbale FIFO mode */
    DMA_InitStructure.DMA_FIFOMode              = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold         = DMA_FIFOThreshold_1QuarterFull;
    /* Trigger DMA transfer every single memory transfer. */
    DMA_InitStructure.DMA_MemoryBurst           = DMA_MemoryBurst_Single;
    /* Trigger DMA transfer every single peripheral transfer. */
    DMA_InitStructure.DMA_PeripheralBurst       = DMA_PeripheralBurst_Single;
    DMA_Init(dma_tx_stream, &DMA_InitStructure);
    
    USART_DMACmd(usart, USART_DMAReq_Tx, ENABLE);
    /* Don't start DMA in tranmission mode */
    // DMA_Cmd(dma_tx_stream, ENABLE);
    
    return true;
}

bool DMA_USART_RX_Config(USART_TypeDef* usart, uint32_t address, uint16_t buffsize, bool isstart) {
    DMA_Stream_TypeDef* dma_rx_stream;
    uint32_t            dma_rx_channel;
    DMA_InitTypeDef     DMA_InitStructure;
    
  #ifdef USING_SP_USART1_RX
    if(usart == USART1) {
        dma_rx_stream = spDMA_USART1_rx_stream;
        dma_rx_channel = spDMA_USART1_rx_chnl;
        goto uart_configuration;
    }
  #endif
  #ifdef USING_SP_USART2_RX
    if(usart == USART2) {
        dma_rx_stream = spDMA_USART2_rx_stream;
        dma_rx_channel = spDMA_USART2_rx_chnl;
        goto uart_configuration;
    }
  #endif
  #ifdef USING_SP_USART3_RX
    if(usart == USART3) {
        dma_rx_stream = spDMA_USART3_rx_stream;
        dma_rx_channel = spDMA_USART3_rx_chnl;
        goto uart_configuration;
    }
  #endif
  #ifdef USING_SP_UART4_RX
    if(usart == UART4) {
        dma_rx_stream = spDMA_UART4_rx_stream;
        dma_rx_channel = spDMA_UART4_rx_chnl;
        goto uart_configuration;
    }
  #endif
  #ifdef USING_SP_UART5_RX
    if(usart == UART5) {
        dma_rx_stream = spDMA_UART5_rx_stream;
        dma_rx_channel = spDMA_UART5_rx_chnl;
        goto uart_configuration;
  #endif
  #ifdef USING_SP_USART6_RX
    if(usart == USART6) {
        dma_rx_stream = spDMA_USART6_rx_stream;
        dma_rx_channel = spDMA_USART6_rx_chnl;
        goto uart_configuration;
    }
  #endif
  #ifdef USING_SP_UART7_RX
    if(usart == UART7) {
        dma_rx_stream = spDMA_UART7_rx_stream;
        dma_rx_channel = spDMA_UART7_rx_chnl;
        goto uart_configuration;
    }
  #endif
  #ifdef USING_SP_UART8_RX
    if(usart == UART8) {
        dma_rx_stream = spDMA_UART8_rx_stream;
        dma_rx_channel = spDMA_UART8_rx_chnl;
        goto uart_configuration;
    }
  #endif
    return false;
uart_configuration:
    /* Return false if DAM is occupied */
    if(dma_rx_stream->CR & DMA_SxCR_EN) {
        return false;
    }
    
    if((uint32_t)dma_rx_stream < DMA2_BASE) {
        spRCC_Set_DMA1();
    } else {
        spRCC_Set_DMA2();
    }
    DMA_DeInit(dma_rx_stream);
    while(DMA_GetCmdStatus(dma_rx_stream)!=DISABLE); 
    
    DMA_InitStructure.DMA_Channel               = dma_rx_channel;
    DMA_InitStructure.DMA_DIR                   = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_PeripheralBaseAddr    = (uint32_t)&(usart->DR);
    DMA_InitStructure.DMA_Memory0BaseAddr       = address;
    DMA_InitStructure.DMA_BufferSize            = buffsize;
    DMA_InitStructure.DMA_PeripheralInc         = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_PeripheralDataSize    = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryInc             = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_MemoryDataSize        = DMA_PeripheralDataSize_Byte;  
    DMA_InitStructure.DMA_Mode                  = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority              = DMA_Priority_High;
    /* Disbale FIFO mode */
    DMA_InitStructure.DMA_FIFOMode              = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold         = DMA_FIFOThreshold_1QuarterFull;
    /* Trigger DMA transfer every single memory transfer. */
    DMA_InitStructure.DMA_MemoryBurst           = DMA_MemoryBurst_Single;
    /* Trigger DMA transfer every single peripheral transfer. */
    DMA_InitStructure.DMA_PeripheralBurst       = DMA_PeripheralBurst_Single;
    DMA_Init(dma_rx_stream, &DMA_InitStructure);
    
    USART_DMACmd(usart, USART_DMAReq_Rx, ENABLE);

    if(isstart) {
        DMA_Cmd(dma_rx_stream, ENABLE);
    }
    return true;
}


/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/
