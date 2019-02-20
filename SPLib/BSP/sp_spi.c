/**
  ******************************************************************************
  * @file       sp_spi.c
  * @author     YTom
  * @version    v0.1
  * @date       2019.Jan.21
  * @brief      CAN-bus module controller
  ******************************************************************************
  * @license
  *
  ******************************************************************************
  */

#include "sp_spi.h"

static inline uint16_t spSPI_Write(SPI_TypeDef* spi, uint16_t data) {
    while(!(spi->SR & SPI_SR_TXE));
    spi->DR = data;
    return 0;
}


static inline uint16_t spSPI_ReadWriteByte(SPI_TypeDef* spi, uint16_t data) {
    uint16_t retry=0;
    while(!(spi->SR & SPI_SR_TXE));
    spi->DR = data;
    retry=0;
    while(!(spi->SR & SPI_SR_RXNE) && retry<10000) retry++;
    return spi->DR;
}

static inline void spSPI_ChipSelect(SPI_PinsType* spi_pins)  {
    GPIO_ResetBits(spi_pins->nss.gpio, spGPIO_PinFromPinSource(spi_pins->nss.pin_source));
}

static inline void spSPI_ChipRelease(SPI_PinsType* spi_pins)  {
    GPIO_SetBits(spi_pins->nss.gpio, spGPIO_PinFromPinSource(spi_pins->nss.pin_source));
}




struct __SPI_Manager_Type spSPI_Manager = {
    .read_write_b = spSPI_ReadWriteByte,
    .write_b = spSPI_Write,
    .select = spSPI_ChipSelect,
    .release = spSPI_ChipRelease
};




void SPI5_Init(void){
    SPI_InitTypeDef     spi_initer;
    
    spRCC_Set_SPI5();
    spRCC_Set_GPIOF();
    
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource7, GPIO_AF_SPI5); 
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource8, GPIO_AF_SPI5);
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource9, GPIO_AF_SPI5);
    GPIO_AF_Config(GPIOF, GPIO_Pin_7 |  GPIO_Pin_8|  GPIO_Pin_9, GPIO_OType_PP,
        GPIO_PuPd_UP, GPIO_Speed_100MHz);
    
    GPIO_OUT_Config(GPIOF, GPIO_Pin_6, GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_100MHz);
    GPIO_SetBits(GPIOF, GPIO_Pin_6);
    
    /* f_APB2 = 84MHz, f_MPU_SCLK_Max = 1MHz */
    spi_initer.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
    spi_initer.SPI_CPOL         = SPI_CPOL_High;
    spi_initer.SPI_CPHA         = SPI_CPHA_2Edge;
    spi_initer.SPI_DataSize     = SPI_DataSize_8b;
    spi_initer.SPI_Direction    = SPI_Direction_2Lines_FullDuplex;
    spi_initer.SPI_FirstBit     = SPI_FirstBit_MSB;
    spi_initer.SPI_Mode         = SPI_Mode_Master;
    spi_initer.SPI_NSS          = SPI_NSS_Soft;
    // spi_initer.SPI_CRCPolynomial = 0x00;
    SPI_Init(SPI5, &spi_initer);
    
    SPI_Cmd(SPI5, ENABLE);
}

//void SPI5_DMA() {
//    DMA_InitTypeDef     dma_initer;
//    
//    SPI_I2S_DMACmd(SPI5, SPI_I2S_DMAReq_Rx, ENABLE);
//    
//    if((uint32_t)spDMA_SPI5_rx_stream < DMA2_BASE) {
//        spRCC_Set_DMA1();
//    } else {
//        spRCC_Set_DMA2();
//    }
//    DMA_DeInit(spDMA_SPI5_rx_stream);
//    while(DMA_GetCmdStatus(spDMA_SPI5_rx_stream)!=DISABLE); 
//    dma_initer.DMA_Channel               = spDMA_SPI5_rx_chnl;
//    dma_initer.DMA_DIR                   = DMA_DIR_PeripheralToMemory;
//    dma_initer.DMA_PeripheralBaseAddr    = (uint32_t)&(SPI5->DR);
//    dma_initer.DMA_Memory0BaseAddr       = (uint32_t)mpu_data_buffer;
//    dma_initer.DMA_BufferSize            = sizeof(mpu_data_buffer);
//    dma_initer.DMA_PeripheralInc         = DMA_PeripheralInc_Disable;
//    dma_initer.DMA_PeripheralDataSize    = DMA_PeripheralDataSize_Byte;
//    dma_initer.DMA_MemoryInc             = DMA_MemoryInc_Enable;
//    dma_initer.DMA_MemoryDataSize        = DMA_MemoryDataSize_Byte;  
//    dma_initer.DMA_Mode                  = DMA_Mode_Circular;
//    dma_initer.DMA_Priority              = DMA_Priority_High;
//    /* Disbale FIFO mode */
//    dma_initer.DMA_FIFOMode              = DMA_FIFOMode_Disable;
//    dma_initer.DMA_FIFOThreshold         = DMA_FIFOThreshold_1QuarterFull;
//    /* Trigger DMA transfer every single memory transfer. */
//    dma_initer.DMA_MemoryBurst           = DMA_MemoryBurst_Single;
//    /* Trigger DMA transfer every single peripheral transfer. */
//    dma_initer.DMA_PeripheralBurst       = DMA_PeripheralBurst_Single;
//    DMA_Init(spDMA_SPI5_rx_stream, &dma_initer);
//    DMA_Cmd(spDMA_SPI5_rx_stream, ENABLE);
//}

//uint8_t SPI5_Write(uint8_t data) {
//    while(!(SPI5->SR & SPI_SR_TXE));
//    SPI5->DR = data;
//    return 0;
//}


//uint8_t SPI5_ReadWriteByte(uint8_t data) {
//    uint16_t retry=0;
//    while(!(SPI5->SR & SPI_SR_TXE)){
//        retry++;
//        if(retry>600) {
//            return false;
//        }
//    }
//    SPI5->DR = data;
//    retry=0;
//    while(!(SPI5->SR & SPI_SR_RXNE)){
//        retry++;
//        if(retry>600){
//            return false;
//        }
//    }
//    return SPI5->DR;
//}



void SPI4_Init(void){
    
    spRCC_Set_SPI4();
    spRCC_Set_GPIOE();
    
    /* Config pins */
    GPIO_PinAFConfig(SPI4_Pins.sck.gpio, SPI4_Pins.sck.pin_source, GPIO_AF_SPI4);
    GPIO_PinAFConfig(SPI4_Pins.miso.gpio, SPI4_Pins.miso.pin_source, GPIO_AF_SPI4);
    GPIO_PinAFConfig(SPI4_Pins.mosi.gpio, SPI4_Pins.mosi.pin_source, GPIO_AF_SPI4);
    
    GPIO_AF_Config(SPI4_Pins.sck.gpio, spGPIO_PinFromPinSource(SPI4_Pins.sck.pin_source), GPIO_OType_PP,
        GPIO_PuPd_UP, GPIO_Speed_100MHz);
    GPIO_AF_Config(SPI4_Pins.miso.gpio, spGPIO_PinFromPinSource(SPI4_Pins.miso.pin_source), GPIO_OType_PP,
        GPIO_PuPd_UP, GPIO_Speed_100MHz);
    GPIO_AF_Config(SPI4_Pins.mosi.gpio, spGPIO_PinFromPinSource(SPI4_Pins.mosi.pin_source), GPIO_OType_PP,
        GPIO_PuPd_UP, GPIO_Speed_100MHz);
    
    GPIO_OUT_Config(SPI4_Pins.nss.gpio, spGPIO_PinFromPinSource(SPI4_Pins.nss.pin_source), GPIO_OType_PP, 
        GPIO_PuPd_UP, GPIO_Speed_100MHz);
    spSPI_Manager.release(&SPI4_Pins);
    
    /* f_APB2 = 84MHz, f_MPU_SCLK_Max = 1MHz */
    SPI4_Pins.spi_initer.SPI_BaudRatePrescaler    = SPI_BaudRatePrescaler_128;
    SPI4_Pins.spi_initer.SPI_CPOL                 = SPI_CPOL_High;
    SPI4_Pins.spi_initer.SPI_CPHA                 = SPI_CPHA_2Edge;
    SPI4_Pins.spi_initer.SPI_DataSize             = SPI_DataSize_16b;
    SPI4_Pins.spi_initer.SPI_Direction            = SPI_Direction_2Lines_FullDuplex;
    
    SPI4_Pins.spi_initer.SPI_FirstBit             = SPI_FirstBit_MSB;
    SPI4_Pins.spi_initer.SPI_Mode                 = SPI_Mode_Master;
    SPI4_Pins.spi_initer.SPI_NSS                  = SPI_NSS_Soft;
    // spi_initer.SPI_CRCPolynomial = 0x00;
    SPI_Init(SPI4_Pins.this, &SPI4_Pins.spi_initer);
    SPI_Cmd(SPI4_Pins.this, ENABLE);
    
    
//    spRCC_Set_SYSCFG();
//    GPIO_IN_Config(GPIOF, GPIO_Pin_1, GPIO_PuPd_UP, GPIO_Speed_100MHz);
//    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, GPIO_PinSource8); 
//    EXTI_InitTypeDef            exit_initer;
//    exit_initer.EXTI_Line       = EXTI_Line1;
//    exit_initer.EXTI_LineCmd    = ENABLE;
//    exit_initer.EXTI_Mode       = EXTI_Mode_Interrupt;
//    exit_initer.EXTI_Trigger    = EXTI_Trigger_Rising;
//    EXTI_Init(&exit_initer);
}


/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/

