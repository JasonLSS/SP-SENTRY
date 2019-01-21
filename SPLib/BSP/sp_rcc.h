/**
  ******************************************************************************
  * @file       sp_rcc.h
  * @author     YTom
  * @version    v0.0-alpha
  * @date       2018.Nov.24
  * @brief      General RCC config module
  * @usage      
  ******************************************************************************
  * @license
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SP_RCC_H
#define __SP_RCC_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_rcc.h"

/** @addtogroup SP
  * @brief      SuperPower
  * @{
  */

/** @defgroup RCC
  * @brief    RCC Module
  * @{
  */


/** @defgroup RCC_Peripherals
  * @brief    Interrupt Handlers for CAN1/CAN2
  * @ingroup  RCC
  * @{
  */

#define RCC_GPIOA             RCC_AHB1Periph_GPIOA
#define RCC_GPIOB             RCC_AHB1Periph_GPIOB
#define RCC_GPIOC             RCC_AHB1Periph_GPIOC
#define RCC_GPIOD             RCC_AHB1Periph_GPIOD
#define RCC_GPIOE             RCC_AHB1Periph_GPIOE
#define RCC_GPIOF             RCC_AHB1Periph_GPIOF
#define RCC_GPIOG             RCC_AHB1Periph_GPIOG
#define RCC_GPIOH             RCC_AHB1Periph_GPIOH
#define RCC_GPIOI             RCC_AHB1Periph_GPIOI
#define RCC_GPIOJ             RCC_AHB1Periph_GPIOJ
#define RCC_GPIOK             RCC_AHB1Periph_GPIOK
#define RCC_CRC               RCC_AHB1Periph_CRC
#define RCC_BKPSRAM           RCC_AHB1Periph_BKPSRAM
#define RCC_CCMDATARAMEN      RCC_AHB1Periph_CCMDATARAMEN
#define RCC_DMA1              RCC_AHB1Periph_DMA1
#define RCC_DMA2              RCC_AHB1Periph_DMA2
#define RCC_DMA2D             RCC_AHB1Periph_DMA2D
#define RCC_ETH_MAC           RCC_AHB1Periph_ETH_MAC
#define RCC_ETH_MAC_Tx        RCC_AHB1Periph_ETH_MAC_Tx
#define RCC_ETH_MAC_Rx        RCC_AHB1Periph_ETH_MAC_Rx
#define RCC_ETH_MAC_PTP       RCC_AHB1Periph_ETH_MAC_PTP
#define RCC_OTG_HS            RCC_AHB1Periph_OTG_HS
#define RCC_OTG_HS_ULPI       RCC_AHB1Periph_OTG_HS_ULPI

#define RCC_DCMI              RCC_AHB2Periph_DCMI
#define RCC_CRYP              RCC_AHB2Periph_CRYP
#define RCC_HASH              RCC_AHB2Periph_HASH
#define RCC_RNG               RCC_AHB2Periph_RNG
#define RCC_OTG_FS            RCC_AHB2Periph_OTG_FS

#define RCC_TIM2              RCC_APB1Periph_TIM2
#define RCC_TIM3              RCC_APB1Periph_TIM3
#define RCC_TIM4              RCC_APB1Periph_TIM4
#define RCC_TIM5              RCC_APB1Periph_TIM5
#define RCC_TIM6              RCC_APB1Periph_TIM6
#define RCC_TIM7              RCC_APB1Periph_TIM7
#define RCC_TIM12             RCC_APB1Periph_TIM12
#define RCC_TIM13             RCC_APB1Periph_TIM13
#define RCC_TIM14             RCC_APB1Periph_TIM14
#define RCC_LPTIM1            RCC_APB1Periph_LPTIM1
#define RCC_WWDG              RCC_APB1Periph_WWDG
#define RCC_SPI2              RCC_APB1Periph_SPI2
#define RCC_SPI3              RCC_APB1Periph_SPI3
#define RCC_SPDIF             RCC_APB1Periph_SPDIF
#define RCC_USART2            RCC_APB1Periph_USART2
#define RCC_USART3            RCC_APB1Periph_USART3
#define RCC_UART4             RCC_APB1Periph_UART4
#define RCC_UART5             RCC_APB1Periph_UART5
#define RCC_I2C1              RCC_APB1Periph_I2C1
#define RCC_I2C2              RCC_APB1Periph_I2C2
#define RCC_I2C3              RCC_APB1Periph_I2C3
#define RCC_FMPI2C1           RCC_APB1Periph_FMPI2C1
#define RCC_CAN1              RCC_APB1Periph_CAN1
#define RCC_CAN2              RCC_APB1Periph_CAN2
#define RCC_CEC               RCC_APB1Periph_CEC
#define RCC_PWR               RCC_APB1Periph_PWR
#define RCC_DAC               RCC_APB1Periph_DAC
#define RCC_UART7             RCC_APB1Periph_UART7
#define RCC_UART8             RCC_APB1Periph_UART8

#define RCC_TIM1              RCC_APB2Periph_TIM1
#define RCC_TIM8              RCC_APB2Periph_TIM8
#define RCC_USART1            RCC_APB2Periph_USART1
#define RCC_USART6            RCC_APB2Periph_USART6
#define RCC_ADC1              RCC_APB2Periph_ADC1
#define RCC_ADC2              RCC_APB2Periph_ADC2
#define RCC_ADC3              RCC_APB2Periph_ADC3
#define RCC_SDIO              RCC_APB2Periph_SDIO
#define RCC_SPI1              RCC_APB2Periph_SPI1
#define RCC_SPI4              RCC_APB2Periph_SPI4
#define RCC_SYSCFG            RCC_APB2Periph_SYSCFG
#define RCC_EXTIT             RCC_APB2Periph_EXTIT
#define RCC_TIM9              RCC_APB2Periph_TIM9
#define RCC_TIM10             RCC_APB2Periph_TIM10
#define RCC_TIM11             RCC_APB2Periph_TIM11
#define RCC_SPI5              RCC_APB2Periph_SPI5
#define RCC_SPI6              RCC_APB2Periph_SPI6
#define RCC_SAI1              RCC_APB2Periph_SAI1
#define RCC_SAI2              RCC_APB2Periph_SAI2
#define RCC_LTDC              RCC_APB2Periph_LTDC
#define RCC_DSI               RCC_APB2Periph_DSI
#define RCC_DFSDM1            RCC_APB2Periph_DFSDM1
#define RCC_DFSDM2            RCC_APB2Periph_DFSDM2
#define RCC_UART9             RCC_APB2Periph_UART9
#define RCC_UART10            RCC_APB2Periph_UART10
/** @} */



/** @defgroup Peripheral_Remap
  * @brief    Peripheral RCC config remap
  * @ingroup  RCC
  * @{
  */

/** 
  * @brief    Redefine @func RCC_SetPeripheral to a easier usage
  */
#define spRCC_Set_GPIOA() \
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE)
#define spRCC_Set_GPIOB() \
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE)
#define spRCC_Set_GPIOC() \
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE)
#define spRCC_Set_GPIOD() \
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE)
#define spRCC_Set_GPIOE() \
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE)
#define spRCC_Set_GPIOF() \
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE)
#define spRCC_Set_GPIOG() \
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE)
#define spRCC_Set_GPIOH() \
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE)
#define spRCC_Set_GPIOI() \
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE)
#define spRCC_Set_GPIOJ() \
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOJ, ENABLE)
#define spRCC_Set_GPIOK() \
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOK, ENABLE)
#define spRCC_Set_CRC() \
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_CRC, ENABLE)
#define spRCC_Set_BKPSRAM() \
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_BKPSRAM, ENABLE)
#define spRCC_Set_CCMDATARAMEN() \
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_CCMDATARAMEN, ENABLE)
#define spRCC_Set_DMA1() \
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE)
#define spRCC_Set_DMA2() \
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE)
#define spRCC_Set_DMA2D() \
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2D, ENABLE)
#define spRCC_Set_ETH_MAC() \
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_ETH_MAC, ENABLE)
#define spRCC_Set_ETH_MAC_Tx() \
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_ETH_MAC_Tx, ENABLE)
#define spRCC_Set_ETH_MAC_Rx() \
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_ETH_MAC_Rx, ENABLE)
#define spRCC_Set_ETH_MAC_PTP() \
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_ETH_MAC_PTP, ENABLE)
#define spRCC_Set_OTG_HS() \
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_OTG_HS, ENABLE)
#define spRCC_Set_OTG_HS_ULPI() \
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_OTG_HS_ULPI, ENABLE)

#define spRCC_Set_DCMI() \
     RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_DCMI, ENABLE)
#define spRCC_Set_CRYP() \
     RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_CRYP, ENABLE)
#define spRCC_Set_HASH() \
     RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_HASH, ENABLE)
#define spRCC_Set_RNG() \
     RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE)
#define spRCC_Set_OTG_FS() \
     RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_OTG_FS, ENABLE)

#define spRCC_Set_TIM2() \
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE)
#define spRCC_Set_TIM3() \
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE)
#define spRCC_Set_TIM4() \
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE)
#define spRCC_Set_TIM5() \
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE)
#define spRCC_Set_TIM6() \
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE)
#define spRCC_Set_TIM7() \
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE)
#define spRCC_Set_TIM12() \
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE)
#define spRCC_Set_TIM13() \
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, ENABLE)
#define spRCC_Set_TIM14() \
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE)
#define spRCC_Set_LPTIM1() \
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_LPTIM1, ENABLE)
#define spRCC_Set_WWDG() \
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE)
#define spRCC_Set_SPI2() \
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE)
#define spRCC_Set_SPI3() \
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE)
#define spRCC_Set_SPDIF() \
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPDIF, ENABLE)
#define spRCC_Set_USART2() \
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE)
#define spRCC_Set_USART3() \
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE)
#define spRCC_Set_UART4() \
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE)
#define spRCC_Set_UART5() \
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE)
#define spRCC_Set_I2C1() \
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE)
#define spRCC_Set_I2C2() \
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE)
#define spRCC_Set_I2C3() \
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C3, ENABLE)
#define spRCC_Set_FMPI2C1() \
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_FMPI2C1, ENABLE)
#define spRCC_Set_CAN1() \
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE)
#define spRCC_Set_CAN2() \
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE)
#define spRCC_Set_CEC() \
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CEC, ENABLE)
#define spRCC_Set_PWR() \
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE)
#define spRCC_Set_DAC() \
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE)
#define spRCC_Set_UART7() \
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART7, ENABLE)
#define spRCC_Set_UART8() \
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART8, ENABLE)

#define spRCC_Set_TIM1() \
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE)
#define spRCC_Set_TIM8() \
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE)
#define spRCC_Set_USART1() \
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE)
#define spRCC_Set_USART6() \
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE)
#define spRCC_Set_ADC1() \
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE)
#define spRCC_Set_ADC2() \
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE)
#define spRCC_Set_ADC3() \
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE)
#define spRCC_Set_SDIO() \
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SDIO, ENABLE)
#define spRCC_Set_SPI1() \
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE)
#define spRCC_Set_SPI4() \
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI4, ENABLE)
#define spRCC_Set_SYSCFG() \
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE)  \
//#define spRCC_Set_EXTIT() \
//   RCC_APB2PeriphClockCmd(RCC_APB2Periph_EXTIT, ENABLE)
#define spRCC_Set_TIM9() \
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE)
#define spRCC_Set_TIM10() \
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE)
#define spRCC_Set_TIM11() \
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11, ENABLE)
#define spRCC_Set_SPI5() \
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI5, ENABLE)
#define spRCC_Set_SPI6() \
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI6, ENABLE)
#define spRCC_Set_SAI1() \
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SAI1, ENABLE)
#define spRCC_Set_SAI2() \
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SAI2, ENABLE)
#define spRCC_Set_LTDC() \
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_LTDC, ENABLE)
#define spRCC_Set_DSI() \
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_DSI, ENABLE)
#define spRCC_Set_DFSDM1() \
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_DFSDM1, ENABLE)
#define spRCC_Set_DFSDM2() \
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_DFSDM2, ENABLE)
//#define spRCC_Set_GPIOx(gpio) \
//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_#gpio, ENABLE)

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

#endif /*__SP_RCC_H */

/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/
