/**
  ******************************************************************************
  * @file       template.c
  * @author     YTom
  * @version    v0.0-alpha
  * @date       2018.Oct.21
  * @brief      Prohect configurations, based on your board.
  ******************************************************************************
  * @license
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SP_CONF_H
#define __SP_CONF_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Exported macro ------------------------------------------------------------*/
/** @defgroup Commonly Used Macros
  * @{
  */
#define spGPIO_FromSource(x)            (0x0001<<(x))   /* Resolve GPIO_Pin_X from GPIO_PinSourceX */
#define spUSART_PRINTF_USARTx           UART8           /* Select USART for overriding printf */
/**
  * @}
  */


/** @defgroup Test Configurations
  * @{
  */

/**
  * @brief  Choose bullet test case 42mm/17mm
  * @note   42mm using motor 203 and 204, 17mm using PWM and motor 201
  */
//#define USING_TEST42
#define USING_TEST17
//#define USING_CHASIS
//#define USING_SENTRY
//#define USING_DM6020
#define USING_USB

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
//#include "stm32f4xx_hal.h"
//#include "stm32f4xx_hal_def.h"
//#include "stm32f4xx_hal_conf.h"

#include "sp_math.h"
#include "sp_pid.h"
#include "sp_filter.h"
#include "sp_kalman.h"

#include "sp_type.h"
#include "sp_rcc.h"
#include "sp_gpio.h"
#include "sp_dma.h"
#include "sp_usart.h"
#include "sp_can.h"
#include "sp_runner.h"
#ifdef USING_USB
    #include "usb.h"
#endif

//#include "sp_motor.h"
//#include "sp_chasis.h"
//#include "sp_monitor.hpp"
//#include "sp_utility.h"
//#include "sp_rc.h"
//#include "mpu6500.h"
//#include "sp_runner.h"

/* Exported types ------------------------------------------------------------*/
typedef struct {
    DMA_Stream_TypeDef*             stream;
    uint32_t                        channel;
} spDMA_SelectorType;


/**
  * @}
  */


/** @defgroup Resource Using Configuration
  * @{
  */
#define USING_SP_USART1_TX
//#define USING_SP_USART2_TX
#define USING_SP_USART3_TX
//#define USING_SP_UART4_TX
//#define USING_SP_UART5_TX
#define USING_SP_USART6_TX
#define USING_SP_UART7_TX
#define USING_SP_UART8_TX

#define USING_SP_USART1_RX
//#define USING_SP_USART2_RX
#define USING_SP_USART3_RX
//#define USING_SP_UART4_RX
//#define USING_SP_UART5_RX
#define USING_SP_USART6_RX
#define USING_SP_UART7_RX
#define USING_SP_UART8_RX


//#define USING_MPU_DMP

/**
  * @}
  */




/** @defgroup USART Pin-Connect Configuration
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

/**
  * @}
  */



/** @defgroup DMA Stream and Channel Configuration
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
extern spDMA_SelectorType spDMA_Mem2Mem[];
//#define spDMA_NULL_stream           DMA2_Stream0
//#define spDMA_NULL_chnl             DMA_Channel_0

/**
  * @}
  */


/* Exported functions --------------------------------------------------------*/
/**
  * @brief    Assembly Delat Functions (sp_delay.s)
  */
extern void TASK_GlobalInit(void);
extern void TASK_TimerInit(void);
extern void TASK_Start(void);
extern void delay_us(uint32_t us);
extern void delay_ms(uint32_t ms);
extern uint32_t TASK_GetCounter(void);
extern void TASK_GetCounterPtr(unsigned long *count);
extern spTimeStamp TASK_GetTimeStamp(void);
extern float TASK_GetSecondFromTimeStamp(spTimeStamp* stamp);
extern void SysTick_Handler(void);

extern void EXTI9_5_IRQHandler(void);
extern void DMA1_Stream3_IRQHandler(void);
extern void DMA2_Stream1_IRQHandler(void);
extern void DMA1_Stream1_IRQHandler(void);
extern void TIM6_DAC_IRQHandler(void);
extern void USART1_IRQHandler(void);
extern void USART2_IRQHandler(void);
extern void USART3_IRQHandler(void);
extern void UART4_IRQHandler(void);
extern void UART5_IRQHandler(void);
extern void USART6_IRQHandler(void);
extern void UART7_IRQHandler(void);
extern void UART8_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /*__SP_CONF_H */

/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/
