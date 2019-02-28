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

//*** <<< Use Configuration Wizard in Context Menu >>> ***

#ifndef __SP_CONF_H
#define __SP_CONF_H

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



/** @defgroup Commonly_Used_Macros
  * @ingroup  CONFIG
  * @{
  */
#if !defined(SP19)
    #error "Please specify SPxx in your porject!"
#endif

#if !defined(SP_FANTRY) && !defined(SP_HERO) && !defined(SP_ENGENEER) \
    && !defined(SP_SENTRY) && !defined(SP_TEST)
    #error "Please specify your robot type from SP_FANTRY, SP_HERO, SP_ENGENEER, SP_SENTRY or SP_TEST!"
#endif

#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include <core_cm4.h>

/* __packed keyword used to decrease the data type alignment to 1-byte */
#if defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
    #define packed_struct       struct __attribute__((packed))
    #define packed_union        union __attribute__((packed))
#else
    #define packed_struct       __packed struct
    #define packed_union        __packed union
#endif

#define spGPIO_FromSource(x)            (0x0001<<(x))   /* Resolve GPIO_Pin_X from GPIO_PinSourceX */
#define spUSART_PRINTF_USARTx           UART8           /* Select USART for overriding printf */

//#define spCLOCK                         Systick
//#define spRCC_Set_spCLOCK               spRCC_Set_TIM14
//#define spClock_IRQn                    TIM8_TRG_COM_TIM14_IRQn
//#define spClock_IRQHandler              TIM8_TRG_COM_TIM14_IRQHandler
    
#define spSYSTIMER                      TIM14
#define spRCC_Set_SYSTIMER              spRCC_Set_TIM14
#define spSYSTIMER_IRQn                 TIM8_TRG_COM_TIM14_IRQn
    
/** @} */



/** @defgroup Test_Configurations
  * @ingroup  CONFIG
  * @{
  */

/**
  * @brief  Choose bullet test case 42mm/17mm
  * @note   42mm using motor 203 and 204, 17mm using PWM and motor 201
  */
//  <h> Test Module Select
//  <c1> USING_CHASIS
//    <i> USING chasis part
#define USING_CHASIS
//  </c>
//  <o>  Gimbal Mode  <0=>Disable  <1=>Normal  <2=>Debug
//       <i> Selects Active Bits in Base Address
#define USING_GIMBAL_MODE       0
//  <c1> USING_USB
//    <i> Enable USB
#define USING_USB
//  </c>
//  <c1> USING_OS
//    <i> Enable FreeRTOS
//#define USING_OS
//  </c>
//  </h>

#define Latitude_At_ShenZhen 22.57025f
#define Latitude_At_ShangHai 31.18333f
/** @} */


/** @defgroup Pool_Size_Configuration
  * @ingroup  CONFIG
  * @{
  */
//  <h> IRQ callback pool size
//    <o> USING_IRQ_POOL_SIZE       <32-256>
#define USING_IRQ_POOL_SIZE         128
//  </h>
/** @} */  

  


/** @defgroup USART_Resource_Using_Configuration
  * @ingroup  CONFIG
  * @{
  */

//  <h> USART Select
//    <h> USART1
//      <c1> USING_SP_USART1_TX
//        <i> Enable USART1 transmit
//#define USING_SP_USART1_TX
//      </c>
//      <c1> USING_SP_USART1_RX
//        <i> Enable USART1 read
#define USING_SP_USART1_RX
//      </c>
//    </h>

//    <h> USART2
//      <c1> USING_SP_USART2_TX
//        <i> Enable USART2 transmit
#define USING_SP_USART2_TX
//      </c>
//      <c1> USING_SP_USART2_RX
//        <i> Enable USART2 read
#define USING_SP_USART2_RX
//      </c>
//    </h>

//    <h> USART3
//      <c1> USING_SP_USART3_TX
//        <i> Enable USART3 transmit
#define USING_SP_USART3_TX
//      </c>
//      <c1> USING_SP_USART3_RX
//        <i> Enable USART3 read
#define USING_SP_USART3_RX
//      </c>
//    </h>

//    <h> UART4
//      <c1> USING_SP_UART4_TX
//        <i> Enable UART4 transmit
//#define USING_SP_UART4_TX
//      </c>
//      <c1> USING_SP_UART4_RX
//        <i> Enable UART4 read
//#define USING_SP_UART4_RX
//      </c>
//    </h>

//    <h> UART5
//      <c1> USING_SP_UART5_TX
//        <i> Enable UART5 transmit
//#define USING_SP_UART5_TX
//      </c>
//      <c1> USING_SP_UART5_RX
//        <i> Enable UART5 read
//#define USING_SP_UART5_RX
//      </c>
//    </h>

//    <h> USART6
//      <c1> USING_SP_USART6_TX
//        <i> Enable USART6 transmit
#define USING_SP_USART6_TX
//      </c>
//      <c1> USING_SP_USART6_RX
//        <i> Enable USART6 read
#define USING_SP_USART6_RX
//      </c>
//    </h>

//    <h> UART7
//      <c1> USING_SP_UART7_TX
//        <i> Enable UART7 transmit
#define USING_SP_UART7_TX
//      </c>
//      <c1> USING_SP_UART7_RX
//        <i> Enable UART7 read
#define USING_SP_UART7_RX
//      </c>
//    </h>

//    <h> UART8
//      <c1> USING_SP_UART8_TX
//        <i> Enable UART8 transmit
#define USING_SP_UART8_TX
//      </c>
//      <c1> USING_SP_UART8_RX
//        <i> Enable UART8 read
#define USING_SP_UART8_RX
//      </c>
//    </h>
//  </h>
/** @} */


/** @defgroup Module_Includes
  * @ingroup  CONFIG
  * @{
  */
#include "sp_type.h"
#include "sp_irq.h"
#include "sp_rcc.h"
#include "sp_gpio.h"
#include "sp_dma.h"
#include "sp_usart.h"
#include "sp_can.h"


#ifdef USING_USB
    #include "usb.h"
#endif

#ifdef USING_OS
    #include "FreeRTOSConfig.h"
    #include "FreeRTOS.h"
    #include "task.h"
    #include "timers.h"
#endif
/** @} */

/** @defgroup SuperPower_Tasks
  * @ingroup  CONFIG
  * @{
  */

extern void TASK_GlobalInit(void);
extern void spCLOCK_Configuration(void);

#ifndef USING_OS
extern void TASK_TimerInit(void);
extern void TASK_Start(void);
#else

#include "sp_delegate.h"
extern void TASK_MonitorTaskInit(void);
extern void TASK_DetectTaskInit(void);
extern void TASK_ExecutorTaskInit(void);
extern void TASK_ClearTaskInit(void);
extern void TASK_UITaskInit(void);

extern TimerHandle_t spDetectTimer;
extern TimerHandle_t spMonitorTimer;
extern TimerHandle_t spExecutorTimer;
extern TimerHandle_t spClearTimer;
extern TimerHandle_t spUITimer;

extern void TASK_MonitorLooper(TimerHandle_t xTimer);
extern void TASK_DetectLooper(TimerHandle_t xTimer);
extern void TASK_ExecutorLooper(TimerHandle_t xTimer);
extern void TASK_ClearLooper(TimerHandle_t xTimer);
extern void TASK_UILooper(TimerHandle_t xTimer);
#endif
/** @} */

  

/** @defgroup General_System_Functions
  * @ingroup  CONFIG
  * @{
  */
extern void delay_us(uint32_t us);
extern void delay_ms(uint32_t ms);
extern uint32_t TASK_GetMicrosecond(void);
extern void TASK_GetMicrosecondPtr(unsigned long *count);
extern spTimeStamp TASK_GetTimeStamp(void);
extern float TASK_GetSecond(void);
//extern float TASK_GetSecondFromTimeStamp(spTimeStamp* stamp);
/** @} */
  
  
/** @defgroup IRQ_Handlers
  * @ingroup  CONFIG
  * @{
  */
extern void TIM8_TRG_COM_TIM14_IRQHandler(void);
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
extern void SysTick_Handler(void);
/** @} */
  
#ifdef __cplusplus
}
#endif

/**
  * @}
  */
  
/**
  * @}
  */

//*** <<< end of configuration section >>>    ***
#endif /*__SP_CONF_H */

/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/
