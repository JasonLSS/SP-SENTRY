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

/** @defgroup CONFIG
  * @brief    Project config
  * @{
  */

/** @defgroup Commonly_Used_Macros
  * @ingroup  CONFIG
  * @{
  */
#if !defined(SP)
    #error "Please define SP in your porject!"
#endif

//  <c> If using board typeA
#define SP_USING_BOARD_TYPEA
//  </c>

//  <o>  Board Mode  <0=>singal  <1=>upper  <2=>bottom
//       <i> Selects Active Bits in Base Address
#define BOARD_MODE       1

//  <e1> Choose your robot type
//    <o> RobotType
//      <1=>FANTRY  <2=>HERO  <3=>ENGINEER  <4=>SENTRY  <5=>NONE
#define sp_choose_robot_type                 4
//  </e>

#if defined(sp_choose_robot_type)
    #if sp_choose_robot_type==0
        #define SP_INFANTRY
    #elif sp_choose_robot_type==2
        #define SP_HERO
    #elif sp_choose_robot_type==3
        #define SP_ENGINEER
    #elif sp_choose_robot_type==4
        #define SP_SENTRY
    #elif sp_choose_robot_type==5
        #define SP_TEST
    #endif
#endif

#if !defined(SP_INFANTRY) && !defined(SP_HERO) && !defined(SP_ENGINEER) \
    && !defined(SP_SENTRY) && !defined(SP_TEST)
    #error "Please specify your robot type from SP_INFANTRY, SP_HERO, SP_ENGENEER, SP_SENTRY or SP_TEST!"
#endif

#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include <core_cm4.h>

/* __packed keyword used to decrease the data type alignment to 1-byte */
#if defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 1)
    #define packed_struct       struct __attribute__((packed))
    #define packed_union        union __attribute__((packed))
#else
    #define packed_struct       __packed struct
    #define packed_union        __packed union
#endif
    
#define spINLINE __inline
#ifndef spFORCE_INLINE
    // #define portFORCE_INLINE __forceinline
    #define spFORCE_INLINE __attribute__((always_inline))
#endif
    
/** @} */



/** @defgroup Test_Configurations
  * @ingroup  CONFIG
  * @{
  */
//  <h> System Module Select
//  <o> spSYSTIMER
//    <1=>TIM13  <2=>TIM14
//    <i> USING chasis part
#define sp_choose_clock                 2
//  </h>

#define spUSART_PRINTF_USARTx           UART8   /*<! Select USART for overriding printf */
#if defined(sp_choose_clock)
    #if sp_choose_clock==1
        #define spSYSTIMER                      TIM13
        #define spRCC_Set_SYSTIMER              spRCC_Set_TIM13
        #define spSYSTIMER_IRQn                 TIM8_UP_TIM13_IRQHandler
    #elif sp_choose_clock==2
        #define spSYSTIMER                      TIM14
        #define spRCC_Set_SYSTIMER              spRCC_Set_TIM14
        #define spSYSTIMER_IRQn                 TIM8_TRG_COM_TIM14_IRQn
    #endif
#else
    #define spSYSTIMER                          TIM14
    #define spRCC_Set_SYSTIMER                  spRCC_Set_TIM14
    #define spSYSTIMER_IRQn                     TIM8_TRG_COM_TIM14_IRQn
#endif

#define spCLOCKTIMER                            spSYSTIMER
#define spRCC_Set_CLOCKTIMER                    spRCC_Set_SYSTIMER
#define spCLOCKTIMER_IRQn                       spSYSTIMER_IRQn


/**
  * @brief  Choose bullet test case 42mm/17mm
  * @note   42mm using motor 203 and 204, 17mm using PWM and motor 201
  */
//  <h> Memory distribute

/** @brief IRQ callback pool size */
// <o> IRQ callback pool size           <32-256>
#define USING_IRQ_POOL_SIZE             128

/** @brief Servo controller pool size */
// <o> Servo controller pool size       <2-32>
#define USING_SERVO_POOL_SIZE           12 

/** @brief Servo controller pool size */
// <o> Key pool size                 <0-32>
#define USING_KEY_POOLSIZE              12

/** @brief USB callback pool size */
// <o> USB pool size                 <0-32>
#define USING_USB_POOLSIZE              16

//  </h>


/**
  * @brief  Choose bullet test case 42mm/17mm
  * @note   42mm using motor 203 and 204, 17mm using PWM and motor 201
  */
//  <h> Test Module Select



//  <c1> USING_SENTRY_CHASIS
//    <i> USING_SENTRY_CHASIS
#define USING_SENTRY_CHASIS
//  </c>

//  <c1> USING_SPEEDBALANCE
//    <i> USING_SPEEDBALANCE
//#define USING_SPEED_BALANCE
//  </c>

//  <c1> USING_FRICTION
//    <i> USING_FRICTION
#define USING_FRICTION
//  </c>

//  <c1> USING_IMU
//    <i> USING_IMU
//#define USING_IMU
//  </c>

//  <c1> USING_FEED_MOTOR
//    <i> USING_FEED_MOTOR
#define USING_FEED_MOTOR
//  </c>

//  <c1> CHASIS_POWER_LIMIT
//    <i> CHASIS_POWER_LIMIT
#define CHASIS_POWER_LIMIT
//  </c>

//  <c1> AIR_PROTECT
//    <i> AIR_PROTECT
#define AIR_PROTECT
//  </c>

//  <c1> USING_BLOCK_CHECK
//    <i> Enable BLOCK_CHECK
#define USING_BLOCK_CHECK
//  </c>

//  <c1> NeuralPID
//    <i> NeuralPID
//#define NeuralPID
//  </c>

//  <o>  distance Mode  <0=>Disable  <1=>I2C  <2=>IO
//       <i> Selects Active Bits in Base Address
#define USING_DISTANCE_MODE       2

//  <o>  Gimbal Mode  <0=>Disable  <1=>Normal  <2=>Debug
//       <i> Selects Active Bits in Base Address
#define USING_GIMBAL_MODE       1


//  <c1> USING_USB
//    <i> Enable USB
#define USING_USB
//  </c>
//  <c1> USING_OS
//    <i> Enable FreeRTOS
//#define USING_OS
//  </c>
//  </h>


//  <h> Config FreeModbus
//  <c1> USING_MODBUS
//    <i> Enable FreeModbus
//#define USING_MODBUS
//  </c>

//    <o> Choose Port
//      <0=>USART6  <1=>USART3
#define sp_choose_modbus_port                   0

//    <o> Choose Timer
//      <0=>TIM12  <1=>TIM11
#define sp_choose_modbus_timer                  0
//  </h>

#ifdef USING_MODBUS
    #if defined(sp_choose_modbus_port)
        #if sp_choose_modbus_port==0
            #define spMODBUS_PORT               USART6
            #define spMODBUS_PORT_IRQ           USART6_IRQn
        #elif sp_choose_modbus_port==1
            #define spMODBUS_PORT               USART3
            #define spMODBUS_PORT_IRQ           USART3_IRQn
        #endif
    #endif
    
    #if defined(sp_choose_modbus_timer)
        #if sp_choose_modbus_timer==0
            #define spMODBUS_TIMER              TIM12
            #define spMODBUS_TIMER_IRQ          TIM8_BRK_TIM12_IRQn
        #elif sp_choose_modbus_timer==1
            #define spMODBUS_TIMER              TIM11
            #define spMODBUS_TIMER_IRQ          TIM1_TRG_COM_TIM11_IRQn
        #endif
    #endif
#endif

#define Latitude_At_ShenZhen 1.57025f
#define Latitude_At_ShangHai 31.18333f
/** @} */


/** @defgroup USART_Resource_Using_Configuration
  * @ingroup  CONFIG
  * @{
  */
//  <h> USART Select

//      <c1> USING_SP_USART1_TX
//        <i> Enable USART1 transmit
//#define USING_SP_USART1_TX
//      </c>
//      <c1> USING_SP_USART1_RX
//        <i> Enable USART1 read
#define USING_SP_USART1_RX
//      </c>


//      <c1> USING_SP_USART2_TX
//        <i> Enable USART2 transmit
#define USING_SP_USART2_TX
//      </c>
//      <c1> USING_SP_USART2_RX
//        <i> Enable USART2 read
#define USING_SP_USART2_RX
//      </c>


//      <c1> USING_SP_USART3_TX
//        <i> Enable USART3 transmit
#define USING_SP_USART3_TX
//      </c>
//      <c1> USING_SP_USART3_RX
//        <i> Enable USART3 read
#define USING_SP_USART3_RX
//      </c>


//      <c1> USING_SP_UART4_TX
//        <i> Enable UART4 transmit
//#define USING_SP_UART4_TX
//      </c>
//      <c1> USING_SP_UART4_RX
//        <i> Enable UART4 read
//#define USING_SP_UART4_RX
//      </c>


//      <c1> USING_SP_UART5_TX
//        <i> Enable UART5 transmit
//#define USING_SP_UART5_TX
//      </c>
//      <c1> USING_SP_UART5_RX
//        <i> Enable UART5 read
//#define USING_SP_UART5_RX
//      </c>


//      <c1> USING_SP_USART6_TX
//        <i> Enable USART6 transmit
#define USING_SP_USART6_TX
//      </c>
//      <c1> USING_SP_USART6_RX
//        <i> Enable USART6 read
#define USING_SP_USART6_RX
//      </c>


#if defined(SP_USING_BOARD_TYPEA)
//      <c1> USING_SP_UART7_TX
//        <i> Enable UART7 transmit
#define USING_SP_UART7_TX
//      </c>
//      <c1> USING_SP_UART7_RX
//        <i> Enable UART7 read
#define USING_SP_UART7_RX
//      </c>
#endif

#if defined(SP_USING_BOARD_TYPEA)
//      <c1> USING_SP_UART8_TX
//        <i> Enable UART8 transmit
#define USING_SP_UART8_TX
//      </c>
//      <c1> USING_SP_UART8_RX
//        <i> Enable UART8 read
#define USING_SP_UART8_RX
//      </c>
#endif

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
#include "sp_timer.h"

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

/**
  * @brief  Counter used in @func TASK_ControlLooper() with period 1ms
  */
void TASK_GetMicrosecondPtr(unsigned long *count);
spTimeStamp TASK_GetTimeStamp(void);
//uint32_t TASK_GetMicroSecond(void);
uint32_t TASK_GetMilliSecond(void);
float TASK_GetSecond(void);

extern void spPortEnterCritical(void);
extern void spPortExitCritical(void);

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
