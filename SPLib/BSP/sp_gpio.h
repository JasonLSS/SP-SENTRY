/**
  ******************************************************************************
  * @file       sp_gpio.c
  * @author     YTom
  * @version    v0.0-alpha
  * @date       2018.Nov.25
  * @brief      General GPIO config module
  * @usage      spGPIO.output_config(GPIOA, GPIO_Pin_0, GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_100MHz);
  *             spGPIO.set(GPIOA, GPIO_Pin_0, true);
  ******************************************************************************
  * @license
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SP_GPIO_H
#define __SP_GPIO_H

#ifdef __cplusplus
 extern "C" {
#endif

/** @addtogroup SP
  * @brief      SuperPower
  * @{
  */

/** @defgroup GPIO
  * @brief    GPIO Module
  * @{
  */

#include "stm32f4xx_gpio.h"
#include "stdbool.h"
#include "sp_rcc.h"
#include "sp_type.h"


#define spGPIO_PinFromPinSource(ps)         (((uint16_t)0x0001)<<(ps))  /* Resolve GPIO_Pin_X from GPIO_PinSourceX */


/** @defgroup GPIO_Declarations
  * @brief    Exported Function Declarations
  * @ingroup  GPIO
  * @{
  */

/**
  * @brief    Remap GPIOx tp spGPIOx for bit operation.
  */

#define spPinOut(gpio)         (((spInt32_t*)&(gpio->ODR))->data)
#define spPinIn(gpio)          (((spInt32_t*)&(gpio->IDR))->data)
#define spPinOutB(gpio)        (((spInt32_t*)&(gpio->ODR))->bits)
#define spPinInB(gpio)         (((spInt32_t*)&(gpio->IDR))->bits)
 
#ifdef GPIOA
    #define spPAOut         (((spInt32_t*)&(GPIOA->ODR))->data)
    #define spPAIn          (((spInt32_t*)&(GPIOA->IDR))->data)
    #define spPAOutB        (((spInt32_t*)&(GPIOA->ODR))->bits)
    #define spPAInB         (((spInt32_t*)&(GPIOA->IDR))->bits)
#endif
#ifdef GPIOB
    #define spPBOut         (((spInt32_t*)&(GPIOB->ODR))->data)
    #define spPBIn          (((spInt32_t*)&(GPIOB->IDR))->data)
    #define spPBOutB        (((spInt32_t*)&(GPIOB->ODR))->bits)
    #define spPBInB         (((spInt32_t*)&(GPIOB->IDR))->bits)
#endif
#ifdef GPIOC
    #define spPCOut         (((spInt32_t*)&(GPIOC->ODR))->data)
    #define spPCIn          (((spInt32_t*)&(GPIOC->IDR))->data)
    #define spPCOutB        (((spInt32_t*)&(GPIOC->ODR))->bits)
    #define spPCInB         (((spInt32_t*)&(GPIOC->IDR))->bits)
#endif
#ifdef GPIOD
    #define spPDOut         (((spInt32_t*)&(GPIOD->ODR))->data)
    #define spPDIn          (((spInt32_t*)&(GPIOD->IDR))->data)
    #define spPDOutB        (((spInt32_t*)&(GPIOD->ODR))->bits)
    #define spPDInB         (((spInt32_t*)&(GPIOD->IDR))->bits)
#endif
#ifdef GPIOE
    #define spPEOut         (((spInt32_t*)&(GPIOE->ODR))->data)
    #define spPEIn          (((spInt32_t*)&(GPIOE->IDR))->data)
    #define spPEOutB        (((spInt32_t*)&(GPIOE->ODR))->bits)
    #define spPEInB         (((spInt32_t*)&(GPIOE->IDR))->bits)
#endif
#ifdef GPIOF
    #define spPFOut         (((spInt32_t*)&(GPIOF->ODR))->data)
    #define spPFIn          (((spInt32_t*)&(GPIOF->IDR))->data)
    #define spPFOutB        (((spInt32_t*)&(GPIOF->ODR))->bits)
    #define spPFInB         (((spInt32_t*)&(GPIOF->IDR))->bits)
#endif
#ifdef GPIOG
    #define spPGOut         (((spInt32_t*)&(GPIOG->ODR))->data)
    #define spPGIn          (((spInt32_t*)&(GPIOG->IDR))->data)
    #define spPGOutB        (((spInt32_t*)&(GPIOG->ODR))->bits)
    #define spPGInB         (((spInt32_t*)&(GPIOG->IDR))->bits)
#endif
#ifdef GPIOH
    #define spPHOut         (((spInt32_t*)&(GPIOH->ODR))->data)
    #define spPHIn          (((spInt32_t*)&(GPIOH->IDR))->data)
    #define spPHOutB        (((spInt32_t*)&(GPIOH->ODR))->bits)
    #define spPHInB         (((spInt32_t*)&(GPIOH->IDR))->bits)
#endif
#ifdef GPIOI
    #define spPIOut         (((spInt32_t*)&(GPIOI->ODR))->data)
    #define spPIIn          (((spInt32_t*)&(GPIOI->IDR))->data)
    #define spPIOutB        (((spInt32_t*)&(GPIOI->ODR))->bits)
    #define spPIInB         (((spInt32_t*)&(GPIOI->IDR))->bits)
#endif
#ifdef GPIOJ
    #define spPJOut         (((spInt32_t*)&(GPIOJ->ODR))->data)
    #define spPJIn          (((spInt32_t*)&(GPIOJ->IDR))->data)
    #define spPJOutB        (((spInt32_t*)&(GPIOJ->ODR))->bits)
    #define spPJInB         (((spInt32_t*)&(GPIOJ->IDR))->bits)
#endif
#ifdef GPIOK
    #define spPKOut         (((spInt32_t*)&(GPIOK->ODR))->data)
    #define spPKIn          (((spInt32_t*)&(GPIOK->IDR))->data)
    #define spPKOutB        (((spInt32_t*)&(GPIOK->ODR))->bits)
    #define spPKInB         (((spInt32_t*)&(GPIOK->IDR))->bits)
#endif


/**
  * @brief    GPIO port-pin pair
  */
typedef struct {
    GPIO_TypeDef*       gpio;
    uint32_t            pin_source;                 /*!< Select pin number, as GPIO_PinSource[0~15] or 0~15 */
} spPinSet;
static const spPinSet spNullPin = {NULL};
/** @} */


/** @defgroup GPIO_APIs
  * @brief    GPIO user operations
  * @ingroup  GPIO
  * @{
  */
extern const struct GPIO_Controllers_Type {
    /** 
      * @brief  General GPIO config
      * @param  GPIOx   where x can be (A..K) to select the GPIO peripheral for STM32F405xx/407xx and STM32F415xx/417xx devices
      *                       x can be (A..I) to select the GPIO peripheral for STM32F42xxx/43xxx devices.
      *                       x can be (A, B, C, D and H) to select the GPIO peripheral for STM32F401xx devices.
      *	@param  Pinx    GPIO_Pin specifies the port bit to read.
      *                 This parameter can be GPIO_Pin_x where x can be (0..15).
      *	@param  Modex   GPIO working mode
      *                 @arg GPIO_Mode_IN, @arg GPIO_Mode_OUT, @arg GPIO_Mode_AF, @arg GPIO_Mode_AN
      *	@param  OTyperx  GPIO output type setting
      *                 @arg GPIO_OType_PP or @arg GPIO_OType_OD
      *	@param  PuPdx  GPIO pull-up/pull-down setting
      *                 @arg GPIO_PuPd_NOPULL, @arg GPIO_PuPd_UP, @arg GPIO_PuPd_DOWN, 
      *	@param  Speedx  GPIO speed setting
      *                 @arg GPIO_Low_Speed, @arg GPIO_Medium_Speed, @arg GPIO_Fast_Speed, @arg GPIO_High_Speed or
      *                 @arg GPIO_Speed_2MHz, @arg GPIO_Speed_25MHz, @arg GPIO_Speed_50MHz, @arg GPIO_Speed_100MHz
      */
    void (*general_config)(GPIO_TypeDef* GPIOx, uint16_t Pinx, GPIOMode_TypeDef Modex, 
        GPIOOType_TypeDef OTyperx, GPIOPuPd_TypeDef PuPdx, GPIOSpeed_TypeDef Speedx);

    /** 
      * @brief  Config GPIO as general output mode
      * @param  GPIOx   where x can be (A..K) to select the GPIO peripheral for STM32F405xx/407xx and STM32F415xx/417xx devices
      *                       x can be (A..I) to select the GPIO peripheral for STM32F42xxx/43xxx devices.
      *                       x can be (A, B, C, D and H) to select the GPIO peripheral for STM32F401xx devices.   
      *	@param  Pinx    GPIO_Pin specifies the port bit to read.
      *                 This parameter can be GPIO_Pin_x where x can be (0..15).
      *	@param  OTyperx  GPIO output type setting
      *                 @arg GPIO_OType_PP or @arg GPIO_OType_OD
      *	@param  PuPdx  GPIO pull-up/pull-down setting
      *                 @arg GPIO_PuPd_NOPULL, @arg GPIO_PuPd_UP, @arg GPIO_PuPd_DOWN, 
      *	@param  Speedx  GPIO speed setting
      *                 @arg GPIO_Low_Speed, @arg GPIO_Medium_Speed, @arg GPIO_Fast_Speed, @arg GPIO_High_Speed or
      *                 @arg GPIO_Speed_2MHz, @arg GPIO_Speed_25MHz, @arg GPIO_Speed_50MHz, @arg GPIO_Speed_100MHz
      * @note   Use as alternative of @func GPIO_Config
      */
    void (*__output_config)(GPIO_TypeDef* GPIOx, uint16_t Pinx, GPIOOType_TypeDef OTyperx, 
        GPIOPuPd_TypeDef PuPdx, GPIOSpeed_TypeDef Speedx);
    void (*output_config)(GPIO_TypeDef* GPIOx, uint16_t Pinx);

    /** 
      * @brief  Config GPIO as general input mode
      * @param  GPIOx   where x can be (A..K) to select the GPIO peripheral for STM32F405xx/407xx and STM32F415xx/417xx devices
      *                       x can be (A..I) to select the GPIO peripheral for STM32F42xxx/43xxx devices.
      *                       x can be (A, B, C, D and H) to select the GPIO peripheral for STM32F401xx devices.   
      *	@param  Pinx    GPIO_Pin specifies the port bit to read.
      *                 This parameter can be GPIO_Pin_x where x can be (0..15).
      *	@param  PuPdx  GPIO pull-up/pull-down setting
      *                 @arg GPIO_PuPd_NOPULL, @arg GPIO_PuPd_UP, @arg GPIO_PuPd_DOWN, 
      *	@param  Speedx  GPIO speed setting
      *                 @arg GPIO_Low_Speed, @arg GPIO_Medium_Speed, @arg GPIO_Fast_Speed, @arg GPIO_High_Speed or
      *                 @arg GPIO_Speed_2MHz, @arg GPIO_Speed_25MHz, @arg GPIO_Speed_50MHz, @arg GPIO_Speed_100MHz
      * @note   Use as alternative of @func GPIO_Config
      */
    void (*__input_config)(GPIO_TypeDef* GPIOx, uint16_t Pinx, GPIOPuPd_TypeDef PuPdx, GPIOSpeed_TypeDef Speedx);
    void (*input_config)(GPIO_TypeDef* GPIOx, uint16_t Pinx);
    /** 
      * @brief  Config GPIO as alternal function mode
      * @param  GPIOx   where x can be (A..K) to select the GPIO peripheral for STM32F405xx/407xx and STM32F415xx/417xx devices
      *                       x can be (A..I) to select the GPIO peripheral for STM32F42xxx/43xxx devices.
      *                       x can be (A, B, C, D and H) to select the GPIO peripheral for STM32F401xx devices.   
      *	@param  Pinx    GPIO_Pin specifies the port bit to read.
      *                 This parameter can be GPIO_Pin_x where x can be (0..15).
      *	@param  OTyperx  GPIO output type setting
      *                 @arg GPIO_OType_PP or @arg GPIO_OType_OD
      *	@param  PuPdx  GPIO pull-up/pull-down setting
      *                 @arg GPIO_PuPd_NOPULL, @arg GPIO_PuPd_UP, @arg GPIO_PuPd_DOWN, 
      *	@param  Speedx  GPIO speed setting
      *                 @arg GPIO_Low_Speed, @arg GPIO_Medium_Speed, @arg GPIO_Fast_Speed, @arg GPIO_High_Speed or
      *                 @arg GPIO_Speed_2MHz, @arg GPIO_Speed_25MHz, @arg GPIO_Speed_50MHz, @arg GPIO_Speed_100MHz
      * @note   Pleas use @func GPIO_PinAFConfig at FIRST.
      *         Use as alternative of @func GPIO_Config
      */
    void (*__alternal_config)(GPIO_TypeDef* GPIOx, uint16_t Pinx, 
        GPIOOType_TypeDef OTyperx, GPIOPuPd_TypeDef PuPdx, GPIOSpeed_TypeDef Speedx);
    void (*alternal_config)(GPIO_TypeDef* GPIOx, uint16_t Pinx);

    /** 
      * @brief  Config GPIO as anolog mode
      * @param  GPIOx   where x can be (A..K) to select the GPIO peripheral for STM32F405xx/407xx and STM32F415xx/417xx devices
      *                       x can be (A..I) to select the GPIO peripheral for STM32F42xxx/43xxx devices.
      *                       x can be (A, B, C, D and H) to select the GPIO peripheral for STM32F401xx devices.   
      *	@param  Pinx    GPIO_Pin specifies the port bit to read.
      *                 This parameter can be GPIO_Pin_x where x can be (0..15).
      * @note   Use as alternative of @func GPIO_Config
      */
    void (*analog_config)(GPIO_TypeDef* GPIOx, uint16_t Pinx);
    
    /** 
      * @brief  Change GPIO pin output
      * @param  GPIOx   where x can be (A..K) to select the GPIO peripheral for STM32F405xx/407xx and STM32F415xx/417xx devices
      *                       x can be (A..I) to select the GPIO peripheral for STM32F42xxx/43xxx devices.
      *                       x can be (A, B, C, D and H) to select the GPIO peripheral for STM32F401xx devices.   
      *	@param  Pinx    GPIO_Pin specifies the port bit to read.
      *                 This parameter can be GPIO_Pin_x where x can be (0..15).
      * @param  high    If change to high level.
      * @note   If mode is @arg GPIO_Mode_OUT, get() returns output state, else will return input state.
      */
    void (*set)(GPIO_TypeDef* GPIOx, uint16_t Pinx, bool high);
    bool (*get)(GPIO_TypeDef* GPIOx, uint16_t Pinx);
    
    /** 
      * @brief  Reverse GPIO pin output
      * @param  GPIOx   where x can be (A..K) to select the GPIO peripheral for STM32F405xx/407xx and STM32F415xx/417xx devices
      *                       x can be (A..I) to select the GPIO peripheral for STM32F42xxx/43xxx devices.
      *                       x can be (A, B, C, D and H) to select the GPIO peripheral for STM32F401xx devices.   
      *	@param  Pinx    GPIO_Pin specifies the port bit to read.
      *                 This parameter can be GPIO_Pin_x where x can be (0..15).
      */
    void (*toggle)(GPIO_TypeDef* GPIOx, uint16_t Pinx);
    
    /** 
      * @brief  Change GPIO mode
      * @param  GPIOx   where x can be (A..K) to select the GPIO peripheral for STM32F405xx/407xx and STM32F415xx/417xx devices
      *                       x can be (A..I) to select the GPIO peripheral for STM32F42xxx/43xxx devices.
      *                       x can be (A, B, C, D and H) to select the GPIO peripheral for STM32F401xx devices.   
      *	@param  Pinx    GPIO_Pin specifies the port bit to read.
      *                 This parameter can be GPIO_Pin_x where x can be (0..15).
      *	@param  mode    @ref GPIOMode_TypeDef
      *                 A param from @arg GPIO_Mode_IN
      *                              @arg GPIO_Mode_OUT
      *                              @arg PIO_Mode_AF
      *                              @arg GPIO_Mode_AN
      * @param  retval  [true]=high level, [false]=low level
      */
    void (*set_mode)(GPIO_TypeDef* GPIOx, uint16_t Pinx, GPIOMode_TypeDef mode);
    GPIOMode_TypeDef (*get_mode)(GPIO_TypeDef* GPIOx, uint16_t Pinx);
} spGPIO;
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

#endif /*__SP_GPIO_H */

/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/
