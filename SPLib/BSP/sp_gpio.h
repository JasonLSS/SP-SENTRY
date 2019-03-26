/**
  ******************************************************************************
  * @file       sp_gpio.c
  * @author     YTom
  * @version    v0.0-alpha
  * @date       2018.Nov.25
  * @brief      General GPIO config module
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
#include "sp_rcc.h"


#define spGPIO_PinFromPinSource(ps)         (0x0001<<(ps))  /* Resolve GPIO_Pin_X from GPIO_PinSourceX */


/** @defgroup GPIO_Declarations
  * @brief    Exported Function Declarations
  * @ingroup  GPIO
  * @{
  */

/**
  * @brief    GPIO port-pin pair
  */
typedef struct {
    GPIO_TypeDef*       gpio;
    uint32_t            pin_source;                 /*!< Select pin number, as GPIO_PinSource[0~15] */
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
      *	@param	Pinx    GPIO_Pin specifies the port bit to read.
      *                 This parameter can be GPIO_Pin_x where x can be (0..15).
      *	@param	Modex   GPIO working mode
      *                 @arg GPIO_Mode_IN, @arg GPIO_Mode_OUT, @arg GPIO_Mode_AF, @arg GPIO_Mode_AN
      *	@param  OTyperx	GPIO output type setting
      *                 @arg GPIO_OType_PP or @arg GPIO_OType_OD
      *	@param	PuPdx	GPIO pull-up/pull-down setting
      *                 @arg GPIO_PuPd_NOPULL, @arg GPIO_PuPd_UP, @arg GPIO_PuPd_DOWN, 
      *	@param  Speedx	GPIO speed setting
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
      *	@param	Pinx    GPIO_Pin specifies the port bit to read.
      *                 This parameter can be GPIO_Pin_x where x can be (0..15).
      *	@param  OTyperx	GPIO output type setting
      *                 @arg GPIO_OType_PP or @arg GPIO_OType_OD
      *	@param	PuPdx	GPIO pull-up/pull-down setting
      *                 @arg GPIO_PuPd_NOPULL, @arg GPIO_PuPd_UP, @arg GPIO_PuPd_DOWN, 
      *	@param  Speedx	GPIO speed setting
      *                 @arg GPIO_Low_Speed, @arg GPIO_Medium_Speed, @arg GPIO_Fast_Speed, @arg GPIO_High_Speed or
      *                 @arg GPIO_Speed_2MHz, @arg GPIO_Speed_25MHz, @arg GPIO_Speed_50MHz, @arg GPIO_Speed_100MHz
      * @note   Use as alternative of @func GPIO_Config
      */
    void (*output_config)(GPIO_TypeDef* GPIOx, uint16_t Pinx, GPIOOType_TypeDef OTyperx, 
        GPIOPuPd_TypeDef PuPdx, GPIOSpeed_TypeDef Speedx);

    /** 
      * @brief  Config GPIO as general input mode
      * @param  GPIOx   where x can be (A..K) to select the GPIO peripheral for STM32F405xx/407xx and STM32F415xx/417xx devices
      *                       x can be (A..I) to select the GPIO peripheral for STM32F42xxx/43xxx devices.
      *                       x can be (A, B, C, D and H) to select the GPIO peripheral for STM32F401xx devices.   
      *	@param	Pinx    GPIO_Pin specifies the port bit to read.
      *                 This parameter can be GPIO_Pin_x where x can be (0..15).
      *	@param	PuPdx	GPIO pull-up/pull-down setting
      *                 @arg GPIO_PuPd_NOPULL, @arg GPIO_PuPd_UP, @arg GPIO_PuPd_DOWN, 
      *	@param  Speedx	GPIO speed setting
      *                 @arg GPIO_Low_Speed, @arg GPIO_Medium_Speed, @arg GPIO_Fast_Speed, @arg GPIO_High_Speed or
      *                 @arg GPIO_Speed_2MHz, @arg GPIO_Speed_25MHz, @arg GPIO_Speed_50MHz, @arg GPIO_Speed_100MHz
      * @note   Use as alternative of @func GPIO_Config
      */
    void (*input_config)(GPIO_TypeDef* GPIOx, uint16_t Pinx, GPIOPuPd_TypeDef PuPdx, GPIOSpeed_TypeDef Speedx);
    /** 
      * @brief  Config GPIO as alternal function mode
      * @param  GPIOx   where x can be (A..K) to select the GPIO peripheral for STM32F405xx/407xx and STM32F415xx/417xx devices
      *                       x can be (A..I) to select the GPIO peripheral for STM32F42xxx/43xxx devices.
      *                       x can be (A, B, C, D and H) to select the GPIO peripheral for STM32F401xx devices.   
      *	@param	Pinx    GPIO_Pin specifies the port bit to read.
      *                 This parameter can be GPIO_Pin_x where x can be (0..15).
      *	@param  OTyperx	GPIO output type setting
      *                 @arg GPIO_OType_PP or @arg GPIO_OType_OD
      *	@param	PuPdx	GPIO pull-up/pull-down setting
      *                 @arg GPIO_PuPd_NOPULL, @arg GPIO_PuPd_UP, @arg GPIO_PuPd_DOWN, 
      *	@param  Speedx	GPIO speed setting
      *                 @arg GPIO_Low_Speed, @arg GPIO_Medium_Speed, @arg GPIO_Fast_Speed, @arg GPIO_High_Speed or
      *                 @arg GPIO_Speed_2MHz, @arg GPIO_Speed_25MHz, @arg GPIO_Speed_50MHz, @arg GPIO_Speed_100MHz
      * @note   Pleas use @func GPIO_PinAFConfig at FIRST.
      *         Use as alternative of @func GPIO_Config
      */
    void (*alternal_config)(GPIO_TypeDef* GPIOx, uint16_t Pinx, 
        GPIOOType_TypeDef OTyperx, GPIOPuPd_TypeDef PuPdx, GPIOSpeed_TypeDef Speedx);

    /** 
      * @brief  Config GPIO as anolog mode
      * @param  GPIOx   where x can be (A..K) to select the GPIO peripheral for STM32F405xx/407xx and STM32F415xx/417xx devices
      *                       x can be (A..I) to select the GPIO peripheral for STM32F42xxx/43xxx devices.
      *                       x can be (A, B, C, D and H) to select the GPIO peripheral for STM32F401xx devices.   
      *	@param	Pinx    GPIO_Pin specifies the port bit to read.
      *                 This parameter can be GPIO_Pin_x where x can be (0..15).
      * @note   Use as alternative of @func GPIO_Config
      */
    void (*analog_config)(GPIO_TypeDef* GPIOx, uint16_t Pinx);
} spGPIO_Controllers;
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
