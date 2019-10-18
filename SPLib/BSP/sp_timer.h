/**
  ******************************************************************************
  * @file       sp_timer.c
  * @author     YTom
  * @version    v0.0-alpha
  * @date       2018.Nov.25
  * @brief      General GPIO config module
  * @usage      spTIMER.init(TIM1, 50, false);
  ******************************************************************************
  * @license
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SP_TIMER_H
#define __SP_TIMER_H

#ifdef __cplusplus
 extern "C" {
#endif

/** @addtogroup SP
  * @brief      SuperPower
  * @{
  */

/** @defgroup TIMER
  * @brief    TIMER Module
  * @{
  */

#include "stm32f4xx_tim.h"
#include "sp_conf.h"


/** @defgroup TIMER_Declarations
  * @brief    Exported Function Declarations
  * @ingroup  TIMER
  * @{
  */
#define __TIMER_MIN_FREQ        (0.05f)
#define __TIMER_MAX_FREQ        (168000000)
/** @} */


/** @defgroup TIMER_APIs
  * @brief    TIMER user operations
  * @ingroup  TIMER
  * @{
  */
extern const struct TIMER_Controllers_Type {
    /** 
      * @brief  Config a timer with base counter.
      * @param  Timx: TIM[1~14]
      * @param  frequency: PWM frequency. [(minest)0.05Hz ~ (?)168MHz]
      * @param  isstart: If start right now.
      * @retval If succeed
      */
    bool (*init)(TIM_TypeDef *Timx,  float frequency, bool isstart);
    /** 
      * @brief  Config PWM output
      * @param  Timx: TIM[1~14]
      * @param  channel: Select from [0~3]
      * @param  portpin: @ref spPinSet
      * @param  duty: PWM output start with duty.
      */
    bool (*pwm_init)(TIM_TypeDef* timx, uint8_t channel, const spPinSet* portpin, float freq, float duty);
    /** 
      * @brief  Set timer channel's compare value (PWM duty).
      * @param  Timx: TIM[1~14]
      * @param  channel: channel [0~3] for channel1~4 if there 4 channels in the  Timx (or [0~1])
      * @param  duty: in unit %.
      */
    void (*set_duty)(TIM_TypeDef *Timx, uint8_t channel, float duty);
    /** 
      * @brief  Config a timer with base counter.
      * @param  Timx: TIM[1~14]
      * @param  channel: channel [0~3] for channel1~4 if there 4 channels in the  Timx (or [0~1])
      * @retval duty in uint %.
      */
    float (*get_duty)(TIM_TypeDef *Timx, uint8_t channel);
    /** 
      * @brief  Config a timer with base counter.
      * @param  Timx: TIM[1~14]
      * @param  frequency: PWM frequency.
      * @param  channel: channel [0~3] for channel1~4 if there 4 channels in the  Timx (or [0~1])
      */
    void (*set_frequency)(TIM_TypeDef *Timx, float frequency, uint8_t channel);
} spTIMER;
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
