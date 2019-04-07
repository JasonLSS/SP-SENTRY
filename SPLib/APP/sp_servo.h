/**
  ******************************************************************************
  * @file       template.h
  * @author     YTom
  * @version    v0.0-alpha
  * @date       2018.Oct.21
  * @brief      Romete controller module driver with USART1_RX
  * @usage      
  ******************************************************************************
  * @license
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SP_SERVO_H
#define __SP_SERVO_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "sp_conf.h"

/** @addtogroup SP
  * @{
  */

/** @addtogroup SERVO
  * @{
  */

/** @defgroup   Definations
  * @note       Exported Macros And Definations
  * @ingroup    SERVO
  * @{
  */

/**
  * @brief  Servo manager struct
  */ 
typedef struct {
    struct {
        float               lower_bound;            /*!< Lower bound of REAL angle servo motor. */
        float               higher_bound;           /*!< Higher bound of REAL angle servo motor. */
        float               offset;                 /*!< Offser for target by PWM signal center(as defaukt 0 deg). */
        float               target;                 /*!< Target of angle. */
        float               current;                /*!< Current angle calculated from timer register.  */
        uint8_t             speed;                  /*!< Max manual rotating speed (real speed depend on
                                                         motor and MCU PWM fresh frequency), unit of **deg/sec**.  */
    } param;
    struct {
        float               time_stamp;             /*!< Last update time */
        TIM_TypeDef*        timx;                   /*!< Select timer */
        uint8_t             channel;                /*!< Select channel from 1~4 */
        spPinSet            portpin;                /*!< Select GPIOx and  */
    } control;
} ServoType;

extern ServoType            ServoPool[USING_SERVO_POOL_SIZE];

/** @} */



/** @defgroup SERVO_APIs
  * @brief    SERVO user operations
  * @ingroup  SERVO
  * @{
  */
extern struct __SERVO_Manager_Type {
    struct {
        void (*init)(void);
        void (*looper)(void);
    } _system;
    struct {
        ServoType* (*get_servo)(
            TIM_TypeDef* timx, uint8_t channel, GPIO_TypeDef* gpio, uint32_t pin_source,
            float offset, float low, float high );
        void (*set_target)(ServoType* servo, float target);
        void (*set_speed)(ServoType* servo, uint8_t speed);
        void (*set_target_with_speed)(ServoType* servo, float target, uint8_t speed);
        float (*get_current_angle)(ServoType* servo);
        void (*spin)(ServoType* servo);
    } user;
} spSERVO;
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

#endif /*__SP_RC_H */

/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/
