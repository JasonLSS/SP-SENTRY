/**
  ******************************************************************************
  * @file       sp_servo.h
  * @author     YTom
  * @version    v0.0-alpha
  * @date       2018.Nov.27
  * @brief      Servo control
  * @usage      
  *         ServoType* servo = spSERVO.user.get_servo(TIM5, 1, GPIOH, GPIO_PinSource10, 2, -80.f, 80.f);
  *         spSERVO.user.set_speed(servo, 200);
  *         spSERVO.user.spin(servo);
  *         spSERVO.user.set_target(servo, target);
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
        uint16_t            speed;                  /*!< Max manual rotating speed (real speed depend on
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
    } _system;
    struct {
        /**
          * @param channel [0~3]
          */
        ServoType* (*get_servo)(
            TIM_TypeDef* timx, uint8_t channel, GPIO_TypeDef* gpio, uint32_t pin_source,
            float offset, float low, float high );
        void (*set_target)(ServoType* servo, float target);
        void (*set_speed)(ServoType* servo, uint16_t speed);
        void (*set_target_with_speed)(ServoType* servo, float target, uint16_t speed);
        float (*get_current_angle)(ServoType* servo);
        void (*spin)(ServoType* servo);
        void (*spin_simple)(ServoType* servo);
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
