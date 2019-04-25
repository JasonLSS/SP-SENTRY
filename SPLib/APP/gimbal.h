/**
  ******************************************************************************
  * @file       template.c
  * @author     YTom
  * @version    v0.0-alpha
  * @date       2018.Oct.21
  * @brief      Prohect configurations, based on your board.
  * @usage      
  ******************************************************************************
  * @license
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SP_GIMBAL_H
#define __SP_GIMBAL_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "sp_conf.h"
#include "sp_motor.h"
#include "sp_dma.h"
#include "sp_rc.h"

//extern struct __GimbalController{
//    float yaw_set;
//    float pitch_set;
//} GimbalController;



#define RC_PARAM        8192.f

/** @defgroup SPI_APIs
  * @brief    SPI user operations
  * @ingroup  SPI
  * @{
  */
extern struct __GIMBAL_Controller_Type {
    struct {
        MOTOR_CrtlType_CAN*         gimbal_yaw_motor;
        MOTOR_CrtlType_CAN*         gimbal_pitch_motor;
				struct {
        PID_Type pitch;
        PID_Type yaw;
				} PID;
        float                       yaw_set;
        float                       pitch_set;
    } _target;
    struct {
        void (*init)(void);                                             /*!< Initialize system gimbal controller. */
        void (*looper)(void);                                           /*!< System gimbal controller looper. */
        bool (*regression)(uint32_t tick);                              /*!< Make gimbal return to its start state. */
				void (*statelooper) (void);                                                /*!< change gimbal state. */
    } _system;
    
    struct {
        void (*update_target_pitch)(float target_pitch);                			/*!< Update gimbal pitch target. */
        void (*update_target_yaw)(float target_yaw);                    			/*!< Update gimbal yaw target. */
        void (*update_target)(float target_pitch, float target_yaw);    			/*!< Update gimbal pitch and yaw target. */
				void (*update_target_limit)(float target_pitch, float target_yaw);    /*!< Update gimbal pitch and yaw target with limit. */
				void (*update_enemy_location)(int id);    														/*!< Update gimbal location limit by enemy direction. */
				void (*pid_init)();    																								/*!< change gimbal pitch and yaw pid to original mode. */
				void (*visual_pid_init) ();                                                /*!< change gimbal pitch and yaw pid to visual mode. */
				void (*cruise_pid_init) ();                                                /*!< change gimbal pitch and yaw pid to cruise mode. */
    } user;
} spGIMBAL_Controller;
/** @} */

#ifdef __cplusplus
}
#endif

#endif /*__SP_CHASIS_H */

/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/
