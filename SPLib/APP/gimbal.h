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
#include "sp_chasis.h"
#include "sp_dma.h"
#include "sp_rc.h"

//extern struct __GimbalController{
//    float yaw_set;
//    float pitch_set;
//} GimbalController;

extern uint8_t auto_aim_flag;
extern uint8_t small_power_flag;

extern MOTOR_CrtlType_CAN* gimbal_yaw_motor;
extern MOTOR_CrtlType_CAN* gimbal_pitch_motor;

#define RC_PARAM        8192.f

void GIMBAL_ControlInit(void);
void GIMBAL_ControlLooper(void);
void GIMBAL_UpdatePitch(float target_pitch);
void GIMBAL_UpdateYaw(float target_yaw);
void GIMBAL_Update(float target_pitch, float target_yaw);
bool GIMBAL_MiddleLooper(uint32_t tick);

#ifdef __cplusplus
}
#endif

#endif /*__SP_CHASIS_H */

/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/
