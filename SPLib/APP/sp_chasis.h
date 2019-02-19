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
#ifndef __SP_CHASIS_H
#define __SP_CHASIS_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "sp_conf.h"
#include "sp_motor.h"


/* Exported types ------------------------------------------------------------*/
typedef enum {
    Motor201=0,
    Motor202,
    Motor203,
    Motor204,
    Motor205,
    Motor206,
    Motor207,
    Motor208,
} CHASIS_MotorIdType;

/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/**
  * @brief  Init chasis control
  */ 
void CHASIS_ControlInit(void);
/**
  * @brief  Enable a motor
  * @param  type: motor type @ref CHASIS_MotorIdType
  * @param  motorx: select motor from @arg Motor201 to @arg Motor208, @ref CHASIS_MotorIdType
  * @param  is_pos_pid: if using position PID control
  * @retval NULL if init failed, or pointer of motor @ref MOTOR_CrtlType_CAN
  */ 
MOTOR_CrtlType_CAN* CHASIS_EnableMotor(CHASIS_MotorIdType motorx, MOTOR_RM_Types type, bool is_pos_pid);
/**
  * @brief  Chasis control looper
  */ 
void CHASIS_ControlLooper(void);
/**
  * @brief  Control single motor's movement
  * @note   Using feedback loop control
  */ 
void CHASIS_SetMotorSpeed(CHASIS_MotorIdType motorx, float speed);
void CHASIS_SetMotorPosition(CHASIS_MotorIdType motorx, float position);
void CHASIS_SetMotorRelativePosition(CHASIS_MotorIdType motorx, float relaposition);
/**
  * @brief  
  * @note   
  */ 
MOTOR_CrtlType_CAN* CHASIS_GetMotor(CHASIS_MotorIdType motorx);



#ifdef __cplusplus
}
#endif

#endif /*__SP_CHASIS_H */

/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/
