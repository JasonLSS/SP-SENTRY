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
#include "sp_rc.h"

/** @addtogroup SP
  * @{
  */
  
/** @addtogroup CHASIS
  * @{
  */

/** @defgroup CHASIS_APIs
  * @brief    CHASIS user operations
  * @ingroup  CHASIS
  * @{
  */
typedef struct {
    const float half_width;
    const float half_length;
    const float wheel_radius;
    struct {
        float x;
        float y;
        float yaw;
        float output[4];
    } state;
    struct {
        float spdx;
        float spdy;
        float spdyaw;
    } target;
    struct {
        PID_Type x;
        PID_Type y;
        PID_Type yaw;
    } PID;
    struct {
        PID_Type x;
        PID_Type yaw;
        float distance;
    } ParallelPID;
} CHASIS_ParamReg_t;

extern struct __CHASIS_Manager_Type {
    struct {
        CHASIS_ParamReg_t params;
        void (*init)(void);
        void (*looper)(uint32_t tick, const RC_DataType *recv);
    } _system;
    struct {
        void (*move)(float speed);
    } user;
} spCHASIS;

extern float chasis_speed_limit;
extern float chasis_speed;
float CHASIS_Legalize(float MotorCurrent , float limit);
void CMWatt_Cal(void);
int IfUsingPowerBuffer(void);
int Enemy_Location(void);
int Empty_Location(void);
float Speed_Change_Limit(float speed);

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



#endif /*__SP_CHASIS_H */

/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/
