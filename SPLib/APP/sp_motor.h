/**
  ******************************************************************************
  * @file       template.c
  * @author     YTom
  * @version    v0.0-alpha
  * @date       2018.Oct.21
  * @brief      project header template
  * @note       This module mainly includes:
                (+) Force(current)/speed/position control
                (+) Motor state monitoring
    ATTENTION:
        Motor's parameter **speed** and **angle** have been muiltiplied by its TRANSMISSION RATION,
    and converted to unit of [rad/s] or [rad].
        For RM3510 and RM3508 motors, I use deadzone_gain() function to modify its performance when low output.
  ******************************************************************************
  * @license
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SP_MOTOR_H
#define __SP_MOTOR_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "string.h"
#include "sp_can.h"
#include "sp_pid.h"
#include "sp_math.h"

/** @addtogroup SP
  * @brief      SuperPower
  * @{
  */

/** @defgroup MOTOR
  * @brief    MOTOR Driver Module
  * @{
  */

/** @defgroup MOTOR_Definations
  * @brief    MOTOR Exported Macros And Definations
  * @ingroup  MOTOR
  * @{
  */

/**
  * @brief  Motor types for RM motros
  */
typedef enum {
    RM_3508_P19,
    RM_3510_P19,
    RM_2006_P36,
    RM_6025_PITCH,
    RM_6623_YAW,
    RM_3510_P27,
    RM_2006_P96,
    GM_3510                 /* NOT YET 20181205 */
} MOTOR_RM_Types;

/** 
  * @brief  Motor Control Struct
  * @note   Motor here is controlled by Electronic Speed Controller(ESC) with CAN-bus.
  */
typedef struct {
#ifndef __cplusplus
    void*                       THIS;
#endif
    /**
      * @brief  State parameters of motor. 
      */
    struct {
        uint8_t                 enable:1;
        uint8_t                 can_mounted:1;
        MOTOR_RM_Types          rm_type;
    } flags;
    /**
      * @brief  Raw data from CAN
      * @note   Each CAN-bus can connect with 8 motors under address 0x200 @ref RoboMaster_Motor_Manual
      *         (+)Freq         1kHz
      *         (+)Angle        0~8192 = 0~360deg (for rotor before deceleration )
      *         (+)SpeedUnit    RPM (for rotor before deceleration )
      *         (+)TempUnit     Celsius Degree
      */
    struct {
        CAN_Receiver            receiver;
        uint8_t                 raw_data[8];
    } data;
    /**
      * @brief  State parameters of motor, data resolved from data.
      */
    struct {
        int16_t                 current;            /* Read from CAN. [uint=?] */
        float                   speed;              /* Read from CAN. [uint=?] */
        float                   angle;              /* Read from CAN. [uint=?] */
        int16_t                 temprature;         /* Read from CAN. [uint=?] */
        uint16_t                mortor_stuckflag;
        int16_t                 __motor_angel_curr;
        int16_t                 __motor_angel_last;
        int16_t                 __motor_angel_first;
    } state;
    /**
      * @brief  SControll parameters of motor.
      */    
    struct {
        PID_Type*               speed_pid;          /* Using speed-feedback-control if not null */
        PID_Type*               position_pid;       /* Using position-feedback-control if not null */
        float                   target;             /** Representing for speed or position, regarded at 
                                                        @ref speed_pid and @ref position_pid, if both are
                                                        null, then will be ignored. */
//        float                   target_limit;       /* Limitation for the target. */
        float                   output;             /* Last output(current). */
        float                   output_limit;       /* Limitation for the last output(current). */
    } control;
} MOTOR_CrtlType_CAN;

/**
  * @brief  CAN feedback data resolver type.
  */
typedef void(*ResolverCallback_t)(CanRxMsg* , void*);

/** 
  * @brief   Pool size of motors, means how many motors will be used.
  */ 
#define MOTOR_POOLSIZE                      32
#define CAN_MOTOR_COUNT                     16              /* Total number of possible motors in CAN1/CAN2 */


/** 
  * @brief    Motor ID on CAN1/CAN2
  */
typedef enum {
    Motor201=0,
    Motor202,
    Motor203,
    Motor204,
    Motor205,
    Motor206,
    Motor207,
    Motor208,
    Motor209,
    Motor20A,
    Motor20B,
    Motor20C,
    Motor20D,
    Motor20E,
    Motor20F,
    Motor216
} CHASIS_MotorIdType;

/** @} */



/** @defgroup MOTOR_APIs
  * @brief    MOTOR user operations
  * @ingroup  MOTOR
  * @{
  */
extern struct __MOTOR_Manager_Type {
    struct {
        void(*init)(void);
        void(*looper)(void);
    } _system;
    struct {
        /**
         * @brief  Enable a motor's controller.
         * @param  type: motor type @ref CHASIS_MotorIdType
         * @param  motorx: select motor from @arg Motor201 to @arg Motor216, @ref CHASIS_MotorIdType
         * @param  is_pos_pid: if using position PID control
         * @retval NULL if init failed, or pointer of motor @ref MOTOR_CrtlType_CAN
         */
        MOTOR_CrtlType_CAN* (*enable)(CAN_TypeDef* CANx, CHASIS_MotorIdType motorx, MOTOR_RM_Types type, bool is_pos_pid);
        
        /**
         * @brief  Enable a motor without controller, which means only monitor a motor.
         * @param  type: motor type @ref CHASIS_MotorIdType
         * @param  motorx: select motor from @arg Motor201 to @arg Motor216, @ref CHASIS_MotorIdType
         * @retval NULL if init failed, or pointer of motor @ref MOTOR_CrtlType_CAN
         */
        MOTOR_CrtlType_CAN* (*enable_simple)(CAN_TypeDef* CANx, CHASIS_MotorIdType motorx, MOTOR_RM_Types type);
        
        /**
         * @brief  Control single motor's movement
         * @note   Using feedback loop control
         */ 
        void (*set_speed)(CAN_TypeDef* CANx, CHASIS_MotorIdType motorx, float speed);
        void (*set_position)(CAN_TypeDef* CANx, CHASIS_MotorIdType motorx, float position);
        void (*set_relative_position)(CAN_TypeDef* CANx, CHASIS_MotorIdType motorx, float relaposition);
        
        MOTOR_CrtlType_CAN* (*get)(CAN_TypeDef* CANx, CHASIS_MotorIdType motorx);
        
        void (*stop)(CAN_TypeDef* CANx, CHASIS_MotorIdType motorx);
    } user;
    struct {
        /**
          * @brief  Get an instance of RM motor @ref MOTOR_CrtlType_CAN
          * @retval Pointer of an instance of @ref MOTOR_CrtlType
          * @param  type: select motor type @ref Motor_RM_Types
          */ 
        MOTOR_CrtlType_CAN* (*get_instance)(MOTOR_RM_Types type);

        /**
          * @brief  Enable a motor and request its resouce
          * @param  motor: Pointer of the motor @ref MOTOR_CrtlType_CAN
          */ 
        void (*enable)(MOTOR_CrtlType_CAN* __motor);

        /**
          * @brief  Disable a motor and release its resouce
          * @param  motor: Pointer of the motor @ref MOTOR_CrtlType_CAN
          */ 
        void (*disable)(MOTOR_CrtlType_CAN* __motor);

        /**
          * @brief  Set motor target
          * @param  motor: Pointer of the motor @ref MOTOR_CrtlType_CAN
          * @param  target: Motion target, differ by motor type (speed, position)
          */ 
        void (*set_target)(MOTOR_CrtlType_CAN* __motor, float target);

        /**
          * @brief  Set motor delta target
          * @param  motor: Pointer of the motor @ref MOTOR_CrtlType_CAN
          * @param  delta: Motion delta target, differ by motor type (speed, position)
          */ 
        void (*set_target_delta)(MOTOR_CrtlType_CAN* __motor, float delta);

        /**
          * @brief  Set limitation for target / output
          * @param  motor: Pointer of the motor @ref MOTOR_CrtlType_CAN
          * @param  limit: Set output(current control value) limit
          */ 
        void (*set_output_limit)(MOTOR_CrtlType_CAN* __motor, float limit);
        //void __MOTOR_SetTargetLimit(MOTOR_CrtlType_CAN* __motor, float limit);

        /**
          * @brief  Rebind PID to motor
          * @param  motor: Pointer of the motor @ref MOTOR_CrtlType_CAN
          * @param  limit: Set output(current control value) limit
          */ 
        void (*set_speed_pid)(MOTOR_CrtlType_CAN* __motor, PID_Type* pid);
        void (*set_position_pid)(MOTOR_CrtlType_CAN* __motor, PID_Type* pid);

        /**
          * @brief  Get an instance of @ref MOTOR_CrtlType
          * @param  motor: @ref MOTOR_CrtlType_CAN motor instance
          * @param  func: @ref tFuncMemberNoParam new data-resolve function or NULL
          * @retval Pointer of an instance of @ref MOTOR_CrtlType
          */ 
        void (*set_resolver)(MOTOR_CrtlType_CAN* motor, void(*func)(CanRxMsg* , void*));
    } motor;
} spMOTOR;
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

#endif /*__SP_MOTOR_H */

/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/
