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
        uint8_t                 pid_inited:1;
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
    /**
      * @brief  Motor implement functions.
      */
    struct {
        void (*set_target)(void*, float);                   /* Set target. */
        void (*set_outputlimit)(void*, float);              /* Set limit of the output value. */
        void (*set_targetlimit)(void*, float);              /* Set limit of the output value. */
        void (*set_speed_pid)(void*, PID_Type*);
        void (*set_position_pid)(void*, PID_Type*);
        void (*mount_can)(void* motor, CAN_TypeDef* canx, uint16_t);
                                                            /* Mount motor to CAN. */
    } implement;
    /**
      * @brief  Motor private member functions.
      */
    struct {
        tFuncMemberNoParam      init;                       /* Init motor module. */
        tFuncMemberNoParam      destroy;                    /* Deinit motor module. */
        tFuncMemberNoParam      data_resolve;               /* Resolving feedback data from ESC via CAN. 
                                                               USER DETERMINE or use predefined function for                                                 RM3508/RM3510/RM2006 motor. */
    } __private;
} MOTOR_CrtlType_CAN;

/**
  * @brief  CAN data transmit/receive manager
  */
typedef enum {
    CAN_MOTOR1=0,
    CAN_MOTOR2,
    CAN_MOTOR3,
    CAN_MOTOR4,
    CAN_MOTOR5,
    CAN_MOTOR6,
    CAN_MOTOR7,
    CAN_MOTOR8
} CAN_MOTORx;


/** 
  * @brief   Pool size of motors, means how many motors will be used.
  */ 
#define MOTOR_POOLSIZE          8



/**
  * @brief  Get an instance of RM motor @ref MOTOR_CrtlType_CAN
  * @retval Pointer of an instance of @ref MOTOR_CrtlType
  * @param  type: select motor type @ref Motor_RM_Types
  */ 
MOTOR_CrtlType_CAN* MOTOR_RM_GetInstance(MOTOR_RM_Types type);

/**
  * @brief  Enable a motor and request its resouce
  * @param  motor: Pointer of the motor @ref MOTOR_CrtlType_CAN
  */ 
void __MOTOR_Init(void* motor);

/**
  * @brief  Disable a motor and release its resouce
  * @param  motor: Pointer of the motor @ref MOTOR_CrtlType_CAN
  */ 
void __MOTOR_Destroy(void* motor);

/**
  * @brief  Set motor target
  * @param  motor: Pointer of the motor @ref MOTOR_CrtlType_CAN
  * @param  target: Motion target, differ by motor type (speed, position)
  */ 
void __MOTOR_SetTarget(void* motor, float target);

/**
  * @brief  Set motor delta target
  * @param  motor: Pointer of the motor @ref MOTOR_CrtlType_CAN
  * @param  delta: Motion delta target, differ by motor type (speed, position)
  */ 
void __MOTOR_SetTargetDelta(void* motor, float delta);

/**
  * @brief  Set limitation for target / output
  * @param  motor: Pointer of the motor @ref MOTOR_CrtlType_CAN
  * @param  limit: Set output(current control value) limit
  */ 
void __MOTOR_SetOutputLimit(void* motor, float limit);
//void __MOTOR_SetTargetLimit(void* motor, float limit);

/**
  * @brief  Rebind PID to motor
  * @param  motor: Pointer of the motor @ref MOTOR_CrtlType_CAN
  * @param  limit: Set output(current control value) limit
  */ 
void __MOTOR_SetSpeedPID(void* motor, PID_Type* pid);
void __MOTOR_SetPositionPID(void* motor, PID_Type* pid);

/**
  * @brief  Get an instance of @ref MOTOR_CrtlType
  * @param  motor: @ref MOTOR_CrtlType_CAN motor instance
  * @param  func: @ref tFuncMemberNoParam new data-resolve function or NULL
  * @retval Pointer of an instance of @ref MOTOR_CrtlType
  */ 
void MOTOR_SetDataResolve(MOTOR_CrtlType_CAN* motor, tFuncMemberNoParam func);



// TODO: Functional speed target
// TODO: Motor protection




/**
  * @brief  Init motor control module
  */ 
void MOTOR_ControlInit(void);

/**
  * @brief  Loop to invoke morotr control
  * @note   Periodic invoke by SYSTEM.
  */
void MOTOR_ControlLooper(void);



#ifdef __cplusplus
}
#endif

#endif /*__SP_MOTOR_H */

/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/
