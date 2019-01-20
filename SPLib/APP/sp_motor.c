/**
  ******************************************************************************
  * @file       template.c
  * @author     YTom
  * @version    v0.0-alpha
  * @date       2018.Oct.21
  * @brief      project source file template
  * @note       This file is suitable for RM3508/RM3510/RM2006 motors 
  *             with C620/C610 ESC.
  ******************************************************************************
  * @license
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "sp_motor.h"

/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define MOTOR_OUTPUTLIMIT_INIT          12000               /* Motor ouput default limitation, real range +-16384 */

#define CAN_MOTOR_ID_MASK               0x0F                /* Motor ID mask for resolving index */
#define CAN_STUCK_FILTER                38.0f               /* Minium of delta encoder angle, 
                                                                delta angle under this will be regarded as stuck */
#define CAN_MOTOR_ENCODER2RAD           2607.5946f          /* Convert encoder value to radial value */
#define CAN1_MOTOR_COUNT                8                   /* Total number of possible motors */
#define CAN2_MOTOR_COUNT                8                   /* Total number of possible motors */
#define MOTOR_STUCK_THRESHOLD           1000


#define CAN_MOTOR_RM3510_P27            (27)
#define CAN_MOTOR_RM2006_P36            (36)
#define CAN_MOTOR_RM3510_3508_P19       (3591/187)          /* Motor's Mechanical trasmission ratio 
                                                                Real value is 3591/187 = 19.2032f @RM3508 */


/* Private variables ---------------------------------------------------------*/
/**
  * @brief  Motor pool (global motor resource)
  */
MOTOR_CrtlType_CAN                      MOTORs[MOTOR_POOLSIZE] = {0x00};


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Limit controller output sending via CAN
  * @param  MotorCurrent: current control value
  * @param  limit: limitation
  */
__STATIC_INLINE float __MOTOR_OutputLimit(MOTOR_CrtlType_CAN* __motor, float value) {
    return (value > __motor->control.output_limit)?__motor->control.output_limit:
        (value < -__motor->control.output_limit)?-__motor->control.output_limit:value;
}
//__STATIC_INLINE float __MOTOR_TargetLimit(MOTOR_CrtlType_CAN* __motor, float value) {
//    return (value > __motor->control.target_limit)?__motor->control.target_limit:
//        (value < -__motor->control.target_limit)?-__motor->control.target_limit:value;
//}

/**
  * @}
  */



/** @defgroup Motor Class Member Function Implement
  * @brief    Implement member functions for @ref MOTOR_CrtlType_CAN
  * @{
  */
/**
  * @brief  Enable a motor and request its resouce
  * @param  motor: Pointer of the motor @ref MOTOR_CrtlType_CAN
  */ 
void __MOTOR_Init(void* motor) {
    if(!motor) return;
    MOTOR_CrtlType_CAN* __motor = (MOTOR_CrtlType_CAN*)motor;
    if(!__motor->flags.enable) {
        __motor->flags.enable = true;
    }
}

/**
  * @brief  Disable a motor and release its resouce
  * @param  motor: Pointer of the motor @ref MOTOR_CrtlType_CAN
  */ 
void __MOTOR_Destroy(void* motor) {
    if(!motor) return;
    MOTOR_CrtlType_CAN* __motor = (MOTOR_CrtlType_CAN*)motor;
    if(__motor->flags.enable) {
        __motor->flags.enable = false;
    }
}

/**
  * @brief  Disable a motor and release its resouce
  * @param  motor: Pointer of the motor @ref MOTOR_CrtlType_CAN
  * @param  target: Motion target, differ by motor type (speed, position)
  */ 
void __MOTOR_SetTarget(void* motor, float target) {
    if(!motor) return;
    MOTOR_CrtlType_CAN* __motor = (MOTOR_CrtlType_CAN*)motor;
     __motor->control.target = target;
//    __motor->control.target = (target > __motor->control.target_limit)?__motor->control.target_limit:
//        (target < -__motor->control.target_limit)?-__motor->control.target_limit:target;
}
void __MOTOR_SetTargetDelta(void* motor, float delta) {
    if(!motor) return;
    MOTOR_CrtlType_CAN* __motor = (MOTOR_CrtlType_CAN*)motor;
     __motor->control.target += delta;
//    __motor->control.target = (target > __motor->control.target_limit)?__motor->control.target_limit:
//        (target < -__motor->control.target_limit)?-__motor->control.target_limit:target;
}

/**
  * @brief  Disable a motor and release its resouce
  * @param  motor: Pointer of the motor @ref MOTOR_CrtlType_CAN
  * @param  limit: Set output(current control value) limit
  */ 
void __MOTOR_SetOutputLimit(void* motor, float limit) {
    if(!motor) return;
    MOTOR_CrtlType_CAN* __motor = (MOTOR_CrtlType_CAN*)motor;
    __motor->control.output_limit = limit;
}
//void __MOTOR_SetTargetLimit(void* motor, float limit) {
//    if(!motor) return;
//    MOTOR_CrtlType_CAN* __motor = (MOTOR_CrtlType_CAN*)motor;
//    __motor->control.target_limit = limit;
//}

/**
  * @brief  Disable a motor and release its resouce
  * @param  motor: Pointer of the motor @ref MOTOR_CrtlType_CAN
  * @param  limit: Set output(current control value) limit
  */ 
void __MOTOR_SetSpeedPID(void* motor, PID_Type* pid) {
    if(!motor) return;
    MOTOR_CrtlType_CAN* __motor = (MOTOR_CrtlType_CAN*)motor;
    __motor->control.speed_pid = pid;
}

void __MOTOR_SetPositionPID(void* motor, PID_Type* pid) {
    if(!motor) return;
    MOTOR_CrtlType_CAN* __motor = (MOTOR_CrtlType_CAN*)motor;
    __motor->control.position_pid = pid;
}

/**
  * @brief  Mount motor on CAN-bus
  */ 
void __MOTOR_DataResolve_RM3510_3508(CanRxMsg* msg_data, void* motor);
void __MOTOR_DataResolve_RM2006(CanRxMsg* msg_data, void* motor);
void __MOTOR_DataResolve_RM6xxx(CanRxMsg* msg_data, void* motor);
void __MOTOR_DataResolve_GM3510(CanRxMsg* msg_data, void* motor);

void __MOTOR_MountCAN(void* motor, CAN_TypeDef* canx, uint16_t stdid) {
    if(!motor) return;
    MOTOR_CrtlType_CAN* __motor = (MOTOR_CrtlType_CAN*)motor;
    if(!__motor->flags.can_mounted && stdid) {
        memset(&__motor->data.receiver, 0x00, sizeof(__motor->data.receiver));
        
//        switch(__motor->flags.rm_type) {
//            case RM_6025_PITCH:
//                __motor->data.receiver.std_id = 0x206;
//                break;
//            case RM_6623_YAW:
//                __motor->data.receiver.std_id = 0x205;
//                break;
//            case RM_3508_P19:
//            case RM_3510_P19:
//            case RM_2006_P36:
//            case RM_3510_P27:
//            case GM_3510:
//            default:
//                break;
//        }
        __motor->data.receiver.std_id = stdid;

        __motor->data.receiver.owner = __motor;
        __motor->data.receiver.rx.addr = __motor->data.raw_data;
        __motor->data.receiver.rx.size = sizeof(__motor->data.raw_data)/sizeof(__motor->data.raw_data[0]);
        
        void __MOTOR_DataResolve(CanRxMsg*, void*);
        switch(__motor->flags.rm_type) {
            case RM_3508_P19:
                __motor->data.receiver.resolver = __MOTOR_DataResolve_RM3510_3508;
                break;
            case RM_3510_P19:
                __motor->data.receiver.resolver = __MOTOR_DataResolve_RM3510_3508;
                break;
            case RM_2006_P36:
                __motor->data.receiver.resolver = __MOTOR_DataResolve_RM2006;
                break;
            case RM_6025_PITCH:
                __motor->data.receiver.resolver = __MOTOR_DataResolve_RM6xxx;
                break;
            case RM_6623_YAW:
                __motor->data.receiver.resolver = __MOTOR_DataResolve_RM6xxx;
                break;
            case RM_3510_P27:
                __motor->data.receiver.resolver = __MOTOR_DataResolve_RM3510_3508;
                break;
            case GM_3510:
                __motor->data.receiver.resolver = __MOTOR_DataResolve_GM3510;
                break;
            default:
                __motor->data.receiver.resolver = NULL;
                break;
        }
        
        if(CAN_RegistReceiver(canx, &__motor->data.receiver)){
            __motor->flags.can_mounted = true;
        }
    }
}

/**
  * @brief  Pre-defined CAN-data resolvoing function for RM3508/RM3510/RM2006
  */ 
void __MOTOR_DataResolve_RM3510_3508(CanRxMsg* msg_data, void* motor) {
    if(!motor) return;
    MOTOR_CrtlType_CAN* __motor = (MOTOR_CrtlType_CAN*)motor;

    /* BYTE0+BYTE1 = mechanical angle */
    __motor->state.__motor_angel_curr = (msg_data->Data[0]<<8) | msg_data->Data[1];
    /* BYTE2+BYTE3 = speed */
    __motor->state.speed = (msg_data->Data[2]<<8) | msg_data->Data[3];
    
    if(__motor->flags.rm_type == RM_3508_P19) {
        /* BYTE2+BYTE3 = current */
        __motor->state.current = (msg_data->Data[4]<<8) | msg_data->Data[5];
        /* BYTE6 = tempreture */
        __motor->state.temprature = msg_data->Data[6];
    }
    
    /* Use angle calculation only when using position PID */
    if(__motor->control.position_pid) {
        float delta = 0.f;
        /* Calculate absolute angle from delta angle */
        if(__motor->state.__motor_angel_last!=-1){
            delta = __motor->state.__motor_angel_curr - __motor->state.__motor_angel_last;
            /* For motor cannot cover 4092 in a sampling period */
            delta += (delta>4096)?-8192:((delta<-4096)?8192:0);

            if(__motor->flags.rm_type == RM_3508_P19 || __motor->flags.rm_type == RM_3510_P19) {
                __motor->state.angle += delta/CAN_MOTOR_RM3510_3508_P19;
            } else if(__motor->flags.rm_type == RM_3510_P27) {
                __motor->state.angle += delta/CAN_MOTOR_RM3510_P27;
            }
            
            /* Too small delta angle is regarded as static/stuck */
            if((delta<CAN_STUCK_FILTER)&&(delta>-CAN_STUCK_FILTER)){
                __motor->state.mortor_stuckflag += 
                    (__motor->state.mortor_stuckflag==(uint16_t)-1)?0:1;
                __motor->state.mortor_stuckflag = (__motor->state.mortor_stuckflag>MOTOR_STUCK_THRESHOLD)?\
                    MOTOR_STUCK_THRESHOLD:__motor->state.mortor_stuckflag;
            }else{
                __motor->state.mortor_stuckflag = 0;
            }
        } else {
            __motor->state.__motor_angel_first = __motor->state.__motor_angel_curr;
        }
        /* Record current angle for next resolving */
        __motor->state.__motor_angel_last = __motor->state.__motor_angel_curr;
    }
    /* Clear flag */
    __motor->data.receiver.rx.changed = false;
}


void __MOTOR_DataResolve_RM2006(CanRxMsg* msg_data, void* motor) {
    if(!motor) return;
    MOTOR_CrtlType_CAN* __motor = (MOTOR_CrtlType_CAN*)motor;

    /* BYTE0+BYTE1 = mechanical angle */
    __motor->state.__motor_angel_curr = (msg_data->Data[0]<<8) | msg_data->Data[1];
    /* BYTE2+BYTE3 = speed */
    __motor->state.speed = (msg_data->Data[2]<<8) | msg_data->Data[3];
    /* BYTE2+BYTE3 = current */
    __motor->state.current = (msg_data->Data[4]<<8) | msg_data->Data[5];
    /* Use angle calculation only when using position PID */
    if(__motor->control.position_pid) {
        float delta = 0.f;
        /* Calculate absolute angle from delta angle */
        if(__motor->state.__motor_angel_last!=-1){
            delta = __motor->state.__motor_angel_curr - __motor->state.__motor_angel_last;
            /* For motor cannot cover 4092 in a sampling period */
            delta += (delta>4096)?-8192:((delta<-4096)?8192:0);
            __motor->state.angle += delta/CAN_MOTOR_RM2006_P36;
            /* Too small delta angle is regarded as static/stuck */
            if((delta<CAN_STUCK_FILTER)&&(delta>-CAN_STUCK_FILTER)){
                __motor->state.mortor_stuckflag += 
                    (__motor->state.mortor_stuckflag==(uint16_t)-1)?0:1;
                __motor->state.mortor_stuckflag = (__motor->state.mortor_stuckflag>MOTOR_STUCK_THRESHOLD)?\
                    MOTOR_STUCK_THRESHOLD:__motor->state.mortor_stuckflag;
            }else{
                __motor->state.mortor_stuckflag = 0;
            }
        } else {
            __motor->state.__motor_angel_first = __motor->state.__motor_angel_curr;
        }
        /* Record current angle for next resolving */
        __motor->state.__motor_angel_last = __motor->state.__motor_angel_curr;
    }
    /* Clear flag */
    __motor->data.receiver.rx.changed = false;
}

void __MOTOR_DataResolve_RM6xxx(CanRxMsg* msg_data, void* motor) {
    if(!motor) return;
    MOTOR_CrtlType_CAN* __motor = (MOTOR_CrtlType_CAN*)motor;

    /* BYTE0+BYTE1 = mechanical angle */
    __motor->state.__motor_angel_curr = (msg_data->Data[0]<<8) | msg_data->Data[1];
    /* BYTE2+BYTE3 = speed */
    __motor->state.speed = (msg_data->Data[2]<<8) | msg_data->Data[3];
    /* BYTE2+BYTE3 = current */
    __motor->state.current = (msg_data->Data[4]<<8) | msg_data->Data[5];
    /* Use angle calculation only when using position PID */
    if(__motor->control.position_pid) {
        float delta = 0.f;
        /* Calculate absolute angle from delta angle */
        if(__motor->state.__motor_angel_last!=-1){
            delta = __motor->state.__motor_angel_curr - __motor->state.__motor_angel_last;
            /* For motor cannot cover 4092 in a sampling period */
            delta += (delta>4096)?-8192:((delta<-4096)?8192:0);
            __motor->state.angle += delta;
            /* Too small delta angle is regarded as static/stuck */
            if((delta<CAN_STUCK_FILTER)&&(delta>-CAN_STUCK_FILTER)){
                __motor->state.mortor_stuckflag += 
                    (__motor->state.mortor_stuckflag==(uint16_t)-1)?0:1;
                __motor->state.mortor_stuckflag = (__motor->state.mortor_stuckflag>MOTOR_STUCK_THRESHOLD)?\
                    MOTOR_STUCK_THRESHOLD:__motor->state.mortor_stuckflag;
            }else{
                __motor->state.mortor_stuckflag = 0;
            }
        } else {
            __motor->state.__motor_angel_first = __motor->state.__motor_angel_curr;
        }
        /* Record current angle for next resolving */
        __motor->state.__motor_angel_last = __motor->state.__motor_angel_curr;
    }
    /* Clear flag */
    __motor->data.receiver.rx.changed = false;
}

void __MOTOR_DataResolve_GM3510(CanRxMsg* msg_data, void* motor) {
    if(!motor) return;
    MOTOR_CrtlType_CAN* __motor = (MOTOR_CrtlType_CAN*)motor;

    /* BYTE0+BYTE1 = mechanical angle */
    __motor->state.__motor_angel_curr = (msg_data->Data[0]<<8) | msg_data->Data[1];
    /* BYTE2+BYTE3 = current */
    __motor->state.current = (msg_data->Data[2]<<8) | msg_data->Data[3];
    /* Use angle calculation only when using position PID */
    if(__motor->control.position_pid) {
        float delta = 0.f;
        /* Calculate absolute angle from delta angle */
        if(__motor->state.__motor_angel_last!=-1){
            delta = __motor->state.__motor_angel_curr - __motor->state.__motor_angel_last;
            /* For motor cannot cover 4092 in a sampling period */
            delta += (delta>4096)?-8192:((delta<-4096)?8192:0);
            __motor->state.angle += delta;
            /* Too small delta angle is regarded as static/stuck */
            if((delta<CAN_STUCK_FILTER)&&(delta>-CAN_STUCK_FILTER)){
                __motor->state.mortor_stuckflag += 
                    (__motor->state.mortor_stuckflag==(uint16_t)-1)?0:1;
                __motor->state.mortor_stuckflag = (__motor->state.mortor_stuckflag>MOTOR_STUCK_THRESHOLD)?\
                    MOTOR_STUCK_THRESHOLD:__motor->state.mortor_stuckflag;
            }else{
                __motor->state.mortor_stuckflag = 0;
            }
        } else {
            __motor->state.__motor_angel_first = __motor->state.__motor_angel_curr;
        }
        /* Record current angle for next resolving */
        __motor->state.__motor_angel_last = __motor->state.__motor_angel_curr;
    }
    /* Clear flag */
    __motor->data.receiver.rx.changed = false;
}






/* Exported functions ---------------------------------------------------------*/
/** @defgroup Motor User Interface API
  * @brief    For system motor resources management.
  * @{
  */

MOTOR_CrtlType_CAN* MOTOR_RM_GetInstance(MOTOR_RM_Types type) {
    uint8_t i, size = sizeof(MOTORs)/sizeof(MOTORs[0]);
    for(i=0; i<size; i++) {
        if(!MOTORs[i].flags.enable) {
            /* Reset motor instance */
            memset(&MOTORs[i], 0x00, sizeof(MOTORs[i]));
//            MOTORs[i].THIS = &MOTORs[i];
            MOTORs[i].flags.rm_type = type;
            
            /* Set out basic parameters */
            MOTORs[i].control.output_limit = MOTOR_OUTPUTLIMIT_INIT;        /* Make motor output default limit. */
//            MOTORs[i].control.target_limit = MOTOR_OUTPUTLIMIT_INIT;      /* Make motor output default limit. */
            MOTORs[i].state.__motor_angel_last = -1;                        /* Save last angle value, init with -1 means not 
                                                                               get value yet. */
            /* Bind menber functions */
            MOTORs[i].__private.init = __MOTOR_Init;
//            MOTORs[i].implement.set_targetlimit = __MOTOR_SetTargetLimit;
            MOTORs[i].implement.set_outputlimit = __MOTOR_SetOutputLimit;
            MOTORs[i].implement.set_target = __MOTOR_SetTarget;
            MOTORs[i].implement.set_speed_pid = __MOTOR_SetSpeedPID;
            MOTORs[i].implement.set_position_pid = __MOTOR_SetPositionPID;
            MOTORs[i].implement.mount_can = __MOTOR_MountCAN;
            
            /* Call init function */
            MOTORs[i].__private.init(&MOTORs[i]);
            
            return &MOTORs[i];
        }
    }
    return NULL;
}

void MOTOR_SetDataResolve(MOTOR_CrtlType_CAN* motor, tFuncMemberNoParam func) {
    if(!motor) return;
    MOTOR_CrtlType_CAN* __motor = (MOTOR_CrtlType_CAN*)motor;
    __motor->__private.data_resolve = func;
}


/**
  * @}
  */


/** @defgroup Motor Control Functions
  * @brief    For system motor resources management.
  * @{
  */

/** 
  * @brief    Low-layer motor control loop.
  */

void MOTOR_ControlInit(void) {
    uint8_t i, size=sizeof(MOTORs)/sizeof(MOTORs[0]);
    for(i=0; i<size; i++) {
        memset(&MOTORs[i], 0x00, sizeof(MOTORs[i]));
    }
}

void MOTOR_ControlLooper(void) {
    uint8_t i, size = sizeof(MOTORs)/sizeof(MOTORs[0]);
    for(i=0; i<size; i++) {
        if(MOTORs[i].flags.enable && MOTORs[i].flags.can_mounted) {
            /* Calc position PID MOTORs[i].control.output */
            // TODO: Make interval time more specific.
            float pid_tmp;
            if(MOTORs[i].control.position_pid) {
                // TODO: Make interval time more specific.
                pid_tmp = PID_ControllerDriver(MOTORs[i].control.position_pid, 
                    MOTORs[i].control.target , MOTORs[i].state.angle);
                pid_tmp = __MOTOR_OutputLimit(&MOTORs[i], pid_tmp);
                if(MOTORs[i].control.speed_pid) {
                    pid_tmp = PID_ControllerDriver(MOTORs[i].control.speed_pid,
                        pid_tmp, MOTORs[i].state.speed);
                }
                MOTORs[i].control.output = __MOTOR_OutputLimit(&MOTORs[i], pid_tmp);
            }
            /* Calc speed PID MOTORs[i].control.output */
            else if(MOTORs[i].control.speed_pid){
                pid_tmp = PID_ControllerDriver(MOTORs[i].control.speed_pid, MOTORs[i].control.target , 
                    MOTORs[i].state.speed);
                MOTORs[i].control.output = __MOTOR_OutputLimit(&MOTORs[i], pid_tmp);
            }
        }
    }
}

/**
  * @}
  */

/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/
