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

#define MOTOR_OUTPUTLIMIT_INIT              12000               /* Motor ouput default limitation, real range +-16384 */

#define MOTOR_BLOCK_ANGLE_FILTER            38.0f
#define MOTOR_BLOCK_CURRETN_FILTER          5000                /* Low angle delta and high current mean motor is stuck. */
#define MOTOR_BLOCK_THRESHOLD               1000                /* Stuck counter threshold. */

#define MOTOR_ENCODER_CONVERT               (0.0439453125f)     /* Convert encoder value 0~8192 to 0~360deg */
#define CAN_MOTOR_RM3510_P27                (27.f)
#define CAN_MOTOR_RM2006_P36                (36.f)
#define CAN_MOTOR_RM2006_P96                (96.f)
#define CAN_MOTOR_RM3510_3508_P19           (3591.f/187.f)       /* Motor's Mechanical trasmission ratio 
                                                                    Real value is 3591/187 = 19.2032f @RM3508 */
                                                                    
#define CAN_TRANS_STDID(x)                  ((x==0)*0x200 + (x==1)*0x1FF + (x==2)*0x2FF)

/**
  * @brief  Motor control parameters
  */
#define MOTOR_MaxOutput                     8000
#define MOTOR_CAN_IDOFFSET                  0x201

/**
  * @brief  PID parameters for motor control
  */
#define PIDVAL_CM_SPEED_IntegralLimit       200
#define PIDVAL_CM_POSI_IntegralLimit        200

/**
  * @brief  Motor pool (global motor resource)
  */
MOTOR_CrtlType_CAN                          MOTORs[MOTOR_POOLSIZE] = {0x00};

/**
  * @brief  CAN1/CAN2 received message buffer
  */
PID_Type                                    __CHASIS_SpeedPID[MOTOR_POOLSIZE];
PID_Type                                    __CHASIS_PositionPID[MOTOR_POOLSIZE];


/**
  * @brief  Limit controller output sending via CAN
  * @param  MotorCurrent: current control value
  * @param  limit: limitation
  */
// __STATIC_INLINE 
float __MOTOR_OutputLimit(MOTOR_CrtlType_CAN* __motor, float value) {
    return (value > __motor->control.output_limit)?__motor->control.output_limit:
        (value < -__motor->control.output_limit)?-__motor->control.output_limit:value;
}

/**
  * @brief  Enable a motor and request its resouce
  * @param  motor: Pointer of the motor @ref MOTOR_CrtlType_CAN
  */ 
void __MOTOR_Init(MOTOR_CrtlType_CAN* __motor) {
    if(!__motor) return;
    if(!__motor->flags.enable) {
        __motor->flags.enable = true;
        __motor->flags.stop = false;
    }
}

/**
  * @brief  Disable a motor and release its resouce
  * @param  motor: Pointer of the motor @ref MOTOR_CrtlType_CAN
  */ 
void __MOTOR_Destroy(MOTOR_CrtlType_CAN* __motor) {
    if(!__motor) return;
    if(__motor->flags.enable) {
        __motor->flags.enable = false;
    }
}

/**
  * @brief  Disable a motor and release its resouce
  * @param  motor: Pointer of the motor @ref MOTOR_CrtlType_CAN
  * @param  target: Motion target, differ by motor type (speed, position)
  */ 
void __MOTOR_SetTarget(MOTOR_CrtlType_CAN* __motor, float target) {
    if(!__motor) return;
        __motor->control.target = target;
}
void __MOTOR_SetTargetDelta(MOTOR_CrtlType_CAN* __motor, float delta) {
    if(!__motor) return;
        __motor->control.target += delta;
}

/**
  * @brief  Disable a motor and release its resouce
  * @param  motor: Pointer of the motor @ref MOTOR_CrtlType_CAN
  * @param  limit: Set output(current control value) limit
  */ 
void __MOTOR_SetOutputLimit(MOTOR_CrtlType_CAN* __motor, float limit) {
    if(!__motor) return;
    __motor->control.output_limit = limit;
}
//void __MOTOR_SetTargetLimit(MOTOR_CrtlType_CAN* __motor, float limit) {
//    if(!__motor) return;
//    __motor->control.target_limit = limit;
//}

/**
  * @brief  Disable a motor and release its resouce
  * @param  motor: Pointer of the motor @ref MOTOR_CrtlType_CAN
  * @param  limit: Set output(current control value) limit
  */ 
void __MOTOR_SetSpeedPID(MOTOR_CrtlType_CAN* __motor, PID_Type* pid) {
    if(!__motor) return;
    __motor->control.speed_pid = pid;
}

void __MOTOR_SetPositionPID(MOTOR_CrtlType_CAN* __motor, PID_Type* pid) {
    if(!__motor) return;
    __motor->control.position_pid = pid;
}



/**
  * @brief  Pre-defined CAN-data resolvoing function for RM3508/RM3510/RM2006
  */ 
void __MOTOR_DataResolve_RM3510_3508(CanRxMsg* msg_data, MOTOR_CrtlType_CAN* __motor) {
    if(!__motor) return;

    /* BYTE0+BYTE1 = mechanical angle */
    __motor->state.__motor_angel_curr = (msg_data->Data[0]<<8) | msg_data->Data[1];
    /* BYTE2+BYTE3 = speed */
    if(__motor->flags.rm_type == RM_3508_P19 || __motor->flags.rm_type == RM_3510_P19) {
        __motor->state.speed = spMATH_RPM2RAD( (int16_t)((msg_data->Data[2]<<8) | msg_data->Data[3])/CAN_MOTOR_RM3510_3508_P19 );
    } else if(__motor->flags.rm_type == RM_3510_P27) {
        __motor->state.speed = spMATH_RPM2RAD( (int16_t)((msg_data->Data[2]<<8) | msg_data->Data[3])/CAN_MOTOR_RM3510_P27 );
    }
    
    if(__motor->flags.rm_type == RM_3508_P19) {
        /* BYTE2+BYTE3 = current */
        __motor->state.current = (msg_data->Data[4]<<8) | msg_data->Data[5];
        /* BYTE6 = tempreture */
        __motor->state.temprature = msg_data->Data[6];
    }
    
    /* Use angle calculation only when using position PID */
//    if(__motor->control.position_pid) {
    float delta = 0.f;
    /* Calculate absolute angle from delta angle */
    if(__motor->state.__motor_angel_last!=-1){
        delta = __motor->state.__motor_angel_curr - __motor->state.__motor_angel_last;
        /* For motor cannot cover 4092 in a sampling period */
        delta += (delta>4096)?-8192:((delta<-4096)?8192:0);

        if(__motor->flags.rm_type == RM_3508_P19 || __motor->flags.rm_type == RM_3510_P19) {
            __motor->state.angle += spMATH_DEG2RAD(delta/CAN_MOTOR_RM3510_3508_P19*MOTOR_ENCODER_CONVERT);
        } else if(__motor->flags.rm_type == RM_3510_P27) {
            __motor->state.angle += spMATH_DEG2RAD(delta/CAN_MOTOR_RM3510_P27*MOTOR_ENCODER_CONVERT);
        }
        
        /* Too small delta angle is regarded as static/stuck */
        if(fabs(delta)<MOTOR_BLOCK_ANGLE_FILTER){
            __motor->state.motor_block_flag += (__motor->state.motor_block_flag>=MOTOR_BLOCK_THRESHOLD) ? 0: 1;
        }else{
            __motor->state.motor_block_flag = 0;
        }
    } else {
        __motor->state.__motor_angel_first = __motor->state.__motor_angel_curr;
    }
    /* Record current angle for next resolving */
    __motor->state.__motor_angel_last = __motor->state.__motor_angel_curr;
//    }
    /* Clear flag */
    __motor->data.receiver.rx.changed = false;
}


void __MOTOR_DataResolve_RM2006(CanRxMsg* msg_data, MOTOR_CrtlType_CAN* __motor) {
    if(!__motor) return;

    /* BYTE0+BYTE1 = mechanical angle */
    __motor->state.__motor_angel_curr = (msg_data->Data[0]<<8) | msg_data->Data[1];
    /* BYTE2+BYTE3 = speed */
    if(__motor->flags.rm_type == RM_2006_P36) {
        __motor->state.speed = spMATH_RPM2RAD( (int16_t)((msg_data->Data[2]<<8) | msg_data->Data[3])/CAN_MOTOR_RM2006_P36 );
    } else if(__motor->flags.rm_type == RM_2006_P96) {
        __motor->state.speed = spMATH_RPM2RAD( (int16_t)((msg_data->Data[2]<<8) | msg_data->Data[3])/CAN_MOTOR_RM2006_P96 );
    }
    /* BYTE2+BYTE3 = current */
    __motor->state.current = (msg_data->Data[4]<<8) | msg_data->Data[5];
    /* Use angle calculation only when using position PID */
//    if(__motor->control.position_pid) {
        float delta = 0.f;
        /* Calculate absolute angle from delta angle */
        if(__motor->state.__motor_angel_last!=-1){
            delta = __motor->state.__motor_angel_curr - __motor->state.__motor_angel_last;
            /* For motor cannot cover 4092 in a sampling period */
            delta += (delta>4096)?-8192:((delta<-4096)?8192:0);
            
            if(__motor->flags.rm_type == RM_2006_P36) {
                __motor->state.angle += spMATH_DEG2RAD(delta/CAN_MOTOR_RM2006_P36*MOTOR_ENCODER_CONVERT);
            } else if(__motor->flags.rm_type == RM_2006_P96) {
                __motor->state.angle += spMATH_DEG2RAD(delta/CAN_MOTOR_RM2006_P96*MOTOR_ENCODER_CONVERT);
            }
            
            /* Too small delta angle is regarded as static/stuck */
            if(fabs(delta)<MOTOR_BLOCK_ANGLE_FILTER){
                __motor->state.motor_block_flag += (__motor->state.motor_block_flag>=MOTOR_BLOCK_THRESHOLD) ? 0: 1;
            }else{
                __motor->state.motor_block_flag = 0;
            }
        } else {
            __motor->state.__motor_angel_first = __motor->state.__motor_angel_curr;
        }
        /* Record current angle for next resolving */
        __motor->state.__motor_angel_last = __motor->state.__motor_angel_curr;
//    }
    /* Clear flag */
    __motor->data.receiver.rx.changed = false;
}

void __MOTOR_DataResolve_RM6xxx(CanRxMsg* msg_data, MOTOR_CrtlType_CAN* __motor) {
    if(!__motor) return;

    /* BYTE0+BYTE1 = mechanical angle */
    __motor->state.__motor_angel_curr = (msg_data->Data[0]<<8) | msg_data->Data[1];
    /* BYTE2+BYTE3 = speed */
    __motor->state.speed = spMATH_RPM2RAD( (int16_t)((msg_data->Data[2]<<8) | msg_data->Data[3]) );
    /* BYTE2+BYTE3 = current */
    __motor->state.current = (msg_data->Data[4]<<8) | msg_data->Data[5];
    /* Use angle calculation only when using position PID */
//    if(__motor->control.position_pid) {
        float delta = 0.f;
        /* Calculate absolute angle from delta angle */
        if(__motor->state.__motor_angel_last!=-1){
            delta = __motor->state.__motor_angel_curr - __motor->state.__motor_angel_last;
            /* For motor cannot cover 4092 in a sampling period */
            delta += (delta>4096)?-8192:((delta<-4096)?8192:0);
            __motor->state.angle += spMATH_DEG2RAD(delta*MOTOR_ENCODER_CONVERT);
           
            /* Too small delta angle is regarded as static/stuck */
            if(fabs(delta)<MOTOR_BLOCK_ANGLE_FILTER){
                __motor->state.motor_block_flag += (__motor->state.motor_block_flag>=MOTOR_BLOCK_THRESHOLD) ? 0: 1;
            }else{
                __motor->state.motor_block_flag = 0;
            }
        } else {
            __motor->state.__motor_angel_first = __motor->state.__motor_angel_curr;
        }
        /* Record current angle for next resolving */
        __motor->state.__motor_angel_last = __motor->state.__motor_angel_curr;
//    }
    /* Clear flag */
    __motor->data.receiver.rx.changed = false;
}

void __MOTOR_DataResolve_GM3510(CanRxMsg* msg_data, MOTOR_CrtlType_CAN* __motor) {
    if(!__motor) return;

    /* BYTE0+BYTE1 = mechanical angle */
    __motor->state.__motor_angel_curr = (msg_data->Data[0]<<8) | msg_data->Data[1];
    /* BYTE2+BYTE3 = current */
    __motor->state.current = (msg_data->Data[2]<<8) | msg_data->Data[3];
    /* Use angle calculation only when using position PID */
//    if(__motor->control.position_pid) {
        float delta = 0.f;
        /* Calculate absolute angle from delta angle */
        if(__motor->state.__motor_angel_last!=-1){
            delta = __motor->state.__motor_angel_curr - __motor->state.__motor_angel_last;
            /* For motor cannot cover 4092 in a sampling period */
            delta += (delta>4096)?-8192:((delta<-4096)?8192:0);
            __motor->state.angle += spMATH_DEG2RAD(delta*MOTOR_ENCODER_CONVERT);
            
            /* Too small delta angle is regarded as static/stuck */
            if(fabs(delta)<MOTOR_BLOCK_ANGLE_FILTER){
                __motor->state.motor_block_flag += (__motor->state.motor_block_flag>=MOTOR_BLOCK_THRESHOLD) ? 0: 1;
            }else{
                __motor->state.motor_block_flag = 0;
            }
        } else {
            __motor->state.__motor_angel_first = __motor->state.__motor_angel_curr;
        }
        /* Record current angle for next resolving */
        __motor->state.__motor_angel_last = __motor->state.__motor_angel_curr;
//    }
    /* Clear flag */
    __motor->data.receiver.rx.changed = false;
}

/**
  * @brief  Mount motor on CAN-bus
  */ 
void __MOTOR_MountCAN(MOTOR_CrtlType_CAN* __motor, CAN_TypeDef* canx, uint16_t stdid) {
    if(!__motor) return;
    
    if(!__motor->flags.can_mounted && stdid) {
        memset(&__motor->data.receiver, 0x00, sizeof(__motor->data.receiver));
        
        __motor->data.receiver.std_id = stdid;

        __motor->data.receiver.owner = __motor;
        __motor->data.receiver.rx.addr = __motor->data.raw_data;
        __motor->data.receiver.rx.size = sizeof(__motor->data.raw_data)/sizeof(__motor->data.raw_data[0]);
        
        switch(__motor->flags.rm_type) {
            case RM_3508_P19:
            case RM_3510_P19:
            case RM_3510_P27:
                __motor->data.receiver.resolver = (ResolverCallback_t)__MOTOR_DataResolve_RM3510_3508;
                break;
            case RM_2006_P36:
            case RM_2006_P96:
                __motor->data.receiver.resolver = (ResolverCallback_t)__MOTOR_DataResolve_RM2006;
                break;
            case RM_6025_PITCH:
            case RM_6623_YAW:
                __motor->data.receiver.resolver = (ResolverCallback_t)__MOTOR_DataResolve_RM6xxx;
                break;
            case GM_3510:
                __motor->data.receiver.resolver = (ResolverCallback_t)__MOTOR_DataResolve_GM3510;
                break;
            default:
                __motor->data.receiver.resolver = NULL;
                break;
        }
        
        if(spCAN.user.registe_receiver(canx, &__motor->data.receiver)){
            __motor->flags.can_mounted = true;
        }
    }
}

/**
  * @brief  Get an instance of @ref MOTOR_CrtlType_CAN type.
  */ 
MOTOR_CrtlType_CAN* MOTOR_RM_GetInstance(MOTOR_RM_Types type) {
    uint8_t i, size = sizeof(MOTORs)/sizeof(MOTORs[0]);
    for(i=0; i<size; i++) {
        if(!MOTORs[i].flags.enable) {
            /* Reset motor instance */
            memset(&MOTORs[i], 0x00, sizeof(MOTORs[i]));
            MOTORs[i].THIS = &MOTORs[i];
            MOTORs[i].flags.rm_type = type;
            
            /* Set out basic parameters */
            MOTORs[i].control.output_limit = MOTOR_OUTPUTLIMIT_INIT;        /* Make motor output default limit. */
//            MOTORs[i].control.target_limit = MOTOR_OUTPUTLIMIT_INIT;      /* Make motor output default limit. */
            MOTORs[i].state.__motor_angel_last = -1;                        /* Save last angle value, init with -1 means not 
                                                                               get value yet. */
            /* Call init function */
            __MOTOR_Init(&MOTORs[i]);
            
            return &MOTORs[i];
        }
    }
    return NULL;
}

/**
  * @brief  Config function to resolve feedback data.
  * @note   Function type @ref tFuncMemberNoParam
  */ 
void MOTOR_SetDataResolve(MOTOR_CrtlType_CAN* __motor, ResolverCallback_t func) {
    if(!__motor) return;
    __motor->data.receiver.resolver = func;
}






  //TODO: CAN monitor
  //TODO: Stuck monitor

typedef struct {
    CAN_TypeDef*                 canx;
    MOTOR_CrtlType_CAN*          motors[CAN_MOTOR_COUNT];
    /**
      * @brief  Raw data from CAN
      * @note   Each CAN-bus can connect with 8 motors under address 0x200 @ref RoboMaster_Motor_Manual
      *         (+)Freq         1kHz
      *         (+)Angle        0~8192 = 0~360deg (for rotor before deceleration )
      *         (+)SpeedUnit    RPM (for rotor before deceleration )
      *         (+)TempUnit     Celsius Degree
      */
    struct {
        CAN_Transmitter         transmitter;
        uint8_t                 raw_data[8];
    } datas[4];       /* For motor201~motor204 */
//    struct {
//        CAN_Transmitter         transmitter;
//        int16_t                 raw_data[4];
//    } dataB;        /* For motor205~motor208 */
//    struct {
//        CAN_Transmitter         transmitter;
//        int16_t                 raw_data[4];
//    } dataC;        /* For motor209~motor20C */
} __MOTOR_BusManagerType;

__MOTOR_BusManagerType            __MOTOR_CAN1_Manager;
__MOTOR_BusManagerType            __MOTOR_CAN2_Manager;



/**
  * @brief  Control single motor's movement
  */
MOTOR_CrtlType_CAN* CHASIS_AddMonitor(
    CAN_TypeDef* CANx,
    CHASIS_MotorIdType motorx, 
    MOTOR_RM_Types type) 
{
    if(CANx!=CAN1 && CANx!=CAN2) return NULL;
    __MOTOR_BusManagerType *manager = (CANx==CAN1)? &__MOTOR_CAN1_Manager:&__MOTOR_CAN2_Manager;

    /* Check accessibility */
    if(!((uint8_t)motorx <= sizeof(manager->motors)/sizeof(manager->motors[0]))) {
        return NULL;
    }
    if( manager->motors[(uint8_t)motorx] && manager->motors[(uint8_t)motorx]->flags.enable ) {
        return manager->motors[(uint8_t)motorx];
    }
    
    /* Init new motor */
    manager->motors[(uint8_t)motorx] = MOTOR_RM_GetInstance(type);
    __MOTOR_SetTarget(manager->motors[(uint8_t)motorx], 0);
    __MOTOR_MountCAN(manager->motors[(uint8_t)motorx], manager->canx, MOTOR_CAN_IDOFFSET+(uint8_t)motorx);
    
    __MOTOR_SetSpeedPID(manager->motors[(uint8_t)motorx], NULL);
    __MOTOR_SetPositionPID(manager->motors[(uint8_t)motorx], NULL);
    return manager->motors[(uint8_t)motorx];
}

MOTOR_CrtlType_CAN* CHASIS_EnableMotor(
    CAN_TypeDef* CANx,
    CHASIS_MotorIdType motorx, 
    MOTOR_RM_Types type, 
    bool is_pos_pid) 
{
    if(CANx!=CAN1 && CANx!=CAN2) return NULL;
    __MOTOR_BusManagerType *manager = (CANx==CAN1)? &__MOTOR_CAN1_Manager:&__MOTOR_CAN2_Manager;

    /* Check accessibility */
    if(!((uint8_t)motorx <= sizeof(manager->motors)/sizeof(manager->motors[0]))) {
        return NULL;
    }
    if( manager->motors[(uint8_t)motorx] && manager->motors[(uint8_t)motorx]->flags.enable ) {
        return manager->motors[(uint8_t)motorx];
    }
    
    /* Init new motor */
    manager->motors[(uint8_t)motorx] = MOTOR_RM_GetInstance(type);
    __MOTOR_SetTarget(manager->motors[(uint8_t)motorx], 0);
    __MOTOR_MountCAN(manager->motors[(uint8_t)motorx], manager->canx, MOTOR_CAN_IDOFFSET+(uint8_t)motorx);
    
    PID_ControllerInit(&__CHASIS_SpeedPID[(uint8_t)motorx], PIDVAL_CM_SPEED_IntegralLimit, 
        0xFFFF, MOTOR_MaxOutput);
    
    __MOTOR_SetSpeedPID(manager->motors[(uint8_t)motorx], &__CHASIS_SpeedPID[(uint8_t)motorx]);
    
    /* Init position PID controller if used */
    if(is_pos_pid) {
        PID_ControllerInit(&__CHASIS_PositionPID[(uint8_t)motorx], PIDVAL_CM_POSI_IntegralLimit, 
            0xFFFF, MOTOR_MaxOutput);
        
        __MOTOR_SetPositionPID(manager->motors[(uint8_t)motorx], &__CHASIS_PositionPID[(uint8_t)motorx]);
    }
    
    return manager->motors[(uint8_t)motorx];
}

void CHASIS_SetMotorSpeed(CAN_TypeDef* CANx, CHASIS_MotorIdType motorx, float speed) {
    if(CANx!=CAN1 && CANx!=CAN2) return;
    __MOTOR_BusManagerType *manager = (CANx==CAN1)? &__MOTOR_CAN1_Manager:&__MOTOR_CAN2_Manager;
    if((uint8_t)motorx <= sizeof(manager->motors)/sizeof(manager->motors[0]) &&
        manager->motors[(uint8_t)motorx] &&
        manager->motors[(uint8_t)motorx]->flags.enable && manager->motors[(uint8_t)motorx]->control.speed_pid ) {
        __MOTOR_SetTarget(manager->motors[(uint8_t)motorx], speed);
    }
}

void CHASIS_SetMotorPosition(CAN_TypeDef* CANx, CHASIS_MotorIdType motorx, float position) {
    if(CANx!=CAN1 && CANx!=CAN2) return;
    __MOTOR_BusManagerType *manager = (CANx==CAN1)? &__MOTOR_CAN1_Manager:&__MOTOR_CAN2_Manager;
    if((uint8_t)motorx <= sizeof(manager->motors)/sizeof(manager->motors[0]) &&
        manager->motors[(uint8_t)motorx] &&
        manager->motors[(uint8_t)motorx]->flags.enable && manager->motors[(uint8_t)motorx]->control.position_pid ) {
        __MOTOR_SetTarget(manager->motors[(uint8_t)motorx], position);
    }
}

void CHASIS_SetMotorRelativePosition(CAN_TypeDef* CANx, CHASIS_MotorIdType motorx, float relaposition) {
    if(CANx!=CAN1 && CANx!=CAN2) return;
    __MOTOR_BusManagerType *manager = (CANx==CAN1)? &__MOTOR_CAN1_Manager:&__MOTOR_CAN2_Manager;
    if((uint8_t)motorx <= sizeof(manager->motors)/sizeof(manager->motors[0]) &&
        manager->motors[(uint8_t)motorx] &&
        manager->motors[(uint8_t)motorx]->flags.enable && manager->motors[(uint8_t)motorx]->control.position_pid ) {
        __MOTOR_SetTargetDelta(manager->motors[(uint8_t)motorx], relaposition);
    }
}

MOTOR_CrtlType_CAN* CHASIS_GetMotor(CAN_TypeDef* CANx, CHASIS_MotorIdType motorx) {
    if(CANx!=CAN1 && CANx!=CAN2) return NULL;
    __MOTOR_BusManagerType *manager = (CANx==CAN1)? &__MOTOR_CAN1_Manager:&__MOTOR_CAN2_Manager;
    if((uint8_t)motorx <= sizeof(manager->motors)/sizeof(manager->motors[0]) &&
        manager->motors[(uint8_t)motorx] && manager->motors[(uint8_t)motorx]->flags.enable) {
        return manager->motors[(uint8_t)motorx];
    } else
        return NULL;
}

/**
  * @brief  Stop chasis movement via CAN-bus
  */
void __CHASIS_Stop(CAN_TypeDef* CANx, CHASIS_MotorIdType motorx) {
    if(CANx!=CAN1 && CANx!=CAN2) return;
    __MOTOR_BusManagerType *manager = (CANx==CAN1)? &__MOTOR_CAN1_Manager:&__MOTOR_CAN2_Manager;
    if((uint8_t)motorx <= sizeof(manager->motors)/sizeof(manager->motors[0]) &&
        manager->motors[(uint8_t)motorx] &&
        manager->motors[(uint8_t)motorx]->flags.enable) {
        manager->motors[(uint8_t)motorx]->flags.stop = true;
        manager->motors[(uint8_t)motorx]->control.output = 0;
    }
 }

/**
  * @brief  If motor is stopping
  */
bool __CHASIS_GetStopSatus(CAN_TypeDef* CANx, CHASIS_MotorIdType motorx) {
    if(CANx!=CAN1 && CANx!=CAN2) return true;
    __MOTOR_BusManagerType *manager = (CANx==CAN1)? &__MOTOR_CAN1_Manager:&__MOTOR_CAN2_Manager;
    if((uint8_t)motorx <= sizeof(manager->motors)/sizeof(manager->motors[0]) &&
        manager->motors[(uint8_t)motorx] &&
        manager->motors[(uint8_t)motorx]->flags.enable) {
        return !manager->motors[(uint8_t)motorx]->flags.stop;
    }
    return true;
 }
 
/**
  * @brief  Start chasis movement via CAN-bus
  */
void __CHASIS_Start(CAN_TypeDef* CANx, CHASIS_MotorIdType motorx) {
    if(CANx!=CAN1 && CANx!=CAN2) return;
    __MOTOR_BusManagerType *manager = (CANx==CAN1)? &__MOTOR_CAN1_Manager:&__MOTOR_CAN2_Manager;
    if((uint8_t)motorx <= sizeof(manager->motors)/sizeof(manager->motors[0]) &&
        manager->motors[(uint8_t)motorx] &&
        manager->motors[(uint8_t)motorx]->flags.enable) {
        manager->motors[(uint8_t)motorx]->flags.stop = false;
    }
 }




/** 
  * @brief    Low-layer motor control loop.
  */

void MOTOR_ControlInit(void) {
    uint8_t i, size=sizeof(MOTORs)/sizeof(MOTORs[0]);
    for(i=0; i<size; i++) {
        memset(&MOTORs[i], 0x00, sizeof(MOTORs[i]));
    }

    memset(&__MOTOR_CAN1_Manager, 0x00, sizeof(__MOTOR_CAN1_Manager));
    memset(&__MOTOR_CAN2_Manager, 0x00, sizeof(__MOTOR_CAN2_Manager));
    
    __MOTOR_CAN1_Manager.canx = CAN1;
    __MOTOR_CAN2_Manager.canx = CAN2;
    
    size = sizeof(__MOTOR_CAN1_Manager.datas)/sizeof(__MOTOR_CAN1_Manager.datas[0]);
    for(i=0; i<size; i++) {
        __MOTOR_CAN1_Manager.datas[i].transmitter.std_id = CAN_TRANS_STDID(i);
        __MOTOR_CAN1_Manager.datas[i].transmitter.tx.size = sizeof(__MOTOR_CAN1_Manager.datas[i].raw_data)/sizeof(__MOTOR_CAN1_Manager.datas[i].raw_data[0]);
        __MOTOR_CAN1_Manager.datas[i].transmitter.tx.addr = __MOTOR_CAN1_Manager.datas[i].raw_data;
        spCAN.user.registe_transmitter(__MOTOR_CAN1_Manager.canx, &__MOTOR_CAN1_Manager.datas[i].transmitter);
    }
    
    size = sizeof(__MOTOR_CAN2_Manager.datas)/sizeof(__MOTOR_CAN2_Manager.datas[0]);
    for(i=0; i<size; i++) {
        __MOTOR_CAN2_Manager.datas[i].transmitter.std_id = CAN_TRANS_STDID(i);
        __MOTOR_CAN2_Manager.datas[i].transmitter.tx.size = sizeof(__MOTOR_CAN2_Manager.datas[i].raw_data)/sizeof(__MOTOR_CAN2_Manager.datas[i].raw_data[0]);
        __MOTOR_CAN2_Manager.datas[i].transmitter.tx.addr = __MOTOR_CAN2_Manager.datas[i].raw_data;
        spCAN.user.registe_transmitter(__MOTOR_CAN2_Manager.canx, &__MOTOR_CAN2_Manager.datas[i].transmitter);
    }
}

void MOTOR_ControlLooper(void) {
    /* Update output value with PID controller */
    for(uint8_t i=0; i<sizeof(MOTORs)/sizeof(MOTORs[0]); i++) {
        if(MOTORs[i].flags.enable && MOTORs[i].flags.can_mounted) {
            if(MOTORs[i].flags.stop) {
                /* Force output to zero when stop mode. */
                MOTORs[i].control.output = 0;
            } else {
                /* Calc position PID MOTORs[i].control.output */
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
    
    
    /* Submit data update to CAN */
    for(uint8_t i=0; i<sizeof(__MOTOR_CAN1_Manager.motors)/sizeof(__MOTOR_CAN1_Manager.motors[0]); i++) {
        int16_t speed = (int16_t)__MOTOR_CAN1_Manager.motors[i]->control.output;
        if(__MOTOR_CAN1_Manager.motors[i]->flags.rm_type == RM_3510_P19 ||
            __MOTOR_CAN1_Manager.motors[i]->flags.rm_type == RM_3508_P19 ||
            __MOTOR_CAN1_Manager.motors[i]->flags.rm_type == RM_3510_P27) {
            speed = deadzone_gain(speed, 200, 600);
        }
        __MOTOR_CAN1_Manager.datas[i/4].raw_data[2*(i%4)] = (speed>>8)&0xff;
        __MOTOR_CAN1_Manager.datas[i/4].raw_data[2*(i%4)+1] = speed&0xff;
        
    }

    for(uint8_t i=0; i<sizeof(__MOTOR_CAN2_Manager.motors)/sizeof(__MOTOR_CAN2_Manager.motors[0]); i++) {
        int16_t speed = (int16_t)__MOTOR_CAN2_Manager.motors[i]->control.output;
        if(__MOTOR_CAN2_Manager.motors[i]->flags.rm_type == RM_3510_P19 ||
            __MOTOR_CAN2_Manager.motors[i]->flags.rm_type == RM_3508_P19 ||
            __MOTOR_CAN2_Manager.motors[i]->flags.rm_type == RM_3510_P27) {
            speed = deadzone_gain(speed, 200, 600);
        }
        __MOTOR_CAN2_Manager.datas[i/4].raw_data[2*(i%4)] = (speed>>8)&0xff;
        __MOTOR_CAN2_Manager.datas[i/4].raw_data[2*(i%4)+1] = speed&0xff;
    }
    

    spCAN.user.send(&__MOTOR_CAN1_Manager.datas[0].transmitter);
    spCAN.user.send(&__MOTOR_CAN1_Manager.datas[1].transmitter);
    spCAN.user.send(&__MOTOR_CAN1_Manager.datas[2].transmitter);
    spCAN.user.send(&__MOTOR_CAN1_Manager.datas[3].transmitter);
    spCAN.user.send(&__MOTOR_CAN2_Manager.datas[0].transmitter);
    spCAN.user.send(&__MOTOR_CAN2_Manager.datas[1].transmitter);
    spCAN.user.send(&__MOTOR_CAN2_Manager.datas[2].transmitter);
    spCAN.user.send(&__MOTOR_CAN2_Manager.datas[3].transmitter);
}



struct __MOTOR_Manager_Type spMOTOR = {
    ._system = {
        .init = MOTOR_ControlInit,
        .looper = MOTOR_ControlLooper,
    },
    .user = {
        .enable = CHASIS_EnableMotor,
        .enable_simple = CHASIS_AddMonitor,
        .set_speed = CHASIS_SetMotorSpeed,
        .set_position = CHASIS_SetMotorPosition,
        .set_relative_position = CHASIS_SetMotorRelativePosition,
        .get = CHASIS_GetMotor,
        .stop = __CHASIS_Stop,
        .isstop = __CHASIS_GetStopSatus,
        .start = __CHASIS_Start,
    },
    .motor = {
        .get_instance = MOTOR_RM_GetInstance,
        .enable = __MOTOR_Init,
        .disable = __MOTOR_Destroy,
        .set_target = __MOTOR_SetTarget,
        .set_target_delta = __MOTOR_SetTargetDelta,
        .set_output_limit = __MOTOR_SetOutputLimit,
        .set_speed_pid = __MOTOR_SetSpeedPID,
        .set_position_pid = __MOTOR_SetPositionPID,
        .set_resolver = MOTOR_SetDataResolve,
    }
};

/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/
