/**
  ******************************************************************************
  * @file       CMControl.c
  * @author     @YangTianhao,490999282@qq.com; @TangJiaxin, tjx1024@126.com
  * @version    v1.0
  * @date       2017.Dec.11
  * @brief      Control Chassis Motors.
                CMControlLoop() shows the way to control the motion of chassis in different states.   
                Use PID to optimize Chassis motor control.        
  ******************************************************************************
  * @license
  *
  ******************************************************************************
  */
  
  //TODO: CAN monitor
  //TODO: Stuck monitor

/* Includes ------------------------------------------------------------------*/
#include "sp_chasis.h"

/* Private typedef -----------------------------------------------------------*/
typedef struct {
    MOTOR_CrtlType_CAN*          motors[8];
    
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
    } dataA;       /* For motor201~motor204 */
    
    struct {
        CAN_Transmitter         transmitter;
        uint8_t                 raw_data[8];
    } dataB;        /* For motor205~motor208 */
    
} __CAHSIS_ManagerType;
__CAHSIS_ManagerType            __CAHSIS_Manager;        /* Chasis1 for 201~204, chasis2 for 205~208 */


/* Private macro -------------------------------------------------------------*/
/**
  * @brief  Motor control parameters
  */
#define __CHASIS_kLinearSpeed               6       /*  */
#define __CHASIS_kAngularSpeed              6       /*  */
// #define __CHASIS_FilterAngular           40      /*  */
/**
  * @brief  Motor control parameters
  */
#define __CHASIS_kRCAngular                 0.6 /* Convert RC value to normal speed value */
#define __CHASIS_kRCAngularSlow             0.3     /* Convert RC value to slower speed value */
#define __CHASIS_MaxOutput                  8000

#define MOTOR_CAN_IDOFFSET                  0x201

/**
  * @brief  PID parameters for motor control
  */
//#define PIDVAL_CM_SPEED_p                   7.5f
//#define PIDVAL_CM_SPEED_i                   5.0f
//#define PIDVAL_CM_SPEED_d                   0.f
#define PIDVAL_CM_SPEED_limit               200

//#define PIDVAL_CM_POSI_p                    7.2f
//#define PIDVAL_CM_POSI_i                    0.f
//#define PIDVAL_CM_POSI_d                    0.36f
#define PIDVAL_CM_POSI_limit                200


/**
  * @brief  CAN1/CAN2 received message buffer
  */
PID_Type        __CHASIS_SpeedPID[8];
PID_Type        __CHASIS_PositionPID[8];
PID_Type        __CHASIS_FollowPID;

/* Private functions ---------------------------------------------------------*/
void            __CHASIS_Stop(void);


/**
  * @brief  Stop chasis movement via CAN-bus
  */
inline void __CHASIS_Stop(void) {
    uint8_t i, size=sizeof(__CAHSIS_Manager.motors)/sizeof(__CAHSIS_Manager.motors[0]);
    for(i=0; i<size; i++) {
        if(__CAHSIS_Manager.motors[i]->flags.enable) {
            __CAHSIS_Manager.motors[i]->implement.set_target(__CAHSIS_Manager.motors[i], 0);
        }
    }
}










/**
  * @brief  Control single motor's movement
  */
MOTOR_CrtlType_CAN* CHASIS_EnableMotor(CHASIS_MotorIdType motorx, MOTOR_RM_Types type, bool is_pos_pid) {
    if(!((uint8_t)motorx <= sizeof(__CAHSIS_Manager.motors)/sizeof(__CAHSIS_Manager.motors[0]))) {
        return NULL;
    }
    if( __CAHSIS_Manager.motors[(uint8_t)motorx] && __CAHSIS_Manager.motors[(uint8_t)motorx]->flags.enable ) {
        return __CAHSIS_Manager.motors[(uint8_t)motorx];
    }
    
    /* Init new motor */
    __CAHSIS_Manager.motors[(uint8_t)motorx] = MOTOR_RM_GetInstance(type);
    __CAHSIS_Manager.motors[(uint8_t)motorx]->implement.set_target(
        __CAHSIS_Manager.motors[(uint8_t)motorx], 0);
    __CAHSIS_Manager.motors[(uint8_t)motorx]->implement.mount_can(
        __CAHSIS_Manager.motors[(uint8_t)motorx], CAN1, MOTOR_CAN_IDOFFSET+(uint8_t)motorx);
    // __CAHSIS_Manager.motors[(uint8_t)motorx]->implement.set_limit(
    //    __CAHSIS_Manager.motors[(uint8_t)motorx], 5000);
    
    PID_ControllerInit(&__CHASIS_SpeedPID[(uint8_t)motorx], PIDVAL_CM_SPEED_limit, 
        0xFFFF, __CHASIS_MaxOutput, 0.01);
    
    __CAHSIS_Manager.motors[(uint8_t)motorx]->implement.set_speed_pid(
        __CAHSIS_Manager.motors[(uint8_t)motorx], &__CHASIS_SpeedPID[(uint8_t)motorx]);
    
    /* Init position PID controller if used */
    if(is_pos_pid) {
        PID_ControllerInit(&__CHASIS_PositionPID[(uint8_t)motorx], PIDVAL_CM_POSI_limit, 
            0xFFFF, __CHASIS_MaxOutput, 0.01);
        
        __CAHSIS_Manager.motors[(uint8_t)motorx]->implement.set_position_pid(
            __CAHSIS_Manager.motors[(uint8_t)motorx], &__CHASIS_PositionPID[(uint8_t)motorx]);
    }
    
    return __CAHSIS_Manager.motors[(uint8_t)motorx];
}

void CHASIS_SetMotorSpeed(CHASIS_MotorIdType motorx, float speed) {
    if((uint8_t)motorx <= sizeof(__CAHSIS_Manager.motors)/sizeof(__CAHSIS_Manager.motors[0]) &&
        __CAHSIS_Manager.motors[(uint8_t)motorx] &&
        __CAHSIS_Manager.motors[(uint8_t)motorx]->flags.enable && __CAHSIS_Manager.motors[(uint8_t)motorx]->control.speed_pid ) {
        __CAHSIS_Manager.motors[(uint8_t)motorx]->implement.set_target(__CAHSIS_Manager.motors[(uint8_t)motorx], speed);
    }
}

void CHASIS_SetMotorPosition(CHASIS_MotorIdType motorx, float position) {
    if((uint8_t)motorx <= sizeof(__CAHSIS_Manager.motors)/sizeof(__CAHSIS_Manager.motors[0]) &&
        __CAHSIS_Manager.motors[(uint8_t)motorx] &&
        __CAHSIS_Manager.motors[(uint8_t)motorx]->flags.enable && __CAHSIS_Manager.motors[(uint8_t)motorx]->control.position_pid ) {
        __CAHSIS_Manager.motors[(uint8_t)motorx]->implement.set_target(__CAHSIS_Manager.motors[(uint8_t)motorx], position);
    }
}

void CHASIS_SetMotorRelativePosition(CHASIS_MotorIdType motorx, float relaposition) {
    if((uint8_t)motorx <= sizeof(__CAHSIS_Manager.motors)/sizeof(__CAHSIS_Manager.motors[0]) &&
        __CAHSIS_Manager.motors[(uint8_t)motorx] &&
        __CAHSIS_Manager.motors[(uint8_t)motorx]->flags.enable && __CAHSIS_Manager.motors[(uint8_t)motorx]->control.position_pid ) {
        __MOTOR_SetTargetDelta(__CAHSIS_Manager.motors[(uint8_t)motorx], relaposition);
    }
}

MOTOR_CrtlType_CAN* CHASIS_GetMotor(CHASIS_MotorIdType motorx) {
    if((uint8_t)motorx <= sizeof(__CAHSIS_Manager.motors)/sizeof(__CAHSIS_Manager.motors[0]) &&
        __CAHSIS_Manager.motors[(uint8_t)motorx] && __CAHSIS_Manager.motors[(uint8_t)motorx]->flags.enable) {
        return __CAHSIS_Manager.motors[(uint8_t)motorx];
    } else
        return NULL;
}

//const MOTOR_CrtlType_CAN* CHASIS_GetMotor(CHASIS_MotorIdType motorx) {
//    if(id <= sizeof(__CAHSIS_Manager.motors)/sizeof(__CAHSIS_Manager.motors[0]) &&
//        __CAHSIS_Manager.motors[id] &&
//        __CAHSIS_Manager.motors[id]->flags.enable && __CAHSIS_Manager.motors[id]->control.position_pid ) {
//        return __CAHSIS_Manager.motors[id];
//    } else
//        return NULL;
//}

  
  





/**
  * @brief  Init chasis control
  */
void CHASIS_ControlInit() {
    memset(&__CAHSIS_Manager, 0x00, sizeof(__CAHSIS_Manager));
    
    /* Mount mortor201~mortor204 to CAN chasis control */
    __CAHSIS_Manager.dataA.transmitter.std_id = 0x200;
    __CAHSIS_Manager.dataA.transmitter.tx.size = 
        sizeof(__CAHSIS_Manager.dataA.raw_data)/sizeof(__CAHSIS_Manager.dataA.raw_data[0]);
    __CAHSIS_Manager.dataA.transmitter.tx.addr = __CAHSIS_Manager.dataA.raw_data;
    CAN_RegistTransmitter(CAN1, &__CAHSIS_Manager.dataA.transmitter);
    
    /* Mount mortor205~mortor208 to CAN chasis control */
    __CAHSIS_Manager.dataB.transmitter.std_id = 0x1FF;
    __CAHSIS_Manager.dataB.transmitter.tx.size = 
        sizeof(__CAHSIS_Manager.dataB.raw_data)/sizeof(__CAHSIS_Manager.dataB.raw_data[0]);
    __CAHSIS_Manager.dataB.transmitter.tx.addr = __CAHSIS_Manager.dataB.raw_data;
    CAN_RegistTransmitter(CAN1, &__CAHSIS_Manager.dataB.transmitter);
}

void CHASIS_ControlLooper(void) {
    int16_t speed;
    // uint8_t i, size=sizeof(__CAHSIS_Manager.motors)/sizeof(__CAHSIS_Manager.motors[0]);
    for(uint8_t i=0; i<4; i++) {
        speed = (int16_t)__CAHSIS_Manager.motors[i]->control.output;
        __CAHSIS_Manager.dataA.raw_data[2*i] = (speed>>8)&0xff;
        __CAHSIS_Manager.dataA.raw_data[2*i+1] = speed&0xff;
    }
    
    for(uint8_t i=0; i<4; i++) {
        speed = (int16_t)__CAHSIS_Manager.motors[i+4]->control.output;
        __CAHSIS_Manager.dataB.raw_data[2*i] = (speed>>8)&0xff;
        __CAHSIS_Manager.dataB.raw_data[2*i+1] = speed&0xff;
    }
    
    CAN_SubmitChange(&__CAHSIS_Manager.dataA.transmitter);
    CAN_SubmitChange(&__CAHSIS_Manager.dataB.transmitter);
}

/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/
