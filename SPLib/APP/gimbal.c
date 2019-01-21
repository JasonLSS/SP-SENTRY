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
  
/* Includes ------------------------------------------------------------------*/
#include "gimbal.h"

#define GM_ID               4
static struct {
    CAN_Transmitter         transmitter;
    uint8_t                 raw_data[8];
} gm_data;
PID_Type                    gm_spd;
PID_Type                    gm_pos;

struct __GimbalController{
    float yaw_set;
    float pitch_set;
} GimbalController;

MOTOR_CrtlType_CAN*         gimbal_yaw_motor;
MOTOR_CrtlType_CAN*         gimbal_pitch_motor;

uint8_t auto_aim_flag = 0;
uint8_t small_power_flag = 0;


static const float yaw_kp_0 = 8.0f;
static const float yaw_ki_0 = 10.0f;

void GIMBAL_ControlInit(void) {
    /* Init new motor */
    gimbal_yaw_motor  = MOTOR_RM_GetInstance(RM_6623_YAW);
    gimbal_yaw_motor->implement.set_target(gimbal_yaw_motor, 0);
    gimbal_yaw_motor->implement.set_outputlimit(gimbal_yaw_motor, 20000);
    
//        PID_ControllerInit(&gm_spd, 200, 0xFFFF, 16000, 0.01);
//        PID_SetGains(&gm_spd, 1.5f, 0, 0);
    PID_ControllerInit(&gm_pos, 500, 0xFFFF, 8000, 0.01f);
    PID_SetGains(&gm_pos, yaw_kp_0, yaw_ki_0*2, 0.15f);     // For init
    gm_pos.intergration_separation = 100;
    gimbal_yaw_motor->implement.set_speed_pid(gimbal_yaw_motor, NULL);
    gimbal_yaw_motor->implement.set_position_pid(gimbal_yaw_motor, &gm_pos);

    gimbal_yaw_motor->implement.mount_can(gimbal_yaw_motor, CAN1, 0x204+GM_ID);
    if(GM_ID<=4)
        gm_data.transmitter.std_id = 0x1FF;
    else
        gm_data.transmitter.std_id = 0x2FF;
    /* Mount mortor201~mortor204 to CAN chasis control */
    gm_data.transmitter.tx.addr = gm_data.raw_data;
    gm_data.transmitter.tx.size = sizeof(gm_data.raw_data)/sizeof(gm_data.raw_data[0]);
    CAN_RegistTransmitter(CAN1, &gm_data.transmitter);
    
    gimbal_pitch_motor = CHASIS_EnableMotor(Motor205, GM_3510, true);
    // PID_SetGains(gimbal_pitch_motor->control.speed_pid, 1.2f, 0, 0);
    gimbal_pitch_motor->implement.set_speed_pid(gimbal_pitch_motor, NULL);
    gimbal_pitch_motor->control.position_pid->intergration_separation = 100;
    gimbal_pitch_motor->control.position_pid->intergration_limit = 400;
    gimbal_pitch_motor->control.position_pid->output_limit = 8000;
    PID_SetGains(gimbal_pitch_motor->control.position_pid, 8.f, 15.0f, 0.4f);

}


void GIMBAL_ControlLooper(void) {
    
    // POS0: 3510-7620 6020-
    // -1200~1000
    
    if(fabs(gimbal_yaw_motor->state.angle - GimbalController.yaw_set) < 10.f) {
        gimbal_yaw_motor->control.position_pid->Kp = 0.6f*yaw_kp_0;
        gimbal_yaw_motor->control.position_pid->Ki = 1.5f*yaw_ki_0;
    } else {
        gimbal_yaw_motor->control.position_pid->Kp = yaw_kp_0;
        gimbal_yaw_motor->control.position_pid->Ki = yaw_ki_0;
    }
    
    gimbal_yaw_motor->implement.set_target(gimbal_yaw_motor, GimbalController.yaw_set);         // Yaw
    CHASIS_SetMotorPosition(Motor205, GimbalController.pitch_set);                              // Pitch

    int16_t speed = (int16_t)gimbal_yaw_motor->control.output;
    #if GM_ID<=4 
        gm_data.raw_data[2*GM_ID-2] = (speed>>8)&0xff;
        gm_data.raw_data[2*GM_ID-1] = speed&0xff;
    #else
        gm_data.raw_data[2*(GM_ID-4)-2] = (speed>>8)&0xff;
        gm_data.raw_data[2*(GM_ID-4)-1] = speed&0xff;
    #endif
    CAN_SubmitChange(&gm_data.transmitter);

}

void GIMBAL_Update(float target_pitch, float target_yaw) {
    GimbalController.pitch_set = target_pitch;
    GimbalController.yaw_set = target_yaw;
}

void GIMBAL_UpdatePitch(float target_pitch) {
    GimbalController.pitch_set = target_pitch;
}

void GIMBAL_UpdateYaw(float target_yaw) {
    GimbalController.yaw_set = target_yaw;
}


#define MIDDLE_YAW          2000
#define MIDDLE_PITCH        7620
bool GIMBAL_MiddleLooper(uint32_t tick) {
    static uint16_t pass_flag = 0;
    if(pass_flag >= 800) {
        gimbal_pitch_motor->state.angle = 0;
        CHASIS_SetMotorPosition(Motor205, 0);
        gimbal_pitch_motor->control.position_pid->Ki = yaw_ki_0;
        
        gimbal_yaw_motor->state.angle = 0;
        gimbal_yaw_motor->control.target = 0;
        gimbal_yaw_motor->control.position_pid->Ki = yaw_ki_0;
        return true;
    } else {
//        if(abs(gimbal_pitch_motor->state.__motor_angel_curr - MIDDLE_PITCH)<20 &&
        if( abs(gimbal_yaw_motor->state.__motor_angel_curr - MIDDLE_YAW)<1/0.0439f ) {
            pass_flag ++;
        } else {
            pass_flag = 0;
        }
        
        if(tick%10==1) {
            if(gimbal_yaw_motor->state.__motor_angel_last!=-1) {
                GIMBAL_UpdateYaw(MIDDLE_YAW - gimbal_yaw_motor->state.__motor_angel_first);
            }
            if(gimbal_pitch_motor->state.__motor_angel_last!=-1) {
                GIMBAL_UpdatePitch(MIDDLE_PITCH - gimbal_pitch_motor->state.__motor_angel_first);
            }
        }
    }
    return false;
}


/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/
