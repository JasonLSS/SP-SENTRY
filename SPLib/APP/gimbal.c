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
#include "sp_shoot.h"
#include "Auto_aim.h"
#include "sp_chasis.h"

#define MOTOR_ENCODER_CONVERT               (0.0439453125f)     /* Convert encoder value 0~8192 to 0~360deg */
bool                        gimbal_inited = false;
char  usart3_buff[256];
#if defined(SP_FANTRY)

#elif defined(SP_SENTRY)
#define GM_ID               4

struct __GimbalController{
    float yaw_set;
    float pitch_set;
} GimbalController;

MOTOR_CrtlType_CAN*         gimbal_yaw_motor;
MOTOR_CrtlType_CAN*         gimbal_pitch_motor;

extern RC_DataType recv;
static RobotMode robotMode_ex;

static const float yaw_kp_0 = 5000.0f;
static const float yaw_ki_0 = 1000.0f;
static const float yaw_kd_0 = 100.f;

static const float pitch_kp_0 = 0.0f;
static const float pitch_ki_0 = 0.f;
static const float pitch_kd_0 = 0.f;

static float yaw_limit_max = 6000.f/8192.f*2.f*PI;
static float yaw_limit_min = -4000.f/8192.f*2.f*PI;
static float pitch_limit_max = 1500.f/8192.f*2.f*PI;
static float pitch_limit_min = 200.f/8192.f*2.f*PI;

static const float visual_yaw_kp = 6000.0f;
static const float visual_yaw_ki = 1000.0f;
static const float visual_yaw_kd = 300.f;

static const float visual_pitch_kp = 0.0f;
static const float visual_pitch_ki = 0.0f;
static const float visual_pitch_kd = 0.0f;

static const float cruise_yaw_kp = 6000.0f;
static const float cruise_yaw_ki = 1000.0f;
static const float cruise_yaw_kd = 300.f;

static const float cruise_pitch_kp = 0.0f;
static const float cruise_pitch_ki = 0.0f;
static const float cruise_pitch_kd = 0.0f;

static const float yaw_cruise_speed = 20.f/8192.f*2.f*PI;//task_lss
static const float pitch_cruise_speed = 6.f/8192.f*2.f*PI;

float yaw_limit[7]= { 0 , 1500.f/8192.f*2.f*PI, 3000.f/8192.f*2.f*PI , -1250.f/8192.f*2.f*PI , 
											-2500.f/8192.f*2.f*PI , -3750.f/8192.f*2.f*PI , -5000.f/8192.f*2.f*PI };//!!!!!!


#define MIDDLE_YAW          1400
#define MIDDLE_PITCH        4000  //7620

void GIMBAL_ControlInit(void) {
    /* Init new motor */
		gimbal_yaw_motor  = spMOTOR.user.enable_simple(CAN1, Motor208, RM_6623_YAW);
    gimbal_pitch_motor = spMOTOR.user.enable_simple(CAN1, Motor205, RM_2006_P36);
	
		PID_ControllerInit(&spGIMBAL_Controller._target.PID.yaw, 5*PI, -1, 5000);
		PID_ControllerInit(&spGIMBAL_Controller._target.PID.pitch, 5*PI, -1, 8000);
	
    spGIMBAL_Controller._target.PID.yaw.intergration_separation = PI;
    spGIMBAL_Controller._target.PID.yaw.output_limit = 5000;
	
		spGIMBAL_Controller._target.PID.pitch.intergration_separation = PI;
    spGIMBAL_Controller._target.PID.pitch.output_limit = 8000;
	
    PID_SetGains(&spGIMBAL_Controller._target.PID.yaw, yaw_kp_0 * 2, yaw_ki_0 * 2, yaw_kd_0);     // For init
    PID_SetGains(&spGIMBAL_Controller._target.PID.pitch, pitch_kp_0, pitch_ki_0*2.f, pitch_kd_0);
    
    spGIMBAL_Controller._target.gimbal_yaw_motor = gimbal_yaw_motor;
    spGIMBAL_Controller._target.gimbal_pitch_motor = gimbal_pitch_motor;

}


void GIMBAL_ControlLooper(void) {
		float target_yaw,target_pitch;
		target_yaw = PID_ControllerDriver(&spGIMBAL_Controller._target.PID.yaw,GimbalController.yaw_set,gimbal_yaw_motor->state.angle);
		target_pitch = PID_ControllerDriver(&spGIMBAL_Controller._target.PID.pitch,GimbalController.pitch_set,gimbal_pitch_motor->state.angle);
		gimbal_yaw_motor->control.output = target_yaw;
		gimbal_pitch_motor->control.output = target_pitch;
		if(robotMode == STANDBY_MODE&& gimbal_inited){
			gimbal_yaw_motor->control.output = 0;
			gimbal_pitch_motor->control.output = 0;
		}
		u8 size = sprintf(usart3_buff, "%f,%f,%f,%f\r\n",gimbal_yaw_motor->state.angle, gimbal_pitch_motor->state.angle,
													gimbal_yaw_motor->control.output,gimbal_pitch_motor->control.output);
		spDMA.controller.start(spDMA_USART3_tx_stream, (uint32_t)usart3_buff, (uint32_t)&USART3->DR, size);
}

void GIMBAL_State(void){
		static float yaw_set = 0;
		static float pitch_set = 0;
		static int yaw_direction = 1;
		static int pitch_direction = 1;
		int id; 
		if(robotMode == REMOTE_MODE && robotMode^robotMode_ex){
			spGIMBAL_Controller.user.pid_init();
			id = 0;
		}
		if(robotMode == CRUISE_MODE && robotMode^robotMode_ex){
			yaw_direction = 1;
			pitch_direction = 1;
			spGIMBAL_Controller.user.cruise_pid_init(); 
			id = 0;
		}
		if(robotMode == ESCAPE_MODE && robotMode^robotMode_ex){
			spGIMBAL_Controller.user.cruise_pid_init(); 
			id = 0;
		}
		if(robotMode == STATIC_ATTACK_MODE && robotMode^robotMode_ex){
			spGIMBAL_Controller.user.visual_pid_init(); 
		}
		if(robotMode == DYNAMIC_ATTACK_MODE && robotMode^robotMode_ex){
			spGIMBAL_Controller.user.visual_pid_init(); 
		}
		if(robotMode == ESCAPE_ATTACK_MODE && robotMode^robotMode_ex){
			spGIMBAL_Controller.user.visual_pid_init(); 
		}
		
		
		if(robotMode == REMOTE_MODE){
			yaw_set +=  (abs(recv.rc.ch0)<20?0:recv.rc.ch0)/15.f/1320.0f;
			pitch_set += (abs(recv.rc.ch1)<20?0:recv.rc.ch1)/15.f/1320.0f;
		}
		else if(robotMode == STANDBY_MODE){
			
		}
		else if(robotMode == CRUISE_MODE){
			yaw_set += yaw_cruise_speed*yaw_direction;
			pitch_set = 0.7f;
		}
		else if(robotMode == ESCAPE_MODE){
			
		}
		else if(robotMode == STATIC_ATTACK_MODE){	
			id = enemy_area;
			if(if_if_newframe){
					yaw_set = gimbal_yaw_motor->state.angle + frame_visual.yaw * 0.0349f;
					pitch_set = gimbal_pitch_motor->state.angle + frame_visual.pitch * 0.0349f;
					id = 0;
			}
			else{
					yaw_set += yaw_cruise_speed*yaw_direction;
			}
		}
		else if(robotMode == DYNAMIC_ATTACK_MODE || robotMode == ESCAPE_ATTACK_MODE || robotMode == CURVE_ATTACK_MODE){
			id = enemy_area;
			if(if_if_newframe){
					yaw_set = gimbal_yaw_motor->state.angle + frame_visual.yaw * 0.0349f + chasis_speed * -0.0066666f;//task_lss
					pitch_set = gimbal_pitch_motor->state.angle + frame_visual.pitch * 0.0349f;
					id = 0;
			}
			else{
					yaw_set += yaw_cruise_speed*yaw_direction;
			}
		}

		spGIMBAL_Controller.user.update_enemy_location(id);//task_lss

		if(yaw_set >= yaw_limit_max)
			yaw_direction = -1;
		else if (yaw_set <= yaw_limit_min)
			yaw_direction = 1;
		
		if(pitch_set >= pitch_limit_max)
			pitch_direction = -1;
		else if (pitch_set <= pitch_limit_min)
			pitch_direction = 1;
		
		pitch_set = (pitch_set > pitch_limit_max ? pitch_limit_max : 
								 pitch_set < pitch_limit_min ? pitch_limit_min : pitch_set);
		yaw_set   = (yaw_set > yaw_limit_max ? yaw_limit_max : 
								 yaw_set < yaw_limit_min ? yaw_limit_min : yaw_set);
		
		robotMode_ex = robotMode;
		spGIMBAL_Controller.user.update_target_limit(pitch_set,yaw_set);

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

void GIMBAL_Update_Limit(float target_pitch, float target_yaw)  {
		GimbalController.pitch_set = (target_pitch > pitch_limit_max ? pitch_limit_max : 
																	target_pitch < pitch_limit_min ? pitch_limit_min : target_pitch);
    GimbalController.yaw_set   = (target_yaw > yaw_limit_max ? yaw_limit_max : 
																	target_yaw < yaw_limit_min ? yaw_limit_min : target_yaw);
}

void GIMBAL_Update_enemy_Location(int id){
		switch(id){
			case 0:yaw_limit_min = -4000.f/8192.f*2.f*PI;yaw_limit_max = 6000.f/8192.f*2.f*PI;break;
			case 1:yaw_limit_min = yaw_limit[1];yaw_limit_max = yaw_limit[2];break;
			case 2:yaw_limit_min = yaw_limit[0];yaw_limit_max = yaw_limit[1];break;
			case 3:yaw_limit_min = yaw_limit[3];yaw_limit_max = yaw_limit[0];break;
			case 4:yaw_limit_min = yaw_limit[4];yaw_limit_max = yaw_limit[3];break;
			case 5:yaw_limit_min = yaw_limit[5];yaw_limit_max = yaw_limit[4];break;
			case 6:yaw_limit_min = yaw_limit[6];yaw_limit_max = yaw_limit[5];break;
		}
}

bool GIMBAL_MiddleLooper(uint32_t tick) {
    static uint16_t pass_flag = 0;
    if(pass_flag >= 800) {
        gimbal_pitch_motor->state.angle = 0;
        GimbalController.pitch_set = 0;
        spGIMBAL_Controller._target.PID.pitch.Kp = pitch_kp_0;
        spGIMBAL_Controller._target.PID.pitch.Ki = pitch_ki_0;
        
        gimbal_yaw_motor->state.angle = 0;
				GimbalController.yaw_set = 0;
        spGIMBAL_Controller._target.PID.yaw.Kp = yaw_kp_0;
        spGIMBAL_Controller._target.PID.yaw.Ki = yaw_ki_0;
				gimbal_inited = true;
        return true;
    } else {
        //if(abs(gimbal_pitch_motor->state.__motor_angel_curr - MIDDLE_PITCH)<5.f/0.0439f &&
        if(abs(gimbal_yaw_motor->state.__motor_angel_curr - MIDDLE_YAW)<1.f/0.0439f ) {
            pass_flag ++;
        } else {
            pass_flag = 0;
        }
        
        if(tick%10==1) {
            if(gimbal_yaw_motor->state.__motor_angel_last!=-1) {
                GIMBAL_UpdateYaw(spMATH_DEG2RAD(MOTOR_ENCODER_CONVERT*(
									MIDDLE_YAW - gimbal_yaw_motor->state.__motor_angel_first)));
            }
            
        }
    }
		gimbal_inited = false;
    return false;
}

void GIMBAL_PID_Init(void)
{
	PID_SetGains(&spGIMBAL_Controller._target.PID.yaw, yaw_kp_0, yaw_ki_0, yaw_kd_0); 
	PID_SetGains(&spGIMBAL_Controller._target.PID.pitch, pitch_kp_0, pitch_ki_0, pitch_kd_0);
}

void GIMBAL_VISUAL_PID_Init(void)
{
	PID_SetGains(&spGIMBAL_Controller._target.PID.yaw, visual_yaw_kp, visual_yaw_ki, visual_yaw_kd);
	PID_SetGains(&spGIMBAL_Controller._target.PID.pitch, visual_pitch_kp, visual_pitch_ki, visual_pitch_kd);
}

void GIMBAL_CRUISE_PID_Init(void)
{
	PID_SetGains(&spGIMBAL_Controller._target.PID.yaw, cruise_yaw_kp, cruise_yaw_ki, cruise_yaw_kd);
	PID_SetGains(&spGIMBAL_Controller._target.PID.pitch, cruise_pitch_kp, cruise_pitch_ki, cruise_pitch_kd);
}

#endif



struct __GIMBAL_Controller_Type spGIMBAL_Controller = {
    ._system = {
        .init = GIMBAL_ControlInit,
        .looper = GIMBAL_ControlLooper,
        .regression = GIMBAL_MiddleLooper,
				.statelooper = GIMBAL_State
    },
    .user = {
        .update_target_pitch = GIMBAL_UpdatePitch,
        .update_target_yaw = GIMBAL_UpdateYaw,
        .update_target = GIMBAL_Update,
				.update_target_limit = GIMBAL_Update_Limit,
				.update_enemy_location = GIMBAL_Update_enemy_Location,
				.pid_init = GIMBAL_PID_Init,
				.visual_pid_init = GIMBAL_VISUAL_PID_Init,
				.cruise_pid_init = GIMBAL_CRUISE_PID_Init,
    }
};

/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/
