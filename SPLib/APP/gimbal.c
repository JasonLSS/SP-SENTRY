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
bool gimbal_inited = false;
char usart3_buff[256];

static const float yaw_kp_0 = 17000.0f;
static const float yaw_ki_0 = 1000.0f;
static const float yaw_kd_0 = 800.f;

static const float pitch_kp_0 = 6000.0f;
static const float pitch_ki_0 = 2000.f;
static const float pitch_kd_0 = 1000.f;

static const float cruise_yaw_kp = 7000.0f;
static const float cruise_yaw_ki = 500.0f;
static const float cruise_yaw_kd = 100.f;

static const float cruise_pitch_kp = 8000.0f;
static const float cruise_pitch_ki = 2000.0f;
static const float cruise_pitch_kd = 1000.0f;

static float yaw_limit_max = 4.3f;
static float yaw_limit_min = -2.6f;
static float pitch_limit_max = 2000.f/8192.f*2.f*PI;
static float pitch_limit_min = 100.f/8192.f*2.f*PI;

float visual_syaw_kp = 8.0f;
float visual_syaw_ki = 0.2f;
float visual_syaw_kd = 0.1f;

float visual_lyaw_kp = 10.0f;
float visual_lyaw_ki = 0.0f;
float visual_lyaw_kd = 1.f;

float visual_spitch_kp = 11.0f;
float visual_spitch_ki = 2.f;
float visual_spitch_kd = 0.2f;

float visual_lpitch_kp = 8.0f;
float visual_lpitch_ki = 0.0f;
float visual_lpitch_kd = 0.0f;

float yaw_seperate_limit = 15.f;
float pitch_seperate_limit = 8.f;



static float yaw_cruise_speed = 7.5f/8192.f*2.f*PI;//task_lss
static float pitch_cruise_speed = 6.f/8192.f*2.f*PI;

float yaw_limit[7]= { -1365.3f*2/8192.f*2.f*PI , -1365.3f/8192.f*2.f*PI, 0.f/8192.f*2.f*PI , 1365.3f/8192.f*2.f*PI , 
											1365.3f*2/8192.f*2.f*PI , 1365.3f*3/8192.f*2.f*PI ,1365.3f*4/8192.f*2.f*PI };//!!!!!!


#define MIDDLE_YAW          1400
#define MIDDLE_PITCH        4000 

void GIMBAL_ControlInit(void) {
    /* Init new motor */
		gimbal_yaw_motor  = spMOTOR.user.enable(CAN1, Motor208, RM_6623_YAW, true);  
//		spMOTOR.motor.set_speed_pid(gimbal_yaw_motor, NULL);
    
    gimbal_yaw_motor->control.position_pid->intergration_separation = 0.5f;
    gimbal_yaw_motor->control.position_pid->intergrations_sum_error_limit = 5*PI;
    gimbal_yaw_motor->control.position_pid->output_limit = 5000;
    PID_SetGains(gimbal_yaw_motor->control.position_pid, yaw_kp_0*2.f, yaw_ki_0*2.f, yaw_kd_0);
	
		PID_SetGains(gimbal_yaw_motor->control.speed_pid, 1, 0, 0);
    
    gimbal_yaw_motor->control.output_limit = 5000;

    gimbal_pitch_motor = spMOTOR.user.enable(CAN1, Motor205, RM_2006_P36, true);
//		spMOTOR.motor.set_speed_pid(gimbal_pitch_motor, NULL);
    gimbal_pitch_motor->control.position_pid->intergration_separation = 0.2f;
    gimbal_pitch_motor->control.position_pid->intergrations_sum_error_limit = 3.f;
    gimbal_pitch_motor->control.position_pid->output_limit = 8000;
    PID_SetGains(gimbal_pitch_motor->control.position_pid, pitch_kp_0, pitch_ki_0*2.f, pitch_kd_0);
	
    PID_SetGains(gimbal_pitch_motor->control.speed_pid, 1, 0, 0);
	
    gimbal_pitch_motor->control.output_limit = 8000;
    
    spGIMBAL_Controller._target.gimbal_yaw_motor = gimbal_yaw_motor;
    spGIMBAL_Controller._target.gimbal_pitch_motor = gimbal_pitch_motor;

}


void GIMBAL_ControlLooper(void) {
		spMOTOR.user.set_position(CAN1, Motor208, GimbalController.yaw_set);
		spMOTOR.user.set_position(CAN1, Motor205, GimbalController.pitch_set);
	
		u8 size = sprintf(usart3_buff, "%f,%f,%f,%f\r\n",gimbal_yaw_motor->state.angle, frame_visual.yaw,
													gimbal_yaw_motor->control.target,gimbal_pitch_motor->control.target);
		spDMA.controller.start(spDMA_USART3_tx_stream, (uint32_t)usart3_buff, (uint32_t)&USART3->DR, size);
}

static float times_S = 0;
static float times_D = 0;

void GIMBAL_State(void){
		static float yaw_set = 0;
		static float pitch_set = 0;
		static int yaw_direction = 1;
		static int pitch_direction = 1;
		int id = 6;
		if(robotMode == REMOTE_MODE && robotMode^robotMode_ex){
			spGIMBAL_Controller.user.update_enemy_location(6);
			spGIMBAL_Controller.user.pid_init();
		}
		if(robotMode == CRUISE_MODE && robotMode^robotMode_ex){
			yaw_direction = 1;
			pitch_direction = 1;
			spGIMBAL_Controller.user.update_enemy_location(6);
			spGIMBAL_Controller.user.cruise_pid_init(); 
		}
		if(robotMode == ESCAPE_MODE && robotMode^robotMode_ex){
			spGIMBAL_Controller.user.update_enemy_location(6);
			spGIMBAL_Controller.user.cruise_pid_init(); 
		}
		if(robotMode == STATIC_ATTACK_MODE && robotMode^robotMode_ex){
			spGIMBAL_Controller.user.cruise_pid_init(); 
			spGIMBAL_Controller.user.visual_sp_pid();
			times_D= 100;
		}
		if(robotMode == DYNAMIC_ATTACK_MODE && robotMode^robotMode_ex){
			spGIMBAL_Controller.user.cruise_pid_init(); 
			times_D = 100;
		}
		if(robotMode == ESCAPE_ATTACK_MODE && robotMode^robotMode_ex){
			spGIMBAL_Controller.user.cruise_pid_init(); 
			times_D = 100;
		}
		
		if(robotMode == STANDBY_MODE && gimbal_inited){
			gimbal_yaw_motor->flags.stop = true;
			gimbal_pitch_motor->flags.stop = true;
		}else{
			gimbal_yaw_motor->flags.stop = false;
			gimbal_pitch_motor->flags.stop = false;
		}
		
		id = enemy_area;
		if(robotMode == REMOTE_MODE){
			yaw_set +=  (abs(recv.rc.ch0)<20?0:recv.rc.ch0)/30.f/1320.0f;
			pitch_set += (abs(recv.rc.ch1)<20?0:recv.rc.ch1)/30.f/1320.0f;
		}
		else if(robotMode == CRUISE_MODE){
			yaw_set += yaw_cruise_speed*yaw_direction;
			pitch_set = 0.7f;
		}
		else if(robotMode == ESCAPE_MODE){
			
		}
//		else if(robotMode == STATIC_ATTACK_MODE){
//			if(if_if_newframe){
//					spGIMBAL_Controller.user.visual_pid_init(); 
//					yaw_set = gimbal_yaw_motor->state.angle + frame_visual.yaw * 0.0349f;
//					pitch_set = gimbal_pitch_motor->state.angle + frame_visual.pitch * 0.0349f;
//					id = 6;
//					if_if_newframe = 0;
//					times_S = 0;
//			}
//			else{
//					times_S++;
//					if(times_S>100){
//						spGIMBAL_Controller.user.cruise_pid_init(); 
//						spGIMBAL_Controller.user.update_enemy_location(id);
//						yaw_set += yaw_cruise_speed*yaw_direction;
//						times_S = 100;
//					}
//			}
//		}
		else if(robotMode == DYNAMIC_ATTACK_MODE || robotMode == ESCAPE_ATTACK_MODE || robotMode == CURVE_ATTACK_MODE || robotMode == STATIC_ATTACK_MODE){
			if(if_if_newframe){
					yaw_set = gimbal_yaw_motor->state.angle + frame_visual.yaw * 0.0349f;
					pitch_set = gimbal_pitch_motor->state.angle + frame_visual.pitch * 0.0349f;
					//yaw seperate
					static float large_time = 0.f;
					static float large_seperate = 6.f;
					if(fabs(frame_visual.yaw) > yaw_seperate_limit){
						spGIMBAL_Controller.user.visual_ly_pid();
						large_time ++;
						if(large_time > large_seperate){
							yaw_set = gimbal_yaw_motor->state.angle +  frame_visual.yaw * 0.0349f/large_time*large_seperate;
						}
						if(large_time > 2 * large_seperate){
							large_time = 2 * large_seperate;
						}
					}else{
						spGIMBAL_Controller.user.visual_sy_pid();
						large_time = 0;
					}
					
					
					
					if(fabs(frame_visual.pitch) > pitch_seperate_limit){
						spGIMBAL_Controller.user.visual_lp_pid();
					}else{
						spGIMBAL_Controller.user.visual_sp_pid();
					}
					
					id = 6;
					spGIMBAL_Controller.user.update_enemy_location(id);
					if_if_newframe = 0;
					times_D = 0;
			}
			else{
					times_D++;
					if(times_D>300){
						spGIMBAL_Controller.user.update_enemy_location(id);
						yaw_set += yaw_cruise_speed*yaw_direction;
						pitch_set = 0.5f;
						times_D = 300;
					}
			}
		}

		

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
			case 6:yaw_limit_min = -2.6f;yaw_limit_max = 4.3f;break;
			case 0:yaw_limit_min = yaw_limit[0];yaw_limit_max = yaw_limit[1];break;
			case 1:yaw_limit_min = yaw_limit[1];yaw_limit_max = yaw_limit[2];break;
			case 2:yaw_limit_min = yaw_limit[2];yaw_limit_max = yaw_limit[3];break;
			case 3:yaw_limit_min = yaw_limit[3];yaw_limit_max = yaw_limit[4];break;
			case 4:yaw_limit_min = yaw_limit[4];yaw_limit_max = yaw_limit[5];break;
			case 5:yaw_limit_min = yaw_limit[5];yaw_limit_max = yaw_limit[6];break;
		}
}

bool GIMBAL_MiddleLooper(uint32_t tick) {
    static uint16_t pass_flag = 0;
    if(pass_flag >= 800) {
        gimbal_pitch_motor->state.angle = 0;
        gimbal_pitch_motor->control.target = 0;
        gimbal_pitch_motor->control.position_pid->Kp = pitch_kp_0;
        gimbal_pitch_motor->control.position_pid->Ki = pitch_ki_0;
        
        gimbal_yaw_motor->state.angle = 0;
        gimbal_yaw_motor->control.target = 0;
        gimbal_yaw_motor->control.position_pid->Kp = yaw_kp_0;
        gimbal_yaw_motor->control.position_pid->Ki = yaw_ki_0;
				gimbal_inited = true;
        return true;
    } else {
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
	PID_SetGains(gimbal_yaw_motor->control.position_pid, yaw_kp_0, yaw_ki_0, yaw_kd_0); 
	PID_SetGains(gimbal_pitch_motor->control.position_pid, pitch_kp_0, pitch_ki_0, pitch_kd_0);
	PID_SetGains(gimbal_yaw_motor->control.speed_pid, 1, 0, 0);
	PID_SetGains(gimbal_pitch_motor->control.speed_pid, 1, 0, 0);
}

void GIMBAL_VISUAL_SY_PID_Init(void)
{
	PID_SetGains(gimbal_yaw_motor->control.position_pid, visual_syaw_kp, visual_syaw_ki, visual_syaw_kd);
	PID_SetGains(gimbal_yaw_motor->control.speed_pid, 2000, 0, 0);
}


void GIMBAL_VISUAL_SP_PID_Init(void)
{
	PID_SetGains(gimbal_pitch_motor->control.position_pid, visual_spitch_kp, visual_spitch_ki, visual_spitch_kd);
	PID_SetGains(gimbal_pitch_motor->control.speed_pid, 2000, 0, 0);
	gimbal_pitch_motor->control.position_pid->sum_error = 0;
}

void GIMBAL_VISUAL_LY_PID_Init(void)
{
	PID_SetGains(gimbal_yaw_motor->control.position_pid, visual_lyaw_kp, visual_lyaw_ki, visual_lyaw_kd);
	PID_SetGains(gimbal_yaw_motor->control.speed_pid, 2000, 0, 0);
}

void GIMBAL_VISUAL_LP_PID_Init(void)
{
	PID_SetGains(gimbal_pitch_motor->control.position_pid, visual_lpitch_kp, visual_lpitch_ki, visual_lpitch_kd);
	PID_SetGains(gimbal_pitch_motor->control.speed_pid, 2000, 0, 0);
	gimbal_pitch_motor->control.position_pid->sum_error = 0;
}

void GIMBAL_CRUISE_PID_Init(void)
{
	PID_SetGains(gimbal_yaw_motor->control.position_pid, cruise_yaw_kp, cruise_yaw_ki, cruise_yaw_kd);
	PID_SetGains(gimbal_pitch_motor->control.position_pid, cruise_pitch_kp, cruise_pitch_ki, cruise_pitch_kd);
	PID_SetGains(gimbal_yaw_motor->control.speed_pid, 1, 0, 0);
	PID_SetGains(gimbal_pitch_motor->control.speed_pid, 1, 0, 0);
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
				.visual_sy_pid = GIMBAL_VISUAL_SY_PID_Init,
				.visual_sp_pid = GIMBAL_VISUAL_SP_PID_Init,
				.visual_ly_pid = GIMBAL_VISUAL_LY_PID_Init,
				.visual_lp_pid = GIMBAL_VISUAL_LP_PID_Init,
				.cruise_pid_init = GIMBAL_CRUISE_PID_Init,
    }
};

/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/
