/**
  ******************************************************************************
  * @file       sp_chasis.c
  * @author     LSS
  * @version    v0.0-alpha
  * @date       2019.May.12
  * @brief      chasis control
  * @usage      
  ******************************************************************************
  * @license
  *
  ******************************************************************************
  */


#include "sp_chasis.h"
#include "sp_sensor.h"
#include "sp_conf.h"
#include "sp_shoot.h"
#include "referee.h"
#include "sp_rng.h"
#include "Auto_aim.h"
#include "infrared.h"

#define __CHASIS_OuputLimit                 7000 


static float speed_last = 0 ;
static RobotMode robotMode_ex;
RC_DataType recv_ex;
float target_motor201 = 0;
float target_motor202 = 0;
float speed = 0;
float chasis_speed = 0;
float cruise_speed = 19.0f;
float chasis_speed_limit = 80.0f;
float	chasis_out_limit=800.0f;
float speedA,speedB;
uint16_t L_distance = 0, R_distance = 0;
uint16_t distance_Threshold = 250;
float chasis_direction = 1;
float Distance_Limit = 450.f;
float SPEED_CHANGE_LIMIT =0.5f;
bool L_flag = true;
bool R_flag = true;
bool change_flag = false;
PID_Type speedbalance;

float test_output = 0;

void CHASIS_Move(float speed) {
    // TODO: Make interval time more specific.
	
		MOTOR_CrtlType_CAN* motor201 = spMOTOR.user.get(CAN1, Motor201);
		MOTOR_CrtlType_CAN* motor202 = spMOTOR.user.get(CAN1, Motor202);
		spCHASIS._system.params.state.x = motor201->state.speed;
		spCHASIS._system.params.state.y = motor202->state.speed;
	
		#ifdef USING_SPEED_BALANCE
				if(motor201 && motor202) {
					float SpeedDifference = motor201->state.speed - motor202->state.speed;
					float speedchange = PID_ControllerDriver(&speedbalance,0,SpeedDifference);
					speedA = CHASIS_Legalize((speed + speedchange),chasis_speed_limit);
					speedB = CHASIS_Legalize(speed,chasis_speed_limit);			
				} else {
						speedA = CHASIS_Legalize(speed,chasis_speed_limit);
						speedB = CHASIS_Legalize(speed,chasis_speed_limit);
				}
					
		#else
				speedA = CHASIS_Legalize(speed,chasis_speed_limit);
				speedB = CHASIS_Legalize(speed,chasis_speed_limit);
		#endif
		target_motor201 = PID_ControllerDriver(&spCHASIS._system.params.PID.x, 
		speedA, spCHASIS._system.params.state.x);
		target_motor202 = PID_ControllerDriver(&spCHASIS._system.params.PID.y, 
		speedB, spCHASIS._system.params.state.y);
		
		#ifdef CHASIS_POWER_LIMIT
			CMWatt_Cal();
			static float target_all = 0;
			target_all = fabs(target_motor201) + fabs(target_motor202);
			
			if(target_all < chasis_out_limit){
				motor201->control.output = target_motor201;
				motor202->control.output = target_motor202;
			}
			else{
				motor201->control.output = target_motor201/target_all*chasis_out_limit;
				motor202->control.output = target_motor202/target_all*chasis_out_limit;
			}
			if(chasis_out_limit == 0){
				motor201->control.output = 0;
				motor202->control.output = 0;
			}
		#else
			motor201->control.output = target_motor201;
			motor202->control.output = target_motor202;
		#endif
		
		static int stop_time = 0;
		static bool last_change = false;
		if(!change_flag && last_change)
			stop_time = 0;
		if((change_flag == false) && ((fabs((target_motor201)/5000.f-(motor201->state.speed/20.f)) > 0.2f)||
												(fabs((target_motor202)/5000.f-(motor202->state.speed/20.f)) > 0.2f))){
			stop_time++;
			if(stop_time > 100){
				change_flag = true;
				stop_time = 100;
			}
		}
		else{
			stop_time = 0;
			change_flag = false;
		}
		last_change = change_flag;
		if(test_output != 0){
			motor201->control.output = test_output;
			motor202->control.output = test_output;
		}
//		static char usart3_buff[128];
//		u8 size = sprintf(usart3_buff, "%f,%f,%f,%f,%f,%f\r\n",motor201->state.speed, speedA,
//													target_motor201,motor202->state.speed, speedB,
//													target_motor202);
//		spDMA.controller.start(spDMA_USART3_tx_stream, (uint32_t)usart3_buff, (uint32_t)&USART3->DR, size);
}




void CHASIS_Init(void) {
		MOTOR_CrtlType_CAN* motor201 = spMOTOR.user.enable_simple(CAN1, Motor201, RM_3508_P19);
		MOTOR_CrtlType_CAN* motor202 = spMOTOR.user.enable_simple(CAN1, Motor202, RM_3508_P19);
	
		assert_param(motor201 || motor202);
		
		PID_ControllerInit(&spCHASIS._system.params.PID.x, 5*PI, -1, __CHASIS_OuputLimit);
    spCHASIS._system.params.PID.x.intergration_separation = PI;
    spCHASIS._system.params.PID.x.output_limit = __CHASIS_OuputLimit;
    
    PID_ControllerInit(&spCHASIS._system.params.PID.y, 5*PI, -1, __CHASIS_OuputLimit);
    spCHASIS._system.params.PID.y.intergration_separation = PI;
    spCHASIS._system.params.PID.y.output_limit = __CHASIS_OuputLimit;
    
    PID_SetGains(&spCHASIS._system.params.PID.x, 500.0f, 0.f, 10.f);
    PID_SetGains(&spCHASIS._system.params.PID.y, 500.0f, 0.f, 10.f);
		
		#ifdef USING_SPEED_BALANCE
        PID_ControllerInit(&speedbalance, 5, 0xFFFF, 2);
        PID_SetGains(&speedbalance, 0.1f, 0.2f, 0.001f);     // For init
        speedbalance.intergration_separation = PI/8.f;
		#endif
		
}


void CM_ParallelYawAuto(float NowPosition, float TargetPosition);
void CM_ParallelXAuto(float NowPosition, float TargetPosition);

void CHASIS_Looper(uint32_t tick, const RC_DataType *recv) {
    if(tick%10 == 1) {
        L_distance = TOF10120_3distance();
				R_distance = TOF10120_2distance();
				static int L_down_time = 0;
				static int R_down_time = 0;
				static int DOWN_TIME = 30;
				if(L_distance == 0){
					L_down_time ++;
					if(L_down_time > DOWN_TIME){
						L_flag = false;
						L_down_time = DOWN_TIME;
					}
				}
				else{
						L_flag = true;
						L_down_time = 0;
					}
				if(R_distance == 0){
					R_down_time ++;
					if(R_down_time > DOWN_TIME){
						R_flag = false;
						R_down_time = DOWN_TIME;
					}
				}
				else{
					R_flag = true;
					R_down_time = 0;
				}
    } else if(tick%10 == 3) { 

    } else if(tick%10 == 7) {
				if(robotMode == REMOTE_MODE && robotMode^robotMode_ex){
						speed = 0;

				}
				if(robotMode == CRUISE_MODE && robotMode^robotMode_ex){
						speed = cruise_speed;
				}

				if(robotMode == REMOTE_MODE){
						speed = (abs(recv->rc.ch2)<20?0:recv->rc.ch2)/8.f;
				}
				else if(robotMode == STANDBY_MODE){
						speed = 0;
				}
				else if(robotMode == CRUISE_MODE){
						speed = cruise_speed;
				}
				else if(robotMode == STATIC_ATTACK_MODE){
						speed = 0;
				}
				else if(robotMode == DYNAMIC_ATTACK_MODE){
						static float timeticket = 0;
						static float EnemyCoefficient = 1.0f;
						if(timeticket < 500){
							timeticket++;
						}
						else{
							timeticket = 0 ;
							EnemyCoefficient = RNG_Get_RandomRange(Enemy_Location() - 50.f,Enemy_Location() + 50.f)/50.f;//task_lss
							if(EnemyCoefficient < 0 && EnemyCoefficient > -0.8f)
								EnemyCoefficient = EnemyCoefficient - 0.8f;
							else if(EnemyCoefficient > 0 && EnemyCoefficient < 0.8f)
								EnemyCoefficient = EnemyCoefficient + 0.8f;
							else
								EnemyCoefficient = 1;
							speed = cruise_speed * fabs(EnemyCoefficient);
							chasis_direction = sign(EnemyCoefficient);
						}
				}
				else if(robotMode == ESCAPE_ATTACK_MODE || robotMode == ESCAPE_MODE){//task_lss
						float EscapeCoefficient = 3.0f;    //(1.5/2.0/2.5)
						static float timeticket = 0;
						static float EnemyCoefficient = 1.0f;   //  (0.8 ~ 2)+(0.8 ~ 1.6)
						if(timeticket < 100){
							timeticket++;
						}
						else{
							timeticket = 0 ;
							EnemyCoefficient = RNG_Get_RandomRange(Empty_Location() - 50.f,Empty_Location() + 50.f)/50.f;//task_lss
							if(EnemyCoefficient < 0 && EnemyCoefficient > -0.8f)
								EnemyCoefficient = EnemyCoefficient - 0.8f;         
							else if(EnemyCoefficient > 0 && EnemyCoefficient < 0.8f)
								EnemyCoefficient = EnemyCoefficient + 0.8f;
							else
								EnemyCoefficient = 1;
						}
						if(IfUsingPowerBuffer()){
							EscapeCoefficient += 0.5f;
						}
						else
							EscapeCoefficient = 1.5f; 
						speed = fabs(EscapeCoefficient * cruise_speed * EnemyCoefficient);   // (1.2~5)cruise_speed = (14.4 ~ 60)
				}
				else if(robotMode == CURVE_ATTACK_MODE){//task_lss
						static float time = 0;
						speed = cruise_speed + RNG_Get_RandomRange(-5,+15);
						if(Infrared_Flag)
							time ++;
						if(time > 1)
							time ++;
						if(time > 100){
							chasis_direction = - chasis_direction;
							time = 0;
						}
				}
				else{
						speed = 0;
				}
				
				if((L_distance>0&&L_distance<distance_Threshold))
						chasis_direction = 1;
				if((R_distance>0&&R_distance<distance_Threshold))
						chasis_direction = -1;
				
#ifdef AIR_PROTECT
				MOTOR_CrtlType_CAN* motor201 = spMOTOR.user.get(CAN1, Motor201);
				if(motor201->state.angle > 0){
						chasis_direction = -1;
				}
#endif
				
				if(!R_flag || !L_flag){
					if(change_flag){
						chasis_direction = -chasis_direction;
						change_flag = false;
					}
				}
				
				chasis_speed = Speed_Change_Limit(speed * chasis_direction);                           //new program
        recv_ex = *recv;
				robotMode_ex = robotMode;
				
				CHASIS_Move(chasis_speed);
    }
}


float CHASIS_Legalize(float MotorCurrent , float limit)
{
	return MotorCurrent<-limit?-limit:(MotorCurrent>limit?limit:MotorCurrent);
}

void CMWatt_Cal(void)//task_lss
{
		if(robotMode == ESCAPE_ATTACK_MODE || robotMode == ESCAPE_MODE){
			chasis_out_limit=12000.0f;
			if(ext_power_heat_data.chassis_power_buffer<50&&ext_power_heat_data.chassis_power_buffer > 0){
				chasis_out_limit=12000.f-11000.f*(ext_power_heat_data.chassis_power/40.0f);
				if(chasis_out_limit < 0)
					chasis_out_limit = 0;
				chasis_out_limit=chasis_out_limit*(ext_power_heat_data.chassis_power_buffer)/50.0f;
			}
			if(ext_power_heat_data.chassis_power_buffer == 0)
					chasis_out_limit = 0;
		}
		else{
			chasis_out_limit=6000.0f;
			if(ext_power_heat_data.chassis_power_buffer<200&&ext_power_heat_data.chassis_power_buffer > 0){
				chasis_out_limit=10000.f-4000.f*(ext_power_heat_data.chassis_power/20.0f);
				chasis_out_limit=chasis_out_limit*(ext_power_heat_data.chassis_power_buffer-150)/50.0f;
			}
			if(ext_power_heat_data.chassis_power_buffer<50&&ext_power_heat_data.chassis_power_buffer > 0){
				chasis_out_limit=10000.f-4000.f*(ext_power_heat_data.chassis_power/20.0f);
				chasis_out_limit=chasis_out_limit*(ext_power_heat_data.chassis_power_buffer)/50.0f;
			}
			if(ext_power_heat_data.chassis_power_buffer == 0)
					chasis_out_limit = 0;
			if(chasis_out_limit < 0)
					chasis_out_limit = 0;
		}
}

int IfUsingPowerBuffer(void){
	float PowerBufferLimit = 200;
	float PowerBufferMin = 50;
	if(ext_power_heat_data.chassis_power_buffer < PowerBufferLimit && ext_power_heat_data.chassis_power_buffer > PowerBufferMin){
		return 1;
	}
	else return 0;
}

int Enemy_Location(void){
	int Left = 25, Right = -25;
  if(enemy_area == 1 || enemy_area == 3)
		return Left;
	else if(enemy_area == 6)
		return 0;
	else if(enemy_area == 2 || enemy_area == 5)
		return Right;
	else
		return 0;
}

int Empty_Location(void){
	int Left = -50, Right = 50;
  if(enemy_empty_area == 1 || enemy_empty_area == 3){
		return Left;
	}
	else if(enemy_empty_area == 6)
		return 0;
	else if(enemy_empty_area == 2 || enemy_empty_area == 5)
		return Right;
	else
		return 0;
}

float Speed_Change_Limit(float speed){                        
	if(fabs(speed_last - speed) > SPEED_CHANGE_LIMIT )
		speed = speed_last + sign(speed - speed_last)*SPEED_CHANGE_LIMIT;
		speed_last = speed;
	return speed;
}

struct __CHASIS_Manager_Type spCHASIS = {
    ._system = {
        .params = {
            .half_width = 0.660f,
            .half_length = 0.560f,
            .wheel_radius = 0.075f,
            .ParallelPID.distance = 300.f
        },
        .init = CHASIS_Init,
        .looper = CHASIS_Looper,
    },
    .user = {
        .move = CHASIS_Move,
    }
};


/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/
