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

#include "sp_chasis.h"
#include "sp_sensor.h"
#include "sp_conf.h"
#include "sp_shoot.h"
#include "referee.h"
#include "sp_rng.h"
#include "Auto_aim.h"
#include "infrared.h"

#define __CHASIS_OuputLimit                 7000 

#ifdef USING_SPEED_BALANCE
				PID_Type speedbalance;
				int16_t speed1;
				int16_t speed2;
#endif
static float speed_last = 0 ;
static RobotMode robotMode_ex;
RC_DataType recv_ex;
float target_motor201 = 0;
float target_motor202 = 0;
float speed = 0;
float chasis_speed = 0;
float cruise_speed = 20.0f;
float chasis_speed_limit = 30.0f;
float speedA,speedB;
uint16_t L_distance = 0, R_distance = 0;
uint16_t distance_Threshold = 150;
float chasis_direction = 1;
float Distance_Limit = 450.f;
float SPEED_CHANGE_LIMIT =200.f;

void CHASIS_Move(float speed) {
    // TODO: Make interval time more specific.
		#ifdef CHASIS_POWER_LIMIT
				CMWatt_Cal();
		#endif
	
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
		
		motor201->control.output = target_motor201;
		motor202->control.output = target_motor202;
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
        L_distance = TOF10120_1distance();
    } else if(tick%10 == 3) {
        R_distance = TOF10120_2distance();
    } else if(tick%10 == 7) {
				if(robotMode == REMOTE_MODE && robotMode^robotMode_ex){
						speed = 0;

				}
				if(robotMode == CRUISE_MODE && robotMode^robotMode_ex){
						speed = cruise_speed;
				}

				if(robotMode == REMOTE_MODE){
						speed = (abs(recv->rc.ch2)<20?0:recv->rc.ch2)/20.f;
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
						if(timeticket < 100){
							timeticket++;
						}
						else{
							timeticket = 0 ;
							EnemyCoefficient = RNG_Get_RandomRange(Enemy_Location() - 50.f,Enemy_Location() + 50.f)/50.f;//task_lss
							if(EnemyCoefficient < 0 && EnemyCoefficient > -0.8f)
								EnemyCoefficient = EnemyCoefficient - 0.8f;
							else if(EnemyCoefficient > 0 && EnemyCoefficient < 0.8f)
								EnemyCoefficient = EnemyCoefficient + 0.8f;
							speed = cruise_speed * fabs(EnemyCoefficient);
							chasis_direction = sign(EnemyCoefficient);
						}
				}
				else if(robotMode == ESCAPE_ATTACK_MODE || robotMode == ESCAPE_MODE){//task_lss
						float EscapeCoefficient = 2.0f;    //(1.5/2.0/2.5)
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
						}
						if(IfUsingPowerBuffer()){
							EscapeCoefficient += 0.5f;
						}
						else
							EscapeCoefficient = 1.5f; 
						speed = fabs(EscapeCoefficient * cruise_speed * EnemyCoefficient);   // (1.2~5)cruise_speed = (14.4 ~ 60)
						chasis_direction = sign(EnemyCoefficient);
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

/*-------------  功率监视程序  -------------*/
void CMWatt_Cal(void)//task_lss
{
		if(ext_power_heat_data.chassis_power_buffer<30&&ext_power_heat_data.chassis_power_buffer > 0)
			chasis_speed_limit=chasis_speed_limit-0.001f*(1.3f-ext_power_heat_data.chassis_power_buffer/60.0f)*chasis_speed_limit;
		else 
			chasis_speed_limit=30.0f;
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
