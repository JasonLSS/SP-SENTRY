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
#include "RefereeInfo.h"
#include "sp_rng.h"
#include "Auto_aim.h"
#include "infrared.h"

#ifdef USING_SPEED_BALANCE
				PID_Type speedbalance;
				int16_t speed1;
				int16_t speed2;
#endif

static RobotMode robotMode_ex;
RC_DataType recv_ex;

float speed = 0;
float chasis_speed = 0;
float cruise_speed = 20.0f;
float chasis_speed_limit = 30.0f;
float speedA,speedB;
uint16_t L_distance = 0, R_distance = 0;
uint16_t distance_Threshold = 150;
float chasis_direction = 1;
float Distance_Limit = 450.f;

void CHASIS_Move(float speed) {
    // TODO: Make interval time more specific.
		#ifdef CHASIS_POWER_LIMIT
				CMWatt_Cal();
		#endif
		#ifdef USING_SPEED_BALANCE
				MOTOR_CrtlType_CAN* motor201 = spMOTOR.user.get(CAN1, Motor201);
				MOTOR_CrtlType_CAN* motor202 = spMOTOR.user.get(CAN1, Motor202);
				float SpeedDifference = motor201->state.speed - motor202->state.speed;
				float speedchange = PID_ControllerDriver(&speedbalance,0,SpeedDifference);
				speedA = CHASIS_Legalize((speed + speedchange),chasis_speed_limit);
				speedB = CHASIS_Legalize(speed,chasis_speed_limit);
				spMOTOR.user.set_speed(CAN1, Motor201, speedA);
				spMOTOR.user.set_speed(CAN1, Motor202, speedB);
		#else
				speedA = CHASIS_Legalize(speed,chasis_speed_limit);
				speedB = CHASIS_Legalize(speed,chasis_speed_limit);
				spMOTOR.user.set_speed(CAN1, Motor201, speedA);
				spMOTOR.user.set_speed(CAN1, Motor202, speedB);
		#endif
}




void CHASIS_Init(void) {
		MOTOR_CrtlType_CAN* motor201 = spMOTOR.user.enable(CAN1, Motor201, RM_3508_P19, false);
		MOTOR_CrtlType_CAN* motor202 = spMOTOR.user.enable(CAN1, Motor202, RM_3508_P19, false);
		if(motor201) {
				motor201->control.speed_pid->Kp = 1000.0f;
				motor201->control.speed_pid->Ki = 0.0f;
				motor201->control.speed_pid->Kd = 10.0f;
				motor201->control.speed_pid->intergration_limit = 5*PI;
				motor201->control.speed_pid->intergration_separation = PI;
				motor201->control.output_limit = 10000;
		}
		if(motor202) {
				motor202->control.speed_pid->Kp = 1000.0f;
				motor202->control.speed_pid->Ki = 0.0f;
				motor202->control.speed_pid->Kd = 10.0f;
				motor202->control.speed_pid->intergration_limit = 5*PI;
				motor202->control.speed_pid->intergration_separation = PI;
				motor202->control.output_limit = 10000;
		}
		
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
				
			//speed = Speed_D_Limit(fabs(speed));                           //new program
				
				chasis_speed = speed * chasis_direction;
        recv_ex = *recv;
				robotMode_ex = robotMode;
				
				CHASIS_Move(chasis_speed);
    }
}


/***************************************************************************************
 *Name     : CM_YawAuto
 *Function : Parallel_yaw轴角度控制
 *Input    : NowPosition, TargetPosition
 *Output   : PID out
 *Description : 陀螺仪做位置环，ToF发来的数据做误差控制目标值
****************************************************************************************/
void CM_ParallelYawAuto(float NowPosition, float TargetPosition)
{
    spCHASIS._system.params.target.spdyaw = PID_ControllerDriver(
        &spCHASIS._system.params.ParallelPID.yaw, TargetPosition, NowPosition); //位置环输出，速度
}

/***************************************************************************************
 *Name     : CM_YawAuto
 *Function : Parallel_yaw轴角度控制
 *Input    : NowPosition, TargetPosition
 *Output   : PID out
 *Description : 陀螺仪做位置环，ToF发来的数据做误差控制目标值
****************************************************************************************/
void CM_ParallelXAuto(float NowPosition, float TargetPosition)
{
    spCHASIS._system.params.target.spdx = PID_ControllerDriver(
        &spCHASIS._system.params.ParallelPID.x, TargetPosition, NowPosition);   //位置环输出，速度
}



/*
Unit:
    linear speed -> m/s
    angular speed -> rad/s
    length -> m
*/
void CHASIS_Mecanum(float spx, float spy, float spyaw, float out_speed[4]) {
    
    out_speed[0] = (
        ( spx - spy - (spCHASIS._system.params.half_width+spCHASIS._system.params.half_length)*spyaw)/spCHASIS._system.params.wheel_radius);
    out_speed[1] = (
        ( spx + spy + (spCHASIS._system.params.half_width+spCHASIS._system.params.half_length)*spyaw)/spCHASIS._system.params.wheel_radius);
    out_speed[2] = (
        ( spx - spy + (spCHASIS._system.params.half_width+spCHASIS._system.params.half_length)*spyaw)/spCHASIS._system.params.wheel_radius);
    out_speed[3] = (
        ( spx + spy - (spCHASIS._system.params.half_width+spCHASIS._system.params.half_length)*spyaw)/spCHASIS._system.params.wheel_radius);

    out_speed[1] = -out_speed[1];
    out_speed[2] = -out_speed[2];
}
void CHASIS_Mecanum_Inv(float speed[4], float* spdx, float *spdy, float *spyaw ) {
    
    speed[1] = -speed[1];
    speed[2] = -speed[2];
    
    if(spdx) *spdx = (
        (speed[0] + speed[1] + speed[2] + speed[3])*spCHASIS._system.params.wheel_radius/4.f );
    if(spdy) *spdy = (
        (-speed[0] + speed[1] - speed[2] + speed[3])*spCHASIS._system.params.wheel_radius/4.f );
    if(spyaw) *spyaw = (
        (-speed[0] + speed[1] + speed[2] - speed[3])*spCHASIS._system.params.wheel_radius/
        (spCHASIS._system.params.half_width+spCHASIS._system.params.half_length)/4.f );
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

/*-------------  ????程序  -------------*/
int IfUsingPowerBuffer(void){
	float PowerBufferLimit = 200;
	float PowerBufferMin = 50;
	if(ext_power_heat_data.chassis_power_buffer < PowerBufferLimit && ext_power_heat_data.chassis_power_buffer > PowerBufferMin){
		return 1;
	}
	else return 0;
}

/*-------------  ????程序  -------------*/
int Enemy_Location(void){
	int Left = 25, Right = -25;
  if(enemy_area == 1 || enemy_area == 2 || enemy_area == 6){
		return Left;
	}
	else if(enemy_area == 0)
		return 0;
	else return Right;
}

int Empty_Location(void){
	int Left = -50, Right = 50;
  if(enemy_empty_area == 1 || enemy_empty_area == 2 || enemy_empty_area == 6){
		return Left;
	}
	else if(enemy_area == 0)
		return 0;
	else return Right;
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
        .mecanum = CHASIS_Mecanum,
        .mecanum_inv = CHASIS_Mecanum_Inv,
        .move = CHASIS_Move,
    }
};

/*-------------  ????程序  -------------*/
float Speed_D_Limit(float speed){                        //new program
	if(chasis_direction == 1 && R_distance < Distance_Limit && R_distance > distance_Threshold){ // (150~450)
			if(speed > (R_distance - 50.f) * 0.1f){		
				speed = (R_distance - 50.f) * 0.1f;
			}
			else ;
		}
	else if(chasis_direction == -1 && L_distance < Distance_Limit && L_distance > distance_Threshold){ // (150~450)
			if(speed > (L_distance - 50.f) * 0.1f){		
				speed = (L_distance - 50.f) * 0.1f;   //(10~40)
			}
			else ;
		}
	return speed;
}
/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/
