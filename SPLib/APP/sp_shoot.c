/**
  ******************************************************************************
  * @file       sp_shoot.c
  * @author     LSS
  * @version    v0.0-alpha
  * @date       2019.Mar.12
  * @brief      shooting control
  * @usage      
  ******************************************************************************
  * @license
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "sp_shoot.h"
#include "Auto_aim.h"
#include "referee.h"

/** @addtogroup SP
  * @{
  */

/** @defgroup   Shoot
  * @{
  */



#define Friction_SPEED_p               0.06f
#define Friction_SPEED_i               0.005f
#define Friction_SPEED_d               0.01f
#define Friction_INTE_limitI           60.f
#define DELTA_TIME                     0.01f
#define USING_FRICTION_FILTER                    /*<! Using input filter */

float Feed_SPEED = 20.f;

static uint16_t max_shoot_speed = 60;
static int Shoot_Cooling_Time = 200;
static int Cooling_tickets = 10;

PWMFriction_Type    Friction_CH1;
PWMFriction_Type    Friction_CH2;
MOTOR_CrtlType_CAN* Feed_motor;
RobotMode robotMode=		STANDBY_MODE;
RobotMode robotMode_ex=	STANDBY_MODE;

extern RC_DataType recv;

/**
  * @brief  
  * @note   
  * @param  
  * @retval 
  */ 
void PulseCapture(void) {
    
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG , ENABLE);//
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 ;    //ó?óú?¤1a        //ó?óú?????¤1a
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//??í¨ê?3??￡ê?
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//í?íìê?3?
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//é?à-
    GPIO_Init(GPIOG, &GPIO_InitStructure);//3?ê??ˉ    
    LASER_ON();
    
    TIM_TimeBaseInitTypeDef             TIM_TimeBaseStructure;
    TIM_ICInitTypeDef                   TIM_ICInitStructure;
 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2|RCC_APB1Periph_TIM3,ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_DMA1, ENABLE);

    GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM2);
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_TIM2);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA,&GPIO_InitStructure);
    
    TIM_TimeBaseStructure.TIM_Period=50000-1;   // 20Hz
    TIM_TimeBaseStructure.TIM_Prescaler=84-1;
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);
    
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV4;
    TIM_ICInitStructure.TIM_ICFilter = 0x10;        // Attention
    TIM_ICInit(TIM2, &TIM_ICInitStructure);
    
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
    TIM_ICInit(TIM2, &TIM_ICInitStructure);
    
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_DMACmd(TIM2, TIM_DMA_CC2|TIM_DMA_CC4, ENABLE);
    TIM_SetCounter(TIM2, 0);

//    NVIC_IRQEnable(TIM2_IRQn, 0, 2);
    
    //TODO: TIM2_CH2/4 IRQ
    /* -------------- Configure DMA -----------------------------------------*/
    DMA_InitTypeDef dma;
    DMA_DeInit(DMA1_Stream5);
    dma.DMA_Channel = DMA_Channel_3;    // TIM2_CH1
    dma.DMA_PeripheralBaseAddr = (uint32_t)&(TIM2->CCR1);
    dma.DMA_Memory0BaseAddr = (uint32_t)Friction_CH1.counters;   
    dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
    dma.DMA_BufferSize = sizeof(Friction_CH1.counters)/sizeof(Friction_CH1.counters[0]);
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    dma.DMA_Mode = DMA_Mode_Circular;
    dma.DMA_Priority = DMA_Priority_High;
    dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
    dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    dma.DMA_MemoryBurst = DMA_Mode_Normal;
    dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//    DMA_Init(DMA1_Stream5, &dma);
//    DMA_ITConfig(DMA1_Stream5,DMA_IT_TC,ENABLE);
//    DMA_Cmd(DMA1_Stream5,ENABLE);
    
    DMA_DeInit(DMA1_Stream6);                     
    dma.DMA_Channel = DMA_Channel_3;    // TIM2_CH2
    dma.DMA_PeripheralBaseAddr = (uint32_t)&(TIM2->CCR2);
    dma.DMA_Memory0BaseAddr = (uint32_t)Friction_CH2.counters;   
    dma.DMA_BufferSize = sizeof(Friction_CH2.counters)/sizeof(Friction_CH2.counters[0]);
    DMA_Init(DMA1_Stream6, &dma);
    DMA_ITConfig(DMA1_Stream6,DMA_IT_TC,ENABLE);
    DMA_Cmd(DMA1_Stream6,ENABLE);
    
//    NVIC_IRQEnable(DMA1_Stream5_IRQn, 0, 1);
//    NVIC_IRQEnable(DMA1_Stream6_IRQn, 0, 1);
    
    TIM_Cmd(TIM2, ENABLE);
}

void PWM_Output(void){
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);      //TIM14时钟使能    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI,ENABLE);     //使能PORTF时钟    
    
    GPIO_PinAFConfig(GPIOI,GPIO_PinSource5,GPIO_AF_TIM8); //GPIOF9复用为定时器
    GPIO_PinAFConfig(GPIOI,GPIO_PinSource6,GPIO_AF_TIM8); //GPIOF9复用为定时器
    GPIO_PinAFConfig(GPIOI,GPIO_PinSource7,GPIO_AF_TIM8); //GPIOF9复用为定时器
    GPIO_PinAFConfig(GPIOI,GPIO_PinSource2,GPIO_AF_TIM8); //GPIOF9复用为定时器
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_2;    //GPIOI
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //速度100MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
    GPIO_Init(GPIOI,&GPIO_InitStructure);               //初始化PF9
      
    TIM_TimeBaseStructure.TIM_Prescaler=168*4-1;  //定时器分频
    TIM_TimeBaseStructure.TIM_Period=1000-1;   //自动重装载值
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
    TIM_TimeBaseInit(TIM8,&TIM_TimeBaseStructure);//初始化定时器14
    
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式2
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性低
    TIM_OCInitStructure.TIM_Pulse = 0;
    
    TIM_OC1Init(TIM8, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);

    TIM_OC2Init(TIM8, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
    
    TIM_OC3Init(TIM8, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);
    
    TIM_OC4Init(TIM8, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);
 
    TIM_ARRPreloadConfig(TIM8,ENABLE);//ARPE使能 
    TIM_CtrlPWMOutputs(TIM8,ENABLE);    //For TIM8 PWM
    TIM_Cmd(TIM8, ENABLE);  //使能TIM14
}


void TIM2_IRQHandler(void) {      //50ms产生一次中断
    if(TIM_GetITStatus(TIM2,TIM_IT_Update) == SET) {
        Friction_CH1.stopping ++;
        if(Friction_CH1.stopping >= 5) {
        #ifdef USING_FRICTION_FILTER
            MovingAverageFilter_f32((float*)Friction_CH1.speed, 
                sizeof(Friction_CH1.speed)/sizeof(Friction_CH1.speed[0]), 0, 15);
        #else
            Friction_CH1.speed[0] = 0;
        #endif
            Friction_CH1.stopping = 5;
        }
        
        
        Friction_CH2.stopping ++;
        if(Friction_CH2.stopping >= 5) {
        #ifdef USING_FRICTION_FILTER
            MovingAverageFilter_f32((float*)Friction_CH2.speed, 
                sizeof(Friction_CH2.speed)/sizeof(Friction_CH2.speed[0]), 0, 15);
        #else
            Friction_CH2.speed[0] = 0;
        #endif
            Friction_CH2.stopping = 5;
        }
        
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}

// 1.615*set+53.87  = 10^6/counter (div8)
float COMPENSATE  = -0.f;
float MAFilter_Threshold = 50.f;
void dmaFrictionUpdata(PWMFriction_Type* friction) {
    uint8_t overflow = 0;
    uint8_t size=sizeof(friction->counters)/sizeof(friction->counters[0])-1;
    
    friction->stopping = 0;
    for(uint8_t i=1; i<size; i++) {
        overflow += (friction->counters[i]<friction->counters[i-1]);
    }
    friction->counter = (friction->counters[size-1] +
        overflow * (TIM2->ARR+1) - friction->counters[0])/(size-1);
    /* Using limited moving average */
    #ifdef USING_FRICTION_FILTER
        MovingAverageFilter_f32((float*)friction->speed, 
                sizeof(friction->speed)/sizeof(friction->speed[0]), 
                (friction->counter>0)? (619195.05f/(2*friction->counter)-33.356f+COMPENSATE):0, 
                MAFilter_Threshold);
    #else
        friction->speed[0] = (friction->counter>0)? (619195.05f/(2*friction->counter)-33.356f+COMPENSATE):0;
    #endif
    friction->changed = 1;
//        friction->counter = counter;
}

float delta_lim = 2;
float dspd_gain = 1.6f, dspd_gain2 = 0.0f;
void looperUpdateFriction(PWMFriction_Type* friction) {
    friction->output[2] = friction->output[1];
    friction->output[1] = friction->output[0];
    
    float d_spd = friction->speed[0] - friction->speed[1];
    float d2_spd = friction->speed[1] - friction->speed[2];
    if(friction->target < 10) {
        friction->output[0] = 0;
        friction->pid.sum_error = 0;
    }
    else if(d_spd < 0) {
        friction->output[0] += fabs(d_spd) * dspd_gain + fabs(d2_spd)*dspd_gain2;
    }
    else {
        friction->output[0] = friction->target;
    }
    
    friction->speed[1] = friction->speed[0];
    friction->speed[2] = friction->speed[1];
    
//    else if(friction->output[0] < friction->target - 25) {
//        friction->output[0] +=  delta_lim;
//    } else {
//        friction->output[0] = friction->target;
//    }
    
//    else if(friction->output[0]<friction->target - 3) {
//        friction->output[0] = friction->target + 10;
//    } else {
//        friction->output[0] = friction->target;
//    }
    
//        // PID_ControllerDriver_Incremental
//        if(friction->speed[0] - friction->target < -3) {
//            friction->output[0] = friction->target*2 - friction->speed[0];
//        } else {
//            friction->output[0] = friction->target;
//        }
////        friction->output[0] = friction->output[0] + PID_ControllerDriver(&friction->pid, 
////            friction->target, friction->speed[0]);
}


//void DMA1_Stream5_IRQHandler(void)
//{
//    if(DMA_GetITStatus(DMA1_Stream5, DMA_IT_TCIF5)) {
//        
//        dmaFrictionUpdata(&Friction_CH1);
//        
//        DMA_ClearFlag(DMA1_Stream5, DMA_IT_TCIF5);
//        DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF5);
//    }
//}

void DMA1_Stream6_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_Stream6, DMA_IT_TCIF6)) {
        
        dmaFrictionUpdata(&Friction_CH2);
        
        DMA_ClearFlag(DMA1_Stream6, DMA_IT_TCIF6);
        DMA_ClearITPendingBit(DMA1_Stream6, DMA_IT_TCIF6);
    }
}



void Friction_Init(void) {
    PWM_Output();
//    PulseCapture();
    
    PID_ControllerInit(&Friction_CH1.pid, Friction_INTE_limitI, (uint16_t)-1, 160);
    Friction_CH1.pid.Kp = Friction_SPEED_p;
    Friction_CH1.pid.Ki = Friction_SPEED_i;
    Friction_CH1.pid.Kd = Friction_SPEED_d;
    Friction_CH1.pid.intergration_separation = 20.f;
    
    PID_ControllerInit(&Friction_CH2.pid, Friction_INTE_limitI, (uint16_t)-1, 160);
    Friction_CH2.pid.Kp = Friction_SPEED_p;
    Friction_CH2.pid.Ki = Friction_SPEED_i;
    Friction_CH2.pid.Kd = Friction_SPEED_d;
    Friction_CH2.pid.intergration_separation = 20.f;
}



uint8_t DELTA_SPEED = 3;
float SPEED_THRESHOLD = 4;
uint32_t check = 0;
void Friction_Looper(void) {
    static uint16_t speed_shoot = 0;
		static uint16_t timeticket = 0;
		if(frictionState == Friction_OFF) {
				speed_shoot = 0;
				timeticket = 0;
		} 
		else if(frictionState == Friction_ON){
				frictionState = Friction_ON;
				timeticket++;
				if(timeticket%5 == 0)
						speed_shoot += (speed_shoot<max_shoot_speed)?10:0;
				if(timeticket>10000)
					timeticket = 0;
		}
    Friction_CH1.target = Friction_CH2.target = speed_shoot;
    looperUpdateFriction(&Friction_CH1);
    looperUpdateFriction(&Friction_CH2);
    if(frictionState == Friction_ON){
			TIM_SetCompare1(TIM8, 800 - Friction_CH1.output[0] );
			TIM_SetCompare2(TIM8, 800 - Friction_CH2.output[0] );
			TIM_SetCompare3(TIM8, check);
			TIM_SetCompare4(TIM8, check);
    }
		else{
			TIM_SetCompare1(TIM8, 800);
			TIM_SetCompare2(TIM8, 800);
			TIM_SetCompare3(TIM8, check);
			TIM_SetCompare4(TIM8, check);
		}
//    TIM_SetCompare1(TIM8, target + 200);
//    TIM_SetCompare2(TIM8, target + 200);
//    printf("%d,%d,%d\r\n", target, Friction_CH1.counter, Friction_CH2.counter);
}

void Shooting_Control_Init (void){
        #ifdef USING_FRICTION
					Friction_Init();
					extern uint16_t frictionState;
        #endif
        
        #ifdef USING_FEED_MOTOR
					MOTOR_CrtlType_CAN* motor203 = spMOTOR.user.enable(CAN1, Motor203, RM_2006_P36, false);
					motor203->control.speed_pid->Kp = 500.0f;
					motor203->control.speed_pid->Ki = 0.0f;
					motor203->control.speed_pid->Kd = 1.f;
					motor203->control.speed_pid->intergrations_sum_error_limit = 5*PI;
					motor203->control.speed_pid->intergration_separation = PI;
					motor203->control.output_limit = 9000;
					Feed_motor = motor203;
        #endif
}

void Feed_Motor_ON(void){
	spMOTOR.user.set_speed(CAN1, Motor203, Feed_SPEED);
}

void Feed_Motor_OFF(void){
	spMOTOR.user.set_speed(CAN1, Motor203, 0);
}

void Feed_Motor_BACK(void){
	spMOTOR.user.set_speed(CAN1, Motor203, -Feed_SPEED);
}

void Feed_Motor_Looper(void){
	static int16_t stop_flag = 0;
	static int16_t times = 0;
	static float motor_position = 0;
	static float motor_position_ex = 0.1f;
	motor_position = Feed_motor->state.current;
	if(shootState == Shoot_OFF){
		Feed_Motor_OFF();
	}
	if(shootState == Shoot_ON){
		if(stop_flag == 0){
			Feed_Motor_ON();
		}
		else{
			times++;
			if(times<50){
				Feed_Motor_BACK();
			}
			else if(times<100){
				Feed_Motor_ON();
			}
			else{
				times = 0;
				stop_flag = 0;
			}
		}
	 static int16_t times2 = 0;
		if(fabs(motor_position - motor_position_ex) < 10.0f/8192.f*2.f*PI && stop_flag == 0){
			times2++;
			if(times2 > 3){
				stop_flag = 1;
				times2 = 0;
			}
		}
		else{
			times2 = 0;
		}
		
	}
	motor_position_ex = motor_position;
}

void Shooting_Control_Looper (void){
		static RC_DataType recv_ex;
		#ifdef USING_FRICTION
			if(recv.rc.s2 == RC_SW_MID) {
					frictionState = Friction_OFF;
			} 
			else if(recv.rc.s2==RC_SW_DOWN){
					if(recv.rc.s1==RC_SW_MID  && recv_ex.rc.s1==RC_SW_UP) {
							frictionState = Friction_ON;
					}
			}
			else if(recv.rc.s2==RC_SW_UP){
				frictionState = Friction_OFF;
			}
			Friction_Looper();
		#endif
			
		#ifdef USING_FEED_MOTOR
			if(recv.rc.s2 == RC_SW_MID) {
				shootState = Shoot_OFF;
			} 
			else if(recv.rc.s2==RC_SW_DOWN){
				if(recv.rc.s1==RC_SW_DOWN)
					shootState = Shoot_ON;
				else
					shootState = Shoot_OFF;
				
				if(ext_power_heat_data.shooter_heat0 > 250){
					shootState = Shoot_OFF;
					Cooling_tickets = 0;
				}
				if(ext_power_heat_data.shooter_heat0 == 0){
					Cooling_tickets = Shoot_Cooling_Time;
				}
				if(Cooling_tickets < Shoot_Cooling_Time){
					shootState = Shoot_OFF;
					Cooling_tickets++;
				}
				else if(Cooling_tickets > 10000)
					Cooling_tickets = Shoot_Cooling_Time;
				
			}
			else if(recv.rc.s2==RC_SW_UP){
				if(recv.rc.s1==RC_SW_DOWN)
					shootState = Shoot_OFF;
				else if(recv.rc.s1==RC_SW_MID)
					shootState = Shoot_OFF;
				else if(recv.rc.s1==RC_SW_UP){
					if(auto_aim_flag == 1)
						shootState = Shoot_ON;
					else
						shootState = Shoot_OFF;
					
					if(ext_power_heat_data.shooter_heat0 > 250){
						shootState = Shoot_OFF;
						Cooling_tickets = 0;
					}
					if(ext_power_heat_data.shooter_heat0 == 0){
						Cooling_tickets = Shoot_Cooling_Time;
					}
					if(Cooling_tickets < Shoot_Cooling_Time){
						shootState = Shoot_OFF;
						Cooling_tickets++;
					}
					else if(Cooling_tickets > 10000)
						Cooling_tickets = Shoot_Cooling_Time;

				}
			}
			Feed_Motor_Looper();
		#endif
		recv_ex = recv;
}

/**
  * @}
  */

/**
  * @}
  */


/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/
