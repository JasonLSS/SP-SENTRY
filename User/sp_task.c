/**
  ******************************************************************************
  * @file       task.c
  * @author     LSS
  * @version    v0.2-alpha
  * @date       2019.Mar.27
  * @brief      Background/Basic task manager module
  * @usage      This module mainly includes:
  *             (+) Global task initialization and module configuration
  *             (+) Config global timer interrupt for task control
  *             Using SYSTICK as global timer
  ******************************************************************************
  * @license
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "sp_conf.h"

#include "sp_utility.h"
#include "sp_rc.h"
#include "sp_imu.h"

#include <math.h>
#include <string.h>
#include "sp_can.h"
#include "sp_pid.h"
#include "gimbal.h"
#include "Auto_aim.h"
#include "sp_shoot.h"
#include "sp_sensor.h"
#include "sp_chasis.h"
#include "referee.h"
#include "infrared.h"

/** @addtogroup SP
  * @brief      SuperPower
  * @{
  */

/** @defgroup TASK
  * @brief    TASK Control
  * @{
  */


/** @defgroup System_Time_Support
  * @brief    Using spCLOCK(default TIM14) as clock source with 1ms(count 1000) period.
  * @ingroup  TASK
  * @{
  */

/**
  * @brief  Counter used in @func TASK_ControlLooper() with period 1ms
  */
static struct {
    volatile uint32_t  ms;
    volatile uint32_t*  us;
} spClock = {0, &spCLOCKTIMER->CNT};

void spClockHandler(void) {
    spClock.ms ++;
}

void TASK_GetMicrosecondPtr(unsigned long *count) {
    count[0] = spClock.ms;
}

spTimeStamp TASK_GetTimeStamp(void) {
    spTimeStamp stamp;
    stamp.us = (*spClock.us);
    stamp.ms = spClock.ms;
    return stamp;
}

//uint32_t TASK_GetMicroSecond(void) {
//    return *spClock.us;
//}

uint32_t TASK_GetMilliSecond(void) {
    return spClock.ms;
}

float TASK_GetSecond(void) {
    return spClock.ms*1.0f/1000 + (*spClock.us)*1.0f/1000000;
}

/** @} */


/** @defgroup Port_Critical_Zone
  * @brief    Enter and exit critical
  * @ingroup  TASK
  * @{
  */
/*-----------------------------------------------------------*/
/* From FreeRTOS                                             */
/*-----------------------------------------------------------*/
#define portNVIC_INT_CTRL_REG           ( * ( ( volatile uint32_t * ) 0xe000ed04 ) )
/* Masks off all bits but the VECTACTIVE bits in the ICSR register. */
#define portVECTACTIVE_MASK             ( 0xFFUL )

static uint16_t uxCriticalNesting;

void spFORCE_INLINE spPortEnterCritical( void )
{
    __disable_irq();
    uxCriticalNesting++;

//    /* This is not the interrupt safe version of the enter critical function so
//    assert() if it is being called from an interrupt context.  Only API
//    functions that end in "FromISR" can be used in an interrupt.  Only assert if
//    the critical nesting count is 1 to protect against recursive calls if the
//    assert function also uses a critical section. */
//    if( uxCriticalNesting == 1 )
//    {
//        assert_param( ( portNVIC_INT_CTRL_REG & portVECTACTIVE_MASK ) == 0 );
//    }
}

void spFORCE_INLINE spPortExitCritical( void )
{
    assert_param( uxCriticalNesting );
    uxCriticalNesting--;
    if( uxCriticalNesting == 0 )
    {
        __enable_irq();
    }
}
/** @} */




/**
  * @brief  Counter used in @func TASK_ControlLooper() with period 1ms
  */
volatile uint32_t           task_counter = 0;
/**
  * @brief  Flag for system initialization
  */
bool                        task_inited = false;
struct {
    uint8_t     buffer[256];
    uint16_t    capacity;
    uint16_t    curr_ptr, last_ptr;
    uint16_t size0, size1;
} USART_Transfer = {
    {0x00}, 256,
    0, 0, 0, 0
};




/**
  * @brief  Enable system base IRQ and modules.
  */
inline void TASK_Start(void) {
    //#warning "Place your IRQ staring on system entering here."
//    spCLOCK_Configuration();

    NVIC_IRQEnable(CAN1_RX0_IRQn, 0, 2);
    NVIC_IRQEnable(CAN2_RX1_IRQn, 0, 2);

    NVIC_IRQEnable(USART1_IRQn, 0, 0);          // RC
    NVIC_IRQEnable(DMA2_Stream5_IRQn, 0, 3);    // RC

    NVIC_IRQEnable(USART2_IRQn, 0, 2);          // View-USART2

    NVIC_IRQEnable(EXTI9_5_IRQn, 0, 3);         // MPU int
    NVIC_IRQEnable(EXTI2_IRQn, 0, 5);           // Key
    NVIC_IRQEnable(EXTI0_IRQn, 0, 5);           // IMU_ADI
	
		NVIC_IRQEnable(USART3_IRQn, 0, 3);
    NVIC_IRQEnable(USART6_IRQn, 0, 3);
    NVIC_IRQEnable(UART7_IRQn, 0, 3);
    NVIC_IRQEnable(UART8_IRQn, 0, 3);
}

/**
  * @brief  Enable general IRQ and modules.
  */
void TASK_Enable() {
    //#warning "Place your IRQ staring on system inited here."
    task_inited = true;
    

    NVIC_IRQEnable(TIM1_TRG_COM_TIM11_IRQn, 0, 1);
    NVIC_IRQEnable(TIM8_BRK_TIM12_IRQn, 0, 1);
}

/**
  * @brief  Init modules in system layer control
  * @note   User CANNOT involve this functions
  *         This function is called in the .s link file
  */
void TASK_GlobalInit() {
    /**
      * @brief  Low layer initialize
      */
    {
        spIRQ.init();

        Led_Configuration();
        Led8_Configuration();
//        Button_Configuration();
        Buzzer_Init();
        // Enable CAN1/CAN2, baudrate=1Mbps
        spCAN._system.init(CAN1,CAN_SJW_1tq,CAN_BS2_4tq,CAN_BS1_9tq,3,CAN_Mode_Normal);
        spCAN._system.init(CAN2,CAN_SJW_1tq,CAN_BS2_4tq,CAN_BS1_9tq,3,CAN_Mode_Normal);
        // DMA memory-to-memory tranfer config
        spDMA.mem2mem.init(NULL, NULL, 0);

        // Start general USART8 for general communication
        USART_TX_Config(UART8, 115200);
        USART_RX_Config(UART8, 115200);
        USART_Cmd(UART8, ENABLE);

        // Start USART and DMA for send data
        USART_TX_Config(USART3, 115200);
        USART_RX_Config(USART3, 115200);
        DMA_USART_TX_Config(USART3);
        DMA_USART_RX_Config(USART3, (uint32_t)USART_Transfer.buffer, sizeof(USART_Transfer.buffer), true);
        USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
        USART_Cmd(USART3, ENABLE);
#ifndef USING_USB
        Autoaim_Init();
#endif				
				IIC_Init();
				Infrared_Init();
				
    }

    /**
      * @brief  System layer initialize
      */
    {
				spMOTOR._system.init();
			
				referee_init();
			
				/* Enable Remote Controller Receiver */
				RC_ReceiverInit();
			
#if defined(USING_GIMBAL_MODE) && USING_GIMBAL_MODE==1
				spGIMBAL_Controller._system.init();
#endif

				/* IMU module init */
//				IMU_Controllers.operations.init();

#ifdef USING_USB
				USB_TaskInit();
				Autoaim_Init();
#endif
    }

    /**
      * @brief  Peripheral layer initialize
      */
    {
			
				Shooting_Control_Init();
				spCHASIS._system.init();



    }
    /**
      * @brief  Sundries and initialize
      */
    {

    }
}


RC_DataType recv;


/**
  * @brief  System layer control loop in background
  */
void TASK_ControlLooper() {
/*------------------------------------------------------------------------*/
/* Sundries and uiltilies looper [PROTECTED] */
/*------------------------------------------------------------------------*/
    task_counter++;
    RC_GetState(&recv);
#ifdef USING_USB
		USB_TaskLoop();
#endif
/*------------------------------------------------------------------------*/
/* Peripheral layer looper [PRIVATE] */
/*------------------------------------------------------------------------*/
    {
				#ifdef USING_SENTRY_CHASIS
						spCHASIS._system.looper(task_counter, &recv);
				#endif
				static RC_DataType recv_ex;
				static uint16_t remain_HP;
				static uint16_t remain_HP_ex;
        if(task_counter%10 == 1) {
						if(recv.rc.s2==RC_SW_UP) {
								if(recv.rc.s1==RC_SW_MID) {
										robotMode = CRUISE_MODE;
								} else if(recv.rc.s1==RC_SW_DOWN){
										robotMode = DYNAMIC_ATTACK_MODE;
								}else if(recv.rc.s1==RC_SW_UP){
										//all auto mode
										static bool attacked = 0;
										static float attacked_time = 0;
										remain_HP = ext_game_robot_state.remain_HP;
										if(remain_HP_ex > remain_HP){
												attacked_time = 200;
												attacked = 1;
										}
										if(attacked_time == 0 )
												attacked = 0;
										else
												attacked_time--;
										//task_lss
										if(enemy_area == 6 && !attacked)
											robotMode = CRUISE_MODE;
										if(ext_game_robot_state.remain_HP > 300 && enemy_area != 6)
											robotMode = DYNAMIC_ATTACK_MODE;
				 						if(ext_game_robot_state.remain_HP > 500 && enemy_area != 6 && !attacked)
											robotMode = STATIC_ATTACK_MODE;
										if(ext_game_robot_state.remain_HP < 300 && 
											 ext_game_robot_state.remain_HP > 100 && enemy_area != 6)
											robotMode = ESCAPE_ATTACK_MODE;
										if((enemy_area == 6 ||ext_game_robot_state.remain_HP < 100)&& attacked)
											robotMode = ESCAPE_MODE;
										
										remain_HP_ex = remain_HP;
								}
						} else if(recv.rc.s2==RC_SW_DOWN) {  
								// hand operation mode
								robotMode = REMOTE_MODE;
						} else {
								robotMode = STANDBY_MODE;
						}
        } else if(task_counter%10 == 3) {
						Shooting_Control_Looper();
        } else if(task_counter%2 == 1) {
						#if defined(USING_GIMBAL_MODE) && USING_GIMBAL_MODE==1
								spGIMBAL_Controller._system.statelooper();
						#endif
        } else if(task_counter%10 == 7) {
						Infrared_Update();//task_lss
        }
//				if(task_counter%5 == 0)
//				{
//					sendtoComputer();
//				}
		}
/*------------------------------------------------------------------------*/
/* System layer looper [PRIVATE] */
/*------------------------------------------------------------------------*/
    {
        
    }
/*------------------------------------------------------------------------*/
/* Low layer looper [PUBLIC] */
/*------------------------------------------------------------------------*/
    {
        /* Singal light */
        if(RC_isValid()) {
            if(task_counter%499 == 0) {
                LED_G_TOGGLE(); LED_R_TOGGLE();
            }
        } else {
            LED_G_OFF(); LED_R_ON();
        }
        
    }
}


void TASK_Backend(void) {
    uint32_t ctime = TASK_GetMilliSecond();
    
    RC_ReceiverChecker(ctime);

    if(!task_inited) {
        static uint16_t tick_init = 0;
        tick_init ++;

#if defined(USING_GIMBAL_MODE) && USING_GIMBAL_MODE>0
        if(spGIMBAL_Controller._system.regression(tick_init)) {
            task_inited = true;
            TASK_Enable();
        }
#else
        if(tick_init>= 30) {
            task_inited = true;
            TASK_Enable();
        }
#endif
    } else {
        TASK_ControlLooper();
    }

    if(ctime%10 == 0) {
#if defined(USING_GIMBAL_MODE) && USING_GIMBAL_MODE==1
        spGIMBAL_Controller._system.looper();
#endif
        spMOTOR._system.looper();
    }
    spCAN._system.transmit_looper(CAN1);
    spCAN._system.transmit_looper(CAN2);
}

void TASK_SerPendSV(void) {
    SCB->ICSR |= ((uint32_t)1<<28);
}
void PendSV_Handler(void) {
    SCB->ICSR |= ((uint32_t)1<<27);
    TASK_Backend();
}

/**
  * @brief  Init systick for global timer control
  * @note   The SysTick calibration value is fixed to 18750(10ms), which gives a reference time base of 1 ms
  *         with the SysTick clock set to 18.75 MHz (HCLK/8, with HCLK set to 150 MHz).
  */
void TASK_TimerInit(void) {
    spRCC_Set_SYSTIMER();
    /* 1000Hz */
    spTIMER.init(spSYSTIMER, 1000, false);
    TIM_ITConfig(spSYSTIMER, TIM_IT_Update, ENABLE);
    NVIC_IRQEnable(spSYSTIMER_IRQn, 0, 0);
    TIM_Cmd(spSYSTIMER, ENABLE);
    spIRQ.registe(spSYSTIMER_IRQn, TIM_IT_Update, spClockHandler);
    spIRQ.registe(spSYSTIMER_IRQn, TIM_IT_Update, PendSV_Handler);
    
    NVIC_IRQEnable(spSYSTIMER_IRQn, 0, 0);
    NVIC_IRQEnable(spCLOCKTIMER_IRQn, 0, 0);
}
/** @} */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/
