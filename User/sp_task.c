/**
  ******************************************************************************
  * @file       task.c
  * @author     YTom
  * @version    v0.0-alpha
  * @date       2018.Nov.11
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

#include "sp_chasis.h"
#include "sp_utility.h"
#include "sp_rc.h"
#include "sp_imu.h"

#include <math.h>

#include "sp_can.h"
#include "sp_pid.h"
#include "gimbal.h"
#include "Auto_aim.h"
#include "sp_shoot.h"

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

#ifdef USING_SPEED_BALANCE
PID_Type speedbalance;
int16_t speed1;
int16_t speed2;
#endif



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
}

/**
  * @brief  Enable general IRQ and modules.
  */
void TASK_Enable() {
    //#warning "Place your IRQ staring on system inited here."
    task_inited = true;
    NVIC_IRQEnable(USART3_IRQn, 0, 3);
    NVIC_IRQEnable(USART6_IRQn, 0, 3);
    NVIC_IRQEnable(UART7_IRQn, 0, 3);
    NVIC_IRQEnable(UART8_IRQn, 0, 3);

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
        spIRQ_Manager.init();

        Led_Configuration();
        Led8_Configuration();
        KEY_Configuration();
        Buzzer_Init();
        // Enable CAN1/CAN2, baudrate=1Mbps
        spCAN_Controllers._system.init(CAN1,CAN_SJW_1tq,CAN_BS2_4tq,CAN_BS1_9tq,3,CAN_Mode_Normal);
        spCAN_Controllers._system.init(CAN2,CAN_SJW_1tq,CAN_BS2_4tq,CAN_BS1_9tq,3,CAN_Mode_Normal);
        // DMA memory-to-memory tranfer config
        spDMA_Controllers.mem2mem.init(NULL, NULL, 0);

        // Start general USART8 for general communication
        USART_TX_Config(UART8, 115200);
        USART_RX_Config(UART8, 115200);
        USART_Cmd(UART8, ENABLE);

        // Start USART and DMA for send data
        USART_TX_Config(UART7, 115200);
        USART_RX_Config(UART7, 115200);
        DMA_USART_TX_Config(UART7);
        DMA_USART_RX_Config(UART7, (uint32_t)USART_Transfer.buffer, sizeof(USART_Transfer.buffer), true);
        USART_ITConfig(UART7, USART_IT_IDLE, ENABLE);
        USART_Cmd(UART7, ENABLE);

        Autoaim_Init();


    }

    /**
      * @brief  System layer initialize
      */
    {
        MOTOR_ControlInit();
        CHASIS_ControlInit();
        spGIMBAL_Controller._system.init();
        /* Enable Remote Controller Receiver */
        RC_ReceiverInit();

        /* IMU module init */
        IMU_Controllers.operations.init();



#ifdef USING_USB
        USB_TaskInit();
#endif
    }

    /**
      * @brief  Peripheral layer initialize
      */
    {
#ifdef USING_SENTRY_CHASIS
        MOTOR_CrtlType_CAN* motor201 = CHASIS_EnableMotor(Motor201, RM_3508_P19, false);
        MOTOR_CrtlType_CAN* motor202 = CHASIS_EnableMotor(Motor202, RM_3508_P19, false);

        if(motor201) {
            motor201->control.speed_pid->Kp = 10.0f;
            motor201->control.speed_pid->Ki = 0.0f;
            motor201->control.speed_pid->Kd = 1.0f;
            motor201->control.speed_pid->intergration_limit = 1000;
            motor201->control.speed_pid->intergration_separation = 800;

            motor201->control.output_limit = 10000;
        }

        if(motor202) {
            motor202->control.speed_pid->Kp = 9.0f;
            motor202->control.speed_pid->Ki = 0.0f;
            motor202->control.speed_pid->Kd = 1.0f;
            motor202->control.speed_pid->intergration_limit = 1000;
            motor202->control.speed_pid->intergration_separation = 800;

            motor202->control.output_limit = 10000;
        }
#endif
#ifdef USING_SPEED_BALANCE
        PID_ControllerInit(&speedbalance, 500, 0xFFFF, 100, 0.01f);
        PID_SetGains(&speedbalance, 1.0f, 5.0f, 0.001f);     // For init
        speedbalance.intergration_separation = 100;
#endif
    }

    Shooting_Control_Init();
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
    
    task_counter++;
    
    RC_GetState(&recv);

    /** 
      * @brief  Sundries and uiltilies looper
      */
#ifdef USING_USB
    USB_TaskLoop();
#endif
    
    /** 
      * @brief  Peripheral layer looper
      */
    {
        if(task_counter%10 == 1) {
					
        } else if(task_counter%10 == 3) {
					Shooting_Control_Looper();
        } else if(task_counter%10 == 5) {
          #ifdef USING_SENTRY_CHASIS
						static int16_t speed = 0;
           	if(recv.rc.s2==RC_SW_UP) {
                auto_aim_flag = 0; 
								small_power_flag = 0xff;
            } else if(recv.rc.s2==RC_SW_DOWN) {
							  auto_aim_flag = 0xff;
								small_power_flag = 0;
								speed = (abs(recv.rc.ch2)<20?0:recv.rc.ch2);
							
            } else {
                CHASIS_SetMotorSpeed(Motor201, 0);
								CHASIS_SetMotorSpeed(Motor202, 0);
                auto_aim_flag = 0; small_power_flag = 0;
            }
						#ifdef USING_SPEED_BALANCE
								MOTOR_CrtlType_CAN* motor201 = CHASIS_GetMotor(Motor201);
								MOTOR_CrtlType_CAN* motor202 = CHASIS_GetMotor(Motor202);
								int16_t SpeedDifference = motor201->state.speed - motor202->state.speed;
								float speedchange = PID_ControllerDriver(&speedbalance,0,SpeedDifference);
								CHASIS_SetMotorSpeed(Motor201, speed + speedchange);
							  CHASIS_SetMotorSpeed(Motor202, speed);
						#else
								CHASIS_SetMotorSpeed(Motor201, speed);
								CHASIS_SetMotorSpeed(Motor202, speed);
						#endif
          #endif
        } else if(task_counter%10 == 7) {
						static RC_DataType recv_ex;
            #if defined(USING_GIMBAL_MODE) && USING_GIMBAL_MODE==1
							if(recv.rc.s2 ^ recv_ex.rc.s2) {
                if(recv.rc.s2==RC_SW_UP) {
                    auto_aim_flag = 0;
                    small_power_flag = 0xff;
                    /*visual pid change*/
                    spGIMBAL_Controller.user.visual_pid_init();
                } else if(recv.rc.s2==RC_SW_DOWN) {
                    auto_aim_flag = 0xff;
                    small_power_flag = 0;
                    spGIMBAL_Controller.user.pid_init();    //change pid to normal
                } else {
                    auto_aim_flag = 0;
                    small_power_flag = 0;
                }
							} 
							
							if(recv.rc.s2==RC_SW_UP) {
								robotMode = CRUISE_MODE;
							} else if(recv.rc.s2==RC_SW_DOWN) {  // hand operation mode
									robotMode = REMOTE_MODE;
							} else {
									robotMode = STANDBY_MODE;
							}
							spGIMBAL_Controller._system.statelooper();
						#endif
            recv_ex = recv;
        }
        

    }
    /** 
      * @brief  System layer looper
      */
    {
        
    }
    /** 
      * @brief  Low layer looper
      */
    {
        // Singal light
        if(task_counter%499 == 0) {
            LED_G_TOGGLE(); LED_R_TOGGLE();
        }
    }
}


void TASK_Backend(void) {
    RC_ReceiverChecker();

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

    /* System background */
    uint32_t ctime = TASK_GetMilliSecond();

    if(ctime%10 == 0) {
#ifndef USING_MOTOR_TEST
        MOTOR_ControlLooper();
#endif
#if defined(USING_GIMBAL_MODE) && USING_GIMBAL_MODE==1
        spGIMBAL_Controller._system.looper();
#endif
        CHASIS_ControlLooper();
    }
    spCAN_Controllers._system.transmit_looper(CAN1);
    spCAN_Controllers._system.transmit_looper(CAN2);
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
    TIM_Init(spSYSTIMER, 1000, false);
    TIM_ITConfig(spSYSTIMER, TIM_IT_Update, ENABLE);
    NVIC_IRQEnable(spSYSTIMER_IRQn, 0, 0);
    TIM_Cmd(spSYSTIMER, ENABLE);
    spIRQ_Manager.registe(spSYSTIMER_IRQn, TIM_IT_Update, spClockHandler);
    spIRQ_Manager.registe(spSYSTIMER_IRQn, TIM_IT_Update, PendSV_Handler);
    
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
