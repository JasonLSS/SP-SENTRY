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

/** @defgroup System_Time_Support
  * @brief    Using spCLOCK(default TIM14) as clock source with 1ms(count 1000) period.
  * @{
  */

/**
  * @brief  Counter used in @func TASK_ControlLooper() with period 1ms
  */
static struct {
    volatile uint32_t  ms;
//    volatile uint32_t  us;
} spClock = {0};

void spCLOCK_Configuration(void) {
    SysTick->CTRL = 0x00;                               /* Reset SysTick control */
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);    /* Using SystemCoreClock as clock source */
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;          /* Enbale systick interrupt */
    SysTick->LOAD = SystemCoreClock/1000;               /* Config to generate 1ms peroid interrupt */
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;           /* Enbale systick */
}

void SysTick_Handler(void) {
    spClock.ms ++;
}

uint32_t TASK_GetMicrosecond(void) {
    return spClock.ms;
}

void TASK_GetMicrosecondPtr(unsigned long *count) {
    count[0] = spClock.ms;
}

spTimeStamp TASK_GetTimeStamp(void) {
    spTimeStamp stamp;
//    stamp.us = 1000 - spClock.us;
    stamp.ms = spClock.ms;
    return stamp;
}

float TASK_GetSecond(void) {
    return spClock.ms*1.0f/1000;
//    return spClock.ms*1.0f/1000 + (1000 - spClock.us)*1.0f/1000000;
}

//float TASK_GetSecondFromTimeStamp(spTimeStamp* stamp) {
//    return stamp->ms*1.0f/1000 + stamp->us*1.0f/1000000;
//}

/**
  * @}
  */




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
  * @brief  Init systick for global timer control
  * @note   The SysTick calibration value is fixed to 18750(10ms), which gives a reference time base of 1 ms
  *         with the SysTick clock set to 18.75 MHz (HCLK/8, with HCLK set to 150 MHz).
  */
void TASK_TimerInit(void) {
    /* 84MHz */
    spRCC_Set_SYSTIMER();
    /* 1000Hz */
    TIM_TimeBaseInitTypeDef tim_base_initer;
    tim_base_initer.TIM_Prescaler = 84-1;
    tim_base_initer.TIM_Period = 1000-1;
    tim_base_initer.TIM_ClockDivision = TIM_CKD_DIV1;
    tim_base_initer.TIM_CounterMode = TIM_CounterMode_Up;
    
    TIM_TimeBaseInit(spSYSTIMER, &tim_base_initer);
    TIM_ITConfig(spSYSTIMER, TIM_IT_Update, ENABLE);
    NVIC_IRQEnable(spSYSTIMER_IRQn, 0, 0);
    TIM_Cmd(spSYSTIMER, ENABLE);
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
        // Enable CAN1, baudrate=1Mbps
        CAN1_Init(CAN_SJW_1tq,CAN_BS2_4tq,CAN_BS1_9tq,3,CAN_Mode_Normal);
        // Enable CAN2, baudrate=1Mbps
        CAN2_Init(CAN_SJW_1tq,CAN_BS2_4tq,CAN_BS1_9tq,3,CAN_Mode_Normal);
        // DMA memory-to-memory tranfer config
        DMA_InitNull(NULL, NULL, 0);
        
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
    }
    
    /** 
      * @brief  System layer initialize
      */
    {
        MOTOR_ControlInit();
        CHASIS_ControlInit();
        
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
//        MOTOR_CrtlType_CAN* motor207 = CHASIS_EnableMotor(Motor207, RM_2006_P36, false);
//        if(motor207) {
//            PID_SetGains(motor207->control.speed_pid, 3.f, 6.f, 0.f);
//            motor207->control.speed_pid->intergration_limit = 1000;
//            motor207->control.speed_pid->intergration_separation = 800;
//            
//            motor207->control.output_limit = 5000;
//        }
    #ifdef USING_CHASIS
        MOTOR_CrtlType_CAN* motor201 = CHASIS_EnableMotor(Motor201, RM_3508_P19, false);
        MOTOR_CrtlType_CAN* motor202 = CHASIS_EnableMotor(Motor202, RM_3508_P19, false);
        MOTOR_CrtlType_CAN* motor203 = CHASIS_EnableMotor(Motor203, RM_3508_P19, false);
        MOTOR_CrtlType_CAN* motor204 = CHASIS_EnableMotor(Motor204, RM_3508_P19, false);
        
        motor201->control.speed_pid = NULL;
        motor202->control.speed_pid = NULL;
        motor203->control.speed_pid = NULL;
        motor204->control.speed_pid = NULL;
        
        PID_ControllerInit(&CHASIS_Param_Reg.PID.x, 50, -1, 4000, 0.01f);
        CHASIS_Param_Reg.PID.x.intergration_separation = 20;
        PID_ControllerInit(&CHASIS_Param_Reg.PID.y, 50, -1, 4000, 0.01f);
        CHASIS_Param_Reg.PID.y.intergration_separation = 20;
        PID_ControllerInit(&CHASIS_Param_Reg.PID.yaw, 50, -1, 4000, 0.01f);
        CHASIS_Param_Reg.PID.yaw.intergration_separation = 20;
        
        PID_SetGains(&CHASIS_Param_Reg.PID.x, 15.0f, 1.f, 1.f);
        PID_SetGains(&CHASIS_Param_Reg.PID.y, 15.0f, 1.f, 1.f);
        PID_SetGains(&CHASIS_Param_Reg.PID.yaw, 15.0f, 1.f, 1.f);
        
//        // 4, 10
//        if(motor201 && motor202 && motor203 && motor204) {
//            PID_SetGains(motor201->control.speed_pid, 15.f, 7.f, 1.f);
//            motor201->control.speed_pid->intergration_limit = 1000;
//            motor201->control.speed_pid->intergration_separation = 200;
//            motor201->control.output_limit = 4000;

//            PID_SetGains(motor202->control.speed_pid, 15.f, 7.f, 1.f);
//            motor202->control.speed_pid->intergration_limit = 1000;
//            motor202->control.speed_pid->intergration_separation = 200;
//            motor202->control.output_limit = 4000;

//            PID_SetGains(motor203->control.speed_pid, 15.f, 7.f, 1.f);
//            motor203->control.speed_pid->intergration_limit = 1000;
//            motor203->control.speed_pid->intergration_separation = 200;
//            motor203->control.output_limit = 4000;

//            PID_SetGains(motor204->control.speed_pid, 15.f, 7.f, 1.f);
//            motor204->control.speed_pid->intergration_limit = 1000;
//            motor204->control.speed_pid->intergration_separation = 200;
//            motor204->control.output_limit = 4000;
//        }
    #endif
    }
    /** 
      * @brief  Sundries and initialize
      */
    {
        
    }
}

/**
  * @brief  Enable systick to start running tasks
  */
inline void TASK_Start(void) {
    spCLOCK_Configuration();
    
    NVIC_IRQEnable(CAN1_RX0_IRQn, 0, 2);
    NVIC_IRQEnable(CAN2_RX1_IRQn, 0, 2);
    NVIC_IRQEnable(USART1_IRQn, 0, 0);          // RC
    NVIC_IRQEnable(DMA2_Stream5_IRQn, 0, 3);    // RC
    NVIC_IRQEnable(USART2_IRQn, 0, 2);          // View-USART2
    NVIC_IRQEnable(EXTI9_5_IRQn, 0, 3);         // MPU int
    NVIC_IRQEnable(EXTI2_IRQn, 0, 5);           // Key
    
    extern void sendtoComputerInit(void);
    sendtoComputerInit();
}

/**
  * @brief  Enabel each modules
  */
void TASK_Enable() {

    /**
      * @brief  NVIC config
      */
    {
        NVIC_IRQEnable(USART3_IRQn, 1, 1);
        NVIC_IRQEnable(USART6_IRQn, 1, 1);
        NVIC_IRQEnable(UART7_IRQn, 1, 1);
        NVIC_IRQEnable(UART8_IRQn, 1, 1);
    }

}

RC_DataType recv;
RC_DataType recv_ex;
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
          #ifdef USING_CHASIS
//            float speed[4];
//            // rad/s
//            speed[0] = CHASIS_GetMotor(Motor201)->state.speed;
//            speed[1] = CHASIS_GetMotor(Motor202)->state.speed;
//            speed[2] = CHASIS_GetMotor(Motor203)->state.speed;
//            speed[3] = CHASIS_GetMotor(Motor204)->state.speed;
//            CHASIS_Mecanum_Inv(speed, &CHASIS_Param_Reg.state.x, &CHASIS_Param_Reg.state.y, &CHASIS_Param_Reg.state.yaw);
          #endif
        } else if(task_counter%10 == 3) {
            
        } else if(task_counter%10 == 5) {
          #ifdef USING_CHASIS
            if(recv.rc.s1 == RC_SW_MID) {
                #define __CHASIS_kLinearSpeedX              0.004545f          /*  */
                #define __CHASIS_kLinearSpeedY              0.004545f          /*  */
                #define __CHASIS_kAngularSpeed              0.004545f          /*  */
                CHASIS_Param_Reg.target.spdx = (abs(recv.rc.ch1)<20?0:recv.rc.ch1) * __CHASIS_kLinearSpeedX;
                CHASIS_Param_Reg.target.spdy = -(abs(recv.rc.ch0)<20?0:recv.rc.ch0) * __CHASIS_kLinearSpeedY;
                CHASIS_Param_Reg.target.spdyaw = -(abs(recv.rc.ch2)<20?0:recv.rc.ch2) * __CHASIS_kAngularSpeed;
                CHASIS_Move(
                    CHASIS_Param_Reg.target.spdx, 
                    CHASIS_Param_Reg.target.spdy,
                    CHASIS_Param_Reg.target.spdyaw);
            } else {
                CHASIS_Move(0, 0, 0);
            }
          #endif
        } else if(task_counter%10 == 7) {
            
        }
        
        recv_ex = recv;
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
    
    if(!task_inited){
        static uint16_t tick_init = 0;
        tick_init ++;
        
        #if defined(USING_GIMBAL_MODE) && USING_GIMBAL_MODE>0
        if(spGIMBAL_Controller._system.regression(tick_init)) {
            task_inited = true;
            TASK_Enable();
        }
        #else
        if(tick_init>= 3000) {
            task_inited = true;
            TASK_Enable();
        }
        #endif
    }else{
        TASK_ControlLooper();
    }
    
    /* System background */
    uint32_t ctime = TASK_GetMicrosecond();

    if(ctime%10 == 0) {
    #ifndef USING_MOTOR_TEST
        MOTOR_ControlLooper();
    #endif
    #if defined(USING_GIMBAL_MODE) && USING_GIMBAL_MODE==1
        spGIMBAL_Controller._system.looper();
    #endif
        CHASIS_ControlLooper();
    }
    CAN1_MsgSendLoop();
    CAN2_MsgSendLoop();
}

void TIM8_TRG_COM_TIM14_IRQHandler(void) {
    if(TIM_GetITStatus(TIM14,TIM_IT_Update) == SET) {
        
        TASK_Backend();
        
        TIM_ClearITPendingBit(TIM14, TIM_IT_Update);
    }
}

/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/

//        // Start general USART8 for general communication
//        extern uint8_t referee_buffer[128];
//        USART_RX_Config(USART6, 115200);
//        DMA_USART_RX_Config(USART6, (uint32_t)referee_buffer, sizeof(referee_buffer), true);
//        USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);
//        DMA_ITConfig(spDMA_USART6_rx_stream, DMA_IT_TC, ENABLE);
//        DMA_Cmd(spDMA_USART6_rx_stream, ENABLE);
//        USART_Cmd(USART6, ENABLE);
       
//        /* Registe autoaims to USART2 */
//        /* For view communication via USART2 */
//    #if defined(USING_GIMBAL_MODE) && USING_GIMBAL_MODE==1
//        extern UsartBuffer_t view_buffer;
//        USART_RX_Config(USART2, 115200);
//        DMA_USART_RX_Config(USART2, (uint32_t)view_buffer.buffer, view_buffer.size, false);
//        
//        USART_TX_Config(USART2, 115200);
//        DMA_USART_TX_Config(USART2);
//        
//        USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
//        DMA_ITConfig(spDMA_USART2_rx_stream, DMA_IT_TC, ENABLE);
//        DMA_Cmd(spDMA_USART2_rx_stream, ENABLE);
//        USART_Cmd(USART2, ENABLE);
//        spIRQ_Manager.registe(USART2_IRQn, Autoaim_USART_Interface);
//    #endif
    
        /* ADI IMU */
//        extern void SPI4_Init(void);
//        SPI4_Init();
//        delay_ms(1);
//        uint16_t prod_id[24];
//        
//        for(uint8_t i=0; i<24; i++) {
//            spSPI_Manager.select(&SPI4_Pins);
//            prod_id[i] = spSPI_Manager.read_write_b(SPI4, 0x7200);
//            spSPI_Manager.release(&SPI4_Pins);
//            delay_us(20);
//        }

//          #ifdef USING_MOTOR_TEST
//            extern float __MOTOR_OutputLimit(MOTOR_CrtlType_CAN* __motor, float value);
//            MOTOR_CrtlType_CAN* motor = CHASIS_GetMotor(Motor206);

//            if(MOTOR_ControllReg.data.flushed) {
//                MOTOR_ControllReg.data.flushed = false;
//                
//                motor->control.speed_pid->Kp = MOTOR_ControllReg.data.skp;
//                motor->control.speed_pid->Ki = MOTOR_ControllReg.data.ski;
//                motor->control.speed_pid->Kd = MOTOR_ControllReg.data.skd;
//                motor->control.position_pid->Kp = MOTOR_ControllReg.data.pkp;
//                motor->control.position_pid->Ki = MOTOR_ControllReg.data.pki;
//                motor->control.position_pid->Kd = MOTOR_ControllReg.data.pkd;
//                
//                tar_spd = motor->state.angle + 8192;
//            }
//            
//            /* PID controller */
//            float pid_tmp;
//            CHASIS_SetMotorPosition(Motor206, tar_spd);
//            pid_tmp = PID_ControllerDriver_test(motor->control.position_pid, 
//                motor->control.target , motor->state.angle);
//            pid_tmp = __MOTOR_OutputLimit(motor, pid_tmp);
//            if(motor->control.speed_pid) {
//                pid_tmp = PID_ControllerDriver_test(motor->control.speed_pid,
//                    pid_tmp, motor->state.speed);
//            }
//            motor->control.output = __MOTOR_OutputLimit(motor, pid_tmp);

////            static int tar_spd = 2000;
////            static bool using_key = true;
////            if(using_key && spUserKey.on_press) {
////                spUserKey.on_press = false;
////                motor201->control.output = (motor201->control.output==0)*tar_spd;
////            } else {
////                // TODO: Make interval time more specific.
////                float pid_tmp;
////                pid_tmp = PID_ControllerDriver_test(motor201->control.position_pid, 
////                    motor201->control.target , motor201->state.angle);
////                pid_tmp = __MOTOR_OutputLimit(motor201, pid_tmp);
////                if(motor201->control.speed_pid) {
////                    pid_tmp = PID_ControllerDriver_test(motor201->control.speed_pid,
////                        pid_tmp, motor201->state.speed);
////                }
////                motor201->control.output = __MOTOR_OutputLimit(motor201, pid_tmp);
////            }
//          #endif