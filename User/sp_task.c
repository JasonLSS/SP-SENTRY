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
#include "mpu6500.h"

#include <math.h>

#include "sp_can.h"
#include "sp_pid.h"
#include "gimbal.h"
#include "Auto_aim.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/**
  * @brief  Counter used in @func TASK_ControlLooper() with period 1ms
  */
volatile uint32_t           task_counter = 0;
/**
  * @brief  Flag for system initialization
  */
bool                        task_inited = false;
/**
  * @brief  Values for test
  */
char                        uart8_buff[16] = "Hello world!\r\n";
char                        uart_buff[256] = {0x00};

/* Private functions ---------------------------------------------------------*/
/** @defgroup Task Initialization and Configuration Functions
  * @brief    Implement member functions for @ref MOTOR_CrtlType
  * @{
  */

/**
  * @brief  Init systick for global timer control
  * @note   The SysTick calibration value is fixed to 18750(10ms), which gives a reference time base of 1 ms
  *         with the SysTick clock set to 18.75 MHz (HCLK/8, with HCLK set to 150 MHz).
  */
void TASK_TimerInit(void) {
    SysTick->CTRL = 0x00;                               /* Reset SysTick control */
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);    /* Using SystemCoreClock as clock source */
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;          /* Enbale systick interrupt */
    SysTick->LOAD = SystemCoreClock/1000;               /* Config to generate 1ms peroid interrupt */
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;          /* Enbale systick */
}

/**
  * @brief  Enable systick to start running tasks
  */
inline void TASK_Start(void) {
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;           /* Enbale systick */
    spCLOCK_Configuration();
    
    extern void sendtoComputerInit(void);
    sendtoComputerInit();
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
        Led_Configuration();
        Led8_Configuration();
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
        DMA_USART_TX_Config(UART7);
        USART_Cmd(UART7, ENABLE);
        
        
//        // Start general USART8 for general communication
//        extern uint8_t referee_buffer[128];
//        USART_RX_Config(USART6, 115200);
//        DMA_USART_RX_Config(USART6, (uint32_t)referee_buffer, sizeof(referee_buffer), true);
//        USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);
//        DMA_ITConfig(spDMA_USART6_rx_stream, DMA_IT_TC, ENABLE);
//        DMA_Cmd(spDMA_USART6_rx_stream, ENABLE);
//        USART_Cmd(USART6, ENABLE);
        
        // Enable Remote Controller Receiver
        RC_ReceiverInit();
        /* Start basis functions */
        NVIC_IRQEnable(DMA2_Stream5_IRQn, 1, 0);    // RC
        
        // IMU module init
        MPU6500_Init();

        // Init DMA null stream for memory-to-memory transfer
        DMA_InitNull(NULL, NULL, 0);
        
    #ifdef USING_USB
        USB_TaskInit();
    #endif
    }
    
    /** 
      * @brief  System layer initialize
      */
    {
        
    }
    
    /** 
      * @brief  Peripheral layer initialize
      */
    {
        MOTOR_ControlInit();
        CHASIS_ControlInit();
        
    #ifdef USING_DM6020
        /* For view communication via USART2 */
        extern UsartBuffer_t view_buffer;
        USART_RX_Config(USART2, 115200);
        DMA_USART_RX_Config(USART2, (uint32_t)view_buffer.buffer, view_buffer.size, false);
        
        USART_TX_Config(USART2, 115200);
        DMA_USART_TX_Config(USART2);
        
        USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
        DMA_ITConfig(spDMA_USART2_rx_stream, DMA_IT_TC, ENABLE);
        DMA_Cmd(spDMA_USART2_rx_stream, ENABLE);
        USART_Cmd(USART2, ENABLE);
        
        GIMBAL_ControlInit();
    #endif
        
    #ifdef USING_GM3510Test
        MOTOR_CrtlType_CAN*         gm3510;
        gm3510 = CHASIS_EnableMotor(Motor205, GM_3510, true);
        // PID_SetGains(gm3510->control.speed_pid, 1.2f, 0, 0);
        gm3510->implement.set_speed_pid(gm3510, NULL);
        gm3510->control.position_pid->intergration_separation = 100;
        gm3510->control.position_pid->intergration_limit = 400;
        gm3510->control.position_pid->output_limit = 8000;
        PID_SetGains(gm3510->control.position_pid, 4.f, 10.0f, 0.4f);
    #endif
    
    #ifdef USING_CHASIS
        MOTOR_CrtlType_CAN* motor201 = CHASIS_EnableMotor(Motor201, RM_3508_P19, false);
        MOTOR_CrtlType_CAN* motor202 = CHASIS_EnableMotor(Motor202, RM_3508_P19, false);
        MOTOR_CrtlType_CAN* motor203 = CHASIS_EnableMotor(Motor203, RM_3508_P19, false);
        MOTOR_CrtlType_CAN* motor204 = CHASIS_EnableMotor(Motor204, RM_3508_P19, false);
        
        if(motor201) {
            motor201->control.speed_pid->Kp = 4.f;
            motor201->control.speed_pid->Ki = 10.f;
            motor201->control.speed_pid->Kd = 0;
            motor201->control.speed_pid->intergration_limit = 1000;
            motor201->control.speed_pid->intergration_separation = 800;
            
            motor201->control.output_limit = 8000;
        }
        
        if(motor202) {
            motor202->control.speed_pid->Kp = 4.f;
            motor202->control.speed_pid->Ki = 10.f;
            motor202->control.speed_pid->Kd = 0;
            motor202->control.speed_pid->intergration_limit = 1000;
            motor202->control.speed_pid->intergration_separation = 800;

            motor202->control.output_limit = 8000;
        }
        
        if(motor203) {
            motor203->control.speed_pid->Kp = 4.f;
            motor203->control.speed_pid->Ki = 10.f;
            motor203->control.speed_pid->Kd = 0;
            motor203->control.speed_pid->intergration_limit = 1000;
            motor203->control.speed_pid->intergration_separation = 800;
            
            motor203->control.output_limit = 8000;
        }
        
        if(motor204) {
            motor204->control.speed_pid->Kp = 4.f;
            motor204->control.speed_pid->Ki = 10.f;
            motor204->control.speed_pid->Kd = 0;
            motor204->control.speed_pid->intergration_limit = 1000;
            motor204->control.speed_pid->intergration_separation = 800;
            
            motor204->control.output_limit = 8000;
        }
    #endif
    
    #ifdef USING_SENTRY
        MOTOR_CrtlType_CAN* motor201 = CHASIS_EnableMotor(Motor201, RM_3510_P19, false);
        MOTOR_CrtlType_CAN* motor202 = CHASIS_EnableMotor(Motor202, RM_3510_P19, false);
        
        if(motor201) {
            motor201->control.speed_pid->Kp = 3.0f;
            motor201->control.speed_pid->Ki = 12.f;
            motor201->control.speed_pid->Kd = 0.015f;
            motor201->control.speed_pid->intergration_limit = 1000;
            motor201->control.speed_pid->intergration_separation = 800;
            
            motor201->control.output_limit = 6000;
        }
        
        if(motor202) {
            motor202->control.speed_pid->Kp = 2.0f;
            motor202->control.speed_pid->Ki = 9.f;
            motor202->control.speed_pid->Kd = 0.01f;
            motor202->control.speed_pid->intergration_limit = 1000;
            motor202->control.speed_pid->intergration_separation = 800;

            motor202->control.output_limit = 6000;
        }
    #endif
    
    #ifdef USING_TEST17
        MOTOR_CrtlType_CAN* motor205 = CHASIS_EnableMotor(Motor205, RM_2006_P36, true);
        MOTOR_CrtlType_CAN* gimbal_pitch_motor = CHASIS_EnableMotor(Motor206, RM_2006_P36, false);
        Friction_Init();
        
        if(motor205) {
            motor205->control.speed_pid->Kp = 7.f;
            motor205->control.speed_pid->Ki = 5.f;
            motor205->control.speed_pid->Kd = 0.06f;
            motor205->control.speed_pid->intergration_limit = 500;
            motor205->control.speed_pid->intergration_separation = 400;
            
            motor205->control.position_pid->Kp = 2;
            motor205->control.position_pid->Ki = 0;
            motor205->control.position_pid->Kd = 0;
            
            motor205->control.output_limit = 8000;
        }
        
        if(gimbal_pitch_motor) {
            gimbal_pitch_motor->control.speed_pid->Kp = 7.f;
            gimbal_pitch_motor->control.speed_pid->Ki = 5.f;
            gimbal_pitch_motor->control.speed_pid->Kd = 0.06f;
            gimbal_pitch_motor->control.speed_pid->intergration_limit = 500;
            gimbal_pitch_motor->control.speed_pid->intergration_separation = 400;
            
            gimbal_pitch_motor->control.output_limit = 8000;
        }
    #endif
    }
    /** 
      * @brief  Sundries and initialize
      */
    {
        
    }
}


/**
  * @brief  Enabel each modules
  */
void TASK_Enable() {

    /**
      * @brief  NVIC config
      */
    {
        NVIC_IRQEnable(TIM2_IRQn, 0, 2);            // Friction
        NVIC_IRQEnable(DMA1_Stream5_IRQn, 0, 1);    // Friction pulse capture
        NVIC_IRQEnable(DMA1_Stream6_IRQn, 0, 1);    // Friction pulse capture
        NVIC_IRQEnable(EXTI9_5_IRQn, 1, 3);         // MPU int
        
//        NVIC_IRQEnable(USART1_IRQn, 1, 1);
//        NVIC_IRQEnable(USART2_IRQn, 1, 1);
        NVIC_IRQEnable(USART3_IRQn, 1, 1);
//        NVIC_IRQEnable(UART4_IRQn, 1, 1);
//        NVIC_IRQEnable(UART5_IRQn, 1, 1);
        NVIC_IRQEnable(USART6_IRQn, 1, 1);
        NVIC_IRQEnable(UART7_IRQn, 1, 1);
        NVIC_IRQEnable(UART8_IRQn, 1, 1);
        
//    DMA_Cmd(spDMA_USART1_rx_stream, ENABLE);    
//    NVIC_IRQEnable(TIM6_DAC_IRQn, 2, 1);
    }

}

extern char uart6_buff[256];
/**
  * @brief  System layer control loop in background
  */
void TASK_ControlLooper() {
    
    task_counter++;
    
    static RC_DataType recv;
    static RC_DataType recv_ex;
    
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
          #ifdef USING_TEST42
            static float speed = 0;
            CHASIS_SetMotorSpeed(Motor207, speed);
            CHASIS_SetMotorSpeed(Motor208, -speed);
          #endif
            
          #ifdef USING_TEST17
            static float speed = 0;
            static uint32_t target = 0;
            // static uint32_t posicount = 0;
            static float delta = 1366;
            
            if((recv.key.bit.g^recv_ex.key.bit.g) && recv.key.bit.g){
                if(delta>0)
                    delta = 0;
                else 
                    delta = 1366;
            }
            
            if((recv.key.bit.a^recv_ex.key.bit.a) && recv.key.bit.a){
                speed += 500;
            }
            if((recv.key.bit.d^recv_ex.key.bit.d) && recv.key.bit.d){
                speed -= 500;
            }
            if((recv.key.bit.s^recv_ex.key.bit.s) && recv.key.bit.s){
                speed = 0;
            }
            if((recv.key.bit.r^recv_ex.key.bit.r) && recv.key.bit.r){
                speed = 4000;
            }
            speed = (speed>8000)?8000:speed;
            CHASIS_SetMotorSpeed(Motor206, speed);
            
            if((recv.key.bit.q^recv_ex.key.bit.q) && recv.key.bit.q){
                CHASIS_SetMotorRelativePosition(Motor205, delta);
            }
            if((recv.key.bit.e^recv_ex.key.bit.e) && recv.key.bit.e){
                CHASIS_SetMotorRelativePosition(Motor205, -delta);
            }
            if((recv.key.bit.w^recv_ex.key.bit.w) && recv.key.bit.w){
                CHASIS_SetMotorPosition(Motor205, 0);
            }
            
            if((recv.key.bit.z^recv_ex.key.bit.z) && recv.key.bit.z){
                target += 10;
            }
            if((recv.key.bit.x^recv_ex.key.bit.x) && recv.key.bit.x){
                target -= 10;
            }
            if((recv.key.bit.c^recv_ex.key.bit.c) && recv.key.bit.c){
                target = 0;
            }
            target = (target>200)?200:target;
            
            Friction_Looper(target);
          #endif
        } else if(task_counter%10 == 5) {
          #ifdef USING_CHASIS
            CHASIS_Move(
                recv.rc.ch0=abs(recv.rc.ch0)<20?0:recv.rc.ch0, 
                recv.rc.ch1=abs(recv.rc.ch1)<20?0:recv.rc.ch1, 
                recv.rc.ch2=abs(recv.rc.ch2)<20?0:recv.rc.ch2);
          #endif
          
          #ifdef USING_SENTRY
            int16_t speed = (abs(recv.rc.ch0)<20?0:recv.rc.ch0)*5;
            CHASIS_SetMotorSpeed(Motor201, speed);
            CHASIS_SetMotorSpeed(Motor202, speed);
          #endif
        } else if(task_counter%10 == 7) {
          #ifdef USING_TEST17
            extern PWMFriction_Type Friction_CH1, Friction_CH2;
            printf("%.4f,%.4f\r\n", Friction_CH1.speed[0], Friction_CH2.speed[0]);
          #endif 
          #ifdef USING_SENTRY
            const MOTOR_CrtlType_CAN* motor201 = CHASIS_GetMotor(Motor201);
            const MOTOR_CrtlType_CAN* motor202 = CHASIS_GetMotor(Motor202);
            printf("%d,%d\r\n", motor201->state.speed, motor202->state.speed);
          #endif
        }  else if(task_counter%10 == 9) {
            recv_ex = recv;
        }
        
      #ifdef USING_DM6020
        if(task_counter%5 == 2) {
            extern void sendtoComputer(void);
            sendtoComputer();
        }
        if(task_counter%10 == 8) {
            if(recv.rc.s2==RC_SW_UP) {
                auto_aim_flag = 0; small_power_flag = 0xff;
            } else if(recv.rc.s2==RC_SW_DOWN) {
                auto_aim_flag = 0xff; small_power_flag = 0;
                GIMBAL_Update( recv.rc.ch1 , recv.rc.ch0*4.0f);
            } else {
                GIMBAL_Update(0, 0);
                auto_aim_flag = 0; small_power_flag = 0;
            }
            
            extern frame fram; 
            uint8_t size = sprintf(uart6_buff, "%f,%f\r\n", gimbal_yaw_motor->state.angle, 
                -fram.yaw/0.04394f);
            DMA_Start(spDMA_UART7_tx_stream, (uint32_t)uart6_buff, (uint32_t)&UART7->DR, size);
            
        }
      #endif
        
      #ifdef USING_GM3510Test
        static float pose = 0.f;
        CHASIS_SetMotorPosition(Motor205, pose);
      #endif
      
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

/**
  * @}
  */



/** @defgroup Task Implement Functions
  * @brief    Implement task control by interrupt
  * @{
  */

/**
  * @brief  Systick interrupt handler for task control
  */
void SysTick_Handler(void) {
    RC_ReceiverChecker();
    
    if(!task_inited){
        static uint16_t tick_init = 0;
        tick_init ++;
        
        #ifndef USING_DM6020
        if(tick_init>= 3000) {
            task_inited = true;
            TASK_Enable();
        }
        #else
        if(GIMBAL_MiddleLooper(tick_init)) {
            task_inited = true;
            TASK_Enable();
        }
        #endif
    }else{
        TASK_ControlLooper();
    }
    

    /* System background */
    uint32_t ctime = TASK_GetMicrosecond();
    MOTOR_ControlLooper();
    
    if(ctime%10 == 0) {
    #ifdef USING_DM6020
        GIMBAL_ControlLooper();
    #endif
        CHASIS_ControlLooper();
    }
    CAN1_MsgSendLoop();
    CAN2_MsgSendLoop();
}

/**
  * @}
  */


/** @defgroup System Time Support
  * @brief    Using spCLOCK(default TIM14) as clock source with 1ms(count 1000) period.
  * @{
  */

/**
  * @brief  Counter used in @func TASK_ControlLooper() with period 1ms
  */
static struct {
    volatile uint32_t  ms;
    volatile uint32_t* us;
} spClock = {0, &spCLOCK->CNT};

void spCLOCK_Configuration(void) {
    /* 84MHz */
    spRCC_Set_spCLOCK();
    /* 1000Hz */
    TIM_TimeBaseInitTypeDef tim_base_initer;
    tim_base_initer.TIM_Prescaler = 84-1;
    tim_base_initer.TIM_Period = 1000-1;
    tim_base_initer.TIM_ClockDivision = TIM_CKD_DIV1;
    tim_base_initer.TIM_CounterMode = TIM_CounterMode_Up;
    
    TIM_TimeBaseInit(spCLOCK, &tim_base_initer);
    TIM_ITConfig(spCLOCK, TIM_IT_Update, ENABLE);
    NVIC_IRQEnable(spClock_IRQn, 0, 0);
    TIM_Cmd(spCLOCK, ENABLE);
}

void spClock_IRQHandler(void) {
    if(TIM_GetITStatus(spCLOCK,TIM_IT_Update) == SET) {
        spClock.ms ++;
        TIM_ClearITPendingBit(spCLOCK, TIM_IT_Update);
    }
}

uint32_t TASK_GetMicrosecond(void) {
    return spClock.ms;
}

void TASK_GetMicrosecondPtr(unsigned long *count) {
    count[0] = spClock.ms;
}

spTimeStamp TASK_GetTimeStamp(void) {
    spTimeStamp stamp;
    stamp.us = *spClock.us;
    stamp.ms = spClock.ms;
    return stamp;
}

float TASK_GetSecond(void) {
    return spClock.ms*1.0f/1000 + (*spClock.us)*1.0f/1000000;
}

float TASK_GetSecondFromTimeStamp(spTimeStamp* stamp) {
    return stamp->ms*1.0f/1000 + stamp->us*1.0f/1000000;
}

/**
  * @}
  */

/**
  * @}
  */
/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/

