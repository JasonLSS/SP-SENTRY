/**
  ******************************************************************************
  * @file       main.c
  * @author     YTom
  * @version    v0.0
  * @date       2018.Nov.10
  * @brief      project entry
  ******************************************************************************
  * @license
  *
  ******************************************************************************
  */
  
  /*
    PID 10 12 0.95~0.11 for 203 204
    How can PID has rhythm?
    TODO list
    1.Communication Layer: SPI/IIC/USB
    2.Algoritm Layer: PID/Fuzzy
    3.Hardware Layer: MPU/RC
    4.System Layer: Monitor/TimerScript
    5.Integrating departments / changeable proportion / bang-bang control
        Dalta output limit
    6.PID ctrl_reg
    7.Overtime checker
    
    2006 sp 7 5 0.06 pos 2 0 0
    
    3510(19) sp 5.5 20 0.09 lim is2000 il800 ol 12000  -- no load
             sp 4.0 10 0 lim is2000 il800 ol8000  -- CM control
             po 2 0 0.0035
        (27) sp 3 6 0.07 is2000 im 800 ol8000
             po 4 0 0.04
    3508 CM Hard ^^^^ po 2 0 0
    
    6623 limout 5000 is500 il500 sp-pid 2 0.2 0 using filters
         po-pid -0.7 0 -0.05    xxx
         
    GM3510 po 8.0, 15.0, 0.4 noSPD is100 il400 ol8000
    
    GM6020 pos 8.0f, 0, 0.15f is100 li500 om8000 noSPD
  */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdio.h>

#include "sp_conf.h"
#include "sp_utility.h"
#include "sp_imu.h"


float frame[6];

void Power_Configuration(void)
{
    spRCC_Set_GPIOH();
    GPIO_OUT_Config(GPIOH, GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5, 
        GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_100MHz);
    GPIO_SetBits(GPIOH,GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5);
}

int main(void)
{
    __disable_irq();
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);     /* 4 bits for pre-emption priority
                                                           0 bits for subpriority */
    Power_Configuration();
    spRCC_Set_RNG();    RNG_Cmd(ENABLE);
    TASK_GlobalInit();
    TASK_TimerInit();
    
    /* System init finish signal */
    //BUZZER_ON(1500); 
    delay_ms(500); BUZZER_OFF();
    LED_G_ON();LED_R_OFF();
    TASK_Start();
    
    __enable_irq();
    
    while(1) {
        extern char uart_buff[256];
        const uint32_t ctime = TASK_GetMicrosecond();
        
        if(ctime % 10 == 0) {
//            frame[0] = IMU_Controllers.imu_state.kalman.mag_angle[0];
//            frame[1] = IMU_Controllers.imu_state.kalman.mag_angle[1];
//            frame[2] = IMU_Controllers.imu_state.kalman.mag_angle[2];
//            frame[3] = IMU_Controllers.imu_state.ahrs.gyro[0];
//            frame[4] = IMU_Controllers.imu_state.ahrs.gyro[1];
//            frame[5] = IMU_Controllers.imu_state.ahrs.gyro[2];
//            uint8_t size = sprintf(uart_buff, "%f,%f,%f\r\n", 
//                IMU_Controllers.imu_state.kalman.euler.roll, 
//                IMU_Controllers.imu_state.kalman.euler.pitch,
//                IMU_Controllers.imu_state.kalman.euler.yaw);
//            DMA_Start(spDMA_UART7_tx_stream, (uint32_t)uart_buff, (uint32_t)&UART7->DR, size);
        }
        
        
    }
}

