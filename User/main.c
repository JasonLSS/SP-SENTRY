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

volatile uint8_t                uartx_buff[128];
char                            uart6_buff[256];

void Power_Configuration(void)
{
    spRCC_Set_GPIOH();
    GPIO_OUT_Config(GPIOH, GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5, 
        GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_100MHz);
    GPIO_SetBits(GPIOH,GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5);
}

int main(void)
{
    Power_Configuration();
    spRCC_Set_RNG();    RNG_Cmd(ENABLE);
    TASK_GlobalInit();
    TASK_TimerInit();
    
    NVIC_IRQEnable(CAN1_RX0_IRQn, 0, 1);
    NVIC_IRQEnable(CAN2_RX1_IRQn, 0, 1);
    NVIC_IRQEnable(DMA2_Stream5_IRQn, 0, 3);    // RC
    
    /* System init finish signal */
    BUZZER_ON(1500); delay_ms(500); BUZZER_OFF();
    LED_G_ON();LED_R_OFF();
    TASK_Start();

    while(1) {
        const uint32_t ctime = TASK_GetCounter();
    }
}

