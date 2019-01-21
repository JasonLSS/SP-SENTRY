/**
  ******************************************************************************
  * @file       sp_irq.c
  * @author     YTom
  * @version    v0.0-alpha
  * @date       2018.Nov.27
  * @brief      General interrupt request functions
  * @usage      
  ******************************************************************************
  * @license
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "sp_conf.h"
#include <string.h>
#include "mpu6500.h"
#include "sp_utility.h"
#include "sp_kalman.h"
#include "mpu6500.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

uint32_t count = 0;

void EXTI9_5_IRQHandler(void) {
    if(EXTI_GetITStatus(EXTI_Line8)) {

        EXTI_ClearITPendingBit(EXTI_Line8);
    }
}


void DMA1_Stream3_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_Stream3, DMA_IT_TCIF3)) {

        DMA_ClearFlag(DMA1_Stream3, DMA_IT_TCIF3);
        DMA_ClearITPendingBit(DMA1_Stream3, DMA_IT_TCIF3);
    }
}

void DMA2_Stream1_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA2_Stream1, DMA_IT_TCIF1)) {

        DMA_ClearFlag(DMA2_Stream1, DMA_IT_TCIF1);
        DMA_ClearITPendingBit(DMA2_Stream1, DMA_IT_TCIF1);
    }
}

void DMA1_Stream1_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_Stream1, DMA_IT_TCIF1)) {

        DMA_ClearFlag(DMA1_Stream1, DMA_IT_TCIF1);
        DMA_ClearITPendingBit(DMA1_Stream1, DMA_IT_TCIF1);
    }
}



void TIM6_DAC_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM6, TIM_IT_Update)) {

    }
    TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
}


#include "sp_rc.h"
void USART1_IRQHandler (void) {
    if(USART_GetITStatus(USART1, USART_IT_IDLE)) {
        RC_OnBusIdle();
        USART_ClearITPendingBit(USART1, USART_IT_IDLE);
    }
}
void USART2_IRQHandler (void) {
}
void USART3_IRQHandler (void) {
}
void UART4_IRQHandler (void) {
}
void UART5_IRQHandler (void) {
}
void USART6_IRQHandler (void) {
}
void UART7_IRQHandler (void) {
    if(USART_GetITStatus(UART7, USART_IT_IDLE)) {
        
        USART_ClearITPendingBit(UART7, USART_IT_IDLE);
    }
}
void UART8_IRQHandler (void) {
}


/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/
