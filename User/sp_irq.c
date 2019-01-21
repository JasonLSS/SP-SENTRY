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



void DMA1_Stream5_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_Stream5, DMA_IT_TCIF5)) {
        
        DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF5);
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

#include "Auto_aim.h"
// View-USART2
static uint8_t __view_buffer[128];
UsartBuffer_t view_buffer = {
    .buffer = __view_buffer,
    .size = 128,
    .curr_ptr = 0,
    .last_ptr = 0
};
void USART2_IRQHandler (void) {
    if(USART_GetITStatus(USART2, USART_IT_IDLE)) {
//        uint16_t size;
//        uint8_t buffer[128];
//        view_buffer.curr_ptr = view_buffer.size - spDMA_USART2_rx_stream->NDTR;
//        if(view_buffer.curr_ptr > view_buffer.last_ptr) {
//            size = view_buffer.curr_ptr - view_buffer.last_ptr;
//            DMA_CopyMem2Mem(
//                (uint32_t)buffer, 
//                (uint32_t)(&view_buffer.buffer[view_buffer.last_ptr]), 
//                size);
//        } else if(view_buffer.curr_ptr < view_buffer.last_ptr) {
//            size = view_buffer.size - view_buffer.last_ptr;
//            DMA_CopyMem2Mem(
//                (uint32_t)buffer, 
//                (uint32_t)(&view_buffer.buffer[view_buffer.last_ptr]), 
//                size);
//            DMA_CopyMem2Mem(
//                (uint32_t)(&buffer[size]), 
//                (uint32_t)(view_buffer.buffer), 
//                view_buffer.curr_ptr);
//            size += view_buffer.curr_ptr;
//        }
//        view_buffer.last_ptr = view_buffer.curr_ptr;
    
        uint16_t size = view_buffer.size - spDMA_USART2_rx_stream->NDTR;
        DMA_Restart(spDMA_USART2_rx_stream, (uint32_t)view_buffer.buffer, 
             (uint32_t)&USART2->DR, view_buffer.size);
        Auto_aim(view_buffer.buffer, size);
        
        USART_ClearITPendingBit(USART2, USART_IT_IDLE);
    }
}
void USART3_IRQHandler (void) {
}
void UART4_IRQHandler (void) {
}
void UART5_IRQHandler (void) {
}


extern uint8_t referee_buffer[128];
void Referee_OnBusIdle(void) {
    DMA_Start(spDMA_USART6_rx_stream, (uint32_t)&USART6->DR, (uint32_t)referee_buffer, sizeof(referee_buffer));
}
void USART6_IRQHandler (void) {
    if(USART_GetITStatus(USART6, USART_IT_IDLE)) {
        Referee_OnBusIdle();
        USART_ClearITPendingBit(USART6, USART_IT_IDLE);
    }
}
void UART7_IRQHandler (void) {
    if(USART_GetITStatus(UART7, USART_IT_IDLE)) {
        
        USART_ClearITPendingBit(UART7, USART_IT_IDLE);
    }
}
void UART8_IRQHandler (void) {
}


/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/
