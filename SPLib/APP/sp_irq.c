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


#include "sp_irq.h"

void EXTI0_IRQHandler(void) {
    spIRQ.invoke(EXTI0_IRQn, (void*)EXTI_Line0,
        (spIRQ_GetITStatus)EXTI_GetITStatus, 
        (spIRQ_ClearPending)EXTI_ClearITPendingBit);
}

void EXTI9_5_IRQHandler(void) {
    spIRQ.invoke(EXTI9_5_IRQn, (void*)EXTI_Line8, 
        (spIRQ_GetITStatus)EXTI_GetITStatus, 
        (spIRQ_ClearPending)EXTI_ClearITPendingBit);
}

void DMA2_Stream5_IRQHandler(void) {
    spIRQ.invoke(DMA2_Stream5_IRQn, DMA2_Stream5, 
        (spIRQ_GetITStatus)DMA_GetITStatus, 
        (spIRQ_ClearPending)DMA_ClearITPendingBit);
}

void DMA2_Stream1_IRQHandler(void) {
    spIRQ.invoke(DMA2_Stream1_IRQn, DMA2_Stream1, 
        (spIRQ_GetITStatus)DMA_GetITStatus, 
        (spIRQ_ClearPending)DMA_ClearITPendingBit);
}

void USART1_IRQHandler (void) {
    spIRQ.invoke(USART1_IRQn, USART1, 
        (spIRQ_GetITStatus)USART_GetITStatus, 
        (spIRQ_ClearPending)USART_ClearITPendingBit);
}
void USART2_IRQHandler (void) {
    spIRQ.invoke(USART2_IRQn, USART2, 
        (spIRQ_GetITStatus)USART_GetITStatus, 
        (spIRQ_ClearPending)USART_ClearITPendingBit);
}
void USART3_IRQHandler (void) {
    spIRQ.invoke(USART3_IRQn, USART3, 
        (spIRQ_GetITStatus)USART_GetITStatus, 
        (spIRQ_ClearPending)USART_ClearITPendingBit);
}
void UART4_IRQHandler (void) {
    spIRQ.invoke(UART4_IRQn, UART4, 
        (spIRQ_GetITStatus)USART_GetITStatus, 
        (spIRQ_ClearPending)USART_ClearITPendingBit);
}
void UART5_IRQHandler (void) {
    spIRQ.invoke(UART5_IRQn, UART5, 
        (spIRQ_GetITStatus)USART_GetITStatus, 
        (spIRQ_ClearPending)USART_ClearITPendingBit);
}
void USART6_IRQHandler (void) {
    spIRQ.invoke(USART6_IRQn, USART6, 
        (spIRQ_GetITStatus)USART_GetITStatus, 
        (spIRQ_ClearPending)USART_ClearITPendingBit);
}
void UART7_IRQHandler (void) {
    spIRQ.invoke(UART7_IRQn, UART7, 
        (spIRQ_GetITStatus)USART_GetITStatus, 
        (spIRQ_ClearPending)USART_ClearITPendingBit);
}
void UART8_IRQHandler (void) {
    spIRQ.invoke(UART8_IRQn, UART8, 
        (spIRQ_GetITStatus)USART_GetITStatus, 
        (spIRQ_ClearPending)USART_ClearITPendingBit);
}




void TIM1_TRG_COM_TIM11_IRQHandler(void) {
    spIRQ.invoke(TIM1_TRG_COM_TIM11_IRQn, TIM11, 
        (spIRQ_GetITStatus)TIM_GetITStatus, 
        (spIRQ_ClearPending)TIM_ClearITPendingBit);
}


void TIM8_BRK_TIM12_IRQHandler(void) {
    spIRQ.invoke(TIM8_BRK_TIM12_IRQn, TIM12, 
        (spIRQ_GetITStatus)TIM_GetITStatus, 
        (spIRQ_ClearPending)TIM_ClearITPendingBit);
}

void TIM8_UP_TIM13_IRQHandler(void) {
    spIRQ.invoke(TIM8_UP_TIM13_IRQn, TIM13, 
        (spIRQ_GetITStatus)TIM_GetITStatus, 
        (spIRQ_ClearPending)TIM_ClearITPendingBit);
//    if(TIM_GetITStatus(TIM14,TIM_IT_Update) == SET) {
//        TASK_Backend();
//        TIM_ClearITPendingBit(TIM14, TIM_IT_Update);
//    }
}

void TIM8_TRG_COM_TIM14_IRQHandler(void) {
    spIRQ.invoke(TIM8_TRG_COM_TIM14_IRQn, TIM14, 
        (spIRQ_GetITStatus)TIM_GetITStatus, 
        (spIRQ_ClearPending)TIM_ClearITPendingBit);
}




//TODO: Manager Core IRQs
/* IQR Manager --------------------------------------------------------------------*/

spIRQ_CbUnit_t                   spIRQ_CbPool[USING_IRQ_POOL_SIZE];
spIRQ_CbUnit_t*                  spIRQ_CbEntries[91];

void IRQ_ManagerInit(void) {
    memset(spIRQ_CbEntries, NULL, sizeof(spIRQ_CbEntries)/sizeof(spIRQ_CbEntries[0]));
    for(uint16_t i=0; i<sizeof(spIRQ_CbPool)/sizeof(spIRQ_CbPool[0]); i++) {
        spIRQ_CbPool[i] = spIRQ_CbNull;
    }
}

spIRQ_CbUnit_t* IRQ_Regeiste(IRQn_Type irq, uint32_t it_flag, IRQ_Callback_t cb) {
    if(!cb) return NULL;
    if(irq<0) return NULL;
    
    /* Malloc a new callback-unit from pool */
    spIRQ_CbUnit_t* pCb;
    for(uint16_t i=0; i<sizeof(spIRQ_CbPool)/sizeof(spIRQ_CbPool[0]); i++) {
        if(spIRQ_CbPool[i].irq_type == spIRQ_CbNull.irq_type) {
            pCb = &spIRQ_CbPool[i];
            break;
        }
    }
    
    pCb->irq_type = irq;
    pCb->interrupt_request_flag = it_flag;
    pCb->callback = cb;
    /* Add it to corresponding callback list */
    if(spIRQ_CbEntries[irq] == NULL) {
        spIRQ_CbEntries[irq] = pCb;
    } else {
        spIRQ_CbUnit_t* pCurrCb = spIRQ_CbEntries[irq];
        while(pCurrCb->next) pCurrCb = pCurrCb->next;
        pCurrCb->next = pCb;
    }
    return pCb;
}

void IRQ_Invoke(IRQn_Type irq, void* peripheral, 
    spIRQ_GetITStatus get_it_status,
    spIRQ_ClearPending clear_pending) {
    
    spIRQ_CbUnit_t* pCurrCb = spIRQ_CbEntries[irq];
    /* Invoke callback */
    while(pCurrCb) {
        if(get_it_status && get_it_status(peripheral, pCurrCb->interrupt_request_flag)) {
            if(pCurrCb->callback) pCurrCb->callback();
        }
        pCurrCb = pCurrCb->next;
    }
    /* Clear pending flag */
    pCurrCb = spIRQ_CbEntries[irq];
    while(pCurrCb) {
        if(clear_pending) clear_pending(peripheral, pCurrCb->interrupt_request_flag);
        pCurrCb = pCurrCb->next;
    }
}


struct __IRQ_Manager_Type spIRQ = {
    .init = IRQ_ManagerInit,
    .registe = IRQ_Regeiste,
    .invoke = IRQ_Invoke,
};


/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/
