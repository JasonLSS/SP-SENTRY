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

#include <string.h>
#include "sp_imu.h"
#include "sp_utility.h"

#include "sp_imu.h"

void EXTI9_5_IRQHandler(void) {
    if(EXTI_GetITStatus(EXTI_Line8)) {
        // Read IMU
        //mag: -37.8 37.2 42.5
        IMU_Controllers.operations.read_stream(
            IMU_Controllers.imu_state.ahrs.gyro, 
            IMU_Controllers.imu_state.ahrs.accel_0,
            &IMU_Controllers.imu_state.ahrs.temp, 
            IMU_Controllers.imu_state.ahrs.mag);
        
        if(IMU_Controllers.imu_state.kalman.pass_filter.lpf_enbale) {
            IMU_Controllers.imu_state.ahrs.accel[0] = 
                LPF_FirstOrder_filter(IMU_Controllers.imu_state.kalman.pass_filter.lpf+0,
                IMU_Controllers.imu_state.ahrs.accel_0[0]);
            IMU_Controllers.imu_state.ahrs.accel[1] = 
                LPF_FirstOrder_filter(IMU_Controllers.imu_state.kalman.pass_filter.lpf+1,
                IMU_Controllers.imu_state.ahrs.accel_0[1]);
            IMU_Controllers.imu_state.ahrs.accel[2] = 
                LPF_FirstOrder_filter(IMU_Controllers.imu_state.kalman.pass_filter.lpf+2,
                IMU_Controllers.imu_state.ahrs.accel_0[2]);
        }
        
        float time = TASK_GetSecond();
        float dt = time - IMU_Controllers.imu_state.timestamp;
        IMU_Controllers.imu_state.freq = 1.f/dt;
        if(!IMU_Controllers.imu_state.inited) {
            IMU_Controllers.imu_state.inited = true;
        } else {
            KalmanFilter(&IMU_Controllers.imu_state.kalman,
                IMU_Controllers.imu_state.ahrs.gyro,
                IMU_Controllers.imu_state.ahrs.accel, 
                IMU_Controllers.imu_state.ahrs.mag, dt);
        }
        // Make log
        IMU_Controllers.imu_state.timestamp = time;
        IMU_Controllers.imu_state.count ++;
        
        EXTI_ClearITPendingBit(EXTI_Line8);
    }
}

void DMA2_Stream5_IRQHandler(void) {
    spIRQ_Manager.invoke(DMA2_Stream5_IRQn, DMA2_Stream5, 
        (spIRQ_GetITStatus)DMA_GetITStatus, 
        (spIRQ_ClearPending)DMA_ClearITPendingBit);
}

void USART1_IRQHandler (void) {
    spIRQ_Manager.invoke(USART1_IRQn, USART1, 
        (spIRQ_GetITStatus)USART_GetITStatus, 
        (spIRQ_ClearPending)USART_ClearITPendingBit);
}
void USART2_IRQHandler (void) {
    spIRQ_Manager.invoke(USART2_IRQn, USART2, 
        (spIRQ_GetITStatus)USART_GetITStatus, 
        (spIRQ_ClearPending)USART_ClearITPendingBit);
}
void USART3_IRQHandler (void) {
    spIRQ_Manager.invoke(USART3_IRQn, USART3, 
        (spIRQ_GetITStatus)USART_GetITStatus, 
        (spIRQ_ClearPending)USART_ClearITPendingBit);
}
void UART4_IRQHandler (void) {
    spIRQ_Manager.invoke(UART4_IRQn, UART4, 
        (spIRQ_GetITStatus)USART_GetITStatus, 
        (spIRQ_ClearPending)USART_ClearITPendingBit);
}
void UART5_IRQHandler (void) {
    spIRQ_Manager.invoke(UART5_IRQn, UART5, 
        (spIRQ_GetITStatus)USART_GetITStatus, 
        (spIRQ_ClearPending)USART_ClearITPendingBit);
}
void USART6_IRQHandler (void) {
    spIRQ_Manager.invoke(USART6_IRQn, USART6, 
        (spIRQ_GetITStatus)USART_GetITStatus, 
        (spIRQ_ClearPending)USART_ClearITPendingBit);
}
void UART7_IRQHandler (void) {
    spIRQ_Manager.invoke(UART7_IRQn, UART7, 
        (spIRQ_GetITStatus)USART_GetITStatus, 
        (spIRQ_ClearPending)USART_ClearITPendingBit);
}
void UART8_IRQHandler (void) {
    spIRQ_Manager.invoke(UART8_IRQn, UART8, 
        (spIRQ_GetITStatus)USART_GetITStatus, 
        (spIRQ_ClearPending)USART_ClearITPendingBit);
}




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
    
    /* Malloc a new callback-unit from pool */
    spIRQ_CbUnit_t* pCb;
    for(uint16_t i=0; i<sizeof(spIRQ_CbPool)/sizeof(spIRQ_CbPool[0]); i++) {
        if(spIRQ_CbPool[i].irq_type == (IRQn_Type)-1) {
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
    while(pCurrCb) {
        if(get_it_status(peripheral, pCurrCb->interrupt_request_flag)) {
            if(pCurrCb->callback) {
                pCurrCb->callback();
            }
            clear_pending(peripheral, pCurrCb->interrupt_request_flag);
        }
        pCurrCb = pCurrCb->next;
    }
}


struct __IRQ_Manager_Type spIRQ_Manager = {
    .init = IRQ_ManagerInit,
    .registe = IRQ_Regeiste,
    .invoke = IRQ_Invoke,
};


/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/
