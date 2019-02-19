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
        
//        static float data[10];
//        IMU_Controllers.operations.read_stream(data+3, data, &temp, data+6);
        float time = TASK_GetSecond();
        float dt = time - IMU_Controllers.imu_state.timestamp;
        IMU_Controllers.imu_state.freq = 1.f/dt;
        // Calculate AHRS
        //TM_AHRSIMU_UpdateAHRS(&IMU_Controllers.imu_state.ahrs, gyro[0], gyro[1], gyro[2],
        //    accel[0], accel[1], accel[2], mag[0], mag[1], mag[2]);
        if(!IMU_Controllers.imu_state.inited) {
//            Madgwick_Init(&IMU_Controllers.imu_state.mad, 0.1f, 200.f);
//            AHRS_init(IMU_Controllers.imu_state.ahrs.q, 
//                IMU_Controllers.imu_state.ahrs.accel, 
//                IMU_Controllers.imu_state.ahrs.mag);
            IMU_Controllers.imu_state.inited = true;
        } else {
            KalmanFilter(&IMU_Controllers.imu_state.kalman,
                IMU_Controllers.imu_state.ahrs.gyro,
                IMU_Controllers.imu_state.ahrs.accel, 
                IMU_Controllers.imu_state.ahrs.mag, dt);
//            AHRS_update(IMU_Controllers.imu_state.ahrs.q, 
//                IMU_Controllers.imu_state.freq,
//                IMU_Controllers.imu_state.ahrs.gyro, 
//                IMU_Controllers.imu_state.ahrs.accel, 
//                IMU_Controllers.imu_state.ahrs.mag);
//            get_angle(IMU_Controllers.imu_state.ahrs.q, &IMU_Controllers.imu_state.ahrs.y, 
//                &IMU_Controllers.imu_state.ahrs.p, &IMU_Controllers.imu_state.ahrs.r);
//            Madgwick_update(&IMU_Controllers.imu_state.mad,
//                IMU_Controllers.imu_state.ahrs.gyro[0],
//                IMU_Controllers.imu_state.ahrs.gyro[1],
//                IMU_Controllers.imu_state.ahrs.gyro[2],
//                IMU_Controllers.imu_state.ahrs.accel[0],
//                IMU_Controllers.imu_state.ahrs.accel[1],
//                IMU_Controllers.imu_state.ahrs.accel[2],
//                IMU_Controllers.imu_state.ahrs.mag[0],
//                IMU_Controllers.imu_state.ahrs.mag[1],
//                IMU_Controllers.imu_state.ahrs.mag[2]);
//            Madgwick_computeAngles(&IMU_Controllers.imu_state.mad);
        }
        // Make log
        IMU_Controllers.imu_state.timestamp = time;
        IMU_Controllers.imu_state.count ++;
        
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

        TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
    }
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
        
        spIRQ_Manager.invoke(UART7_IRQn);
        
        USART_ClearITPendingBit(UART7, USART_IT_IDLE);
    }
}
void UART8_IRQHandler (void) {
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

spIRQ_CbUnit_t* IRQ_Regeiste(IRQn_Type irq, IRQ_Callback_t cb) {
    /* Malloc a new callback-unit from pool */
    spIRQ_CbUnit_t* pCb;
    for(uint16_t i=0; i<sizeof(spIRQ_CbPool)/sizeof(spIRQ_CbPool[0]); i++) {
        if(spIRQ_CbPool[i].irq_type == (IRQn_Type)-1) {
            pCb = &spIRQ_CbPool[i];
            break;
        }
    }
    
    pCb->irq_type = irq;
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

void IRQ_Invoke(IRQn_Type irq) {
    
    spIRQ_CbUnit_t* pCurrCb = spIRQ_CbEntries[irq];
    while(pCurrCb) {
        if(pCurrCb->callback) {
            pCurrCb->callback();
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
