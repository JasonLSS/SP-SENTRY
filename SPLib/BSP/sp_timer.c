/**
  ******************************************************************************
  * @file       sp_timer.c
  * @author     YTom
  * @version    v0.0-alpha
  * @date       2018.Nov.25
  * @brief      General GPIO config module
  ******************************************************************************
  * @license
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "sp_timer.h"


bool TIM_Init(
    TIM_TypeDef *Timx, 
    float frequency,
    bool isstart)
{
    /* If timer is not being ocupied. */
    if(Timx->CR1 & TIM_CR1_CEN_Msk)
        return false;
    /* Enbale TIM clock */
    if(Timx==TIM1) {
        spRCC_Set_TIM1();
    }
    else if(Timx==TIM2) {
        spRCC_Set_TIM2();
    }
    else if(Timx==TIM3) {
        spRCC_Set_TIM3();
    }
    else if(Timx==TIM4) {
        spRCC_Set_TIM4();
    }
    else if(Timx==TIM5) {
        spRCC_Set_TIM5();
    }
    else if(Timx==TIM6) {
        spRCC_Set_TIM6();
    }
    else if(Timx==TIM7) {
        spRCC_Set_TIM7();
    }
    else if(Timx==TIM8) {
        spRCC_Set_TIM8();
    }
    else if(Timx==TIM9) {
        spRCC_Set_TIM9();
    }
    else if(Timx==TIM10) {
        spRCC_Set_TIM10();
    }
    else if(Timx==TIM11) {
        spRCC_Set_TIM11();
    }
    else if(Timx==TIM12) {
        spRCC_Set_TIM12();
    }
    else if(Timx==TIM13) {
        spRCC_Set_TIM13();
    }
    else if(Timx==TIM14) {
        spRCC_Set_TIM14();
    }
    else
        return false;
    
    /* Calculate arrangement value(peroid) and prescaler. */
    uint32_t bus_freq = SystemCoreClock;
    
    if(Timx==TIM2  | Timx==TIM3  | Timx==TIM4  | Timx==TIM5  | Timx==TIM6 | \
       Timx==TIM7  | Timx==TIM12 | Timx==TIM13 | Timx==TIM14) {
        bus_freq /= 2;
    }
    uint32_t peroid = bus_freq / frequency;
    uint32_t prescaler = 1;
    /* NOTE: Only TIM2 and TIM5 is 32-bit conter for STM32F427II. */
    if(peroid > 0x10000 && Timx!=TIM2 && Timx!=TIM5) {
        prescaler = bus_freq / 1000000;
        peroid /= prescaler;
    }
    TIM_TimeBaseInitTypeDef     tim_initstruct;
    tim_initstruct.TIM_Prescaler            =   prescaler - 1;
    tim_initstruct.TIM_Period               =   peroid - 1;
    tim_initstruct.TIM_CounterMode          =   TIM_CounterMode_Up;
    tim_initstruct.TIM_ClockDivision        =   TIM_CKD_DIV1; 
    TIM_TimeBaseInit(Timx, &tim_initstruct);
    
    if(isstart) TIM_Cmd(Timx, ENABLE);
    return true;
}

void TIM_SetDuty(TIM_TypeDef *Timx, uint8_t channel, float duty) {
    if(channel > 2) return;
    duty = (duty>100.f)?100.f:( (duty<0.f)?0.f:duty );
    ((uint32_t*)(&Timx->CCR1))[channel] = (uint32_t)((Timx->ARR+1)*duty/100.f);
}

float TIM_GetDuty(TIM_TypeDef *Timx, uint8_t channel) {
    if(channel > 2) return 0.f;
    return (((uint32_t*)(&Timx->CCR1))[channel])*1.f/((uint32_t)(Timx->ARR+1))*100.f;
}

void TIM_SetFrequency(TIM_TypeDef *Timx, float frequency, uint8_t channel) {
    
    float duty = TIM_GetDuty(Timx, channel);
    
    /* Calculate arrangement value(peroid) and prescaler. */
    uint32_t bus_freq = SystemCoreClock;
    
    if(Timx==TIM2  | Timx==TIM3  | Timx==TIM4  | Timx==TIM5  | Timx==TIM6 | \
       Timx==TIM7  | Timx==TIM12 | Timx==TIM13 | Timx==TIM14) {
        bus_freq /= 2;
    }
    uint32_t peroid = bus_freq / frequency;
    uint32_t prescaler = 1;
    /* NOTE: Only TIM2 and TIM5 is 32-bit conter for STM32F427II. */
    if(peroid > 0x10000 && Timx!=TIM2 && Timx!=TIM5) {
        prescaler = bus_freq / 1000000;
        peroid /= prescaler;
    }
    
    TIM_Cmd(Timx, DISABLE);
    Timx->PSC = prescaler - 1;
    Timx->ARR = peroid - 1;
    TIM_Cmd(Timx, ENABLE);
    
    TIM_SetDuty(Timx, channel, duty);
}




const struct TIMER_Controllers_Type spTIMER = {
    .init = TIM_Init,
    .set_duty = TIM_SetDuty,
    .get_duty = TIM_GetDuty,
    .set_frequency = TIM_SetFrequency,
};

/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/
