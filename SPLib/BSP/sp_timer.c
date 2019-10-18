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

const float __prescaler_factor[] = {168, 10, 10, 2, 1.6};

void __calc_psc_period(TIM_TypeDef *Timx, float frequency, uint32_t* psc, uint32_t* per) {
    /* Calculate arrangement value(peroid) and prescaler. */
    if(frequency < __TIMER_MIN_FREQ) {
        frequency = __TIMER_MIN_FREQ;
    } else if(frequency > __TIMER_MAX_FREQ) {
        frequency = __TIMER_MAX_FREQ;
    }
    
    uint32_t bus_freq = SystemCoreClock;
    if(Timx==TIM2  | Timx==TIM3  | Timx==TIM4  | Timx==TIM5  | Timx==TIM6 | \
       Timx==TIM7  | Timx==TIM12 | Timx==TIM13 | Timx==TIM14) {
        bus_freq /= 2;
    }
       
    uint32_t peroid0 = bus_freq / frequency;
    uint32_t prescaler = 1;
    /* NOTE: Only TIM2 and TIM5 is 32-bit conter for STM32F427II. */
    if(Timx!=TIM2 && Timx!=TIM5) {
        uint32_t peroid = peroid0;
        for(int i=0; i<sizeof(__prescaler_factor)/sizeof(__prescaler_factor[0]); ++i) {
            prescaler *= __prescaler_factor[i];
            peroid = peroid0/prescaler;
            if(peroid < 0x10000) {
                *psc = prescaler;
                *per = peroid;
                return;
            }
        }
        *psc = 0x10000;
        *per = peroid0 / (*psc);
    } else {
        *psc = 1;
        *per = peroid0;
    }
}



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
    uint32_t prescaler, peroid;
    __calc_psc_period(Timx, frequency, &prescaler, &peroid);
    
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
    if(channel > 4) return;
    duty = (duty>100.f)?100.f:( (duty<0.f)?0.f:duty );
    ((uint32_t*)(&Timx->CCR1))[channel] = (uint32_t)((Timx->ARR+1)*duty/100.f);
}

float TIM_GetDuty(TIM_TypeDef *Timx, uint8_t channel) {
    if(channel > 4) return 0.f;
    return (((uint32_t*)(&Timx->CCR1))[channel])*1.f/((uint32_t)(Timx->ARR+1))*100.f;
}

void TIM_SetFrequency(TIM_TypeDef *Timx, float frequency, uint8_t channel) {
    
    float duty = TIM_GetDuty(Timx, channel);
    
    uint32_t prescaler, peroid;
    __calc_psc_period(Timx, frequency, &prescaler, &peroid);
    
    TIM_Cmd(Timx, DISABLE);
    Timx->PSC = prescaler - 1;
    Timx->ARR = peroid - 1;
    TIM_Cmd(Timx, ENABLE);
    
    TIM_SetDuty(Timx, channel, duty);
}

bool TIM_PWM_OutputInit(
    TIM_TypeDef* timx, uint8_t channel, const spPinSet* portpin, float freq, float duty)
{
    /* Enbale TIM clock */
    /* NOTE: TIM6 and TIM7 donnot have PWM. */
    uint8_t gpio_af;
    if(timx==TIM1) { gpio_af = GPIO_AF_TIM1; }
    else if(timx==TIM2) { gpio_af = GPIO_AF_TIM2; }
    else if(timx==TIM3) { gpio_af = GPIO_AF_TIM3; }
    else if(timx==TIM4) { gpio_af = GPIO_AF_TIM4; }
    else if(timx==TIM5) { gpio_af = GPIO_AF_TIM5; }
//    else if(timx==TIM6) { gpio_af = GPIO_AF_TIM6; }
//    else if(timx==TIM7) { gpio_af = GPIO_AF_TIM7; }
    else if(timx==TIM8) { gpio_af = GPIO_AF_TIM8; }
    else if(timx==TIM9) { gpio_af = GPIO_AF_TIM9; }
    else if(timx==TIM10) { gpio_af = GPIO_AF_TIM10; }
    else if(timx==TIM11) { gpio_af = GPIO_AF_TIM11; }
    else if(timx==TIM12) { gpio_af = GPIO_AF_TIM12; }
    else if(timx==TIM13) { gpio_af = GPIO_AF_TIM13; }
    else if(timx==TIM14) { gpio_af = GPIO_AF_TIM14; }
    else return false;
    
    /* Config GPIO */
    spGPIO.alternal_config(
        portpin->gpio,
        spGPIO_PinFromPinSource(portpin->pin_source));
    
    /* Config pins */
    GPIO_PinAFConfig(
        portpin->gpio, 
        portpin->pin_source,
        gpio_af );
    
    /* Frequency must near 50Hz */
    if( !(timx->CR1 & TIM_CR1_CEN_Msk) ) {
        spTIMER.init(timx, freq, false);
    } 

    TIM_OCInitTypeDef           TIM_OCInitStructure;
    memset(&TIM_OCInitStructure, 0x00, sizeof(TIM_OCInitStructure));
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    
    switch(channel) {
    case 0:
        TIM_OC1Init(timx, &TIM_OCInitStructure);
        TIM_OC1PreloadConfig(timx, TIM_OCPreload_Enable);
        break;
    case 1:
        TIM_OC2Init(timx, &TIM_OCInitStructure);
        TIM_OC2PreloadConfig(timx, TIM_OCPreload_Enable);
        break;
    case 2:
        TIM_OC3Init(timx, &TIM_OCInitStructure);
        TIM_OC3PreloadConfig(timx, TIM_OCPreload_Enable);
        break;
    case 3:
        TIM_OC4Init(timx, &TIM_OCInitStructure);
        TIM_OC4PreloadConfig(timx, TIM_OCPreload_Enable);
        break;
    }
    
    TIM_ARRPreloadConfig(timx,ENABLE);
    TIM_CtrlPWMOutputs(timx,ENABLE);
    
    TIM_SetDuty(timx, channel, duty);
    
    TIM_Cmd(timx, ENABLE);
    return true;
}



const struct TIMER_Controllers_Type spTIMER = {
    .init = TIM_Init,
    .pwm_init = TIM_PWM_OutputInit,
    .set_duty = TIM_SetDuty,
    .get_duty = TIM_GetDuty,
    .set_frequency = TIM_SetFrequency,
};

/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/
