/**
  ******************************************************************************
  * @file       sp_utility.c
  * @author     YTom
  * @version    v0.0-alpha
  * @date       2018.Oct.21
  * @brief      project utilities/scripts
  * @usage      
  ******************************************************************************
  * @license
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "sp_utility.h"
#include "sp_filter.h"


/** @addtogroup SP
  * @{
  */

/** @defgroup   Utility
  * @{
  */


#define USING_FRICTION_FILTER                    /*<! Using input filter */


__INLINE void NVIC_IRQEnable(uint8_t irq, uint8_t pri, uint8_t subpri) {
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = (irq);
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = (pri);
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = (subpri);
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

__INLINE void NVIC_IRQDisable(uint8_t irq) {
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = (irq);
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}




void Buzzer_Init(void) {
#if defined(SP_USING_BOARD_TYPEA)
    /* TIM12CH1 + PH6 */
    spGPIO.alternal_config(GPIOH, GPIO_Pin_6, GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_100MHz);
    GPIO_PinAFConfig(GPIOH, GPIO_PinSource6, GPIO_AF_TIM12); 
    
    spTIMER.init(TIM12, 1000, false);
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_Pulse = 0;
    
    TIM_OC1Init(TIM12, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM12, TIM_OCPreload_Enable);
    
    TIM_ARRPreloadConfig(TIM12,ENABLE);
    TIM_CtrlPWMOutputs(TIM12, ENABLE);
    TIM_Cmd(TIM12, ENABLE);
    
    BUZZER_OFF();
#else
    /* TIM3CH1 + PB4 */
    spGPIO.alternal_config(GPIOB, GPIO_Pin_4, GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_100MHz);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3); 
    
    spTIMER.init(TIM3, 2000, false);
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_Pulse = 0;
    
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    
    TIM_ARRPreloadConfig(TIM3,ENABLE);
    TIM_CtrlPWMOutputs(TIM3, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
    
    BUZZER_OFF();
#endif
}
float spBeep_MusicalScale[][8] = {
//      C       D       E       F       G       A       B     
    {0, 16.35,  18.35,  20.60,  21.83,  24.50,  27.50,  30.87 },    // 0
    {0, 32.70,  36.71,  41.20,  43.65,  49.00,  55.00,  61.74 },    // 1
    {0, 65.41,  73.42,  82.41,  87.31,  98.00,  110.00, 123.48},    // 2
    {0, 130.81, 146.83, 164.81, 174.61, 196.00, 220.00, 246.94},    // 3
    {0, 261.63, 293.66, 329.63, 349.23, 392.00, 440.00, 493.88},    // 4
    {0, 523.25, 587.33, 659.25, 698.46, 783.99, 880.00, 987.77},    // 5
    {0, 1046.5, 1174.7, 1318.5, 1396.9, 1568.0, 1760.0, 1975.5},    // 6
    {0, 2093.0, 2349.3, 2637.0, 2793.8, 3136.0, 3520.0, 3951.1},    // 7
    {0, 4186.0, 4698.6, 5274.0, 5587.6, 6271.9, 7040.0, 7902.1},    // 8
};
/*!< f=frequency, d=delay units [each 1ms] */
void spBeep(float f, uint32_t d) {
    spTIMER.set_frequency(BUZZER_TIMER, (f), 0); \
    spTIMER.set_duty(BUZZER_TIMER, 0, 90.f); delay_ms(d); \
    spTIMER.set_duty(BUZZER_TIMER, 0, 0);delay_ms(10);
}



void Led_Configuration(void) {
#if defined(SP_USING_BOARD_TYPEA)
    /* PE11 + PF14 */
    spGPIO.output_config(GPIOE, GPIO_Pin_11, GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_100MHz);
    spGPIO.output_config(GPIOF, GPIO_Pin_14, GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_100MHz);
#else
    /* PF14 + PE7 */
    spGPIO.output_config(GPIOF, GPIO_Pin_14, GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_100MHz);
    spGPIO.output_config(GPIOE, GPIO_Pin_7, GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_100MHz);
#endif
    
    /* Laser output */
    spGPIO.output_config(GPIOG, GPIO_Pin_13, GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_100MHz);
    
    LED_G_OFF();
    LED_R_ON();
    LASER_OFF();
}
void Led8_Configuration(void) {
#if defined(SP_USING_BOARD_TYPEA)
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG , ENABLE);

    GPIO_InitStructure.GPIO_Pin = 0xFF<<1;                  //0x00~0x80
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;           //普通输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;          //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;      //100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;            //下拉（控制逻辑为正常逻辑）
    GPIO_Init(GPIOG, &GPIO_InitStructure);                  //初始化
    
    LED8_OUTPUT(0x00);
#endif
}
void Power_Configuration(void) {
#if defined(SP_USING_BOARD_TYPEA)
    spRCC_Set_GPIOH();
    spGPIO.output_config(GPIOH, GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5, 
        GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_100MHz);
    GPIO_SetBits(GPIOH,GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5);
#endif
}


bool getUserButton(void);
spButton_t spUserButton = {
    .state = {false, false, false, .is_pushing=getUserButton},
    .timestamp = 0.f,
#if defined(SP_USING_BOARD_TYPEA)
    .gpio_pin = {GPIOB, GPIO_PinSource2} };
#else
    .gpio_pin = {GPIOD, GPIO_PinSource10} };
#endif

bool getUserButton(void) {
   return !GPIO_ReadInputDataBit(spUserButton.gpio_pin.gpio, spGPIO_PinFromPinSource(spUserButton.gpio_pin.pin_source));
}
    
//void KEY_Callback(void) {
//    float time = TASK_GetSecond();
//    if(time - spUserButton.timestamp > 0.01f) spUserButton.on_press = true;
//    spUserButton.timestamp = time;
//}

void Button_Configuration(void) {
    spRCC_Set_SYSCFG();
    spGPIO.input_config(spUserButton.gpio_pin.gpio, 
        spGPIO_PinFromPinSource(spUserButton.gpio_pin.pin_source), 
        GPIO_PuPd_UP, GPIO_Speed_100MHz);
    
    if(spUserButton.gpio_pin.gpio==GPIOA) {
        SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, spUserButton.gpio_pin.pin_source); 
    } else if(spUserButton.gpio_pin.gpio==GPIOB) {
        SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, spUserButton.gpio_pin.pin_source); 
    } else if(spUserButton.gpio_pin.gpio==GPIOC) {
        SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, spUserButton.gpio_pin.pin_source); 
    } else if(spUserButton.gpio_pin.gpio==GPIOD) {
        SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, spUserButton.gpio_pin.pin_source); 
    } else if(spUserButton.gpio_pin.gpio==GPIOE) {
        SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, spUserButton.gpio_pin.pin_source); 
    } else if(spUserButton.gpio_pin.gpio==GPIOF) {
        SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOF, spUserButton.gpio_pin.pin_source); 
    } else if(spUserButton.gpio_pin.gpio==GPIOG) {
        SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOG, spUserButton.gpio_pin.pin_source); 
    }
    #ifdef GPIOH
    if(spUserButton.gpio_pin.gpio==GPIOH) {
        SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOH, spUserButton.gpio_pin.pin_source); 
    }
    #endif
    #ifdef GPIOI
    if(spUserButton.gpio_pin.gpio==GPIOI) {
        SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOI, spUserButton.gpio_pin.pin_source); 
    } 
    #endif
    #ifdef GPIOK
    if(spUserButton.gpio_pin.gpio==GPIOJ) {
        SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOJ, spUserButton.gpio_pin.pin_source); 
    } 
    #endif
    #ifdef GPIOK
    if(spUserButton.gpio_pin.gpio==GPIOK) {
        SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOK, spUserButton.gpio_pin.pin_source); 
    }
    #endif
    
    EXTI_InitTypeDef            exit_initer;
    exit_initer.EXTI_Line       = spEXIT_LineFromPinSource(spUserButton.gpio_pin.pin_source);
    exit_initer.EXTI_LineCmd    = ENABLE;
    exit_initer.EXTI_Mode       = EXTI_Mode_Interrupt;
    exit_initer.EXTI_Trigger    = EXTI_Trigger_Rising;  // EXTI_Trigger_Rising_Falling;
    EXTI_Init(&exit_initer);
    
    Key_Add(&spUserButton.state);
//    spIRQ.registe(EXTI15_10_IRQn, NULL, KEY_Callback);
}


spKeyState_t* spKEY_POOL[USING_KEY_POOLSIZE] = {NULL};
void Key_Add(spKeyState_t* key) {
    key->down = key->on = key->up = false;
    for(int i=0; i<sizeof(spKEY_POOL)/sizeof(spKEY_POOL[0]); ++i) {
        if(spKEY_POOL[i]==0x00) {
            spKEY_POOL[i] = key;
            return;
        }
    }
}

/* Using 20Hz frequecy for key freshing */
void Key_Looper(uint32_t tick) {
    if(tick % 50) {
        spKeyState_t** key = spKEY_POOL;
        bool _cstate = false;
        while(*key) {
            if( (*key)->is_pushing ) {
                _cstate = (*key)->is_pushing();
            }
            
            bool _lstate = (*key)->down || (*key)->on;      // Whether last checkpoint is pushing
            if( _lstate ) {
                if(_cstate) {                               // Pushing -> pushing
                    (*key)->on = true;
                    (*key)->down = false;
                    (*key)->up = false;
                } else {                                    // Pushing -> not pushing
                    (*key)->up = true;
                    (*key)->on = false;
                    (*key)->down = false;
                }
            } else {
                if(_cstate) {                               // Not pushing -> pushing
                    (*key)->down = true;
                    (*key)->on = false;
                    (*key)->up = false;
                } else {
                    (*key)->down = false;
                    (*key)->on = false;
                    (*key)->up = false;
                }
            }
            
            ++key;
        }
    }
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/
