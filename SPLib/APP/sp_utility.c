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
#include "sp_chasis.h"
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
    GPIO_PinAFConfig(GPIOH, GPIO_PinSource6, GPIO_AF_TIM12); 
    spGPIO_Controllers.alternal_config(GPIOH, GPIO_Pin_6, GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_100MHz);
    
    TIM_Init(TIM12, 583.3f, false);
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
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3); 
    spGPIO_Controllers.alternal_config(GPIOB, GPIO_Pin_4, GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_100MHz);
    
    TIM_Init(TIM3, 2000, false);
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

void Led_Configuration(void) {
#if defined(SP_USING_BOARD_TYPEA)
    /* PE11 + PF14 */
    spGPIO_Controllers.output_config(GPIOE, GPIO_Pin_11, GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_100MHz);
    spGPIO_Controllers.output_config(GPIOF, GPIO_Pin_14, GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_100MHz);
#else
    /* PF14 + PE7 */
    spGPIO_Controllers.output_config(GPIOF, GPIO_Pin_14, GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_100MHz);
    spGPIO_Controllers.output_config(GPIOE, GPIO_Pin_7, GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_100MHz);
#endif
    
    LED_G_OFF();
    LED_R_ON();
}
void Led8_Configuration(void) {
#if defined(SP_USING_BOARD_TYPEA)
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG , ENABLE);

    GPIO_InitStructure.GPIO_Pin = 0x1fe;                     //0x00~0x80
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;           //普通输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;          //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;      //100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;            //下拉（控制逻辑为正常逻辑）
    GPIO_Init(GPIOG, &GPIO_InitStructure);                  //初始化
    
		spGPIO_Controllers.output_config(GPIOG,GPIO_Pin_13,GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_100MHz);
		GPIO_SetBits(GPIOG,GPIO_Pin_13);
		
    LED8_OUTPUT(0x00);
#endif
}
void Power_Configuration(void) {
#if defined(SP_USING_BOARD_TYPEA)
    spRCC_Set_GPIOH();
    spGPIO_Controllers.output_config(GPIOH, GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5, 
        GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_100MHz);
    GPIO_SetBits(GPIOH,GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5);
#endif
}



//void TIM6_Configuration(void) {
//spRCC_Set_TIM6();       /* 84MHz */

//TIM_TimeBaseInitTypeDef tim_base_initer;
///* 200Hz */
//tim_base_initer.TIM_Prescaler = 840-1;
//tim_base_initer.TIM_Period = 500-1;
//tim_base_initer.TIM_ClockDivision = TIM_CKD_DIV1;
//tim_base_initer.TIM_CounterMode = TIM_CounterMode_Up;

//TIM_TimeBaseInit(TIM6, &tim_base_initer);
//TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);

//TIM_Cmd(TIM6, ENABLE);
//}


void EXTI2_IRQHandler(void) {
    if(EXTI_GetITStatus(EXTI_Line2)) {
        float time = TASK_GetSecond();
        if(time - spUserKey.timestamp > 0.01f) spUserKey.on_press = true;
        spUserKey.timestamp = time;
        EXTI_ClearITPendingBit(EXTI_Line2);
    }
}

spKeyController spUserKey = {
    .on_press = false, .on_release = false, 
    .gpio_pin = {GPIOB, GPIO_PinSource2} };

#define spEXIT_LineFromPinSource(ln)        (0x0001<<(ln))
void KEY_Configuration(void) {
    spRCC_Set_SYSCFG();
    spGPIO_Controllers.input_config(spUserKey.gpio_pin.gpio, 
        spGPIO_PinFromPinSource(spUserKey.gpio_pin.pin_source), 
        GPIO_PuPd_UP, GPIO_Speed_100MHz);
    
    if(spUserKey.gpio_pin.gpio==GPIOA) {
        SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, spUserKey.gpio_pin.pin_source); 
    } else if(spUserKey.gpio_pin.gpio==GPIOB) {
        SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, spUserKey.gpio_pin.pin_source); 
    } else if(spUserKey.gpio_pin.gpio==GPIOC) {
        SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, spUserKey.gpio_pin.pin_source); 
    } else if(spUserKey.gpio_pin.gpio==GPIOD) {
        SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, spUserKey.gpio_pin.pin_source); 
    } else if(spUserKey.gpio_pin.gpio==GPIOE) {
        SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, spUserKey.gpio_pin.pin_source); 
    } else if(spUserKey.gpio_pin.gpio==GPIOF) {
        SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOF, spUserKey.gpio_pin.pin_source); 
    } else if(spUserKey.gpio_pin.gpio==GPIOG) {
        SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOG, spUserKey.gpio_pin.pin_source); 
    }
    #ifdef GPIOH
    else if(spUserKey.gpio_pin.gpio==GPIOH) {
        SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOH, spUserKey.gpio_pin.pin_source); 
    }
    #endif
    #ifdef GPIOI
    else if(spUserKey.gpio_pin.gpio==GPIOI) {
        SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOI, spUserKey.gpio_pin.pin_source); 
    } 
    #endif
    #ifdef GPIOK
    else if(spUserKey.gpio_pin.gpio==GPIOJ) {
        SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOJ, spUserKey.gpio_pin.pin_source); 
    } 
    #endif
    #ifdef GPIOK
    else if(spUserKey.gpio_pin.gpio==GPIOK) {
        SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOK, spUserKey.gpio_pin.pin_source); 
    }
    #endif
    
    EXTI_InitTypeDef            exit_initer;
    exit_initer.EXTI_Line       = spEXIT_LineFromPinSource(spUserKey.gpio_pin.pin_source);
    exit_initer.EXTI_LineCmd    = ENABLE;
    exit_initer.EXTI_Mode       = EXTI_Mode_Interrupt;
    exit_initer.EXTI_Trigger    = EXTI_Trigger_Rising;  // EXTI_Trigger_Rising_Falling;
    EXTI_Init(&exit_initer);
}

uint32_t CRC_CheckSum(uint32_t* buffer, uint16_t size) {
    CRC_ResetDR();
    return CRC_CalcBlockCRC(buffer, size);
}




/*
Unit:
    linear speed -> m/s
    angular speed -> rad/s
    length -> m
*/
struct __CHASIS_Param_Reg CHASIS_Param_Reg = {
    .half_width = 0.185f,
    .half_length = 0.19f,
    .wheel_radius = 0.075f
};
void CHASIS_Mecanum(float spx, float spy, float spyaw, float out_speed[4]) {
    
    out_speed[0] = spMATH_RAD2RPM(
        ( spx - spy - (CHASIS_Param_Reg.half_width+CHASIS_Param_Reg.half_length)*spyaw)/CHASIS_Param_Reg.wheel_radius);
    out_speed[1] = spMATH_RAD2RPM(
        ( spx + spy + (CHASIS_Param_Reg.half_width+CHASIS_Param_Reg.half_length)*spyaw)/CHASIS_Param_Reg.wheel_radius);
    out_speed[2] = spMATH_RAD2RPM(
        ( spx - spy + (CHASIS_Param_Reg.half_width+CHASIS_Param_Reg.half_length)*spyaw)/CHASIS_Param_Reg.wheel_radius);
    out_speed[3] = spMATH_RAD2RPM(
        ( spx + spy - (CHASIS_Param_Reg.half_width+CHASIS_Param_Reg.half_length)*spyaw)/CHASIS_Param_Reg.wheel_radius);

    out_speed[1] = -out_speed[1];
    out_speed[2] = -out_speed[2];
}
void CHASIS_Mecanum_Inv(float speed[4], float* spdx, float *spdy, float *spyaw ) {
    
    speed[1] = -speed[1];
    speed[2] = -speed[2];
    
    if(spdx) *spdx = spMATH_RPM2RAD(
        (speed[0] + speed[1] + speed[2] + speed[3])*CHASIS_Param_Reg.wheel_radius/4.f );
    if(spdy) *spdy = spMATH_RPM2RAD(
        (-speed[0] + speed[1] - speed[2] + speed[3])*CHASIS_Param_Reg.wheel_radius/4.f );
    if(spyaw) *spyaw = spMATH_RPM2RAD(
        (-speed[0] + speed[1] + speed[2] - speed[3])*CHASIS_Param_Reg.wheel_radius/
        (CHASIS_Param_Reg.half_width+CHASIS_Param_Reg.half_length)/4.f );
}

#define __CHASIS_SpeedLimit                 5000
static float fspeed[4][4] = {0};
void CHASIS_Move(float speedX, float speedY, float rad) {
//    float speed[4]={0};
//    CHASIS_Mecanum(speedX, speedY, rad, speed);
//    
//    for(uint8_t i=0; i<4; i++) {
//        if(fabs(speed[i]) > __CHASIS_SpeedLimit) {
//            speed[i] = (speed[i]>0)?__CHASIS_SpeedLimit:-__CHASIS_SpeedLimit;
//        } else if(fabs(speed[i]) < 20.f) {
//            speed[i] = 0;
//        }
//    }
//    
//    memcpy(CHASIS_Param_Reg.state.output, speed, sizeof(speed));
//    
//    CHASIS_SetMotorSpeed(Motor201, speed[0]);
//    CHASIS_SetMotorSpeed(Motor202, speed[1]);
//    CHASIS_SetMotorSpeed(Motor203, speed[2]);
//    CHASIS_SetMotorSpeed(Motor204, speed[3]);
    
    // TODO: Make interval time more specific.
    CHASIS_Param_Reg.target.spdx = speedX;
    CHASIS_Param_Reg.target.spdy = speedY;
    CHASIS_Param_Reg.target.spdyaw = rad;
    
    MOTOR_CrtlType_CAN* motorA = CHASIS_GetMotor(Motor201);
    MOTOR_CrtlType_CAN* motorB = CHASIS_GetMotor(Motor202);
    MOTOR_CrtlType_CAN* motorC = CHASIS_GetMotor(Motor203);
    MOTOR_CrtlType_CAN* motorD = CHASIS_GetMotor(Motor204);
    
    float speed[4];
    // rad/s
    speed[0] = motorA->state.speed;
    speed[1] = motorB->state.speed;
    speed[2] = motorC->state.speed;
    speed[3] = motorD->state.speed;
    CHASIS_Mecanum_Inv(speed, &CHASIS_Param_Reg.state.x, &CHASIS_Param_Reg.state.y, &CHASIS_Param_Reg.state.yaw);
    
    float target[3];
    target[0] = PID_ControllerDriver(&CHASIS_Param_Reg.PID.x, 
        speedX, CHASIS_Param_Reg.state.x);
    target[1] = PID_ControllerDriver(&CHASIS_Param_Reg.PID.y, 
        speedY, CHASIS_Param_Reg.state.y);
    target[2] = PID_ControllerDriver(&CHASIS_Param_Reg.PID.yaw, 
        rad, CHASIS_Param_Reg.state.yaw);
    
    CHASIS_Mecanum(target[0], target[1], target[2], CHASIS_Param_Reg.state.output);
    
    motorA->control.output = CHASIS_Param_Reg.state.output[0];
    motorB->control.output = CHASIS_Param_Reg.state.output[1];
    motorC->control.output = CHASIS_Param_Reg.state.output[2];
    motorD->control.output = CHASIS_Param_Reg.state.output[3];
}



/** 
  * @brief  Config a timer with base counter.
  * @param  Timx: TIM[1~14]
  * @param  frequency: PWM frequency.
  * @param  isstart: If start right now.
  * @rteval If succeed
  */
#include "stm32f4xx_rcc.h"
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

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/
