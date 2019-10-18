/**
  ******************************************************************************
  * @file       sp_servo.c
  * @author     YTom
  * @version    v0.0-alpha
  * @date       2018.Nov.27
  * @brief      Servo control
  * @usage      
  ******************************************************************************
  * @license
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "sp_servo.h"
#include "sp_math.h"
#include "sp_utility.h"

/* Private typedef -----------------------------------------------------------*/
#define SERVO_RESOLVE_DUTY(duty)            ((duty-0.075f)/0.1f*180.f)
#define SERVO_RESOLVE_ANGLE(angle)          ((angle)/180.f*0.1f+0.075f)

/**
  * @brief  
  * @note   
  * @param  
  * @retval 
  */ 
void PWM_OutputInit(ServoType* servo)
{
    /* Enbale TIM clock */
    /* NOTE: TIM6 and TIM7 donnot have PWM. */
    uint8_t gpio_af;
    if(servo->control.timx==TIM1) { gpio_af = GPIO_AF_TIM1; }
    else if(servo->control.timx==TIM2) { gpio_af = GPIO_AF_TIM2; }
    else if(servo->control.timx==TIM3) { gpio_af = GPIO_AF_TIM3; }
    else if(servo->control.timx==TIM4) { gpio_af = GPIO_AF_TIM4; }
    else if(servo->control.timx==TIM5) { gpio_af = GPIO_AF_TIM5; }
//    else if(servo->control.timx==TIM6) { gpio_af = GPIO_AF_TIM6; }
//    else if(servo->control.timx==TIM7) { gpio_af = GPIO_AF_TIM7; }
    else if(servo->control.timx==TIM8) { gpio_af = GPIO_AF_TIM8; }
    else if(servo->control.timx==TIM9) { gpio_af = GPIO_AF_TIM9; }
    else if(servo->control.timx==TIM10) { gpio_af = GPIO_AF_TIM10; }
    else if(servo->control.timx==TIM11) { gpio_af = GPIO_AF_TIM11; }
    else if(servo->control.timx==TIM12) { gpio_af = GPIO_AF_TIM12; }
    else if(servo->control.timx==TIM13) { gpio_af = GPIO_AF_TIM13; }
    else if(servo->control.timx==TIM14) { gpio_af = GPIO_AF_TIM14; }
    else return;
    
    /* Config GPIO */
    spGPIO.alternal_config(
        servo->control.portpin.gpio,
        spGPIO_PinFromPinSource(servo->control.portpin.pin_source));
    
    /* Config pins */
    GPIO_PinAFConfig(
        servo->control.portpin.gpio, 
        servo->control.portpin.pin_source,
        gpio_af );
    
    /* Frequency must near 50Hz */
    if( !(servo->control.timx->CR1 & TIM_CR1_CEN_Msk) ) {
        spTIMER.init(servo->control.timx, 50, false);
    } 

    TIM_OCInitTypeDef           TIM_OCInitStructure;
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_Pulse = SERVO_RESOLVE_ANGLE(servo->param.offset)*(servo->control.timx->ARR+1);
    
    switch(servo->control.channel) {
    case 0:
        TIM_OC1Init(servo->control.timx, &TIM_OCInitStructure);
        TIM_OC1PreloadConfig(servo->control.timx, TIM_OCPreload_Enable);
        break;
    case 1:
        TIM_OC2Init(servo->control.timx, &TIM_OCInitStructure);
        TIM_OC2PreloadConfig(servo->control.timx, TIM_OCPreload_Enable);
        break;
    case 2:
        TIM_OC3Init(servo->control.timx, &TIM_OCInitStructure);
        TIM_OC3PreloadConfig(servo->control.timx, TIM_OCPreload_Enable);
        break;
    case 3:
        TIM_OC4Init(servo->control.timx, &TIM_OCInitStructure);
        TIM_OC4PreloadConfig(servo->control.timx, TIM_OCPreload_Enable);
        break;
    }
    
    TIM_ARRPreloadConfig(servo->control.timx,ENABLE);
    TIM_CtrlPWMOutputs(servo->control.timx,ENABLE);
    TIM_Cmd(servo->control.timx, ENABLE);
    return;
}


/* User operations */
void SERVO_SetTarget(ServoType* servo, float target) {
    servo->param.target = limit_minmax( target, servo->param.lower_bound, servo->param.higher_bound) + servo->param.offset;
}

void SERVO_SetSpeed(ServoType* servo, uint16_t speed) {
    servo->param.speed = speed;
}

void SERVO_SetTargetWithSpeed(ServoType* servo, float target, uint16_t speed) {
    SERVO_SetTarget(servo, target);
    servo->param.speed = speed;
}

/**
 *  \param[low] lower bound of real angle
 *  \param[high] higher bound of real angle
 */
ServoType* SERVO_GetServo(
    TIM_TypeDef* timx, uint8_t channel, GPIO_TypeDef* gpio, uint32_t pin_source,
    float offset, float low, float high ) {
    ServoType* servo = NULL;
    
    uint16_t i;
    for(i=0; i<sizeof(ServoPool)/sizeof(*ServoPool); i++) {
        if(ServoPool[i].control.timx == NULL) {
            servo = &ServoPool[i];
            break;
        }
    }
    if(servo) {
        servo->param.offset = offset;
        servo->param.lower_bound = low;
        servo->param.higher_bound = high;
        
        servo->control.timx = timx;
        servo->control.channel = channel;
        servo->control.portpin.gpio = gpio;
        servo->control.portpin.pin_source = pin_source;
        
        PWM_OutputInit(servo);
    }
    
    SERVO_SetTarget(servo, 0);
    
    return servo;
}

/* This angle stands for dealing data inner servo controller */
float __SERVO_GetInnerAngle(ServoType* servo) {
    servo->param.current = SERVO_RESOLVE_DUTY(
        ((uint32_t*)(&servo->control.timx->CCR1))[servo->control.channel]*1.f/(servo->control.timx->ARR+1) );// 
    return servo->param.current;
}

void SERVO_Spinner(ServoType* servo) {
    float time = TASK_GetSecond();
    
    if(servo->control.channel<4) {
        
        /* Calculate output */
        float output = __SERVO_GetInnerAngle(servo);
        
        /* Speed control */
        if(fabs(servo->param.target - servo->param.current) > 0.1f ) {        // Not get target
            float dt = time - servo->control.time_stamp;
            /* When not get target */
            if(servo->param.target > servo->param.current)
                output = servo->param.current + dt * servo->param.speed;
            else 
                output = servo->param.current - dt * servo->param.speed;
        } else {
            output = servo->param.target;
        }
    
        //TODO:
        /* Restore timer register */
        ((uint32_t*)(&servo->control.timx->CCR1))[servo->control.channel] = 
            SERVO_RESOLVE_ANGLE(output)*(servo->control.timx->ARR+1);
    }
    
    servo->control.time_stamp = time;
}

void SERVO_SimpleSpinner(ServoType* servo) {
    float time = TASK_GetSecond();
    if(servo->control.channel<4) {
        /* Calculate output */
        float output = servo->param.target;
        //TODO:
        /* Restore timer register */
        ((uint32_t*)(&servo->control.timx->CCR1))[servo->control.channel] = 
            SERVO_RESOLVE_ANGLE(output)*(servo->control.timx->ARR+1);
    }
    servo->control.time_stamp = time;
}

/* This angle reflect real angle */
float SERVO_GetReadAngle(ServoType* servo) {
    return __SERVO_GetInnerAngle(servo) - servo->param.offset;
}

/* System operations */
void SERVO_Init(void) {
    memset(ServoPool, 0x00, sizeof(ServoPool));
    for(uint16_t i=0; i<sizeof(ServoPool)/sizeof(*ServoPool); i++) {
        ServoPool[i].param.speed = (uint16_t)-1;
    }
}


ServoType ServoPool[USING_SERVO_POOL_SIZE];

struct __SERVO_Manager_Type spSERVO = {
    ._system = {
        .init = SERVO_Init,
    }, 
    .user = {
        .get_servo = SERVO_GetServo,
        .set_target = SERVO_SetTarget,
        .set_speed = SERVO_SetSpeed,
        .set_target_with_speed = SERVO_SetTargetWithSpeed,
        .get_current_angle = SERVO_GetReadAngle,
        .spin = SERVO_Spinner,
        .spin_simple = SERVO_SimpleSpinner
    }
};

/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/
