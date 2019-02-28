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
    //PB4
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12,ENABLE);        //TIM4时钟使能    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);       //使能PORTB时钟    
    GPIO_PinAFConfig(GPIOH,GPIO_PinSource6,GPIO_AF_TIM12);      //GPIOH9复用为定时器12
    
    /* TIM3 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6  ;                 //GPIOB 6 7
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                //复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;           //速度100MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;              //推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;                //上拉
    GPIO_Init(GPIOH,&GPIO_InitStructure);                       //初始化
      
    TIM_TimeBaseStructure.TIM_Period=2000;          //自动重装载值    
    TIM_TimeBaseStructure.TIM_Prescaler=72-1;       //定时器分频               ///******这里改过
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
    
    TIM_ITConfig(TIM12,TIM_IT_Update,ENABLE);       //允许定时器12更新中断
    TIM_TimeBaseInit(TIM12,&TIM_TimeBaseStructure); //初始化定时器12
    
    //初始化TIM3 Channel1 PWM模式     
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;   //选择定时器模式:TIM脉冲宽度调制模式2
     TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;  //比较输出使能
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High ;      //输出极性:TIM输出比较极性低
    TIM_OC1Init(TIM12, &TIM_OCInitStructure);           //根据T指定的参数初始化外设TIM1 4OC1

    TIM_OC1PreloadConfig(TIM12, TIM_OCPreload_Enable);  //使能TIM4在CCR1上的预装载寄存器
  
    TIM_ARRPreloadConfig(TIM12,ENABLE);                 //ARPE使能 
    TIM_Cmd(TIM12, ENABLE);                             //使能TIM14
    BUZZER_OFF();
}
void Led_Configuration(void) {         
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOF , ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;           //普通输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;          //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;      //100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;            //上拉
    GPIO_Init(GPIOF, &GPIO_InitStructure);                  //初始化

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;              // A-board
    GPIO_Init(GPIOE, &GPIO_InitStructure);                  //初始化

    LED_G_OFF();
    LED_R_ON();
}

void Led8_Configuration(void) {         
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG , ENABLE);

    GPIO_InitStructure.GPIO_Pin = 0xff;                     //0x00~0x80
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;           //普通输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;          //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;      //100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;            //下拉（控制逻辑为正常逻辑）
    GPIO_Init(GPIOG, &GPIO_InitStructure);                  //初始化
    
    LED8_OUTPUT(0x00);
}


void TIM6_Configuration(void) {
    spRCC_Set_TIM6();       /* 84MHz */
    
    TIM_TimeBaseInitTypeDef tim_base_initer;
    /* 200Hz */
    tim_base_initer.TIM_Prescaler = 840-1;
    tim_base_initer.TIM_Period = 500-1;
    tim_base_initer.TIM_ClockDivision = TIM_CKD_DIV1;
    tim_base_initer.TIM_CounterMode = TIM_CounterMode_Up;
    
    TIM_TimeBaseInit(TIM6, &tim_base_initer);
    TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
    
    TIM_Cmd(TIM6, ENABLE);
}





void EXTI2_IRQHandler(void) {
    if(EXTI_GetITStatus(EXTI_Line2)) {
//        if(spUserKey.gpio_pin.gpio->IDR & GPIO_IDR_ID1) {
//            spUserKey.on_release = true;
//        } else {
//            
//        }
        spUserKey.on_press = true;
        EXTI_ClearITPendingBit(EXTI_Line2);
    }
}

spKeyController spUserKey = {
    .on_press = false, .on_release = false, 
    .gpio_pin = {GPIOB, GPIO_PinSource2} };

#define spEXIT_LineFromPinSource(ln)        (0x0001<<(ln))
void KEY_Configuration(void) {
    spRCC_Set_SYSCFG();
    GPIO_IN_Config(spUserKey.gpio_pin.gpio, 
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




PWMFriction_Type    Friction_CH1;
PWMFriction_Type    Friction_CH2;
/**
  * @brief  
  * @note   
  * @param  
  * @retval 
  */ 
void PulseCapture(void) {
    
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG , ENABLE);//
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 ;    //ó?óú?¤1a        //ó?óú?????¤1a
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//??í¨ê?3??￡ê?
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//í?íìê?3?
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//é?à-
    GPIO_Init(GPIOG, &GPIO_InitStructure);//3?ê??ˉ    
    LASER_ON();
    
    TIM_TimeBaseInitTypeDef             TIM_TimeBaseStructure;
    TIM_ICInitTypeDef                   TIM_ICInitStructure;
 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2|RCC_APB1Periph_TIM3,ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_DMA1, ENABLE);

    GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM2);
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_TIM2);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA,&GPIO_InitStructure);
    
    TIM_TimeBaseStructure.TIM_Period=50000-1;   // 20Hz
    TIM_TimeBaseStructure.TIM_Prescaler=84-1;
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);
    
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV4;
    TIM_ICInitStructure.TIM_ICFilter = 0x10;        // Attention
    TIM_ICInit(TIM2, &TIM_ICInitStructure);
    
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
    TIM_ICInit(TIM2, &TIM_ICInitStructure);
    
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_DMACmd(TIM2, TIM_DMA_CC2|TIM_DMA_CC4, ENABLE);
    TIM_SetCounter(TIM2, 0);

//    NVIC_IRQEnable(TIM2_IRQn, 0, 2);
    
    //TODO: TIM2_CH2/4 IRQ
    /* -------------- Configure DMA -----------------------------------------*/
    DMA_InitTypeDef dma;
    DMA_DeInit(DMA1_Stream5);
    dma.DMA_Channel = DMA_Channel_3;    // TIM2_CH1
    dma.DMA_PeripheralBaseAddr = (uint32_t)&(TIM2->CCR1);
    dma.DMA_Memory0BaseAddr = (uint32_t)Friction_CH1.counters;   
    dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
    dma.DMA_BufferSize = sizeof(Friction_CH1.counters)/sizeof(Friction_CH1.counters[0]);
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    dma.DMA_Mode = DMA_Mode_Circular;
    dma.DMA_Priority = DMA_Priority_High;
    dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
    dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    dma.DMA_MemoryBurst = DMA_Mode_Normal;
    dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//    DMA_Init(DMA1_Stream5, &dma);
//    DMA_ITConfig(DMA1_Stream5,DMA_IT_TC,ENABLE);
//    DMA_Cmd(DMA1_Stream5,ENABLE);
    
    DMA_DeInit(DMA1_Stream6);                     
    dma.DMA_Channel = DMA_Channel_3;    // TIM2_CH2
    dma.DMA_PeripheralBaseAddr = (uint32_t)&(TIM2->CCR2);
    dma.DMA_Memory0BaseAddr = (uint32_t)Friction_CH2.counters;   
    dma.DMA_BufferSize = sizeof(Friction_CH2.counters)/sizeof(Friction_CH2.counters[0]);
    DMA_Init(DMA1_Stream6, &dma);
    DMA_ITConfig(DMA1_Stream6,DMA_IT_TC,ENABLE);
    DMA_Cmd(DMA1_Stream6,ENABLE);
    
//    NVIC_IRQEnable(DMA1_Stream5_IRQn, 0, 1);
//    NVIC_IRQEnable(DMA1_Stream6_IRQn, 0, 1);
    
    TIM_Cmd(TIM2, ENABLE);
}

void PWM_Output(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);      //TIM14时钟使能    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI,ENABLE);     //使能PORTF时钟    
    
    GPIO_PinAFConfig(GPIOI,GPIO_PinSource5,GPIO_AF_TIM8); //GPIOF9复用为定时器
    GPIO_PinAFConfig(GPIOI,GPIO_PinSource6,GPIO_AF_TIM8); //GPIOF9复用为定时器
    GPIO_PinAFConfig(GPIOI,GPIO_PinSource7,GPIO_AF_TIM8); //GPIOF9复用为定时器
    GPIO_PinAFConfig(GPIOI,GPIO_PinSource2,GPIO_AF_TIM8); //GPIOF9复用为定时器
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_2;    //GPIOI
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //速度100MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
    GPIO_Init(GPIOI,&GPIO_InitStructure);               //初始化PF9
      
    TIM_TimeBaseStructure.TIM_Prescaler=168*4-1;  //定时器分频
    TIM_TimeBaseStructure.TIM_Period=1000-1;   //自动重装载值
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
    TIM_TimeBaseInit(TIM8,&TIM_TimeBaseStructure);//初始化定时器14
    
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式2
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性低
    TIM_OCInitStructure.TIM_Pulse = 0;
    
    TIM_OC1Init(TIM8, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);

    TIM_OC2Init(TIM8, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
    
    TIM_OC3Init(TIM8, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);
    
    TIM_OC4Init(TIM8, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);
 
    TIM_ARRPreloadConfig(TIM8,ENABLE);//ARPE使能 
    TIM_CtrlPWMOutputs(TIM8,ENABLE);    //For TIM8 PWM
    TIM_Cmd(TIM8, ENABLE);  //使能TIM14
}


void TIM2_IRQHandler(void)      //50ms产生一次中断
{
    if(TIM_GetITStatus(TIM2,TIM_IT_Update) == SET) {
        Friction_CH1.stopping ++;
        if(Friction_CH1.stopping >= 5) {
        #ifdef USING_FRICTION_FILTER
            MovingAverageFilter_f32((float*)Friction_CH1.speed, 
                sizeof(Friction_CH1.speed)/sizeof(Friction_CH1.speed[0]), 0, 15);
        #else
            Friction_CH1.speed[0] = 0;
        #endif
            Friction_CH1.stopping = 5;
        }
        
        
        Friction_CH2.stopping ++;
        if(Friction_CH2.stopping >= 5) {
        #ifdef USING_FRICTION_FILTER
            MovingAverageFilter_f32((float*)Friction_CH2.speed, 
                sizeof(Friction_CH2.speed)/sizeof(Friction_CH2.speed[0]), 0, 15);
        #else
            Friction_CH2.speed[0] = 0;
        #endif
            Friction_CH2.stopping = 5;
        }
        
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}

// 1.615*set+53.87  = 10^6/counter (div8)
float COMPENSATE  = -0.f;
float MAFilter_Threshold = 50.f;
void dmaFrictionUpdata(PWMFriction_Type* friction) {
    uint8_t overflow = 0;
    uint8_t size=sizeof(friction->counters)/sizeof(friction->counters[0])-1;
    
    friction->stopping = 0;
    for(uint8_t i=1; i<size; i++) {
        overflow += (friction->counters[i]<friction->counters[i-1]);
    }
    friction->counter = (friction->counters[size-1] +
        overflow * (TIM2->ARR+1) - friction->counters[0])/(size-1);
    /* Using limited moving average */
    #ifdef USING_FRICTION_FILTER
        MovingAverageFilter_f32((float*)friction->speed, 
                sizeof(friction->speed)/sizeof(friction->speed[0]), 
                (friction->counter>0)? (619195.05f/(2*friction->counter)-33.356f+COMPENSATE):0, 
                MAFilter_Threshold);
    #else
        friction->speed[0] = (friction->counter>0)? (619195.05f/(2*friction->counter)-33.356f+COMPENSATE):0;
    #endif
    friction->changed = 1;
//        friction->counter = counter;
}

float delta_lim = 2;
float dspd_gain = 1.6f, dspd_gain2 = 0.0f;
void looperUpdateFriction(PWMFriction_Type* friction) {
    friction->output[2] = friction->output[1];
    friction->output[1] = friction->output[0];
    
    float d_spd = friction->speed[0] - friction->speed[1];
    float d2_spd = friction->speed[1] - friction->speed[2];
    if(friction->target < 10) {
        friction->output[0] = 0;
        friction->pid.sum_error = 0;
    }
    else if(d_spd < 0) {
        friction->output[0] += fabs(d_spd) * dspd_gain + fabs(d2_spd)*dspd_gain2;
    }
    else {
        friction->output[0] = friction->target;
    }
    
    friction->speed[1] = friction->speed[0];
    friction->speed[2] = friction->speed[1];
    
//    else if(friction->output[0] < friction->target - 25) {
//        friction->output[0] +=  delta_lim;
//    } else {
//        friction->output[0] = friction->target;
//    }
    
//    else if(friction->output[0]<friction->target - 3) {
//        friction->output[0] = friction->target + 10;
//    } else {
//        friction->output[0] = friction->target;
//    }
    
//        // PID_ControllerDriver_Incremental
//        if(friction->speed[0] - friction->target < -3) {
//            friction->output[0] = friction->target*2 - friction->speed[0];
//        } else {
//            friction->output[0] = friction->target;
//        }
////        friction->output[0] = friction->output[0] + PID_ControllerDriver(&friction->pid, 
////            friction->target, friction->speed[0]);
}


//void DMA1_Stream5_IRQHandler(void)
//{
//    if(DMA_GetITStatus(DMA1_Stream5, DMA_IT_TCIF5)) {
//        
//        dmaFrictionUpdata(&Friction_CH1);
//        
//        DMA_ClearFlag(DMA1_Stream5, DMA_IT_TCIF5);
//        DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF5);
//    }
//}

void DMA1_Stream6_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_Stream6, DMA_IT_TCIF6)) {
        
        dmaFrictionUpdata(&Friction_CH2);
        
        DMA_ClearFlag(DMA1_Stream6, DMA_IT_TCIF6);
        DMA_ClearITPendingBit(DMA1_Stream6, DMA_IT_TCIF6);
    }
}


#define Friction_SPEED_p               0.06f
#define Friction_SPEED_i               0.005f
#define Friction_SPEED_d               0.01f
#define Friction_INTE_limitI           60.f
#define DELTA_TIME                     0.01f

void Friction_Init(void) {
    PWM_Output();
    PulseCapture();
    
    PID_ControllerInit(&Friction_CH1.pid, Friction_INTE_limitI, (uint16_t)-1, 160, DELTA_TIME);
    Friction_CH1.pid.Kp = Friction_SPEED_p;
    Friction_CH1.pid.Ki = Friction_SPEED_i;
    Friction_CH1.pid.Kd = Friction_SPEED_d;
    Friction_CH1.pid.intergration_separation = 20.f;
    Friction_CH1.pid.functions.output_filter = MovingAverageFilter_f32;
    
    PID_ControllerInit(&Friction_CH2.pid, Friction_INTE_limitI, (uint16_t)-1, 160, DELTA_TIME);
    Friction_CH2.pid.Kp = Friction_SPEED_p;
    Friction_CH2.pid.Ki = Friction_SPEED_i;
    Friction_CH2.pid.Kd = Friction_SPEED_d;
    Friction_CH2.pid.intergration_separation = 20.f;
    Friction_CH2.pid.functions.output_filter = MovingAverageFilter_f32;
}


uint8_t DELTA_SPEED = 3;
float SPEED_THRESHOLD = 4;
uint32_t check = 0;
void Friction_Looper(uint32_t target) {
    // static uint32_t target = 0, 
    // Clear changed flag
//    Friction_CH1.changed ^= Friction_CH1.changed;
//    Friction_CH2.changed ^= Friction_CH2.changed;
    
    Friction_CH1.target = Friction_CH2.target = target;
    looperUpdateFriction(&Friction_CH1);
    looperUpdateFriction(&Friction_CH2);
    
    TIM_SetCompare1(TIM8, 800-Friction_CH1.output[0] );
    TIM_SetCompare2(TIM8, 800-Friction_CH2.output[0] );
    TIM_SetCompare3(TIM8, check);
    TIM_SetCompare4(TIM8, check);
    
//    TIM_SetCompare1(TIM8, target + 200);
//    TIM_SetCompare2(TIM8, target + 200);
//    printf("%d,%d,%d\r\n", target, Friction_CH1.counter, Friction_CH2.counter);
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/
