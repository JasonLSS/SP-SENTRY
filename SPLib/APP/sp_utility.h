/**
  ******************************************************************************
  * @file       sp_utility.h
  * @author     YTom
  * @version    v0.0-alpha
  * @date       2018.Nov.15
  * @brief      project utilities/scripts
  * @usage      
  ******************************************************************************
  * @license
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SP_UTILITY_H
#define __SP_UTILITY_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "sp_pid.h"
#include "sp_conf.h"


/** @addtogroup SP
  * @{
  */

/** @addtogroup Utility
  * @{
  */

/** @defgroup   Definations
  * @note       Utility Exported Macros And Definations
  * @ingroup    Utility
  * @{
  */
  
typedef struct {
    uint8_t             changed:1;
    uint8_t             stopping:4;
    volatile float      speed[3];
    uint16_t            target;
    volatile int16_t    output[3];
    PID_Type            pid;
    volatile uint32_t   counters[3];
    uint32_t            counter;
//    uint32_t        counter;            // for test
} PWMFriction_Type;

extern float speeder;




typedef struct {
    uint8_t         on_press:1;
    uint8_t         on_release:1;
    spPinSet        gpio_pin;
} spKeyController;

extern spKeyController spUserKey;




#define LASER_ON()          GPIO_SetBits(GPIOG,GPIO_Pin_13)
#define LASER_OFF()         GPIO_ResetBits(GPIOG,GPIO_Pin_13)


#define LED_G_OFF()         GPIO_SetBits(GPIOF,GPIO_Pin_14)
#define LED_G_ON()          GPIO_ResetBits(GPIOF,GPIO_Pin_14)
#define LED_G_TOGGLE()      (GPIOF->ODR) ^= GPIO_Pin_14
#define LED_R_OFF()         GPIO_SetBits(GPIOE,GPIO_Pin_11)
#define LED_R_ON()          GPIO_ResetBits(GPIOE,GPIO_Pin_11)
#define LED_R_TOGGLE()      (GPIOE->ODR) ^= GPIO_Pin_11
// A-board
#define	LED8_BIT0           GPIO_Pin_1
#define	LED8_BIT1           GPIO_Pin_2
#define	LED8_BIT2           GPIO_Pin_3
#define	LED8_BIT3           GPIO_Pin_4
#define	LED8_BIT4           GPIO_Pin_5
#define	LED8_BIT5           GPIO_Pin_6
#define	LED8_BIT6           GPIO_Pin_7
#define	LED8_BIT7           GPIO_Pin_8
#define LED8_BIT_ON(x)      GPIO_ResetBits(GPIOG,x)
#define LED8_BIT_OFF(x)     GPIO_SetBits(GPIOG,x)
#define LED8_BIT_TOGGLE(x)  GPIOG->ODR ^= (x)
#define LED8_OUTPUT(x)      GPIOG->ODR = (GPIOG->ODR&(~0x1fe))|((~x&0xff)<<1)

#define BUZZER_ON(a)        TIM_SetCompare1(TIM12, a);
#define BUZZER_OFF()        TIM_SetCompare1(TIM12, 0);

/**
  * @}
  */


/** @defgroup   BasicFunc
  * @note       Utility of System General Basic Functions
  * @ingroup    Utility
  * @{
  */
/**
  * @brief  NVIC interrupt request function enbale
  * @param  irq     IRQ vector number
  * @param  pri     preemption priority
  * @param  subpri  subpriority
  */
extern void NVIC_IRQEnable(uint8_t irq, uint8_t pri, uint8_t subpri);
/**
  * @brief  NVIC interrupt request function enbale
  * @param  irq     IRQ vector number
  */
extern void NVIC_IRQDisable(uint8_t irq);
/**
  * @}
  */


/** @defgroup   BSPInit
  * @note       Utility of General BSP Initializations
  * @ingroup    Utility
  * @{
  */
void Buzzer_Init(void);
void Led_Configuration(void);
void Led8_Configuration(void);
void TIM6_Configuration(void);
void KEY_Configuration(void);
/**
  * @}
  */


/** @defgroup   Algorithm
  * @note       Utility of General Algorithm Functions
  * @ingroup    Utility
  * @{
  */
/**
  * @brief  Make CRC checksum, combination @func CRC_ResetDR() and @func CRC_CalcBlockCRC()
  * @param  buffer  Data array for CRC checking
  * @param  size    Total length of the data array
  */
uint32_t CRC_CheckSum(uint32_t* buffer, uint16_t size);


/**
  * @brief  Distribute speed accroding to Mecanum wheel principle
  * @param  spx     target linear speed in x-axis
  * @param  spy     target linear speed in y-axis
  * @param  spyaw   target angular speed in yaw-axis
  * @param  out_speed    output speed buffer
  */
void CHASIS_Mecanum(float spx, float spy, float spyaw, float out_speed[4]);

/**
  * @brief  Chasis control looper
  * @brief  speedX: chasis linear speed in X-axis
  * @brief  speedY: chasis linear speed in Y-axis
  * @brief  rad: chasis angular speed yaw
  */
void CHASIS_Move(int16_t speedX, int16_t speedY, int16_t rad);

/**
  * @}
  */

/** @defgroup   FrictionOpe
  * @note       Friction Operations
  * @ingroup    Utility
  * @{
  */
void Friction_Init(void);
void Friction_Looper(uint32_t target);
/**
  * @}
  */


/**
  * @}
  */
  
/**
  * @}
  */
  
#ifdef __cplusplus
}
#endif

#endif /*__SP_UTILITY_H */

/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/
