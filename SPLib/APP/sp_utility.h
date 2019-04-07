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
#define LASER_ON()              GPIO_SetBits(GPIOG,GPIO_Pin_13)
#define LASER_OFF()             GPIO_ResetBits(GPIOG,GPIO_Pin_13)

#if defined(SP_USING_BOARD_TYPEA)
    #define LED_G               spPFOutB.bit14
    #define LED_R               spPEOutB.bit11
    
    #define BUZZER_TIMER        TIM12

    // A-board
    #define LED8_BIT0           GPIO_Pin_1
    #define LED8_BIT1           GPIO_Pin_2
    #define LED8_BIT2           GPIO_Pin_3
    #define LED8_BIT3           GPIO_Pin_4
    #define LED8_BIT4           GPIO_Pin_5
    #define LED8_BIT5           GPIO_Pin_6
    #define LED8_BIT6           GPIO_Pin_7
    #define LED8_BIT7           GPIO_Pin_8
    #define LED8_BIT_ON(x)      GPIO_ResetBits(GPIOG,x)
    #define LED8_BIT_OFF(x)     GPIO_SetBits(GPIOG,x)
    #define LED8_BIT_TOGGLE(x)  GPIOG->ODR ^= (x)
    #define LED8_OUTPUT(x)      GPIOG->ODR = (GPIOG->ODR&(~0x1fe))|((~x&0xff)<<1)
#else
    #define LED_G               spPFOutB.bit14
    #define LED_R               spPEOutB.bit7
    
    #define BUZZER_TIMER        TIM3
#endif

#define LED_G_OFF()             LED_G = true
#define LED_G_ON()              LED_G = false
#define LED_G_TOGGLE()          LED_G = !LED_G

#define LED_R_OFF()             LED_R = true
#define LED_R_ON()              LED_R = false
#define LED_R_TOGGLE()          LED_R = !LED_R

extern float spBeep_MusicalScale[][8];    
#define BUZZER_ON(a)            spTIMER.set_duty(BUZZER_TIMER, 0, 90.f)
#define BUZZER_OFF()            spTIMER.set_duty(BUZZER_TIMER, 0, 0)

#define spEXIT_LineFromPinSource(ln)        (0x0001<<(ln))

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
void spBeep(float frequency, uint32_t delay);

void Led_Configuration(void);
void Led8_Configuration(void);
void Power_Configuration(void);



/**
  * @brief  Keys(RC) and Button(on-board-button)
  * @param  buffer  Data array for CRC checking
  * @param  size    Total length of the data array
  */
typedef struct {
    uint8_t         down:1;
    uint8_t         up:1;
    uint8_t         on:1;
    /* Get current key state. */
    bool (*is_pushing)(void);
} spKeyState_t;
extern spKeyState_t* spKEY_POOL[USING_KEY_POOLSIZE];
/**
  * @brief  Generay Key manager
  * @note   This looper will auto-detect key state by user-provided function and change key state
  */
void Key_Add(spKeyState_t* key);
void Key_Looper(uint32_t tick);

/**
  * @brief  Generay Key manager
  * @note   This part activate when push button on-board
  */
typedef struct {
    spKeyState_t        state;
    float               timestamp;
    spPinSet            gpio_pin;
} spButton_t;
extern spButton_t spUserButton;
void Button_Configuration(void);

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
