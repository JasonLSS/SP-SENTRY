/**
  ******************************************************************************
  * @file       template.h
  * @author     YTom
  * @version    v0.0-alpha
  * @date       2018.Oct.21
  * @brief      Romete controller module driver with USART1_RX
  * @usage      
  ******************************************************************************
  * @license
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SP_RC_H
#define __SP_RC_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "sp_conf.h"

/* Exported types ------------------------------------------------------------*/
/**
  * @brief  Remote controller data structure (data frame via USART1_RX)
  */ 
typedef struct {
    struct {
        int16_t         ch0;
        int16_t         ch1;
        int16_t         ch2;
        int16_t         ch3;
        uint8_t         s1:2;
        uint8_t         s2:2;
    } rc;
    struct {
        int16_t         x;
        int16_t         y;
        int16_t         z;
        uint8_t         press_l:1;
        uint8_t         press_r:1;
    } mouse;
    union {
        uint16_t        bits;
        struct {
            uint16_t    w:1;
            uint16_t    s:1;
            uint16_t    a:1;
            uint16_t    d:1;
            uint16_t    shift:1;
            uint16_t    ctrl:1;
            uint16_t    q:1;
            uint16_t    e:1;
            uint16_t    r:1;
            uint16_t    f:1;
            uint16_t    g:1;
            uint16_t    z:1;
            uint16_t    x:1;
            uint16_t    c:1;
            uint16_t    v:1;
            uint16_t    b:1;
        } bit;
    } key;
    uint16_t            resv;
    spTimeStamp         stamp;              /*!< Time stamp */
} RC_DataType;

typedef enum {
    RC_NULL = 0,
    RC_LOSS_SIGNAL,
    RC_WRONG_DATA,
} RC_InfoType;


/* Exported macro ------------------------------------------------------------*/
/**
  * @brief  Remote controller channel value offset
  */ 
#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

/**
  * @brief  Switch value of three-stop switch
  */ 
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)

/**
  * @brief  PC key-value mask
  */ 
#define RC_KEY_W                ((uint16_t)0x01<<0)
#define RC_KEY_S                ((uint16_t)0x01<<1)
#define RC_KEY_A                ((uint16_t)0x01<<2)
#define RC_KEY_D                ((uint16_t)0x01<<3)
#define RC_KEY_SHIFT            ((uint16_t)0x01<<4)
#define RC_KEY_CTRL             ((uint16_t)0x01<<5)
#define RC_KEY_Q                ((uint16_t)0x01<<6)
#define RC_KEY_E                ((uint16_t)0x01<<7)
#define RC_KEY_R                ((uint16_t)0x01<<8)
#define RC_KEY_F                ((uint16_t)0x01<<9)
#define RC_KEY_G                ((uint16_t)0x01<<10)
#define RC_KEY_Z                ((uint16_t)0x01<<11)
#define RC_KEY_X                ((uint16_t)0x01<<12)
#define RC_KEY_C                ((uint16_t)0x01<<13)
#define RC_KEY_V                ((uint16_t)0x01<<14)
#define RC_KEY_B                ((uint16_t)0x01<<15)

/**
  * @brief  PC mouse-value offset
  */ 
#define RC_MOUSE_X_MAX          ((uint16_t)40)
#define RC_MOUSE_Y_MAX          ((uint16_t)40)
#define RC_MONITOR_MAX          ((uint16_t)100)


/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/** @defgroup Remote Controller Initialization and Configuration
  * @{
  */

/**
  * @brief  Initialize remote controller receiver
  * @retval If module initialization succeed
  * @note   Using USART1_RX on A-board
  */ 
bool RC_ReceiverInit(void);


/**
  * @brief  RC state check looper
  * @note   Used in MAIN_LOOPER
  */ 
void RC_ReceiverChecker(void);

/**
  * @}
  */


/** @defgroup Remote Controller Event Callbacks
  * @{
  */

/**
  * @brief  On USART bus idle state
  * @note   Used in IRQ_HADNLER
  */ 
void RC_OnBusIdle(void);

/**
  * @}
  */


/** @defgroup Remote Controller User Interface
  * @{
  */

/**
  * @brief  Get current received remote controller's values 
  * @param  recv    User buffer for RC data of @ref RC_DataType
  * @retval Length of successfully copied data.
  */ 
uint16_t RC_GetState(RC_DataType* recv);
/**
  * @}
  */


#ifdef __cplusplus
}
#endif

#endif /*__SP_RC_H */

/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/
