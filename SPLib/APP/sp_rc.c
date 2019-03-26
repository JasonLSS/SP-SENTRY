/**
  ******************************************************************************
  * @file       sp_rc.c
  * @author     YTom
  * @version    v0.0-alpha
  * @date       2018.Nov.27
  * @brief      Romete controller module driver with USART1_RX
  * @usage      
  ******************************************************************************
  * @license
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "sp_rc.h"
#include "sp_utility.h"

/* Private typedef -----------------------------------------------------------*/
#pragma pack(1)
typedef struct {
    uint64_t ch0    :11;
    uint64_t ch1    :11;
    uint64_t ch2    :11;
    uint64_t ch3    :11;
    uint64_t s1     :2;
    uint64_t s2     :2;
    uint64_t x      :16;
    uint64_t y      :16;
    uint64_t z      :16;
    uint64_t lmb    :8;
    uint64_t rmb    :8;
    uint64_t key    :16;
    uint16_t reserve;
} __RC_RawDataType;

/**
  * @brief  Remote controller manager struct
  */ 
typedef struct {
    float                           stamp;                      /*!< Time stamp */
    float                           stamp_ex;                   /*!< Time stamp of last valid data*/
    volatile __RC_RawDataType       raw_rc_data;                /*!< Recerived raw data form remote controll */
    RC_DataType                     cvt_rc_data;                /*!< Converted data */
    uint8_t                         valid:1;                    /*!< If curent frame valid*/
} RC_ManagerType;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define RC_VALIDATE_CHANNEL(val)                (((val)<=RC_CH_VALUE_MAX) && ((val)>=RC_CH_VALUE_MIN))
#define RC_VALIDATE_SWITCH(val)                 (((val)==RC_SW_UP)      || \
                                                 ((val)==RC_SW_MID)     || \
                                                 ((val)==RC_SW_DOWN))

/* Private variables ---------------------------------------------------------*/
RC_ManagerType                      RC_Manager;                 /*!< Remote controll data manager */

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @defgroup Remote Controller Internal Functions
  * @{
  */

/**
  * @brief  Convert raw RC data to user-friendly format
  */ 
void __RC_DataConvert(void) {
    
    if( RC_VALIDATE_CHANNEL(RC_Manager.raw_rc_data.ch0) &&
        RC_VALIDATE_CHANNEL(RC_Manager.raw_rc_data.ch1) &&
        RC_VALIDATE_CHANNEL(RC_Manager.raw_rc_data.ch2) &&
        RC_VALIDATE_CHANNEL(RC_Manager.raw_rc_data.ch3) &&
        RC_VALIDATE_SWITCH(RC_Manager.raw_rc_data.s1) &&
        RC_VALIDATE_SWITCH(RC_Manager.raw_rc_data.s2)) {
        
        RC_Manager.cvt_rc_data.rc.ch0 = RC_Manager.raw_rc_data.ch0 - 1024;
        RC_Manager.cvt_rc_data.rc.ch1 = RC_Manager.raw_rc_data.ch1 - 1024;
        RC_Manager.cvt_rc_data.rc.ch2 = RC_Manager.raw_rc_data.ch2 - 1024;
        RC_Manager.cvt_rc_data.rc.ch3 = RC_Manager.raw_rc_data.ch3 - 1024;
        RC_Manager.cvt_rc_data.rc.s1  = RC_Manager.raw_rc_data.s1;
        RC_Manager.cvt_rc_data.rc.s2  = RC_Manager.raw_rc_data.s2;

        RC_Manager.cvt_rc_data.mouse.x = (int16_t)RC_Manager.raw_rc_data.x;
        RC_Manager.cvt_rc_data.mouse.y = (int16_t)RC_Manager.raw_rc_data.y;

        RC_Manager.cvt_rc_data.mouse.press_l = RC_Manager.raw_rc_data.lmb;
        RC_Manager.cvt_rc_data.mouse.press_r = RC_Manager.raw_rc_data.rmb;
        RC_Manager.cvt_rc_data.key.bits = RC_Manager.raw_rc_data.key;

        RC_Manager.stamp_ex = RC_Manager.cvt_rc_data.stamp = RC_Manager.stamp;
    } else {
        return;
    }
}

void __RC_DataClear(void) {
    
    RC_Manager.cvt_rc_data.rc.ch0 = 0;
    RC_Manager.cvt_rc_data.rc.ch1 = 0;
    RC_Manager.cvt_rc_data.rc.ch2 = 0;
    RC_Manager.cvt_rc_data.rc.ch3 = 0;
    RC_Manager.cvt_rc_data.rc.s1  = 0;
    RC_Manager.cvt_rc_data.rc.s2  = 0;

    RC_Manager.cvt_rc_data.mouse.x = 0;
    RC_Manager.cvt_rc_data.mouse.y = 0;

    RC_Manager.cvt_rc_data.mouse.press_l = 0;
    RC_Manager.cvt_rc_data.mouse.press_r = 0;
    RC_Manager.cvt_rc_data.key.bits = 0;
}

void RC_OnError(RC_InfoType info) {
    RC_Manager.valid = false;
#if defined(SP_USING_BOARD_TYPEA)
    if((RC_InfoType)info == RC_WRONG_DATA) {
        LED8_BIT_ON(LED8_BIT1);
    } else if((RC_InfoType)info == RC_LOSS_SIGNAL) {
        LED8_BIT_ON(LED8_BIT0);
    }
#endif
}

void RC_OnRecovery(void) {
    RC_Manager.valid = true;
#if defined(SP_USING_BOARD_TYPEA)
    LED8_BIT_OFF(LED8_BIT0);
    LED8_BIT_OFF(LED8_BIT1);
#endif
}

void RC_OnBusIdle(void) {
    RC_Manager.stamp = TASK_GetSecond();
    /* Start RC DMA and stop USART_DELE IRQ */
    spDMA_Controllers.controller.reset_counter(spDMA_USART1_rx_stream, sizeof(RC_Manager.raw_rc_data));
    __RC_DataConvert();
}

void RC_ReceiverChecker(void) {
    /* Converted data has the same timestamp with manage means data valid */
    float time = TASK_GetSecond();
    if(time - RC_Manager.stamp > 0.1f) {
        /* Long time not get signal. */
        RC_OnError(RC_LOSS_SIGNAL);
        return;
    } else if(RC_Manager.stamp - RC_Manager.stamp_ex > 0.1f) {
        /* Long time not get valid data. */
        RC_OnError(RC_WRONG_DATA);
        return;
    }
    RC_OnRecovery();
}

bool RC_ReceiverInit(void) {
    if(!USART_RX_Config(USART1, 100000)) {
        return false;
    }
    if(!DMA_USART_RX_Config(USART1, (uint32_t)&RC_Manager.raw_rc_data, sizeof(RC_Manager.raw_rc_data), false)) {
        return false;
    }
    /* Clear all flags before enable */
    spDMA_Controllers.controller.clear_stream_bit(spDMA_USART1_rx_stream, DMA_CRx);
    /* Enable stream interrupt and start DMA */
    DMA_ITConfig(spDMA_USART1_rx_stream, DMA_IT_TC, ENABLE);
    DMA_Cmd(spDMA_USART1_rx_stream, ENABLE);
    
    RC_Manager.stamp = TASK_GetSecond();
    __RC_DataClear();
    
    /* For first frame detecting */
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
    USART_Cmd(USART1, ENABLE);
    
    /* Registe IRQ callbacks */
    spIRQ_Manager.registe(USART1_IRQn, USART_IT_IDLE, RC_OnBusIdle);
    spIRQ_Manager.registe(DMA2_Stream5_IRQn, DMA_IT_TCIF5, NULL);
    
    return true;
}

/**
  * @}
  */


/** @defgroup Remote Controller User Interface
  * @{
  */
uint16_t RC_GetState(RC_DataType* recv) {
    if(RC_Manager.valid) {
        DMA_Stream_TypeDef* dmas = spDMA_Controllers.mem2mem.copy((uint32_t)recv, 
            (uint32_t)&RC_Manager.cvt_rc_data, sizeof(RC_Manager.cvt_rc_data));
        while(spDMA_Controllers.controller.get_stream_bit(dmas, DMA_CR_CTCIFx)==false);
        return sizeof(RC_Manager.raw_rc_data) - dmas->NDTR;
    } else {
        __RC_DataClear();
        return 0;
    }
}
/**
  * @}
  */

/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/
