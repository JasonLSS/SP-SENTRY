/**
  ******************************************************************************
  * @file       sp_can.h
  * @author     YTom
  * @version    v0.0-alpha
  * @date       2018.Nov.11
  * @brief      CAN-bus module controller
  @verbatim      
  ==============================================================================
                    ##### About CAN configuration #####
  ==============================================================================
    # Speed Config
    (+)tsjw:����ͬ����Ծʱ�䵥Ԫ. @ref CAN_synchronisation_jump_width    ��Χ: CAN_SJW_1tq ~ CAN_SJW_4tq
    (+)tbs2:ʱ���2��ʱ�䵥Ԫ.    @ref CAN_time_quantum_in_bit_segment_2 ��Χ: CAN_BS2_1tq ~ CAN_BS2_8tq
    (+)tbs1:ʱ���1��ʱ�䵥Ԫ.    @ref CAN_time_quantum_in_bit_segment_1 ��Χ: CAN_BS1_1tq ~ CAN_BS1_16tq
    (+)brp :�����ʷ�Ƶ��.��Χ:1~1024;(ʵ��Ҫ��1,Ҳ����1~1024) tq=(brp)*tpclk1
        ������=Fpclk1/((tsjw+tbs1+tbs2+3)*brp);
    (+)mode: @ref CAN_operating_mode ��Χ��CAN_Mode_Normal,��ͨģʽ;CAN_Mode_LoopBack,�ػ�ģʽ;
    (+)Fpclk1��ʱ���ڳ�ʼ����ʱ������Ϊ36M,�������CAN_Normal_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_LoopBack);
        ������Ϊ:42M/((1+6+7)*6)=500Kbps
        ����ֵ:0,��ʼ��OK;
        ����,��ʼ��ʧ��;
    # Attention
        System time config can affect CAN's communication vary much.
  
  @endverbatim
  ******************************************************************************
  * @license
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SP_CAN_H
#define __SP_CAN_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_can.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "misc.h"
#include <string.h>
#include "sp_type.h"


/* Exported types ------------------------------------------------------------*/
/**
  * @brief  CAN data transmit/receive manager
  */
typedef struct {
    int16_t         std_id;     /*!< Standard CAN id of sender, can be 0 to 0x7FF. */
    void*           owner;      /*!< Pointer to owner of the receiver. */
    struct {
        uint8_t     changed:1;  /*!< If data has changed since last receive */
        uint8_t     size:7;     /*!< Message size for TX/RX in a data frame. */
        uint8_t*    addr;       /*!< Memory address of message data. */
    } rx;
    /**
      * @brief  User-defined data resolving method
      * @param  @ref CanRxMsg*: received messgae struct.
      * @param  @ref void*: pointer of this @ref CAN_Exchanger.rx.addr .
      */ 
    void(*resolver)(CanRxMsg*, void*);
} CAN_Receiver;

typedef struct {
    int16_t         std_id;     /*!< Standard CAN id of sender, can be 0 to 0x7FF. */
    struct {
        uint8_t     changed:1;  /*!< If data has changed since last transmit */
        uint8_t     size:7;     /*!< Message size for TX/RX in a data frame. */
        uint8_t*    addr;       /*!< Memory address of message data. */
    } tx;
//    /**
//      * @brief  Reveive callback
//      * @param  @ref CanRxMsg*: received messgae struct.
//      * @param  @ref void*: pointer of this @ref CAN_Exchanger.rx.addr .
//      */ 
//    void(*call_back)(CanRxMsg*, void*);
} CAN_Transmitter;


/* Exported macro ------------------------------------------------------------*/
#define CAN1_POOLSIZE            8      /*!< How many message receiver can mount on CAN12 */
#define CAN2_POOLSIZE            8      /*!< How many message receiver can mount on CAN12 */


/* Exported parameters -------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/** @defgroup CANx Basic Control Function
  * @brief    Implement basic CAN functions
  * @{
  */

/**
  * @brief  Register a receiver to CAN bus
  * @param  canx: @ref CAN_TypeDef select @arg CAN1 or @arg CAN2 to send message
  * @param  receiver: @ref CAN_Receiver appointed receiver
  * @retval If succeed.
  */ 
bool CAN_RegistReceiver(CAN_TypeDef* canx, CAN_Receiver* receiver);

/**
  * @brief  Register a transmitter to CAN bus
  * @param  canx: @ref CAN_TypeDef select @arg CAN1 or @arg CAN2 to send message
  * @param  transmitter: @ref CAN_Transmitter appointed transmitter
  * @retval If succeed.
  */ 
bool CAN_RegistTransmitter(CAN_TypeDef* canx, CAN_Transmitter* transsmitter);

/**
  * @brief  Submit a change for CAN to transmit
  * @param  transmitter: @ref CAN_Transmitter appointed transmitter
  * @retval If succeed.
  */ 
void CAN_SubmitChange(CAN_Transmitter* transmitter);

/**
  * @brief  Send message via CAN
  * @param  canx: @ref CAN_TypeDef select @arg CAN1 or @arg CAN2 to send message
  * @param  exchanger: @ref CAN_Exchanger appoint an exchanger
  * @note   Only used for send once message.
  */ 
void CAN_SendMsg(CAN_Transmitter* exchanger);


///**
//  * @brief  Send control value to motor via CAN1
//  * @param  id: @ref CAN_MOTORx
//  * @param  candata: -16384~0~16384 stand for -20A~20A control current
//  */ 
//void        CAN1_MotorSend(CAN_MOTORx id, int16_t candata);
///**
//  * @brief  Send control value to motor via CAN2
//  * @param  id: @ref CAN_MOTORx
//  * @param  candata: -16384~0~16384 stand for -20A~20A control current
//  */ 
//void        CAN2_MotorSend(CAN_MOTORx id, int16_t candata);
///**
//  * @brief  CAN2 periodic sending
//  * @param  canx:
//  * @param  id:
//  * @param  candata:
//  */ 
//void        CAN_GeneralSend(CAN_TypeDef *canx, uint16_t id, int16_t candata);
/**
  * @}
  */



/** @defgroup CANx Basic Control Function
  * @brief    Implement basic CAN functions
  * @note     SHOULD NOT call by users.
  * @{
  */
/**
  * @brief  Init CAN1
  * @param  @ref ##### About CAN configuration #####
  * @retval void
  */
void        CAN1_Init(uint8_t tsjw,uint8_t tbs2,uint8_t tbs1,uint16_t brp,uint8_t mode);
/**
  * @brief  CAN1 periodic sending
  * @note   Should be periodicly called
  */ 
void        CAN1_MsgSendLoop(void);
/**
  * @brief  Init CAN2
  * @param  @ref ##### About CAN configuration #####
  * @retval void
  */
void        CAN2_Init(uint8_t tsjw,uint8_t tbs2,uint8_t tbs1,uint16_t brp,uint8_t mode);
/**
  * @brief  CAN2 periodic sending
  * @note   Should be periodicly called
  */ 
void        CAN2_MsgSendLoop(void);

/**
  * @}
  */


#ifdef __cplusplus
}
#endif

#endif /*__SP_CAN_H */

/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/



