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
    (+)tsjw:重新同步跳跃时间单元. @ref CAN_synchronisation_jump_width    范围: CAN_SJW_1tq ~ CAN_SJW_4tq
    (+)tbs2:时间段2的时间单元.    @ref CAN_time_quantum_in_bit_segment_2 范围: CAN_BS2_1tq ~ CAN_BS2_8tq
    (+)tbs1:时间段1的时间单元.    @ref CAN_time_quantum_in_bit_segment_1 范围: CAN_BS1_1tq ~ CAN_BS1_16tq
    (+)brp :波特率分频器.范围:1~1024;(实际要加1,也就是1~1024) tq=(brp)*tpclk1
        波特率=Fpclk1/((tsjw+tbs1+tbs2+3)*brp);
    (+)mode: @ref CAN_operating_mode 范围：CAN_Mode_Normal,普通模式;CAN_Mode_LoopBack,回环模式;
    (+)Fpclk1的时钟在初始化的时候设置为36M,如果设置CAN_Normal_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_LoopBack);
        则波特率为:42M/((1+6+7)*6)=500Kbps
        返回值:0,初始化OK;
        其他,初始化失败;
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


/** @addtogroup SP
  * @brief      SuperPower
  * @{
  */

/** @defgroup CAN 
  * @brief    CAN Module
  * @{
  */

#include "stm32f4xx.h"
#include "stm32f4xx_can.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "misc.h"
#include <string.h>
#include "sp_type.h"


/** @defgroup CNA_Definations
  * @brief    Exported CANx Macros And Definations 
  * @ingroup  CAN
  * @{
*/
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

#define CAN1_POOLSIZE            8      /*!< How many message receiver can mount on CAN12 */
#define CAN2_POOLSIZE            8      /*!< How many message receiver can mount on CAN12 */
/** @} */


/** @defgroup CNA_Declarations
  * @brief    Exported CANx Function Declarations
  * @ingroup  CAN
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
/** @} */


/** @defgroup CNA_Control
  * @brief    Exported CANx Basic Control Function
  * @ingroup  CAN
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
/** @} */


/**
  * @}
  */

/**
  * @}
  */


#ifdef __cplusplus
}
#endif

#endif /*__SP_CAN_H */
