/**
  ******************************************************************************
  * @file       sp_can.c
  * @author     @YangTianhao, 490999282@qq.com; @TangJiaxin, tjx1024@126.com; @YTom, ybb331082@126.com
  * @version    v0.0-alpha
  * @date       2018.Nov.11
  * @brief      CAN-bus module controller
  * @note       BaudRate:   1Mbps
  *             PinMap:     PB13(tx) PB12(rx)
  ******************************************************************************
  * @license
  *
  ******************************************************************************
  */

#include "sp_can.h"


/**
  * @brief  CAN1/CAN2 exchangers
  */
CAN_Receiver*                   __can1_receivers[CAN1_POOLSIZE] = {0x00};
CAN_Receiver*                   __can2_receivers[CAN2_POOLSIZE] = {0x00};
CAN_Transmitter*                __can1_transmitters[CAN1_POOLSIZE] = {0x00};
CAN_Transmitter*                __can2_transmitters[CAN2_POOLSIZE] = {0x00};


/**
  * @brief  CAN1 receive interrupt handler
  */
void CAN1_RX0_IRQHandler(void) {
    
    if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET) {
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
        
        /* Receive can data from CAN peripheral */
        CanRxMsg __can1_rx_message;
        CAN_Receive(CAN1, CAN_FIFO0, &__can1_rx_message);
        
        uint8_t i, size=sizeof(__can1_receivers)/sizeof(__can1_receivers[0]);
        for(i=0; i<size; i++) {
            if(__can1_receivers[i]->std_id == __can1_rx_message.StdId) {
                if(__can1_receivers[i]->rx.size){
                    /* Using user-defined resolver function to resolve data */
                    if(__can1_receivers[i]->resolver) {
                        __can1_receivers[i]->resolver(&__can1_rx_message, __can1_receivers[i]->owner);
                    }
                    __can1_receivers[i]->rx.changed = true;
                }
                break;
            }
        }
    }
}


/**
  * @brief  CAN2 receive interrupt handler
  */
void CAN2_RX1_IRQHandler(void) {
    
    if (CAN_GetITStatus(CAN2,CAN_IT_FMP1)!= RESET) {
        CAN_ClearITPendingBit(CAN2, CAN_IT_FMP1);
        
        /* Receive can data from CAN peripheral */
        CanRxMsg __can2_rx_message;
        CAN_Receive(CAN2, CAN_FIFO1, &__can2_rx_message);
        
        uint8_t i, size=sizeof(__can2_receivers)/sizeof(__can2_receivers[0]);
        for(i=0; i<size; i++) {
            if(__can2_receivers[i]->std_id == __can2_rx_message.StdId) {
                if(__can2_receivers[i]->rx.size){
                    /* Using user-defined resolver function to resolve data */
                    __can2_receivers[i]->rx.changed = true;
                    if(__can2_receivers[i]->resolver) {
                        __can2_receivers[i]->resolver(&__can2_rx_message, __can2_receivers[i]->owner);
                    }
                }
                break;
            }
        }
    }
}

/**
  * @brief  Send message via CAN
  * @param  canx: @ref CAN_TypeDef select @arg CAN1 or @arg CAN2 to send message
  * @param  exchanger: @ref CAN_Exchanger appoint an exchanger
  * @note   Only used for send once message. INNER use;
  */ 
static inline bool __CAN_SendMsg(CAN_TypeDef* canx, CAN_Transmitter* exchanger){

    CanTxMsg TxMessage;
    
    TxMessage.StdId = exchanger->std_id;
    TxMessage.IDE = CAN_Id_Standard;
    TxMessage.RTR = CAN_RTR_Data;
    TxMessage.DLC = exchanger->tx.size;
    memcpy(TxMessage.Data, exchanger->tx.addr, exchanger->tx.size);    
    return CAN_Transmit(canx, &TxMessage) != CAN_TxStatus_NoMailBox;
}






void CAN1_Init(uint8_t tsjw,uint8_t tbs2,uint8_t tbs1,uint16_t brp,uint8_t mode) {
    
    GPIO_InitTypeDef            GPIO_InitStructure; 
    CAN_InitTypeDef             CAN_InitStructure;
    CAN_FilterInitTypeDef       CAN_FilterInitStructure;

    /* Reset variables */
    memset(__can1_receivers, 0x00, sizeof(__can1_receivers));
    memset(__can1_transmitters, 0x00, sizeof(__can1_transmitters));

    //?????????
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//???PORTA??? 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//???CAN1???

    //?????GPIO
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0| GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    //??????????????
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource0,GPIO_AF_CAN1);
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource1,GPIO_AF_CAN1);
        
    //CAN???????
    CAN_InitStructure.CAN_TTCM=DISABLE;
    CAN_InitStructure.CAN_ABOM=ENABLE;
    CAN_InitStructure.CAN_AWUM=DISABLE;
    CAN_InitStructure.CAN_NART=ENABLE;
    CAN_InitStructure.CAN_RFLM=DISABLE;
    CAN_InitStructure.CAN_TXFP=DISABLE;
    CAN_InitStructure.CAN_Mode= mode;
    CAN_InitStructure.CAN_SJW=tsjw;
    CAN_InitStructure.CAN_BS1=tbs1;
    CAN_InitStructure.CAN_BS2=tbs2;
    CAN_InitStructure.CAN_Prescaler=brp;
    CAN_Init(CAN1, &CAN_InitStructure);

    //???¨´?????
    CAN_FilterInitStructure.CAN_FilterNumber=0;
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
    CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32¦ËID
    CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32¦ËMASK
    CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//??????0??????FIFO0
    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);//??????????
    CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0???????§Ø?????.            
  
//      NVIC_IRQEnable(CAN1_RX0_IRQn, 0, 1);
}

void CAN2_Init(uint8_t tsjw,uint8_t tbs2,uint8_t tbs1,uint16_t brp,uint8_t mode) { 
    
    GPIO_InitTypeDef            GPIO_InitStructure; 
    CAN_InitTypeDef             CAN_InitStructure;
    CAN_FilterInitTypeDef       CAN_FilterInitStructure;

    /* Reset variables */
    memset(__can2_receivers, 0x00, sizeof(__can2_receivers));
    memset(__can2_transmitters, 0x00, sizeof(__can2_transmitters));

    //?????????
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//???GPIOB???
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);//???CAN2???

    //?????GPIO
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12| GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//???¨´???
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//???????
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//????
    GPIO_Init(GPIOB, &GPIO_InitStructure);//?????PA11,PA12

    //??????????????
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource12,GPIO_AF_CAN2);
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_CAN2);
        
    //CAN???????
    CAN_InitStructure.CAN_TTCM=DISABLE;
    CAN_InitStructure.CAN_ABOM=ENABLE;
    CAN_InitStructure.CAN_AWUM=DISABLE;
    CAN_InitStructure.CAN_NART=ENABLE;
    CAN_InitStructure.CAN_RFLM=DISABLE;
    CAN_InitStructure.CAN_TXFP=DISABLE;
    CAN_InitStructure.CAN_Mode= mode;
    CAN_InitStructure.CAN_SJW=tsjw;
    CAN_InitStructure.CAN_BS1=tbs1;
    CAN_InitStructure.CAN_BS2=tbs2;
    CAN_InitStructure.CAN_Prescaler=brp;
    CAN_Init(CAN2, &CAN_InitStructure);

    //???¨´?????
    CAN_FilterInitStructure.CAN_FilterNumber=14;
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
    CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;
    CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO1;
    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);

    CAN_ITConfig(CAN2,CAN_IT_FMP1,ENABLE);//FIFO1???????§Ø?????.            
  
//      NVIC_IRQEnable(CAN2_RX1_IRQn, 0, 1);
}



void spCAN_Init(CAN_TypeDef* canx, uint8_t tsjw,uint8_t tbs2,uint8_t tbs1,uint16_t brp,uint8_t mode) {
    
    assert_param(canx == CAN1 || canx == CAN2);
    
    if(canx == CAN1) {
        CAN1_Init(tsjw, tbs2, tbs1, brp, mode);
    } else if(canx == CAN2) {
        CAN2_Init(tsjw, tbs2, tbs1, brp, mode);
    }
}

void spCAN_MsgSendLoop(CAN_TypeDef* canx)
{
    assert_param(canx == CAN1 || canx == CAN2);
    
    if(canx == CAN1) {
        uint8_t i, size=sizeof(__can1_transmitters)/sizeof(__can1_transmitters[0]);
        for(i=0; i<size; i++) {
            if(__can1_transmitters[i] && __can1_transmitters[i]->tx.changed) {
                __CAN_SendMsg(CAN1, __can1_transmitters[i]);
                /* Clear flag */
                __can1_transmitters[i]->tx.changed = false;
            }
        }
    } else if(canx == CAN2) {
        uint8_t i, size=sizeof(__can2_transmitters)/sizeof(__can2_transmitters[0]);
        for(i=0; i<size; i++) {
            if(__can2_transmitters[i] && __can2_transmitters[i]->tx.changed) {
                __CAN_SendMsg(CAN2, __can2_transmitters[i]);
                /* Clear flag */
                __can2_transmitters[i]->tx.changed = false;
            }
        }
    }
}



bool CAN_RegistReceiver(CAN_TypeDef* canx, CAN_Receiver* receiver) {
    if(canx == CAN1) {
        uint8_t i, size=sizeof(__can1_receivers)/sizeof(__can1_receivers[0]);
        for(i=0; i<size; i++) {
            if(!__can1_receivers[i]) {
                __can1_receivers[i] = receiver;
                return true;
            }
        }
    } else if(canx == CAN2) {
        uint8_t i, size=sizeof(__can2_receivers)/sizeof(__can2_receivers[0]);
        for(i=0; i<size; i++) {
            if(!__can2_receivers[i]) {
                __can2_receivers[i] = receiver;
                return true;
            }
        }
    }
    return false;
}

bool CAN_RegistTransmitter(CAN_TypeDef* canx, CAN_Transmitter* transmitter) {
    if(canx == CAN1) {
        uint8_t i, size=sizeof(__can1_transmitters)/sizeof(__can1_transmitters[0]);
        for(i=0; i<size; i++) {
            if(!__can1_transmitters[i]) {
                __can1_transmitters[i] = transmitter;
                return true;
            }
        }
    } else if(canx == CAN2) {
        uint8_t i, size=sizeof(__can2_transmitters)/sizeof(__can2_transmitters[0]);
        for(i=0; i<size; i++) {
            if(!__can2_transmitters[i]) {
                __can2_transmitters[i] = transmitter;
                return true;
            }
        }
    }
    return false;
}

void CAN_SubmitChange(CAN_Transmitter* transmitter) {
    transmitter->tx.changed = true;
}

void CAN_SendMsg(CAN_Transmitter* transmitter) {
    CAN_SubmitChange(transmitter);
}



const struct CAN_Controllers_Type spCAN =  {
    ._system = {
        .init = spCAN_Init,
        .transmit_looper = spCAN_MsgSendLoop
    },
    .user = {
        .registe_receiver = CAN_RegistReceiver,
        .registe_transmitter = CAN_RegistTransmitter,
        .submit = CAN_SendMsg,
        .send = __CAN_SendMsg
    }
};


/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/
