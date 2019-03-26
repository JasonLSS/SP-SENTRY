#include "sp_conf.h"

#include <stdio.h>
#include <string.h>
#include "define.h"
#include "frame.h"
#include "crc_table.h"

#include "gimbal.h"

//created by sui qi
//modified by twb 2017.7.18 add send module
//本文件用于通过周陈旺自制的板子与视觉通信

frame sendtoCom_frame;//数据包
u8 sendbuffer[100];

/***************************************************************************************
 *Name     : sumCheck
 *Function ：求和校验函数
 *Input    ：buff（待校验数据包），len（校验数据长度）
 *Output   ：u8（校验结果）
 *Description :求和校验（未使用）
****************************************************************************************/
u8 sumCheck(u8 *buff,u16 len)
{
    u16 i=0;
    u32 sum=0;
    for(i=0;i<len;i++)
        sum+=buff[i];
    return (u8)sum&0x000000ff;
}
/***************************************************************************************
 *Name     : crc8Check
 *Function ：8位CRC校验函数
 *Input    ：buff（待校验数据包），len（校验数据长度）
 *Output   ：u8（校验结果）
 *Description :8位CRC校验，返回校验结果
****************************************************************************************/
u8 crc8Check(u8 *buff,u16 len)
{
    u8 ucIndex,ucCRC8=(u8)CRC8_INIT;
    while (len--)
    {
        ucIndex = ucCRC8^(*buff++);
        ucCRC8 = CRC8_Table[ucIndex];
    }
    return(ucCRC8);
}
/***************************************************************************************
 *Name     : crc16Check
 *Function ：16位CRC校验函数
 *Input    ：buff（待校验数据包），len（校验数据长度）
 *Output   ：u8（校验结果）
 *Description :16位CRC校验，返回校验结果
****************************************************************************************/
u16 crc16Check(u8 *buff,u32 len)
{
    u8 chData;
    u16 wCRC=(u16)CRC16_INIT;
    if(buff == NULL)
    {
        return 0xFFFF;
    }
    while(len--)
    {
        chData = *buff++;
        (wCRC) = ((u16)(wCRC) >> 8) ^ CRC16_Table[((u16)(wCRC) ^ (u16)(chData)) &0x00ff];
    }
    return wCRC;
}
/***************************************************************************************
 *Name     : unpackFrame
 *Function ：解包函数
 *Input    ：buff（待解数据包），len（数据长度），fram（要存放数据的结构体）
 *Output   ：int（0为成功）
 *Description ：解包同时进行8位CRC校验
****************************************************************************************/
int unpackFrame(u8 *buff,u16 len,frame *fram)
{    
    u16 i=0;
    while(buff[i]!=0xff||buff[i+1]!=0xff)    
    {
        i++;
        if(i>44)
            break;
    }    
    if(i==45)
    {
        if(buff[i]!=0xff||buff[0]!=0xff)
            return 1;
    }
    if(i<23)
    {
      memcpy(fram,(buff+i),sizeof(frame));
    }
    else
    {
        memcpy(fram,(buff+i),46-i);
        memcpy(((u8 *)fram)+46-i,buff,sizeof(frame)-46+i);
    }
    if(fram->crc8check==crc8Check((u8 *)fram,sizeof(frame)-1))
    {
        memset(buff,0x00,46);
        return 0;
    }
    else
        return 2;
}
/***************************************************************************************
 *Name     : packFrame
 *Function ：打包函数
 *Input    ：buff（数据包），fram（要放到数据包里的结构体）
 *Output   ：无
 *Description ：将结构体内的数据装进数据包，并加入帧尾校验
****************************************************************************************/
void packFrame(u8 *buff,frame *fram)
{    
    memcpy(buff,fram,sizeof(frame));
    buff[sizeof(frame)-1] = crc8Check((u8 *)fram,sizeof(frame)-1);
}
/***************************************************************************************
 *Name     : packFrame1
 *Function ：打包函数
 *Input    ：buff（数据包），fram（要放到数据包里的结构体）
 *Output   ：无
 *Description ：将结构体内的数据装进数据包（省略了前6字节），并加入帧尾校验
****************************************************************************************/
void packFrame1(u8 *buff,frame *fram)
{    
    memcpy(buff+6,((u8*)fram)+6,8);//只需向内存移入底盘偏移值
    buff[sizeof(frame)-1] = crc8Check((u8 *)fram,sizeof(frame)-1);
}

/***************************************************************************************
 *Name     : sendtoComputer
 *Function ：将准备发给视觉的数据存入数据包
 *Input    ：无
 *Output   ：无
 *Description ：将结构体内的数据装进数据包
****************************************************************************************/
void sendtoComputer(void)
{
    sendtoCom_frame.yaw = spGIMBAL_Controller._target.gimbal_yaw_motor->state.angle*0.0439f;
    sendtoCom_frame.pitch = spGIMBAL_Controller._target.gimbal_pitch_motor->state.angle*0.0439f;
//    sendtoCom_frame.yaw = (current_position_205-MIDDLE_YAW)*0.0439f;
//    sendtoCom_frame.pitch = (current_position_206-MIDDLE_PITCH)*0.0439f;
    packFrame1(sendbuffer, &sendtoCom_frame);//减少每次搬运内存时间
    spDMA_Controllers.controller.start(spDMA_USART2_tx_stream, (uint32_t)sendbuffer, (uint32_t)&USART2->DR, sizeof(sendtoCom_frame));
//    DMA_Cmd(DMA1_Stream6,ENABLE);
//    USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
}
/***************************************************************************************
 *Name     : sendtoComputerInit
 *Function ：打包初始化函数
 *Input    ：无
 *Output   ：无
 *Description ：首次将数据装入数据包
****************************************************************************************/
void sendtoComputerInit(void)
{
    sendtoCom_frame.head[0]=0xff;
    sendtoCom_frame.head[1]=0xff;
    
    sendtoCom_frame.yaw = spGIMBAL_Controller._target.gimbal_yaw_motor->state.angle*0.0439f;
    sendtoCom_frame.pitch = spGIMBAL_Controller._target.gimbal_pitch_motor->state.angle*0.0439f;
//    sendtoCom_frame.yaw = (current_position_205-MIDDLE_YAW)*0.0439f;
//    sendtoCom_frame.pitch = (current_position_206-MIDDLE_PITCH)*0.0439f;
    
    packFrame(sendbuffer,&sendtoCom_frame);

}
/***************************************************************************************
 *Name     : ChangeToAuto
 *Function ：开启自瞄模式函数
 *Input    ：无
 *Output   ：无
 *Description ：向视觉发送开启自瞄模式的命令（通过CAN2发给自制板）
****************************************************************************************/
void ChangeToAuto(void)//选择打开自瞄模式
{
    
    #ifdef little_board
    
        CanTxMsg TxMessage;
        
        TxMessage.StdId=0x212;               // 标准标识符为0x212
        TxMessage.ExtId=0x00;                 // 设置扩展标示符（29位）（不设置）
        TxMessage.IDE=CAN_ID_STD;                     // 使用标准帧模式
        TxMessage.RTR=CAN_RTR_DATA;                 // 发送的是数据
        TxMessage.DLC=8;                                 // 数据长度为2字节
        TxMessage.Data[0]= 0x01;
        TxMessage.Data[1]= 0x01;    
        TxMessage.Data[2]= 0x00;
        TxMessage.Data[3]= 0x00;    
        TxMessage.Data[4]= 0x00;
        TxMessage.Data[5]= 0x00;    
        TxMessage.Data[6]= 0x00;
        TxMessage.Data[7]= 0x00;    
        
        CAN_Transmit(CAN2, &TxMessage);
    
    #else
    
        u32 AutoFlag[2]={1,1};
        memcpy(sendbuffer+14,&AutoFlag,8);
        
    #endif
}
/***************************************************************************************
 *Name     : ChangeToManual
 *Function ：开启自手动模式函数
 *Input    ：无
 *Output   ：无
 *Description ：向视觉发送开启手动模式的命令（通过CAN2发给自制板）
****************************************************************************************/
void ChangeToManual(void)//选择打开手动模式
{

    #ifdef little_board
    
        CanTxMsg TxMessage;
        
        TxMessage.StdId=0x212;               // 标准标识符为0x212
        TxMessage.ExtId=0x00;                 // 设置扩展标示符（29位）
        TxMessage.IDE=CAN_ID_STD;                     // 使用标准帧模式
        TxMessage.RTR=CAN_RTR_DATA;                 // 发送的是数据
        TxMessage.DLC=8;                                 // 数据长度为2字节
        TxMessage.Data[0]= 0x00;
        TxMessage.Data[1]= 0x00;    
        TxMessage.Data[2]= 0x00;
        TxMessage.Data[3]= 0x00;    
        TxMessage.Data[4]= 0x00;
        TxMessage.Data[5]= 0x00;    
        TxMessage.Data[6]= 0x00;
        TxMessage.Data[7]= 0x00;    
        
        CAN_Transmit(CAN2, &TxMessage);
    
    #else
    
        u32 AutoFlag=0;
        memcpy(sendbuffer+14,&AutoFlag,4);
        
    #endif
    
}
/***************************************************************************************
 *Name     : ChangeToPower
 *Function ：开启小符模式函数
 *Input    ：无
 *Output   ：无
 *Description ：向视觉发送开启小符模式的命令（通过CAN2发给自制板）
****************************************************************************************/
void ChangeToPower(void)//选择打开能量机关模式
{

    #ifdef little_board
    
        CanTxMsg TxMessage;
        
        TxMessage.StdId=0x212;               // 标准标识符为0x212
        TxMessage.ExtId=0x00;                 // 设置扩展标示符（29位）
        TxMessage.IDE=CAN_ID_STD;                     // 使用标准帧模式
        TxMessage.RTR=CAN_RTR_DATA;                 // 发送的是数据
        TxMessage.DLC=8;                                 // 数据长度为2字节
        TxMessage.Data[0]= 0x01;
        TxMessage.Data[1]= 0x03;    
        TxMessage.Data[2]= 0x00;
        TxMessage.Data[3]= 0x00;    
        TxMessage.Data[4]= 0x00;
        TxMessage.Data[5]= 0x00;    
        TxMessage.Data[6]= 0x00;
        TxMessage.Data[7]= 0x00;    
        
        CAN_Transmit(CAN2, &TxMessage);
    
    #else
    
        u32 AutoFlag[2]={1,3};
        memcpy(sendbuffer+14,&AutoFlag,8);
        
    #endif
}
/***************************************************************************************
 *Name     : ChangeToBigPower
 *Function ：开启大符模式函数
 *Input    ：无
 *Output   ：无
 *Description ：向视觉发送开启大符模式的命令（通过CAN2发给自制板）
****************************************************************************************/
void ChangeToBigPower(void)//选择打开能量机关模式
{

    #ifdef little_board
    
        CanTxMsg TxMessage;
        
        TxMessage.StdId=0x212;               // 标准标识符为0x200
        TxMessage.ExtId=0x00;                 // 设置扩展标示符（29位）
        TxMessage.IDE=CAN_ID_STD;                     // 使用标准帧模式
        TxMessage.RTR=CAN_RTR_DATA;                 // 发送的是数据
        TxMessage.DLC=8;                                 // 数据长度为2字节
        TxMessage.Data[0]= 0x01;
        TxMessage.Data[1]= 0x04;    
        TxMessage.Data[2]= 0x00;
        TxMessage.Data[3]= 0x00;    
        TxMessage.Data[4]= 0x00;
        TxMessage.Data[5]= 0x00;    
        TxMessage.Data[6]= 0x00;
        TxMessage.Data[7]= 0x00;    
        
        CAN_Transmit(CAN2, &TxMessage);
    
    #else
    
        u32 AutoFlag[2]={1,4};
        memcpy(sendbuffer+14,&AutoFlag,8);
        
    #endif
}






