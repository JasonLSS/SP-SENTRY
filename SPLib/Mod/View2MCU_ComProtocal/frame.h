#include "define.h"
#ifndef __FRAME
#define __FRAME     

extern u8 sendbuffer[100];
//modified by sui qi
//modified by twb 2017
#pragma  pack(1) 
typedef struct _frame //视觉发来的数据框架    
{
    u8 head[2];        //帧头，为0xffff
    u32 timestamp;//包序号
    float yaw;      //yaw误差值
    float pitch;  //pitch误差值
    u32 extra[2]; //additional imformation    
    u8 crc8check;    //crc8校验位                
}frame;                                            
#pragma  pack() 
/*
#pragma  pack(1) 
typedef struct _frame_send                
{
    u8 head[2];    
    u32 timestamp;

    s16 v_x;
    s16 v_y;
    s16 v_w;                                                                                  
    s16 target_x;//undermined
    s16 target_y;//undermined
    s16 target_w;//undermined        
    float platform_delta_yaw;
    float platform_delta_pitch;
    u8 cmd0;
    u8 cmd1;
    u8 cmd2;
    u8 cmd3;    
    u8 crc8check;                            
}frame_send;                                            
#pragma  pack() 
*/
u8 sumCheck(u8 *buff,u16 len);
u8 crc8Check(u8 *buff,u16 len);
u16 crc16Check(u8 *buff,u32 len);
/*
*return 0: success
*return 1: no frame in buff
*return 2: crc8 error
*/
int unpackFrame(u8 *buff,u16 len,frame *fram);
void packFrame(u8 *buff,frame *fram);
void sendtoComputer(void);
void sendtoComputerInit(void);
void ChangeToAuto(void);
void ChangeToManual(void);
void ChangeToPower(void);
void ChangeToBigPower(void);

#endif
