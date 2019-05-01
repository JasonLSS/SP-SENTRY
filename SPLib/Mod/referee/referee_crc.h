#ifndef __REFEREE_CRC_H
#define __REFEREE_CRC_H

#include "stdbool.h"
#include "stdint.h"
#include "stdlib.h"


//crc8 生成多项式:G(x)=x8+x5+x4+1
extern const unsigned char CRC8_INIT2;
extern const unsigned char CRC8_TAB[256];


//crc16 生成多项式: 未知
extern uint16_t CRC_INIT;
extern const uint16_t wCRC_Table[256];


/*
** Descriptions: CRC8 校验码生成
** Input: 信息数据, 信息数据长, CRC8_INIT2(0xff)
** Output: CRC8校验码
*/
unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);


/*
** Descriptions: CRC8 校验程序
** Input: 需校验数据, 数据长度(= 信息长度 + 校验码长度)
** Output: True or False (CRC Verify Result)
*/
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);


/*
** Descriptions: 增添 CRC8 至信息数据尾部
** Input: 信息数据(尾部需留空1位以加入校验码), 信息数据长度
** Output: 带校验码的数据
*/
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);

/*
** Descriptions: CRC16 校验码生成
** Input: 信息数据, 信息数据长, CRC_INIT(0xffff)
** Output: CRC16校验码
*/
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);


/*
** Descriptions: CRC16 校验程序
** Input: 需校验数据, 数据长度(= 信息长度 + 校验码长度, 即包含了校验码的长度)
** Output: True or False (CRC Verify Result)
*/
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

/*
** Descriptions: 增添 CRC16 至信息数据尾部
** Input: 信息数据(尾部需留空2位以加入校验码), 信息数据长度
** Output: 带校验码的数据
*/
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength);


// 单字节数组转u16(2字节), 高低位反序
uint16_t _bytes2u16(uint8_t * chosen_Message);


// 单字节数组转u32(4字节), 高低位未知
uint32_t _bytes4u32(uint8_t * chosen_Message);


// 单字节数组转float(4字节), 高低位正常
float _bytes2float(uint8_t * chosen_Message);


// float转4长度单字节数组
void float2bytes(float chosen_value, uint8_t * res_message);

#endif // __REFEREE_CRC_H
