#ifndef __I2C_H
#define __I2C_H
#include "stm32f4xx.h"
#include "sys.h"
#include "sp_conf.h"
//IO方向设置

#define Start 	     1
#define NStart       0
#define Wait_ACK     1
#define Wait_NACK    0
#define Send_ACK     1
#define Send_NACK    0

#define T 6
/*IIC's basic functions*/
void IIC_Init(void);                //初始化IIC的IO口				 
//uint8_t IIC_Start(void);			
//void IIC_Stop(void);	  			
//void IIC_Send_Byte(uint8_t txd);			
//uint8_t IIC_Read_Byte(void);
//uint8_t IIC_Wait_Ack(void); 		
//void IIC_Send_Ack(uint8_t i);

void IIC_Start(void);
void IIC_Stop(void);
//uint8_t IIC_Wait_Ack(void);
void IIC_Ack(void);
void IIC_NAck(void);
void IIC_Send_Byte(uint8_t txd);
uint8_t IIC_Read_Byte(unsigned char ack);
/*IIC's protocol functions*/
//uint8_t IIC_BeginTrasnmission(uint8_t Slave_addr);
//uint8_t IIC_Write_Addr(uint8_t addr);
//uint8_t IIC_Write_Data(uint16_t* databuffer);
//uint8_t requestFrom(uint8_t Slave_addr,uint8_t cnt,uint16_t* requestdata);
 
void IIC_endTrasnmission(void);
void	IIC_BeginTrasnmission(uint8_t Slave_addr);
void IIC_Write_Addr(uint8_t addr);
void IIC_Write_Data(uint16_t* databuffer);
void requestFrom(uint8_t Slave_addr,uint8_t cnt,uint16_t* requestdata);

//uint8_t GY56requestRange(uint8_t Slave_Address,uint16_t *databuffer);
//uint8_t SensorWrite(uint8_t Slave_addr,uint8_t addr,uint16_t* databuffer); 
//uint8_t SensorRead(uint8_t Slave_addr,uint8_t addr,uint8_t cnt,uint16_t* requestdata);
//uint8_t GY56takeRangeReading(uint8_t Slave_Address);
//uint8_t GY56requestRange(uint8_t Slave_Address,uint16_t *distance);
#endif
