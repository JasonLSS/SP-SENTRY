/**

------

- @file       sp_sensor.c
- @author     LiuZhe 937150058@qq.com
- @version    v0.0-alpha
- @date       2019.Mar.02
- @brief     	get senors' data(for exmple:distance) via IIC 

------

- @license
- @demo GY56---GY56_distance();TOF10120---TOF10120_distance();GP2Y0E03---GP2Y0E03_distance();

------

*/

/* Includes ------------------------------------------------------------------*/
#include "sp_sensor.h"

/**

- @brief  write data to slave's register
- @param  Slave_addr: i2c addressing uses the high 7 bits(get if from slave's specification)
- @param  addr:the address of slave's regiser.(get if from slave's specification)
- @param  databuffer: the data to write
*/
uint8_t SensorWrite_PF(uint8_t Slave_addr,uint8_t addr,uint16_t* databuffer) 
{
    IIC_BeginTrasnmission_PF(Slave_addr);
    IIC_Write_Addr_PF(addr);
    IIC_Write_Data_PF(databuffer);
    IIC_endTrasnmission();
    return SET;
}

uint8_t SensorWrite_PE(uint8_t Slave_addr,uint8_t addr,uint16_t* databuffer) 
{
    IIC_BeginTrasnmission_PE(Slave_addr);
    IIC_Write_Addr_PE(addr);
    IIC_Write_Data_PE(databuffer);
    IIC_endTrasnmission();
    return SET;
}
/**
- @brief  read data from slave's register
- @param  Slave_addr: i2c addressing uses the high 7 bits(get if from slave's specification)
- @param  addr:the address of slave's regiser.(get if from slave's specification)
- @param  cnt: the number of byte
- @param  requestdata:the data to read
*/
uint8_t SensorRead_PF(uint8_t Slave_addr,uint8_t addr,uint8_t cnt,uint16_t* requestdata)
{
    IIC_BeginTrasnmission_PF(Slave_addr);
    IIC_Write_Addr_PF(addr);
    IIC_endTrasnmission();
    delay_us(50);
    requestFrom_PF(Slave_addr,cnt,requestdata);
    IIC_endTrasnmission();
    return SET;
}

uint8_t SensorRead_PE(uint8_t Slave_addr,uint8_t addr,uint8_t cnt,uint16_t* requestdata)
{
    IIC_BeginTrasnmission_PE(Slave_addr);
    IIC_Write_Addr_PE(addr);
    IIC_endTrasnmission();
    delay_us(50);
    requestFrom_PE(Slave_addr,cnt,requestdata);
    IIC_endTrasnmission();
    return SET;
}
/**
*@brief  read distance from slave's register
*@param  Slave_addr: i2c addressing uses the high 7 bits(get if from slave's specification)
*@param  addr:the address of slave's distance regiser.(get if from slave's specification)
*@param  cnt: the number of byte
*@param  requestdata:the data to read
*/
uint16_t distance(uint8_t Slave_addr,uint8_t addr,uint8_t cnt,uint16_t* requestdata)
{
    uint16_t distance,*requestedata;
    SensorRead_PF(Slave_addr,addr,cnt,requestedata);
    distance=(requestedata[0] << 8) | requestedata[1];
    return distance;
}
/**
*@brief  read distance of GY56 sensor
*/
uint16_t GY56_1_distance(void)
{
    uint16_t current_distance=0, requestedata[2] = {0, 0}, last_distance=0;
    SensorRead_PF(0x70,0x51,2,requestedata);
    current_distance=(requestedata[0] << 8) | requestedata[1];
    if(current_distance > 2000) current_distance=last_distance;
    else last_distance=current_distance;
    return current_distance;
}
uint16_t GY56_2_distance(void)
{
    uint16_t distance=0,requestedata[2]={0,0};
    SensorRead_PF(0x20,0x51,2,requestedata);
    distance=(requestedata[0] << 8) | requestedata[1];
    return distance;
}


//??IIC?? ????8bit,??????0
uint16_t GY56ChangeSlaveAddress(uint8_t NewSlaveAddress)
{
    IIC_BeginTrasnmission_PF(0x70);
    IIC_Write_Addr_PF(0xaa);
    IIC_Write_Addr_PF(0xa5);
    IIC_Write_Addr_PF(NewSlaveAddress);
    IIC_endTrasnmission();
return SET;
}
/**
* @brief  read distance of TOF10120 sensor
*/
uint16_t TOF10120_1distance(void)
{
    uint16_t current_distance=0, requestedata[2] = {0, 0}, last_distance=0;
    SensorRead_PF(0x52,0x04,2,requestedata);
    current_distance=(requestedata[0] << 8) | requestedata[1];
    if(current_distance > 2000) current_distance=last_distance;
    else last_distance=current_distance;
    return current_distance;
}


uint16_t TOF10120_2distance(void)
{
    uint16_t current_distance=0, requestedata[2] = {0, 0}, last_distance=0;
    SensorRead_PF(0x53,0x04,2,requestedata);
    current_distance=(requestedata[0] << 8) | requestedata[1];
    if(current_distance > 2000) current_distance=last_distance;
    else last_distance=current_distance;
    return current_distance;
}

/***********lz***********/
uint16_t TOF10120_3distance(void)
{
    uint16_t current_distance=0, requestedata[2] = {0, 0}, last_distance=0;
    SensorRead_PF(0x50,0x04,2,requestedata);
    current_distance=(requestedata[0] << 8) | requestedata[1];
    if(current_distance > 2000) current_distance=last_distance;
    else last_distance=current_distance;
    return current_distance;
}


uint16_t TOF10120_4distance(void)
{
    uint16_t current_distance=0, requestedata[2] = {0, 0}, last_distance=0;
    SensorRead_PF(0x49,0x04,2,requestedata);
    current_distance=(requestedata[0] << 8) | requestedata[1];
    if(current_distance > 2000) current_distance=last_distance;
    else last_distance=current_distance;
    return current_distance;
}
 uint16_t current_distance5 = 0;
uint16_t TOF10120_5distance(void)
{
    uint16_t /*current_distance5=0,*/ requestedata[2] = {0, 0}, last_distance=0;
    SensorRead_PF(0x48,0x04,2,requestedata);
    current_distance5=(requestedata[0] << 8) | requestedata[1];
    if(current_distance5 > 2000) current_distance5=last_distance;
    else last_distance=current_distance5;
    return current_distance5;
}

uint16_t TOF10120_6distance(void)
{
    uint16_t current_distance=0, requestedata[2] = {0, 0}, last_distance=0;
    SensorRead_PF(0x47,0x04,2,requestedata);
    current_distance=(requestedata[0] << 8) | requestedata[1];
    if(current_distance > 2000) current_distance=last_distance;
    else last_distance=current_distance;
    return current_distance;
}

uint16_t TOF10120_7distance(void)
{
    uint16_t current_distance=0, requestedata[2] = {0, 0}, last_distance=0;
    SensorRead_PF(0x46,0x04,2,requestedata);
    current_distance=(requestedata[0] << 8) | requestedata[1];
    if(current_distance > 2000) current_distance=last_distance;
    else last_distance=current_distance;
    return current_distance;
}


/**
*@brief  read distance of GP2Y0E03 sensor
*@ps GP2Y0E03_distance  it becomes zero sometimes.
*/
uint16_t GP2Y0E03_distance(void)
{
    uint16_t requestedata[2]={0,0},Median_Filter_9=0x20;;
    float distance=0.0;
    SensorWrite_PF(0x40,0x3F,&Median_Filter_9);
    SensorRead_PF(0x40,0x5E,2,requestedata);
    distance=(requestedata[0]*16 + requestedata[1]) /16.0/4.0;
    //		distance2=requestedata[0]/2;
    //		distance3=((requestedata[0] << 8) | requestedata[1]) >> 4; 
    return distance;
}


/***********************PE for TOF************************************/

uint16_t TOF10120_1distance_PE(void)
{
    uint16_t current_distance=0, requestedata[2] = {0, 0}, last_distance=0;
    SensorRead_PE(0x52,0x04,2,requestedata);
    current_distance=(requestedata[0] << 8) | requestedata[1];
    if(current_distance > 2000) current_distance=last_distance;
    else last_distance=current_distance;
    return current_distance;
}


uint16_t TOF10120_2distance_PE(void)
{
    uint16_t current_distance=0, requestedata[2] = {0, 0}, last_distance=0;
    SensorRead_PE(0x51,0x04,2,requestedata);
    current_distance=(requestedata[0] << 8) | requestedata[1];
    if(current_distance > 2000) current_distance=last_distance;
    else last_distance=current_distance;
    return current_distance;
}



uint16_t TOF10120_3distance_PE(void)
{
    uint16_t current_distance=0, requestedata[2] = {0, 0}, last_distance=0;
    SensorRead_PE(0x50,0x04,2,requestedata);
    current_distance=(requestedata[0] << 8) | requestedata[1];
    if(current_distance > 2000) current_distance=last_distance;
    else last_distance=current_distance;
    return current_distance;
}

