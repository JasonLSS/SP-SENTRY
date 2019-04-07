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
uint8_t SensorWrite(uint8_t Slave_addr,uint8_t addr,uint16_t* databuffer) 
{
    IIC_BeginTrasnmission(Slave_addr);
    IIC_Write_Addr(addr);
    IIC_Write_Data(databuffer);
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
uint8_t SensorRead(uint8_t Slave_addr,uint8_t addr,uint8_t cnt,uint16_t* requestdata)
{
    IIC_BeginTrasnmission(Slave_addr);
    IIC_Write_Addr(addr);
    IIC_endTrasnmission();
    delay_us(50);
    requestFrom(Slave_addr,cnt,requestdata);
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
    SensorRead(Slave_addr,addr,cnt,requestedata);
    distance=(requestedata[0] << 8) | requestedata[1];
    return distance;
}
/**
*@brief  read distance of GY56 sensor
*/
uint16_t GY56_1_distance(void)
{
    uint16_t current_distance=0, requestedata[2] = {0, 0}, last_distance=0;
    SensorRead(0x70,0x51,2,requestedata);
    current_distance=(requestedata[0] << 8) | requestedata[1];
		if(current_distance > 2000) current_distance=last_distance;
		else last_distance=current_distance;
    return current_distance;
}
uint16_t GY56_2_distance(void)
{
    uint16_t distance=0,requestedata[2]={0,0};
    SensorRead(0x20,0x51,2,requestedata);
    distance=(requestedata[0] << 8) | requestedata[1];
    return distance;
}

//??IIC?? ????8bit,??????0
uint16_t GY56ChangeSlaveAddress(uint8_t NewSlaveAddress)
{
    IIC_BeginTrasnmission(0x70);
    IIC_Write_Addr(0xaa);
    IIC_Write_Addr(0xa5);
    IIC_Write_Addr(NewSlaveAddress);
    IIC_endTrasnmission();
return SET;
}
/**
* @brief  read distance of TOF10120 sensor
*/
uint16_t TOF10120_1distance(void)
{
    uint16_t current_distance=0, requestedata[2] = {0, 0}, last_distance=0;
    SensorRead(0x52,0x04,2,requestedata);
    current_distance=(requestedata[0] << 8) | requestedata[1];
		if(current_distance > 2000) current_distance=last_distance;
		else last_distance=current_distance;
    return current_distance;
}
uint16_t TOF10120_2distance(void)
{
    uint16_t current_distance=0, requestedata[2] = {0, 0}, last_distance=0;
    SensorRead(0x53,0x04,2,requestedata);
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
    SensorWrite(0x40,0x3F,&Median_Filter_9);
    SensorRead(0x40,0x5E,2,requestedata);
    distance=(requestedata[0]*16 + requestedata[1]) /16.0/4.0;
    //		distance2=requestedata[0]/2;
    //		distance3=((requestedata[0] << 8) | requestedata[1]) >> 4; 
    return distance;
}

