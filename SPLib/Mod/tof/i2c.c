/************************************************************
 *File        :i2c
 *Author    :  @LiuZhe 937150058@qq.com
 *Version    : V1.0 
 *Update    : 2019.02.28
 *Description:     Use I/O to simulate IIC and Test sensors such as TOF10120 GP2Y0E03 and GY56¡.
 *Ps:    ÊµÏÖIICÉÏ¹Ò¶à¸ö´Ó»ú
            GY56 ²âÊÔÍê³É
             GP2Y0E03 ²âÁ¿¾àÀëÓÐÏµÍ³Æ«²îÇÒ²»ÎÈ¶¨
            TOF10120 Ó²¼þÉÕ»µÉÐÎ´²âÊÔÍê±Ï
 ************************************************************/
/**
  ******************************************************************************
  * @file       sp_gpio.c
  * @author     LiuZhe 937150058@qq.com
  * @version    v0.1-beta
  * @date       2019.Mar.02
  * @brief         Use I/O to simulate IIC 
  ******************************************************************************
  * @license
    * @demo:Ä¿Ç°ÐèÒª¸ÄÊ¹ÄÜµÄ GPIO ºÍ PIN Ä¿Ç°¸ÃÎÄ¼þ¶ÔÓ¦µÄÎªPF0(SDA)--I2(A°å) PF1(SCL)---I1(A°å)    
  ******************************************************************************
  */
    
/* Includes ------------------------------------------------------------------*/

    
#include "i2c.h"
#include "sp_gpio.h"

#define SDA_IN()  {GPIOF->MODER&=~(3<<( 0 *2));GPIOF->MODER|=0<< 0 *2;}    //PF0ÊäÈëÄ£Ê½
#define SDA_OUT() {GPIOF->MODER&=~(3<<( 0 *2));GPIOF->MODER|=1<< 0 *2;} //PF0Êä³öÄ£Ê½
//IO²Ù×÷º¯Êý     
#define IIC_SCL    PFout(1) //SCL
#define IIC_SDA    PFout(0) //SDA     
#define READ_SDA   PFin(0)  //ÊäÈëSDA 

void IIC_Init(void)
{            
  GPIO_InitTypeDef  GPIO_InitStructure;
    
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);//Ê¹ÄÜGPIOFÊ±ÖÓ

  //GPIOF0,F1³õÊ¼»¯ÉèÖÃ
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//ÆÕÍ¨Êä³öÄ£Ê½
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//ÍÆÍìÊä³ö
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//ÉÏÀ­
  GPIO_Init(GPIOF, &GPIO_InitStructure);//³õÊ¼»¯
    IIC_SCL=1;
    IIC_SDA=1;
}

void IIC_Start(void)
{
    SDA_OUT();     //sdaÏßÊä³ö
    IIC_SDA=1;            
    IIC_SCL=1;
    delay_us(T);
    IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
    delay_us(T);
    IIC_SCL=0;//Ç¯×¡I2C×ÜÏß£¬×¼±¸·¢ËÍ»ò½ÓÊÕÊý¾Ý 
    delay_us(T);
}      
//²úÉúIICÍ£Ö¹ÐÅºÅ
void IIC_Stop(void)
{
    SDA_OUT();//sdaÏßÊä³ö
    IIC_SCL=0;
    IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
    delay_us(T);
    IIC_SCL=1; 
    delay_us(T);
    IIC_SDA=1;//·¢ËÍI2C×ÜÏß½áÊøÐÅºÅ
    delay_us(T);
}
////µÈ´ýÓ¦´ðÐÅºÅµ½À´
////·µ»ØÖµ£º1£¬½ÓÊÕÓ¦´ðÊ§°Ü
////        0£¬½ÓÊÕÓ¦´ð³É¹¦
//uint8_t IIC_Wait_Ack(void)
//{

//} 
//²úÉúACKÓ¦´ð
void IIC_Ack(void)
{
    IIC_SCL=0;
    SDA_OUT();
    IIC_SDA=0;
    delay_us(T);
    IIC_SCL=1;
    delay_us(T);
    IIC_SCL=0;
    IIC_SDA=1;
    delay_us(T);
}
//²»²úÉúACKÓ¦´ð            
void IIC_NAck(void)
{
    IIC_SCL=0;
    SDA_OUT();
    IIC_SDA=1;
    delay_us(T);
    IIC_SCL=1;
    delay_us(T);
    IIC_SCL=0;
    IIC_SDA=1;
    delay_us(T);
}                                          
//IIC·¢ËÍÒ»¸ö×Ö½Ú
//·µ»Ø´Ó»úÓÐÎÞÓ¦´ð
//1£¬ÓÐÓ¦´ð
//0£¬ÎÞÓ¦´ð              
void IIC_Send_Byte(uint8_t txd)
{                        
    uint8_t t;   
    SDA_OUT();
    IIC_SCL=0;//À­µÍÊ±ÖÓ¿ªÊ¼Êý¾Ý´«Êä
    for(t=0x80; t!=0; t>>=1)
    {              
        IIC_SDA=(txd&0x80)>>7;  //È¥txdµÄ×î¸ßÎ» ±äÎª×îµÍÎ»´æÈë¼Ä´æÆ÷
        txd<<=1;                //txd=txd <<1
        delay_us(T);            //¶ÔTEA5767ÕâÈý¸öÑÓÊ±¶¼ÊÇ±ØÐëµÄ
        IIC_SCL=1;
        delay_us(T); 
        IIC_SCL=0;
    }
    delay_us(T);
    
    IIC_SDA=1;
    SDA_IN();      //SDAÉèÖÃÎªÊäÈë
    delay_us(2);
    IIC_SCL=1;
    uint8_t ret = READ_SDA;
    delay_us(T);
    IIC_SCL=0;//Ê±ÖÓÊä³ö0
    delay_us(T);
    delay_us(10);
//    return ret;
}         
//¶Á1¸ö×Ö½Ú£¬ack=1Ê±£¬·¢ËÍACK£¬ack=0£¬·¢ËÍnACK   
uint8_t IIC_Read_Byte(unsigned char ack)
{
    unsigned char i,receive=0;
    IIC_SCL=0;
    IIC_SDA=1;
    delay_us(T);
    SDA_IN();//SDAÉèÖÃÎªÊäÈë
    for(i=0;i<8;i++)
    {
        delay_us(T);
        IIC_SCL=1;
        receive<<=1;
        if(READ_SDA) receive++;
        delay_us(T);
        IIC_SCL=0;
    }
    if (!ack)
        IIC_NAck();//·¢ËÍnACK
    else
        IIC_Ack(); //·¢ËÍACK   
    return receive;
}
/**
  * @brief  tell the salve to begin to transit via IIC and the slave issues acknowledgement.
  * @param  Slave_addr: i2c addressing uses the high 7 bits 
  */
void    IIC_BeginTrasnmission(uint8_t Slave_addr)
//uint8_t IIC_BeginTrasnmission(uint8_t Slave_addr)
{
    /*Master generates a start condition.----------------------------------------*/
    //if(IIC_Start()==0){IIC_Stop(); return RESET;}
    IIC_Start();
    /**
        Master places 1st byte data that are combined slave address (7bits) 
        with a read/write control bit (R/W=0) to SDA.
    */
    IIC_Send_Byte((Slave_addr<<1) | 0);
    /*Slave issues acknowledgement.---------------------------------------------*/
    //if(!IIC_Wait_Ack()){IIC_Stop(); return RESET;}
//    IIC_Wait_Ack();
    //return SET;
}    

/**
  * @brief  from which register to read data
  * @param  addr: the register's address 
  */
void IIC_Write_Addr(uint8_t addr)
//uint8_t IIC_Write_Addr(uint8_t addr)
{
    IIC_Send_Byte(addr);    //Master places 2nd byte address data on SDA.
    //if(!IIC_Wait_Ack()){IIC_Stop(); return RESET;}//Slave issues acknowledgement.    
//    IIC_Wait_Ack();
    //return SET;
}    

/**
  * @brief  write data 
  * @param  databuffer: data
  */
void IIC_Write_Data(uint16_t* databuffer)
{
//uint8_t IIC_Write_Data(uint16_t* databuffer)
//{
    IIC_Send_Byte(*databuffer);
    //if(!IIC_Wait_Ack()){IIC_Stop(); return RESET;}
//    IIC_Wait_Ack();
//    return SET;
}

/**
  * @brief  IIC stop
  */
void IIC_endTrasnmission(void)
{
    IIC_Stop();
}

/**
  * @brief  request reading from sensor
  * @param  Slave_addr: i2c addressing uses the high 7 bits 
  * @param  cnt: the number of byte
  */
void requestFrom(uint8_t Slave_addr,uint8_t cnt,uint16_t* requestdata)
{
    //if(IIC_Start()==0){IIC_Stop(); return RESET;}//Master generates a start condition.
    IIC_Start();
    IIC_Send_Byte((Slave_addr<<1) | 1);
//    IIC_Wait_Ack();
    for(int i=0; i<cnt; i++)
    {
        requestdata[i] = IIC_Read_Byte( (i<cnt-1)?1:0 );    //8 bits data is read from addr
        delay_us(10);
    }
    //return SET;
}

