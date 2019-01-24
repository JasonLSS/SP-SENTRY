/**
  ******************************************************************************
  * @file       sp_spi.h
  * @author     YTom
  * @version    v0.1
  * @date       2019.Jan.21
  * @brief      CAN-bus module controller
  @verbatim      
        BaudRate:   1Mbps
        PinMap:     PB13(tx) PB12(rx)
      
        PE4  SPI4_MOSI
        PE5  SPI4_MISO
        PE6  SPI4_NSS
        PE12 SPI4_SCK
  @endverbatim
  ******************************************************************************
  * @license
  *
  ******************************************************************************
  */

#ifndef __SP_SPI_H
#define __SP_SPI_H



#include "sp_conf.h"

#ifdef __cplusplus
 extern "C" {
#endif

/** @addtogroup SP
  * @brief      SuperPower
  * @{
  */

/** @defgroup SPI
  * @brief    SPI Module
  * @{
  */


// User Register Memory Map from Table 6
#define FLASH_CNT   	0x00  //Flash memory write count
#define DIAG_STAT   	0x02  //Diagnostic and operational status
#define X_GYRO_LOW  	0x04  //X-axis gyroscope output, lower word
#define X_GYRO_OUT  	0x06  //X-axis gyroscope output, upper word
#define Y_GYRO_LOW  	0x08  //Y-axis gyroscope output, lower word
#define Y_GYRO_OUT  	0x0A  //Y-axis gyroscope output, upper word
#define Z_GYRO_LOW  	0x0C  //Z-axis gyroscope output, lower word
#define Z_GYRO_OUT  	0x0E  //Z-axis gyroscope output, upper word
#define X_ACCL_LOW  	0x10  //X-axis accelerometer output, lower word
#define X_ACCL_OUT  	0x12  //X-axis accelerometer output, upper word
#define Y_ACCL_LOW  	0x14  //Y-axis accelerometer output, lower word
#define Y_ACCL_OUT  	0x16  //Y-axis accelerometer output, upper word
#define Z_ACCL_LOW  	0x18  //Z-axis accelerometer output, lower word
#define Z_ACCL_OUT  	0x1A  //Z-axis accelerometer output, upper word
#define TEMP_OUT    	0x1C  //Temperature output (internal, not calibrated)
#define TIME_STAMP  	0x1E  //PPS mode time stamp
#define X_DELTANG_LOW	0x24  //X-axis delta angle output, lower word
#define X_DELTANG_OUT	0x26  //X-axis delta angle output, upper word
#define Y_DELTANG_LOW	0x28  //Y-axis delta angle output, lower word
#define Y_DELTANG_OUT	0x2A  //Y-axis delta angle output, upper word
#define Z_DELTANG_LOW	0x2C  //Z-axis delta angle output, lower word
#define Z_DELTANG_OUT	0x2E  //Z-axis delta angle output, upper word
#define X_DELTVEL_LOW	0x30  //X-axis delta velocity output, lower word
#define X_DELTVEL_OUT	0x32  //X-axis delta velocity output, upper word
#define Y_DELTVEL_LOW	0x34  //Y-axis delta velocity output, lower word
#define Y_DELTVEL_OUT	0x36  //Y-axis delta velocity output, upper word
#define Z_DELTVEL_LOW	0x38  //Z-axis delta velocity output, lower word
#define Z_DELTVEL_OUT	0x3A  //Z-axis delta velocity output, upper word
#define XG_BIAS_LOW		0x40  //X-axis gyroscope bias offset correction, lower word
#define XG_BIAS_HIGH	0x42  //X-axis gyroscope bias offset correction, upper word
#define YG_BIAS_LOW		0x44  //Y-axis gyroscope bias offset correction, lower word
#define YG_BIAS_HIGH	0x46  //Y-axis gyroscope bias offset correction, upper word
#define ZG_BIAS_LOW		0x48  //Z-axis gyroscope bias offset correction, lower word
#define ZG_BIAS_HIGH	0x4A  //Z-axis gyroscope bias offset correction, upper word
#define XA_BIAS_LOW		0x4C  //X-axis accelerometer bias offset correction, lower word
#define XA_BIAS_HIGH	0x4E  //X-axis accelerometer bias offset correction, upper word
#define YA_BIAS_LOW		0x50  //Y-axis accelerometer bias offset correction, lower word
#define YA_BIAS_HIGH	0x52  //Y-axis accelerometer bias offset correction, upper word
#define ZA_BIAS_LOW		0x54  //Z-axis accelerometer bias offset correction, lower word
#define ZA_BIAS_HIGH	0x56  //Z-axis accelerometer bias offset correction, upper word
#define FILT_CTRL    	0x5C  //Filter control
#define MSC_CTRL    	0x60  //Miscellaneous control
#define UP_SCALE    	0x62  //Clock scale factor, PPS mode
#define DEC_RATE    	0x64  //Decimation rate control (output data rate)
#define NULL_CFG    	0x66  //Auto-null configuration control
#define GLOB_CMD    	0x68  //Global commands
#define FIRM_REV    	0x6C  //Firmware revision
#define FIRM_DM    		0x6E  //Firmware revision date, month and day
#define FIRM_Y    		0x70  //Firmware revision date, year
#define PROD_ID    		0x72  //Product identification 
#define SERIAL_NUM    0x74  //Serial number (relative to assembly lot)
#define USER_SCR1    	0x76  //User scratch register 1 
#define USER_SCR2    	0x78  //User scratch register 2 
#define USER_SCR3    	0x7A  //User scratch register 3 
#define FLSHCNT_LOW   0x7C  //Flash update count, lower word 
#define FLSHCNT_HIGH  0x7E  //Flash update count, upper word 




/** @defgroup Definations
  * @brief    Exported Macros And Definations
  * @ingroup  SPI
  * @{
  */
#define SPI5_NSS_Select     GPIO_ResetBits(GPIOF, GPIO_Pin_6)
#define SPI5_NSS_Release    GPIO_SetBits(GPIOF, GPIO_Pin_6)


typedef struct {
    SPI_TypeDef*        this;
    spPinSet            sck;
    spPinSet            miso;
    spPinSet            mosi;
    spPinSet            nss;
    SPI_InitTypeDef     spi_initer;
} SPI_PinsType;

extern SPI_PinsType SPI4_Pins;

extern struct SPI_Controllers_Type {
    uint16_t (*read_write_b)(SPI_TypeDef* spi, uint16_t data);
    uint16_t (*write_b)(SPI_TypeDef* spi, uint16_t data);
    void     (*select)(SPI_PinsType* spi_pins);
    void     (*release)(SPI_PinsType* spi_pins);
} spSPI_Controllers;
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

#endif /*__SP_SPI_H */

/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/
