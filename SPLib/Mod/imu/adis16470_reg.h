/**
  ******************************************************************************
  * @file       sp_imu.h
  * @author     YTom
  * @version    v0.1
  * @date       2019.Jan.23
  * @brief      IMU(MPU6500 & IST8310) module
  * @usage      
  ******************************************************************************
  * @license
  *
  ******************************************************************************
  */

#ifndef __ADIS16470_REG_H
#define __ADIS16470_REG_H

#ifdef __cplusplus
 extern "C" {
#endif

/** @addtogroup SP
  * @brief      SuperPower
  * @{
  */

/** @defgroup IMU
  * @brief    IMU Module
  * @{
  */

/** @defgroup ADIS16470_Register_Map
  * @brief    Register map of ADIS16470
  * @ingroup  IMU
  * @{
  */
// User Register Memory Map from Table 6
#define ADIS_FLASH_CNT           0x00  //Flash memory write count
#define ADIS_DIAG_STAT           0x02  //Diagnostic and operational status
#define ADIS_X_GYRO_LOW          0x04  //X-axis gyroscope output, lower word
#define ADIS_X_GYRO_OUT          0x06  //X-axis gyroscope output, upper word
#define ADIS_Y_GYRO_LOW          0x08  //Y-axis gyroscope output, lower word
#define ADIS_Y_GYRO_OUT          0x0A  //Y-axis gyroscope output, upper word
#define ADIS_Z_GYRO_LOW          0x0C  //Z-axis gyroscope output, lower word
#define ADIS_Z_GYRO_OUT          0x0E  //Z-axis gyroscope output, upper word
#define ADIS_X_ACCL_LOW          0x10  //X-axis accelerometer output, lower word
#define ADIS_X_ACCL_OUT          0x12  //X-axis accelerometer output, upper word
#define ADIS_Y_ACCL_LOW          0x14  //Y-axis accelerometer output, lower word
#define ADIS_Y_ACCL_OUT          0x16  //Y-axis accelerometer output, upper word
#define ADIS_Z_ACCL_LOW          0x18  //Z-axis accelerometer output, lower word
#define ADIS_Z_ACCL_OUT          0x1A  //Z-axis accelerometer output, upper word
#define ADIS_TEMP_OUT            0x1C  //Temperature output (internal, not calibrated)
#define ADIS_TIME_STAMP          0x1E  //PPS mode time stamp
#define ADIS_X_DELTANG_LOW       0x24  //X-axis delta angle output, lower word
#define ADIS_X_DELTANG_OUT       0x26  //X-axis delta angle output, upper word
#define ADIS_Y_DELTANG_LOW       0x28  //Y-axis delta angle output, lower word
#define ADIS_Y_DELTANG_OUT       0x2A  //Y-axis delta angle output, upper word
#define ADIS_Z_DELTANG_LOW       0x2C  //Z-axis delta angle output, lower word
#define ADIS_Z_DELTANG_OUT       0x2E  //Z-axis delta angle output, upper word
#define ADIS_X_DELTVEL_LOW       0x30  //X-axis delta velocity output, lower word
#define ADIS_X_DELTVEL_OUT       0x32  //X-axis delta velocity output, upper word
#define ADIS_Y_DELTVEL_LOW       0x34  //Y-axis delta velocity output, lower word
#define ADIS_Y_DELTVEL_OUT       0x36  //Y-axis delta velocity output, upper word
#define ADIS_Z_DELTVEL_LOW       0x38  //Z-axis delta velocity output, lower word
#define ADIS_Z_DELTVEL_OUT       0x3A  //Z-axis delta velocity output, upper word
#define ADIS_XG_BIAS_LOW         0x40  //X-axis gyroscope bias offset correction, lower word
#define ADIS_XG_BIAS_HIGH        0x42  //X-axis gyroscope bias offset correction, upper word
#define ADIS_YG_BIAS_LOW         0x44  //Y-axis gyroscope bias offset correction, lower word
#define ADIS_YG_BIAS_HIGH        0x46  //Y-axis gyroscope bias offset correction, upper word
#define ADIS_ZG_BIAS_LOW         0x48  //Z-axis gyroscope bias offset correction, lower word
#define ADIS_ZG_BIAS_HIGH        0x4A  //Z-axis gyroscope bias offset correction, upper word
#define ADIS_XA_BIAS_LOW         0x4C  //X-axis accelerometer bias offset correction, lower word
#define ADIS_XA_BIAS_HIGH        0x4E  //X-axis accelerometer bias offset correction, upper word
#define ADIS_YA_BIAS_LOW         0x50  //Y-axis accelerometer bias offset correction, lower word
#define ADIS_YA_BIAS_HIGH        0x52  //Y-axis accelerometer bias offset correction, upper word
#define ADIS_ZA_BIAS_LOW         0x54  //Z-axis accelerometer bias offset correction, lower word
#define ADIS_ZA_BIAS_HIGH        0x56  //Z-axis accelerometer bias offset correction, upper word
#define ADIS_FILT_CTRL           0x5C  //Filter control
#define ADIS_MSC_CTRL            0x60  //Miscellaneous control
#define ADIS_UP_SCALE            0x62  //Clock scale factor, PPS mode
#define ADIS_DEC_RATE            0x64  //Decimation rate control (output data rate)
#define ADIS_NULL_CFG            0x66  //Auto-null configuration control
#define ADIS_GLOB_CMD            0x68  //Global commands
#define ADIS_FIRM_REV            0x6C  //Firmware revision
#define ADIS_FIRM_DM             0x6E  //Firmware revision date, month and day
#define ADIS_FIRM_Y              0x70  //Firmware revision date, year
#define ADIS_PROD_ID             0x72  //Product identification 
#define ADIS_SERIAL_NUM          0x74  //Serial number (relative to assembly lot)
#define ADIS_USER_SCR1           0x76  //User scratch register 1 
#define ADIS_USER_SCR2           0x78  //User scratch register 2 
#define ADIS_USER_SCR3           0x7A  //User scratch register 3 
#define ADIS_FLSHCNT_LOW         0x7C  //Flash update count, lower word 
#define ADIS_FLSHCNT_HIGH        0x7E  //Flash update count, upper word 

#define ADIS_BURST               0x6800  // Start ince burst read

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

#endif /*__ADIS16470_REG_H */

/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/
