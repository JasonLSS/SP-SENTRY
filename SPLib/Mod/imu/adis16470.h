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

#ifndef __ADIS16470_H
#define __ADIS16470_H

#include "sp_conf.h"
#include "sp_spi.h"

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

/** @defgroup ADIS16470_User_API
  * @brief    ADIS16470 driver for user operations
  * @ingroup  IMU
  * @{
  */
extern struct __ADIS16470_Driver {
    struct {
        
    } __param_const;
    struct {
        bool (*read_byte)(uint8_t addr, uint16_t* output);
        bool (*write_byte)(uint8_t addr, uint16_t data);
        void (*burst_read)(void);
    } register_ope;
    struct {
        bool (*init)(void);
        bool (*check_device)(void);
        void (*update)(void);
        void (*reset)(bool hardware);
        uint8_t (*get_state)(void);
    } user;
    struct {
        float roll, pitch, yaw;
    } data_readonly;
} ADIS_Driver;
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
