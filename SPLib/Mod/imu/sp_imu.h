/**
  ******************************************************************************
  * @file       sp_imu.h
  * @author     YTom
  * @version    v0.1
  * @date       2019.Jan.23
  * @brief      IMU(MPU6500 & IST8310) module
  @verbatim      
 ===============================================================================
                   #####  How to use this driver #####
 ===============================================================================
      (#) Initialize IMU module
        (++) IMU_Controllers.operations.init();
      (#) Using IMU in EXIT IRQ, typical usage:
            
            #include "sp_imu.h"
            void EXTI9_5_IRQHandler(void) {
                if(EXTI_GetITStatus(EXTI_Line8)) {
                    // Read IMU
                    //mag: -37.8 37.2 42.5
                    IMU_Controllers.operations.read_stream(
                        IMU_Controllers.imu_state.ahrs.gyro, 
                        IMU_Controllers.imu_state.ahrs.accel,
                        &IMU_Controllers.imu_state.ahrs.temp, 
                        IMU_Controllers.imu_state.ahrs.mag);
                    
                    if(IMU_Controllers.imu_state.kalman.pass_filter.lpf_enbale) {
                        IMU_Controllers.imu_state.ahrs.accel[0] = 
                            LPF_FirstOrder_filter(IMU_Controllers.imu_state.kalman.pass_filter.lpf+0,
                            IMU_Controllers.imu_state.ahrs.accel[0]);
                        IMU_Controllers.imu_state.ahrs.accel[1] = 
                            LPF_FirstOrder_filter(IMU_Controllers.imu_state.kalman.pass_filter.lpf+1,
                            IMU_Controllers.imu_state.ahrs.accel[1]);
                        IMU_Controllers.imu_state.ahrs.accel[2] = 
                            LPF_FirstOrder_filter(IMU_Controllers.imu_state.kalman.pass_filter.lpf+2,
                            IMU_Controllers.imu_state.ahrs.accel[2]);
                    }
                    
                    float time = TASK_GetSecond();
                    float dt = time - IMU_Controllers.imu_state.timestamp;
                    IMU_Controllers.imu_state.freq = 1.f/dt;
                    if(!IMU_Controllers.imu_state.inited) {
                        IMU_Controllers.imu_state.inited = true;
                    } else {
                        KalmanFilter(&IMU_Controllers.imu_state.kalman,
                            IMU_Controllers.imu_state.ahrs.gyro,
                            IMU_Controllers.imu_state.ahrs.accel, 
                            IMU_Controllers.imu_state.ahrs.mag, dt);
                    }
                    // Make log
                    IMU_Controllers.imu_state.timestamp = time;
                    IMU_Controllers.imu_state.count ++;
                    
                    EXTI_ClearITPendingBit(EXTI_Line8);
                }
            }
  @endverbatim
  ******************************************************************************
  * @license
  *
  ******************************************************************************
  */

#ifndef __SP_IMU_H
#define __SP_IMU_H

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

#include "sp_spi.h"
#include "sp_conf.h"
#include "mpu_reg.h"
#include "imu_kalman.h"

typedef struct {
    float q[4];
    float gyro[3];
    float accel[3];
    float mag[3];
    float temp;
    float r,p,y;
} AHRS_t;



/** @defgroup Declarations
  * @brief    Exported Function Declaration
  * @ingroup  IMU
  * @{
  */
/** 
  * @brief  IMU Controller
  */
extern struct IMU_Controllers_Type {
    /** 
      * @brief  IMU control operations
      */
    struct {
        /** 
          * @brief  Initialize MPU6500
          * @retval NULL
          */
        int (*init)(void);
        /** 
          * @brief  Reset MPU6500 data FIFOs
          * @retval NULL
          */
        uint16_t (*fifo_reset)(void);
        /** 
          * @brief  Read data from MPU6500 FIFOs
          * @retval NULL
          */
        uint16_t (*fifo_read)(void);
        /** 
          * @brief  Read data stream from MPU6500 from FIFO
          * @param  gyro: 3*1 gyroscope output array
          * @param  accel: 3*1 accelerometer output array
          * @retval NULL
          */
        uint16_t (*fifo_read_stream)(short* gyro, short* accel);
        /** 
          * @brief  Read data stream from MPU6500
          * @param  gyro: 3*1 gyroscope output array
          * @param  accel: 3*1 accelerometer output array
          * @param  temp: temprature output pointer
          * @param  mag: 3*1 magnetnometer output array
          */
        void (*read_stream)(float* gyro, float* accel, float* temp, float* mag);
    } operations;

    /** 
      * @brief  IMU informations
      */
    struct {
        AHRS_t          ahrs;                       /*!< AHRS converted data */
        Kalman_t        kalman;
        bool            inited;                     /*!< If AHRS data initialized */
        
        float           temp;                       /*!< Temprature value from MPU6500 */
        float           timestamp;                  /*!< IMU data timestamp */
        float           freq;                       /*!< IMU realtime reading frequency */
        uint32_t        count;                      /*!< IMU data package counters */
    } imu_state;
    
} IMU_Controllers;

/**
  * @brief  MPU sampling frequency
  */ 
#define DEFAULT_MPU_HZ              (200)

/**
  * @brief  Gyroscope, accelerometer and magnetometer's precision.s
  */ 
extern const float ACCEL_SEN;
extern const float GYRO_SEN;
extern const float MAG_SEN;
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

#endif /*__MPU6500_H */

/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/
