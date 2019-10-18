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
        (++) spIMU.operations.init();
      (#) Using IMU in EXIT IRQ, typical usage:

            #include "sp_imu.h"
            void EXTI9_5_IRQHandler(void) {
                if(EXTI_GetITStatus(EXTI_Line8)) {
                    // Read IMU
                    //mag: -37.8 37.2 42.5
                    spIMU.operations.read_stream(
                        spIMU.imu_state.ahrs.gyro,
                        spIMU.imu_state.ahrs.accel,
                        &spIMU.imu_state.ahrs.temp,
                        spIMU.imu_state.ahrs.mag);

                    if(spIMU.imu_state.kalman.pass_filter.lpf_enbale) {
                        spIMU.imu_state.ahrs.accel[0] =
                            LPF_FirstOrder_filter(spIMU.imu_state.kalman.pass_filter.lpf+0,
                            spIMU.imu_state.ahrs.accel[0]);
                        spIMU.imu_state.ahrs.accel[1] =
                            LPF_FirstOrder_filter(spIMU.imu_state.kalman.pass_filter.lpf+1,
                            spIMU.imu_state.ahrs.accel[1]);
                        spIMU.imu_state.ahrs.accel[2] =
                            LPF_FirstOrder_filter(spIMU.imu_state.kalman.pass_filter.lpf+2,
                            spIMU.imu_state.ahrs.accel[2]);
                    }

                    float time = TASK_GetSecond();
                    float dt = time - spIMU.imu_state.timestamp;
                    spIMU.imu_state.freq = 1.f/dt;
                    if(!spIMU.imu_state.inited) {
                        spIMU.imu_state.inited = true;
                    } else {
                        KalmanFilter(&spIMU.imu_state.kalman,
                            spIMU.imu_state.ahrs.gyro,
                            spIMU.imu_state.ahrs.accel,
                            spIMU.imu_state.ahrs.mag, dt);
                    }
                    // Make log
                    spIMU.imu_state.timestamp = time;
                    spIMU.imu_state.count ++;

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
#include "sp_math.h"
#include "sp_gpio.h"

//*** <<< Use Configuration Wizard in Context Menu >>> ***

//  <o> Choose IMU type
//    <1=>Onboard MPU+IST(default)  <2=>Extern ADIS16470
#define sp_choose_imu_type            1

//  <o> Choose IMU calculating method
//    <1=>Euler(default)  <2=>Kalman  <3=>AHRS
#define sp_choose_imu_method          1

#if sp_choose_imu_method == 1
#include "imu_euler.h"
#elif sp_choose_imu_method == 2
#include "imu_kalman.h"
#elif sp_choose_imu_method == 3
#include "imu_ahrs.h"
#endif

//  <o> MPU sampling frequency
//    <1=>100  <2=>200(default)  <3=>250  <4=>500  <5=>1000
#define sp_choose_mpu_hz              2

#if sp_choose_mpu_hz == 1
#define DEFAULT_MPU_HZ                100
#elif sp_choose_mpu_hz == 2
#define DEFAULT_MPU_HZ                200
#elif sp_choose_mpu_hz == 3
#define DEFAULT_MPU_HZ                250
#elif sp_choose_mpu_hz == 4
#define DEFAULT_MPU_HZ                1000
#endif

// <!c1> Disable accelerometer LPF
//   <i> Disable accelerometer low-pass-filter
#define __IMU_USE_ACCEL_LPF
// </c>

// <c1>  Enable magnetometer
//   <i> Enable magnetometer
//#define __IMU_USE_MAGNETOMETER
// </c>

//*** <<< end of configuration section >>>    ***


    /**
      * @brief  Gyroscope, accelerometer and magnetometer's precision.s
      */
    extern const float ACCEL_SEN;
    extern const float GYRO_SEN;
    extern const float MAG_SEN;

    typedef struct {
        float gyro[3];
        float accel_0[3];
        float accel[3];
        float mag[3];
        float temp;
        float r,p,y;
        float q[4];
    } imu_data_t;

    typedef struct {
        LPF_FirstOrder_type lpf[3];
        bool                lpf_enbale;
        HPF_FirstOrder_type hpf[3];
        bool                hpf_enbale;
    } imu_pass_filter_t;


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
            bool (*init)(void);
            /**
              * @brief  Update IMU data
              */
            void (*update)(float* gyro, float* accel);
            /**
              * @brief  Update magnetometer data
              */
#ifdef __IMU_USE_MAGNETOMETER
            void (*update_magnetometer)(float* mag);
#endif
            /**
              * @brief  IMU data resolving in IRQ
              */
            void (*irq_callback)(void);
        } operations;

        /**
          * @brief  IMU informations
          */
        struct {
            imu_data_t              data;           /*!< IMU converted data */
            imu_pass_filter_t       filter;
            bool                    inited;         /*!< If IMU data initialized */
            float                   temp;           /*!< Temprature value from MPU6500 */
            float                   timestamp;      /*!< IMU data timestamp */
            float                   freq;           /*!< IMU realtime reading frequency */
            uint32_t                count;          /*!< IMU data package counters */
        } imu_state;

    } spIMU;
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
