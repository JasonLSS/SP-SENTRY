/**
  ******************************************************************************
  * @file       sp_imu.h
  * @author     YTom
  * @version    v0.1
  * @date       2019.Jan.23
  * @brief      IMU manager
  ******************************************************************************
  * @license
  *
  ******************************************************************************
  */

#include <math.h>
#include "sp_imu.h"

void IMU_Callback(void) {
    // Read IMU
    //mag: -37.8 37.2 42.5
    spIMU.operations.update(
        spIMU.imu_state.data.gyro,
        spIMU.imu_state.data.accel_0);

    // Data filter for accel
    if(spIMU.imu_state.filter.lpf_enbale) {
        spIMU.imu_state.data.accel[0] =
            LPF_FirstOrder_filter(spIMU.imu_state.filter.lpf+0,
                                  spIMU.imu_state.data.accel_0[0]);
        spIMU.imu_state.data.accel[1] =
            LPF_FirstOrder_filter(spIMU.imu_state.filter.lpf+1,
                                  spIMU.imu_state.data.accel_0[1]);
        spIMU.imu_state.data.accel[2] =
            LPF_FirstOrder_filter(spIMU.imu_state.filter.lpf+2,
                                  spIMU.imu_state.data.accel_0[2]);
    }

    float time = TASK_GetSecond();
    float dt = time - spIMU.imu_state.timestamp;
    spIMU.imu_state.freq = 1.f/dt;
    if(!spIMU.imu_state.inited) {
        spIMU.imu_state.inited = true;
    } else {
#if sp_choose_imu_method == 1
        update_euler(spIMU.imu_state.data.gyro,
                     spIMU.imu_state.data.accel,
                     dt,
                     &spIMU.imu_state.data.r,
                     &spIMU.imu_state.data.p,
                     &spIMU.imu_state.data.y);
#elif sp_choose_imu_method == 2
        KalmanFilter(&spIMU.imu_state.kalman,
                     spIMU.imu_state.ahrs.gyro,
                     spIMU.imu_state.ahrs.accel,
                     spIMU.imu_state.ahrs.mag, dt);
#elif sp_choose_imu_method == 3
        
#endif
    }

    // Make log
    spIMU.imu_state.timestamp = time;
    spIMU.imu_state.count ++;
}


/* Config operation functions */
#if sp_choose_imu_type == 1
extern int MPU6500_Init(void);
extern void MPU6500_Stream_GyroAccel(float* gyro, float* accel);
extern void ist8310_get_data(float* mag );
extern IRQn_Type MPU6500_IRQ_Config(void);
#define __IMU_INIT                      MPU6500_Init
#define __IMU_IRQ_CONFIG                MPU6500_IRQ_Config
#define __IMU_GYRO_ACCEL_UPDATE         MPU6500_Stream_GyroAccel
#define __IMU_MAGN_UPDATE               ist8310_get_data

#elif sp_choose_imu_type == 2

#endif

int __init_result = 0;

bool IMU_Init(void) {

    // Module Init
    __init_result = __IMU_INIT();

    // Init data filters
    for(uint8_t i=0; i<sizeof(spIMU.imu_state.filter.lpf)/
            sizeof(spIMU.imu_state.filter.lpf[0]); i++) {
        LPF_FirstOrder_Init(spIMU.imu_state.filter.lpf+i, 10.f, DEFAULT_MPU_HZ);
    }
    
    /* Config interrut IRQ for IMU */
    IRQn_Type irq = __IMU_IRQ_CONFIG();
    
    /* Registe IMU update callback */
    spIRQ.registe(irq, NULL, spIMU.operations.irq_callback);

    return true;
}

struct IMU_Controllers_Type spIMU = {
    .operations = {
        .init = IMU_Init,
        .update = __IMU_GYRO_ACCEL_UPDATE,
        .irq_callback = IMU_Callback,
#ifdef __IMU_USE_MAGNETOMETER
        .update_magnetometer = __IMU_MAGN_UPDATE,
#endif
    },
    .imu_state.filter = {
#ifdef __IMU_USE_ACCEL_LPF
        .lpf_enbale = true,
#else
        .lpf_enbale = false,
#endif
        .hpf_enbale = false,
    }
};

/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/

//    .imu_state.kalman.param = {
//        .xk= {
//            0, 0, 0, 0
//        },
//        .pk= {
//            1.f, 0, 0, 0,
//            0, 1.f, 0, 0,
//            0, 0, 1.f, 0,
//            0, 0, 0, 1.f,
//        },
//        .R= {
//            0.5f, 0,
//            0, 0.5f,
//        },
//        .Q= {
//            0.05f, 0, 0, 0,
//            0, 0.05f, 0, 0,
//            0, 0, 0.05f, 0,
//            0, 0, 0, 0.05f,
//        },
//    },
