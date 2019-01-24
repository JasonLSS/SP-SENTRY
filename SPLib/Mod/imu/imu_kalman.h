#ifndef __IMU_KALMAN_H
#define __IMU_KALMAN_H


#include <stdint.h>
#include <math.h>
#include "sp_conf.h"
#include "sp_math.h"

/*
190124:
                                
    cov-mag_angle =
            1.0e-04 *
            0.0322   -0.0007   -0.0008
           -0.0007    0.0283    0.0008
           -0.0008    0.0008    0.1173
    
    cov-gyro =
            1.0e-04 *
            0.1713    0.0006   -0.0002
            0.0006    0.1693   -0.0030
           -0.0002   -0.0030    0.1679

*/


#define IMU_PI              3.141592653f                /*!< PI definition */
#define IMU_RAD2DEG(x)      ((x) * 57.2957795f)         /*!< Radians to degrees converter */
#define IMU_DEG2RAD(x)      ((x) * 0.0174532925f)       /*!< Radians to degrees converter */

typedef struct {
    struct {
        float dt;
        float xk[9];
        float pk[9];
        float R[9];
        float Q[9];
        float mag_angle[3];
    } param;
    struct {
        LPF_FirstOrder_type lpf[3];
        bool                lpf_enbale;
        HPF_FirstOrder_type hpf[3];
        bool                hpf_enbale;
    } pass_filter;
    struct {
        float roll, pitch, yaw;
    } euler;
    
} Kalman_t;


void KalmanFilter(
        Kalman_t* kalman, 
        const float * gyro, 
        const float * accel,
        const float * mag, float dt);

#endif // __IMU_KALMAN_H
