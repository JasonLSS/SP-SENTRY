#ifndef __KALMAN_H
#define __KALMAN_H


#include <stdint.h>
#include <math.h>
#include "sp_conf.h"
#include "sp_math.h"


#define IMU_PI              3.141592653f                /*!< PI definition */
#define IMU_RAD2DEG(x)      ((x) * 57.2957795f)         /*!< Radians to degrees converter */
#define IMU_DEG2RAD(x)      ((x) * 0.0174532925f)       /*!< Radians to degrees converter */

typedef struct {
    float dt;
    float xk[9];
    float pk[9];
    float R[9];
    float Q[9];
    
    float mag_angle[3];
    
    float roll, pitch, yaw;
} Kalman_t;


void KalmanFilter(
        Kalman_t* kalman, 
        const float * gyro, 
        const float * accel,
        const float * mag, float dt);

#endif // __KALMAN_H
