#include "imu_kalman.h"

static const float I[]= {
    1.f, 0, 0,
    0, 1.f, 0,
    0, 0, 1.f,
};

void matrix_add(const float * mata, const float * matb, float * matc) {
    uint8_t i, j;
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            matc[i * 3 + j] = mata[i * 3 + j] + matb[i * 3 + j];
        }
    }
}

void matrix_sub(const float * mata, const float * matb, float * matc) {
    uint8_t i, j;
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            matc[i * 3 + j] = mata[i * 3 + j] - matb[i * 3 + j];
        }
    }
}

void matrix_multi(const float * mata, const float * matb, float * matc) {
    uint8_t i, j, m;
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            matc[i * 3 + j] = 0.0;
            for (m = 0; m < 3; m++) {
                matc[i * 3 + j] += mata[i * 3 + m] * matb[m * 3 + j];
            }
        }
    }
}


void AngleFromAccelMag(const float* accel, const float* mag, float* angles) {
    
    float normFactor = invSqrt(accel[0]*accel[0]+accel[1]*accel[1]+accel[2]*accel[2]);
    float ax = accel[0] * normFactor;
    float ay = accel[1] * normFactor;
    float az = accel[2] * normFactor;
    
    normFactor = invSqrt(mag[0]*mag[0]+mag[1]*mag[1]+mag[2]*mag[2]);
    float mx = mag[0] * normFactor;
    float my = mag[1] * normFactor;
    float mz = mag[2] * normFactor;
    
    float roll = atan2(-ay, az);       // Roll
    float pitch = asin(-ax);       // Pitch
    float yaw = atan2(-my*cos(roll)+mz*sin(roll), 
        mx*cos(pitch)+my*sin(roll)*sin(pitch)+mz*cos(roll)*sin(pitch)); // Yaw
    
    angles[0] = roll;
    angles[1] = pitch;
    angles[2] = yaw;
}

void KalmanFilter(
        Kalman_t* kalman, 
        const float gyro[3], 
        const float accel[3],
        const float mag[3], float dt) {
    uint8_t i, j;
    
    AngleFromAccelMag(accel, mag, kalman->param.mag_angle);
    float Uk[9] = {
        gyro[0]*dt, 0, 0,
        0, gyro[1]*dt, 0,
        0, 0, gyro[2]*dt,
    };
    float xnew[9] = {
        kalman->param.mag_angle[0], 0, 0,
        0, kalman->param.mag_angle[1], 0,
        0, 0, kalman->param.mag_angle[2],
    };
            
    float yk[9];
    float pk_new[9];
    float K[9];
    float KxYk[9];
    float I_K[9];
    float S[9];
    float S_invert[9];
    float sdet;

    //xk = xk + uk
    matrix_add(kalman->param.xk, Uk, kalman->param.xk);
    //pk = pk + Q
    matrix_add(kalman->param.pk, kalman->param.Q, kalman->param.pk);
    //yk = xnew - xk
    matrix_sub(xnew, kalman->param.xk, yk);
    //S=Pk + R
    matrix_add(kalman->param.pk, kalman->param.R, S);
    //S invert
    sdet = S[0] * S[4] * S[8] + S[1] * S[5] * S[6] + S[2] * S[3] * S[7] - 
        S[2] * S[4] * S[6] - S[5] * S[7] * S[0] - S[8] * S[1] * S[3];
    S_invert[0] = (S[4] * S[8] - S[5] * S[7]) / sdet;
    S_invert[1] = (S[2] * S[7] - S[1] * S[8]) / sdet;
    S_invert[2] = (S[1] * S[7] - S[4] * S[6]) / sdet;
    S_invert[3] = (S[5] * S[6] - S[3] * S[8]) / sdet;
    S_invert[4] = (S[0] * S[8] - S[2] * S[6]) / sdet;
    S_invert[5] = (S[2] * S[3] - S[0] * S[5]) / sdet;
    S_invert[6] = (S[3] * S[7] - S[4] * S[6]) / sdet;
    S_invert[7] = (S[1] * S[6] - S[0] * S[7]) / sdet;
    S_invert[8] = (S[0] * S[4] - S[1] * S[3]) / sdet;
    //K = Pk * S_invert
    matrix_multi(kalman->param.pk, S_invert, K);
    //xk = xk + K * yk
    matrix_multi(K, yk, KxYk);
    matrix_add(kalman->param.xk, KxYk, kalman->param.xk);
    //pk = (I - K)*(pk)
    matrix_sub(I, K, I_K);
    matrix_multi(I_K, kalman->param.pk, pk_new);
    //update pk
    //pk = pk_new;
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            kalman->param.pk[i * 3 + j] = pk_new[i * 3 + j];
        }
    }
    
    kalman->euler.roll = IMU_RAD2DEG(kalman->param.xk[0]);
    kalman->euler.pitch = IMU_RAD2DEG(kalman->param.xk[4]);
    kalman->euler.yaw = IMU_RAD2DEG(kalman->param.xk[8]);
}
