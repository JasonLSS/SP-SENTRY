#include "imu_kalman.h"

// From https://www.embbnux.com/2015/01/30/9_dof_imu_with_kalman_filter_on_avr/

static const float I[4][4]= {
    1.f, 0, 0, 0,
    0, 1.f, 0, 0,
    0, 0, 1.f, 0,
    0, 0, 0, 1.f,
};

#include "arm_math.h"


void AngleFromAccelMag(const float* accel, const float* mag, float* angles) {
    
    float normFactor = inv_sqrt(accel[0]*accel[0]+accel[1]*accel[1]+accel[2]*accel[2]);
    float ax = accel[0] * normFactor;
    float ay = accel[1] * normFactor;
    float az = accel[2] * normFactor;
    
    normFactor = inv_sqrt(mag[0]*mag[0]+mag[1]*mag[1]+mag[2]*mag[2]);
    float mx = mag[0] * normFactor;
    float my = mag[1] * normFactor;
    float mz = mag[2] * normFactor;
    
    float roll = atan2(-ay, az);        // Roll [az=g not -g]
    float pitch = asin(-ax);            // Pitch
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
        const float mag[3], float dt) 
{
    /* Solve angular speed from body-frame. */
    /* From: http://philsal.co.uk/projects/imu-attitude-estimation */
    float roll = kalman->euler.roll;
    float pitch = kalman->euler.pitch;
    float wx = gyro[0] + sin(roll)*tan(pitch)*gyro[1] + cos(roll)*tan(pitch)*gyro[2];
    float wy = cos(roll)*gyro[1] - sin(roll)*gyro[2];
    float wz = sin(roll)/cos(pitch)*gyro[1] + cos(roll)/cos(pitch)*gyro[2];
//    float wx = gyro[0];
//    float wy = gyro[1];
    
    AngleFromAccelMag(accel, mag, kalman->param.angle_with_mag);
            
    arm_matrix_instance_f32 E_4x4;
    arm_mat_init_f32(&E_4x4, 4, 4, (float32_t *)I);
    
    /* Constant matrix */
    static float32_t __A_4x4[16] = {
        1.f,    0,      0,      0,
        0,      1.f,    0,      0,
        0,      0,      1.f,    0,
        0,      0,      0,      1.f,
    };
    __A_4x4[1] = -dt;
    __A_4x4[11] = -dt;
    static float32_t __At_4x4[16] = {
        1.f,    0,      0,      0,
        0,      1.f,    0,      0,
        0,      0,      1.f,    0,
        0,      0,      0,      1.f,
    };
    __At_4x4[4] = -dt;
    __At_4x4[14] = -dt;
    static float32_t __B_4x2[8] = {
        0,      0,
        0,      0,
        0,      0,
        0,      0,
    };
    __B_4x2[0] = dt;
    __B_4x2[5] = dt;
    static float32_t __C_2x4[8] = {
        1.f,    0,      0,      0,
        0,      0,      1.f,    0,
    };
    static float32_t __Ct_4x2[8] = {
        1.f,    0,
        0,      0,
        0,      1.f,
        0,      0,
    };
    arm_matrix_instance_f32 A_4x4;
    arm_mat_init_f32(&A_4x4, 4, 4, (float32_t *)__A_4x4);
    arm_matrix_instance_f32 At_4x4;
    arm_mat_init_f32(&At_4x4, 4, 4, (float32_t *)__At_4x4);
    arm_matrix_instance_f32 B_4x2;
    arm_mat_init_f32(&B_4x2, 4, 2, (float32_t *)__B_4x2);
    arm_matrix_instance_f32 C_2x4;
    arm_mat_init_f32(&C_2x4, 2, 4, (float32_t *)__C_2x4);
    arm_matrix_instance_f32 Ct_4x2;
    arm_mat_init_f32(&Ct_4x2, 4, 2, (float32_t *)__Ct_4x2);
    
    float32_t __U_2x1[2] = { wx, wy};
    arm_matrix_instance_f32 U_2x1;
    arm_mat_init_f32(&U_2x1, 2, 1, (float32_t *)__U_2x1);
    
    float32_t __Z_2x1[2] = { kalman->param.angle_with_mag[0],
        kalman->param.angle_with_mag[1], };
    arm_matrix_instance_f32 Z_2x1;
    arm_mat_init_f32(&Z_2x1, 2, 1, (float32_t *)__Z_2x1);
    
    float32_t __Y_2x1[2];
    arm_matrix_instance_f32 Y_2x1;
    arm_mat_init_f32(&Y_2x1, 2, 1, (float32_t *)__Y_2x1);
    float32_t __K_4x2[8];
    arm_matrix_instance_f32 K_4x2;
    arm_mat_init_f32(&K_4x2, 4, 2, (float32_t *)__K_4x2);
    
    arm_matrix_instance_f32 X_4x1;
    arm_mat_init_f32(&X_4x1, 4, 1, (float32_t *)kalman->param.xk);
    arm_matrix_instance_f32 P_4x4;
    arm_mat_init_f32(&P_4x4, 4, 4, (float32_t *)kalman->param.pk);
    arm_matrix_instance_f32 Q_4x4;
    arm_mat_init_f32(&Q_4x4, 4, 4, (float32_t *)kalman->param.Q);
    arm_matrix_instance_f32 R_2x2;
    arm_mat_init_f32(&R_2x2, 2, 2, (float32_t *)kalman->param.R);
    
    /* Middle values */
    float __mid_AX_4x1[4];
    arm_matrix_instance_f32 mid_AX_4x1;
    arm_mat_init_f32(&mid_AX_4x1, 4, 1, (float32_t *)__mid_AX_4x1);
    float __mid_BU_4x1[4];
    arm_matrix_instance_f32 mid_BU_4x1;
    arm_mat_init_f32(&mid_BU_4x1, 4, 1, (float32_t *)__mid_BU_4x1);
    
    float __mid_PAt_4x4[16];
    arm_matrix_instance_f32 mid_PAt_4x4;
    arm_mat_init_f32(&mid_PAt_4x4, 4, 4, (float32_t *)__mid_PAt_4x4);
    float __mid_APAt_4x4[16];
    arm_matrix_instance_f32 mid_APAt_4x4;
    arm_mat_init_f32(&mid_APAt_4x4, 4, 4, (float32_t *)__mid_APAt_4x4);
    
    float __mid_CX_2x1[2];
    arm_matrix_instance_f32 mid_CX_2x1;
    arm_mat_init_f32(&mid_CX_2x1, 2, 1, (float32_t *)__mid_CX_2x1);
    
    float __mid_PCt_4x2[8];
    arm_matrix_instance_f32 mid_PCt_4x2;
    arm_mat_init_f32(&mid_PCt_4x2, 4, 2, (float32_t *)__mid_PCt_4x2);
    float __mid_CPCt_2x2[4];
    arm_matrix_instance_f32 mid_CPCt_2x2;
    arm_mat_init_f32(&mid_CPCt_2x2, 2, 2, (float32_t *)__mid_CPCt_2x2);
    float __mid_CPCtR_2x2[4];
    arm_matrix_instance_f32 mid_CPCtR_2x2;
    arm_mat_init_f32(&mid_CPCtR_2x2, 2, 2, (float32_t *)__mid_CPCtR_2x2);
    float __mid_Fact_2x2[4];
    arm_matrix_instance_f32 mid_Fact_2x2;
    arm_mat_init_f32(&mid_Fact_2x2, 2, 2, (float32_t *)__mid_Fact_2x2);
            
    float __mid_KY_4x1[4];
    arm_matrix_instance_f32 mid_KY_4x1;
    arm_mat_init_f32(&mid_KY_4x1, 4, 1, (float32_t *)__mid_KY_4x1);
    
    float __mid_KC_4x4[16];
    arm_matrix_instance_f32 mid_KC_4x4;
    arm_mat_init_f32(&mid_KC_4x4, 4, 4, (float32_t *)__mid_KC_4x4);
    float __mid_EKC_4x4[16];
    arm_matrix_instance_f32 mid_EKC_4x4;
    arm_mat_init_f32(&mid_EKC_4x4, 4, 4, (float32_t *)__mid_EKC_4x4);
    float __mid_nP_4x4[16];
    arm_matrix_instance_f32 mid_nP_4x4;
    arm_mat_init_f32(&mid_nP_4x4, 4, 4, (float32_t *)__mid_nP_4x4);
     
    
//    Prediction
//    \vec{x}_{t+1} = \textbf{A} \cdot \vec{x}_t + \textbf{B} \cdot \vec{u}_t
        arm_mat_mult_f32(&A_4x4, &X_4x1, &mid_AX_4x1);
        arm_mat_mult_f32(&B_4x2, &U_2x1, &mid_BU_4x1);
        arm_mat_add_f32(&mid_AX_4x1, &mid_BU_4x1, &X_4x1);
//    \textbf{P} = \textbf{A} \cdot \textbf{P} \cdot \textbf{A}^T + \textbf{Q}
        arm_mat_trans_f32(&A_4x4, &At_4x4);
        arm_mat_mult_f32(&P_4x4, &At_4x4, &mid_PAt_4x4);
        arm_mat_mult_f32(&A_4x4, &mid_PAt_4x4, &mid_APAt_4x4);
        arm_mat_add_f32(&mid_APAt_4x4, &Q_4x4, &P_4x4);
//    Update
//    \widetilde{y}_{t+1} = \vec{z}_{t+1} - \textbf{C} \cdot \vec{x}_{t+1}
        arm_mat_mult_f32(&C_2x4, &X_4x1, &mid_CX_2x1);
        arm_mat_sub_f32(&Z_2x1, &mid_CX_2x1, &Y_2x1);
//    \textbf{S} = \textbf{C} \cdot \textbf{P} \cdot \textbf{C}^T + \textbf{R}
//    \textbf{K} = \textbf{P} \cdot \textbf{C}^T \cdot \textbf{S}^{-1}
        arm_mat_trans_f32(&C_2x4, &Ct_4x2);
        arm_mat_mult_f32(&P_4x4, &Ct_4x2, &mid_PCt_4x2);
        arm_mat_mult_f32(&C_2x4, &mid_PCt_4x2, &mid_CPCt_2x2);
        arm_mat_add_f32(&mid_CPCt_2x2, &R_2x2, &mid_CPCtR_2x2);
        arm_mat_inverse_f32(&mid_CPCtR_2x2, &mid_Fact_2x2);
        arm_mat_mult_f32(&mid_PCt_4x2, &mid_Fact_2x2, &K_4x2);
//    \vec{x}_{t+1} = \vec{x}_{t+1} + \textbf{K} \cdot \widetilde{y}_{t+1}
        arm_mat_mult_f32(&K_4x2, &Y_2x1, &mid_KY_4x1);
        arm_mat_add_f32(&X_4x1, &mid_KY_4x1, &X_4x1);
//    \textbf{P} = (\textbf{I} - \textbf{K} \cdot \textbf{C}) \cdot \textbf{P}
        arm_mat_mult_f32(&K_4x2, &C_2x4, &mid_KC_4x4);
        arm_mat_sub_f32(&E_4x4, &mid_KC_4x4, &mid_EKC_4x4);
        arm_mat_mult_f32(&mid_EKC_4x4, &P_4x4, &mid_nP_4x4);
        arm_mat_mult_f32(&mid_nP_4x4, &E_4x4, &P_4x4);
    
    /* Solve Euler angles */
    roll = kalman->euler.roll = IMU_RAD2DEG(kalman->param.xk[0][0]);
    pitch = kalman->euler.pitch = IMU_RAD2DEG(kalman->param.xk[2][0]);
    float normFactor = inv_sqrt(mag[0]*mag[0]+mag[1]*mag[1]+mag[2]*mag[2]);
    float mx = mag[0] * normFactor;
    float my = mag[1] * normFactor;
    float mz = mag[2] * normFactor;
    kalman->euler.yaw = IMU_RAD2DEG(atan2(-(my*cos(roll)+mz*sin(roll)), 
        mx*cos(pitch)+my*sin(roll)*sin(pitch)+mz*cos(roll)*sin(pitch)));
}
