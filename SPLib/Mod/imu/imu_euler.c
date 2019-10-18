/************************************************************
 *  File    :   euler.c
 *  Author  :   @YangTianhao ,490999282@qq.com,@TangJiaxin ,tjx1024@126.com
 *  Version :   V1.0
 *  Update  :   2017.03.02
 *  Description:    euler caluation functions
 ************************************************************/

#include "sp_math.h"
#include "imu_euler.h"

float Yaw_Offset, Pitch_Offset, Roll_Offset;
int count = 0;
float delay_speed=0;

int euler_count=0;

double gyro_zero_offset[3] = {0.0312f, -1.3835e-04f, -0.0214f};

// Mahony filters
void update_euler(float gyro[3], float accel[3], float euler_dt,float* roll, float* pitch, float* yaw) {

    assert_param(roll);
    assert_param(pitch);
    assert_param(yaw);

    static const float KP=0.5f;
    
    float gx = gyro[0] - gyro_zero_offset[0];
    float gy = gyro[1] - gyro_zero_offset[1];
    float gz = gyro[2] - gyro_zero_offset[2];
    
    float Roll_sin    =   sin(*roll);
    float Roll_cos    =   cos(*roll);
    float Pitch_sin   =   sin(*pitch);
    float Pitch_cos   =   cos(*pitch);

    // Normalize accelerometer data
    float a = inv_sqrt(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
    float ax = accel[0] * a;
    float ay = accel[1] * a;
    float az = accel[2] * a;
    
    /*  Correction of angular velocity error caused by rotation of the body frame
        Assume:
            {B}=body frame, {O}=world frame, (P){O->B}=transfer matrix from {O} to {B}
            [g]{O}=theoretical gravity vector under {O}, [g]{B}=theoretical gravity vector under {B}
            [gr]{B}=real gravity vector under {B}, [err_g]=error between [gr]{B} and [g]{B}
            <dot>=mathematical dot operations, <cross>=mathematical cross operations
        1. [g]{B} = (P){O->B} <dot> [g]{O}
            where [g]{O} = (0, 0, 1)^T
            where (P){O->B} = Rot(x=roll) * Rot(y=pitch)
            and [g]{B} = [sp -sr*cp cr*cp]^T
        2. [err_g] = [gr]{B} <cross> [g]{B}
            where [gr]{B} = [ax ay az]^T
    */
    gx += KP * ( ay*Roll_cos*Pitch_cos - az*Roll_sin*Pitch_cos );
    gy += KP * (-ax*Roll_cos*Pitch_cos - az*Pitch_sin );
    gz += KP * ( ax*Roll_sin*Pitch_cos + ay*Pitch_sin );
    
    // Euler angular velocity transform
    (*roll)  = (*roll)  + (gx + Roll_sin*Pitch_sin*gy + Roll_cos*Pitch_sin*gz)/Pitch_cos *euler_dt;
    (*pitch) = (*pitch) + (Roll_cos*gy - Roll_sin*gz) *euler_dt;
    (*yaw)   = (*yaw)   + (Roll_sin*gy + Roll_cos*gz)/Pitch_cos *euler_dt;
    
    if((*roll) > PI) {
        (*roll) -= 2.0f*PI;
    } else if((*roll)<-PI) {
        (*roll) += 2.0f*PI;
    }

    if((*pitch) > PI/2.0f) {
        (*pitch) -= PI;
    } else if((*pitch) < -PI/2.0f) {
        (*pitch) += PI;
    }

}
