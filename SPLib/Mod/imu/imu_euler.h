/**
  ******************************************************************************
  * @file       imu_euler.h
  * @author     YTom
  * @version    v0.1
  * @date       2019.Jan.23
  * @brief      IMU(MPU6500 & IST8310) module
  @verbatim
  @endverbatim
  ******************************************************************************
  * @license
  *
  ******************************************************************************
  */

#ifndef __IMU_EULER_H
#define __IMU_EULER_H

#ifdef __cplusplus
extern "C" {
#endif

void init_euler(void);
void update_euler(float gyro[3], float accel[3], float euler_dt,
                  float* roll, float* pitch, float* yaw);

//extern float Yaw_gyro,Roll_gyro,Pitch_gyro;
//extern float Yaw_mag,Roll_accel,Pitch_accel;
//extern float Yaw,Roll,Pitch,Yaw_Offset,Pitch_Offset,Roll_Offset;
//extern float delay_speed;

#ifdef __cplusplus
}
#endif

#endif /*__IMU_EULER_H */

/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/

