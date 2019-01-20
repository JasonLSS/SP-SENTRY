/**
  ******************************************************************************
  * @file       sp_kalman.c
  * @author     YTom
  * @version    v0.0-alpha
  * @date       2018.Jun.24
  * @brief      Kalman Filter module
  * @note       MODEL:
  *                 Z = ¦×,¦È,¦Ã
  *                 ax=-gsin¦Ãcos¦È
  *                 ay=gsin¦È
  *                 az=gcos¦Ãcos¦È
  * @usage      
  *
  ******************************************************************************
  * @license
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "sp_kalman.h"

/* Private typedef -----------------------------------------------------------*/
void KALMAN_Init(KALMAM_Type* kalman) {
    memset_f32((float*)&kalman->core, 0, sizeof(kalman->core)/sizeof(float32_t));
    
    arm_mat_init_f32(&kalman->mat_A, 
                     sizeof(kalman->core.A)/sizeof(kalman->core.A[0]),
                     sizeof(kalman->core.A[0])/sizeof(kalman->core.A[0][0]),
                     (float32_t*)kalman->core.A);
    arm_mat_init_f32(&kalman->mat_X, 
                     sizeof(kalman->core.X)/sizeof(kalman->core.X[0]),
                     sizeof(kalman->core.X[0])/sizeof(kalman->core.X[0][0]),
                     (float32_t*)kalman->core.X);
//    arm_mat_init_f32(&kalman->mat_W,
//                     sizeof(kalman->core.W)/sizeof(kalman->core.W[0]),
//                     sizeof(kalman->core.W[0])/sizeof(kalman->core.W[0][0]),
//                     (float32_t*)kalman->core.W);
    arm_mat_init_f32(&kalman->mat_H,
                     sizeof(kalman->core.H)/sizeof(kalman->core.H[0]),
                     sizeof(kalman->core.H[0])/sizeof(kalman->core.H[0][0]),
                     (float32_t*)kalman->core.H);
//    arm_mat_init_f32(&kalman->mat_V,
//                     sizeof(kalman->core.V)/sizeof(kalman->core.V[0]),
//                     sizeof(kalman->core.V[0])/sizeof(kalman->core.V[0][0]),
//                     (float32_t*)kalman->core.V);
    arm_mat_init_f32(&kalman->mat_P, 
                     sizeof(kalman->core.P)/sizeof(kalman->core.P[0]),
                     sizeof(kalman->core.P[0])/sizeof(kalman->core.P[0][0]),
                     (float32_t*)kalman->core.P);
    arm_mat_init_f32(&kalman->mat_K, 
                     sizeof(kalman->core.K)/sizeof(kalman->core.K[0]),
                     sizeof(kalman->core.K[0])/sizeof(kalman->core.K[0][0]),
                     (float32_t*)kalman->core.K);
    arm_mat_init_f32(&kalman->mat_Q, 
                     sizeof(kalman->core.Q)/sizeof(kalman->core.Q[0]),
                     sizeof(kalman->core.Q[0])/sizeof(kalman->core.Q[0][0]),
                     (float32_t*)kalman->core.Q);
    arm_mat_init_f32(&kalman->mat_R,
                     sizeof(kalman->core.R)/sizeof(kalman->core.R[0]),
                     sizeof(kalman->core.R[0])/sizeof(kalman->core.R[0][0]),
                     (float32_t*)kalman->core.R);
}
                                  
                                  
void KALMAN_Filter(
    KALMAM_Type*    kalman, 
    float32_t*      Z ) {
    
    #define SHELL_SET(shell,r,c,p)          shell.numRows = r;\
                                            shell.numCols = c;\
                                            shell.pData = p;
    arm_status                  status;
    arm_matrix_instance_f32     shell_a = {0, 0, NULL};
    arm_matrix_instance_f32     shell_b = {0, 0, NULL};
    arm_matrix_instance_f32     shell_c = {0, 0, NULL};
    
    /* X(k) = A*X(k-1) [+ B*u(k)] */
    float32_t AmX[6];
    SHELL_SET(shell_a, 6, 1, (float32_t*)AmX);
        status = arm_mat_mult_f32(&kalman->mat_A, &kalman->mat_X, &shell_a);
        memcpy(kalman->mat_X.pData, AmX, kalman->mat_X.numCols*kalman->mat_X.numRows*4);
//        status = arm_mat_add_f32(&shell_a, &kalman->mat_W, &kalman->mat_X);
    
    /* P(k) = A*P(k-1)*A' + Q */
    float32_t AmP[36], AT[36], AmPmAT[36];
    SHELL_SET(shell_a, 6, 6, (float32_t*)AmP);
        status = arm_mat_mult_f32(&kalman->mat_A, &kalman->mat_P, &shell_a);
    SHELL_SET(shell_b, 6, 6, (float32_t*)AT);
        status = arm_mat_trans_f32(&kalman->mat_A, &shell_b);
    SHELL_SET(shell_c, 6, 6, (float32_t*)AmPmAT);
        status = arm_mat_mult_f32(&shell_a, &shell_b, &shell_c);
        status = arm_mat_add_f32(&shell_c, &kalman->mat_Q, &kalman->mat_P);
    
    /* K(k) = P(k)*H'/(H*P(k)*H' + R) */
    float32_t PmH[18], HT[18], HmPmH[9], iHmPmH[9];
    SHELL_SET(shell_a, 6, 3, (float32_t*)HT);
        status = arm_mat_trans_f32(&kalman->mat_H, &shell_a);
    SHELL_SET(shell_b, 6, 3, (float32_t*)PmH);
        status = arm_mat_mult_f32(&kalman->mat_P, &shell_a, &shell_b);
    SHELL_SET(shell_a, 3, 3, (float32_t*)HmPmH);
        status = arm_mat_mult_f32(&kalman->mat_H, &shell_b, &shell_a);
        status = arm_mat_add_f32(&shell_a, &kalman->mat_R, &shell_a);
    SHELL_SET(shell_c, 3, 3, (float32_t*)iHmPmH);
        status = arm_mat_inverse_f32(&shell_a, &shell_c);
        status = arm_mat_mult_f32(&shell_b, &shell_c, &kalman->mat_K);
    
    /* X(k) = X(k) + K(k)*(z(k) - H*X(k)) */
    float32_t HmX[3], KmD[6], ZsHmx[3];
    SHELL_SET(shell_a, 3, 1, (float32_t*)HmX);
        status = arm_mat_mult_f32(&kalman->mat_H, &kalman->mat_X, &shell_a);
    SHELL_SET(shell_b, 3, 1, (float32_t*)Z);
        status = arm_mat_sub_f32(&shell_b, &shell_a, &shell_a);
    SHELL_SET(shell_b, 6, 1, (float32_t*)KmD);
        status = arm_mat_mult_f32(&kalman->mat_K, &shell_a, &shell_b);
        status = arm_mat_add_f32(&kalman->mat_X, &shell_b, &kalman->mat_X);
    
    /* P(k) = (1 - K(k)*H)*P(k) */
    float32_t KmH[36];
    float32_t eye[36] = {1,0,0,0,0,0,
                         0,1,0,0,0,0,
                         0,0,1,0,0,0,
                         0,0,0,1,0,0,
                         0,0,0,0,1,0,
                         0,0,0,0,0,1};
    float32_t P[36];
    SHELL_SET(shell_a, 6, 6, (float32_t*)KmH);
        status = arm_mat_mult_f32(&kalman->mat_K, &kalman->mat_H, &shell_a);
    SHELL_SET(shell_b, 6, 6, (float32_t*)eye);
        status = arm_mat_sub_f32(&shell_b, &shell_a, &shell_a);
    SHELL_SET(shell_c, 6, 6, (float32_t*)&P);
        memcpy(P, kalman->mat_P.pData, kalman->mat_P.numCols*kalman->mat_P.numRows*4);
        status = arm_mat_mult_f32(&shell_a, &shell_c, &kalman->mat_P);
                         
//    return &kalman->mat_X;
}


float32_t eye[9] = { 1,0,0,
                     0,1,0,
                     0,0,1,};
float32_t Asx2[9] = {1,2,3,
                     5,3,2,
                     5,6,7,};
float32_t tmp[36] = {0,0,0,0,0,0,
                     0,0,0,0,0,0,
                     0,0,0,0,0,0,
                     0,0,0,0,0,0,
                     0,0,0,0,0,0,
                     0,0,0,0,0,0,};
void KALMAN2_Mul_A_X(void) {
    arm_matrix_instance_f32     matA;
    arm_matrix_instance_f32     matAt;
    arm_matrix_instance_f32     mtmp;
    
    arm_mat_init_f32(&matA, 3, 3, (float32_t*)eye);
    arm_mat_init_f32(&matAt, 3, 3, (float32_t*)Asx2);
    arm_mat_add_f32(&matAt, &matA, &matAt);
    arm_mat_sub_f32(&matAt, &matA, &matAt);
}


/* Private macro -------------------------------------------------------------*/
#define KALMAN2_P       1


/* Private variables ---------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/** @defgroup Publuc Kalman Filter Initialization and Configuration
  * @brief    Kalman filter basic functions
  * @{
  */

void KALMAN2_DeInit(Kalman2_Filter_Type *kf)
{
    kf->state[0]        = 0;
    kf->state[1]        = 0;
    kf->covar[0][0]     = KALMAN2_P;
    kf->covar[0][1]     = 0;
    kf->covar[1][0]     = 0;
    kf->covar[1][1]     = KALMAN2_P;
    //kf->dt = time
    kf->dt              = 0;
    //kf->argH = [1 0]
    kf->argH[0]            = 0;
    kf->argH[1]            = 0;
    /* measure noise convariance */
    kf->argQ[0]            = 0;
    kf->argQ[1]            = 0;
    /* estimated error convariance */
    kf->argR               = 0;
}


float KALMAN2_Filterate(Kalman2_Filter_Type *kf, float gyro, float acceang)
{
    float priorState[2]     	= {0};
    float priorCovar[2][2]      = {0};
    float kalmanGain[2]     	= {0};
    /* Step1: Predict */
    /* Priori estimate
    [ang]   [1 -t].[ang]+[t]        [ang-off*t+gyro*t]
    [off] = [0  1] [off] [0]*gyro = [off             ]    */
    priorState[0] = kf->state[0] - kf->dt*kf->state[1] + kf->dt*gyro;
    priorState[1] = kf->state[1];
    /* Priori covariance
    [p1 p2]   [1 -t].[p1 p2].[1 0 ]+[Qa 0]   [p1-p3*t-t*p2+p4*t*t p2-p4*t]+[Qa 0]
    [p3 p4] = [0 1]  [p3 p4] [-t 1] [0 Qg] = [p3-p4*t             p4     ] [0 Qg]    */
    priorCovar[0][0] += kf->dt*(-kf->covar[1][0]-kf->covar[0][1]+kf->covar[1][1]*kf->dt)+kf->argQ[0];
    priorCovar[0][1] += -kf->dt*kf->covar[1][1];
    priorCovar[1][0] += -kf->dt*kf->covar[1][1];
    priorCovar[1][1] += kf->argQ[1];
    /* Step2: Measurement  */
    /* Kalman gain
    [k0]   [p1 p2].[1] (      [p1 p2].[1]+R)^(-1)   [p1/(p1+R)]
    [k1] = [p3 p4] [0].([1 0].[p3 p4] [0]  )      = [p3/(p1+R)]    */
    kalmanGain[0] = priorCovar[0][0]/(priorCovar[0][0]+kf->argR);
    kalmanGain[1] = priorCovar[1][0]/(priorCovar[0][0]+kf->argR);
    /* Posteriori estimate
    [ang]   [ang] [k0]
    [off] = [off]+[k1]*(z-ang)    */
    kf->state[0] = priorState[0] + kalmanGain[0]*(acceang-priorState[0]);
    kf->state[1] = priorState[1] + kalmanGain[1]*(acceang-priorState[0]);
    /* Posteriori covariance
    [p1 p2]   ([1 0]-[k0]*[1 0]) [p1 p2]   [p1-k0*p1 p2-k0*p2]
    [p3 p4] = ([0 1] [k1]      )*[p3 p4] = [p3-k1*p1 p4-k1*p2]    */
    kf->covar[0][0] -= kalmanGain[0]*priorCovar[0][0];
    kf->covar[0][1] -= kalmanGain[0]*priorCovar[0][1];
    kf->covar[1][0] -= kalmanGain[1]*priorCovar[0][0];
    kf->covar[1][1] -= kalmanGain[1]*priorCovar[0][1];

    return kf->state[0];
}

/**
  * @}
  */



//#define KALMAN2_Q1              2.0
//#define KALMAN2_Q2              1e-3    //0.001~0.05(on friction) (1e-6)
//#define KALMAN2_R               20
//#define KALMAN2_SMAPLE_UINT     0.001
//#define KALMAN_INITSAMPLE       200

//float kalman_acce_data[3];
//float kalman_gyro_data[3];
//float kalman_offset[3];

//Kalman2_Filter_Type     kalmanYaw, kalmanPitch, kalmanRoll;
//IMU_Data_Type           IMU_Data;

//void initKalman(void)
//{
//    float ax, ay, az;
//    float gx, gy, gz;
//    float angel[3]={0.0f}, gyro[3]={0.0f};
//    
//    /* Calculate gyroscope data offset, and the initialized angle */
//    unsigned char i;    
//    for (i = 0; i < KALMAN_INITSAMPLE; i++)
//    {
//        MPU6500ReadACCEL();
//        MPU6500ReadGYRO();
//        ax += mpu6500_real_data2.Accel_X;
//        ay += mpu6500_real_data2.Accel_Y;
//        az += mpu6500_real_data2.Accel_Z;
//        gx += mpu6500_real_data2.Gyro_X;
//        gy += mpu6500_real_data2.Gyro_Y;
//        gz += mpu6500_real_data2.Gyro_Z;
//        delay_ms(5);
//    }
//    ax /= KALMAN_INITSAMPLE;
//    ay /= KALMAN_INITSAMPLE;
//    az /= KALMAN_INITSAMPLE;
//    gx /= KALMAN_INITSAMPLE;
//    gy /= KALMAN_INITSAMPLE;
//    gz /= KALMAN_INITSAMPLE;
//    
//    /* YAW: gz, atan2(ax,ay) */
//    angel[0] = atan2(ax, ay);
//    gyro[0] = gz;
//    /* PITCH: gx, atan2(az,ay) */
//    angel[1] = atan2(az, ay);
//    gyro[1] = gx;
//    /* ROLL: gy, atan2(ax,az) */
//    angel[2] = atan2(ax, az);
//    gyro[2] = gy;

//    /* Init Kalman-Filter's arguments */
//    KALMAN2_DeInit(&kalmanYaw);
//    KALMAN_InitGyroAngle(kalmanYaw,angel[0]);
//    KALMAN_InitGyroBias(kalmanYaw,gyro[0]);
//    KALMAN_InitDeltaTime(kalmanYaw,KALMAN2_SMAPLE_UINT);
//    KALMAN_InitCovarAngle(kalmanYaw,KALMAN2_Q1);
//    KALMAN_InitCovarGyroBias(kalmanYaw,KALMAN2_Q2);
//    KALMAN_InitCovarAcceAngle(kalmanYaw,KALMAN2_R);
//    
//    KALMAN2_DeInit(&kalmanPitch);
//    KALMAN_InitGyroAngle(kalmanPitch,angel[1]);
//    KALMAN_InitGyroBias(kalmanPitch,gyro[1]);
//    KALMAN_InitDeltaTime(kalmanPitch,KALMAN2_SMAPLE_UINT);
//    KALMAN_InitCovarAngle(kalmanPitch,KALMAN2_Q1);
//    KALMAN_InitCovarGyroBias(kalmanPitch,KALMAN2_Q2);
//    KALMAN_InitCovarAcceAngle(kalmanPitch,KALMAN2_R);
//    
//    KALMAN2_DeInit(&kalmanRoll);
//    KALMAN_InitGyroAngle(kalmanRoll,angel[2]);
//    KALMAN_InitGyroBias(kalmanRoll,gyro[2]);
//    KALMAN_InitDeltaTime(kalmanRoll,KALMAN2_SMAPLE_UINT);
//    KALMAN_InitCovarAngle(kalmanRoll,KALMAN2_Q1);
//    KALMAN_InitCovarGyroBias(kalmanRoll,KALMAN2_Q2);
//    KALMAN_InitCovarAcceAngle(kalmanRoll,KALMAN2_R);
//}
//void updateKalman(void)
//{
//    MPU6500ReadACCEL();
//    MPU6500ReadGYRO();
//    /* YAW: gz, atan2(ax,ay) */
//    IMU_Data.yaw = 57.296f*KALMAN2_Filterate(&kalmanYaw, mpu6500_real_data2.Gyro_Z, atan2(mpu6500_real_data2.Accel_X, mpu6500_real_data2.Accel_Y));
//    /* PITCH: gx, atan2(az,ay) */
//    IMU_Data.pitch = 57.296f*KALMAN2_Filterate(&kalmanPitch, mpu6500_real_data2.Gyro_X, atan2(mpu6500_real_data2.Accel_Z, mpu6500_real_data2.Accel_Y));
//    /* ROLL: gy, atan2(ax,az) */
//    IMU_Data.roll = 57.296f*KALMAN2_Filterate(&kalmanRoll, mpu6500_real_data2.Gyro_Y, atan2(mpu6500_real_data2.Accel_X, mpu6500_real_data2.Accel_Z));
//    /* Temperature */
//    MPU6500ReadTEMP();
//    IMU_Data.temp = mpu6500_real_data2.Temp;
//}

//void testKalman(void)
//{
//    printf("KalmanYaw:%f, %f\r\n",kalmanYaw.state[0],kalmanYaw.state[1]);
//}


/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/


/* 
 * @FROM: https://blog.csdn.net/baidu_31872269/article/details/71426301
 */
///** Function
// * @brief   
// *   Init fields of structure @kalman1_state.
// *   I make some defaults in this init function:
// *     A = 1;
// *     H = 1; 
// *   and @q,@r are valued after prior tests.
// *
// *   NOTES: Please change A,H,q,r according to your application.
// *
// * @inputs  
// *   state - Klaman filter structure
// *   init_x - initial x state value   
// *   init_p - initial estimated error convariance
// * @outputs 
// * @retval  
// */
//void kalman1_init(kalman1_state *state, float init_x, float init_p)
//{
//    state->state = init_x;
//    state->covar = init_p;
//    state->dt = 1;
//    state->argH = 1;
//    state->argQ = 2e2;//10e-6;  /* predict noise convariance */
//    state->argR = 5e2;//10e-5;  /* measure error convariance */
//}

///** Function
// * @brief   
// *   1 Dimension Kalman filter
// * @inputs  
// *   state - Klaman filter structure
// *   z_measure - Measure value
// * @outputs 
// * @retval  
// *   Estimated result
// */
//float kalman1_filter(kalman1_state *state, float z_measure)
//{
//    /* Predict */
//    state->state = state->dt * state->state;
//    state->covar = state->dt * state->dt * state->covar + state->argQ;  /* p(n|n-1)=A^2*p(n-1|n-1)+q */

//    /* Measurement */
//    state->argK = state->covar * state->argH / (state->covar * state->argH * state->argH + state->argR);
//    state->state = state->state + state->argK * (z_measure - state->argH * state->state);
//    state->covar = (1 - state->argK * state->argH) * state->covar;

//    return state->state;
//}
