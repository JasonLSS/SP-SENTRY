/**
  ******************************************************************************
  * @file       sp_kalman.h
  * @author     @YTom, ybb331082@126.com
  * @version    v0.0-alpha
  * @date       2018.Jun.24
  * @brief      Kalman Filter module
  * @note       
  *
  *     卡尔曼滤波
  *     (注:噪声均符合高斯分布)
  *     Model:
  *         X(k|k-1)=AX(k-1|k-1)+BU(k)+W(k)
  *         Z(k)=HX(k-1|k-1)+V(k)
  *     Sign:
  *         ^=transpose
  *         '=inverse
  *     Kalman Functions:
  *         X(k|k-1)=AX(k-1|k-1)+BU(k)                  ----1
  *         P(k|k-1)=AP(k-1|k-1)A^+Q                    ----2
  *         Kg(k)= P(k|k-1)H^(HP(k|k-1)H^+R)?           ----3 
  *         X(k|k)= X(k|k-1)+Kg(k)(Z(k)-HX(k|k-1))'     ----4
  *         P(k|k)=(I-Kg(k)H)P(k|k-1)                   ----5
  ******************************************************************************
  * @license
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SP_KALMAN_H
#define __SP_KALMAN_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include <arm_math.h>
#include <string.h>
#include "sp_math.h"

/* Exported types ------------------------------------------------------------*/
/** 
  * @brief  2-D Angular Kalman filter
  */
typedef struct {
    float state[2];         /* state: [0]-m=angle [1]-v=velocity, 2x1 */
//    float argA;             /* prior-estimate factor: 2*2 (here only need time) */
//    float argB;             /* input factor: 2*1(here only need time) */
    float dt;               /* delta time (alternate argA & argB) */
    float argH[2];          /* measurement factor */
    float argQ[2];          /* process(predict) noise convariance,2x1 [q0 q1] */
    float argR;             /* measure noise convariance */
    float covar[2][2];      /* estimated error convariance,2x2 [p0 p1; p2 p3] */
} Kalman2_Filter_Type;

typedef struct {
    float32_t     A[6][6];       /*!< 6*6 */
    float32_t     X[6][1];       /*!< 6*1 */
//    float32_t     W[6][1];       /*!< 6*1 */
    
    float32_t     H[3][6];       /*!< 3*6 */
//    float32_t     V[3][1];       /*!< 3*1 */
    
    float32_t     P[6][6];       /*!< 6*6 */
    float32_t     K[6][3];       /*!< 6*3 */
    
    float32_t     Q[6][6];       /*!< 6*6 */
    float32_t     R[3][3];       /*!< 3*3 */
} KALMAM_CoreType;

typedef struct {
    arm_matrix_instance_f32     mat_A;
    arm_matrix_instance_f32     mat_X;
//    arm_matrix_instance_f32     mat_W;
    
    arm_matrix_instance_f32     mat_H;
//    arm_matrix_instance_f32     mat_V;
    
    arm_matrix_instance_f32     mat_P;
    arm_matrix_instance_f32     mat_K;
    
    arm_matrix_instance_f32     mat_Q;
    arm_matrix_instance_f32     mat_R;
    
    KALMAM_CoreType             core;
} KALMAM_Type;


/* Exported macro ------------------------------------------------------------*/
/** Defines
  * @brief  1-D Kalman filter model
  */
#define KALMAN_InitGyroAngle(filter,x)            (filter.state[0]=x)
#define KALMAN_InitGyroBias(filter,x)             (filter.state[1]=x)

#define KALMAN_InitDeltaTime(filter,x)            (filter.dt=x)
#define KALMAN_InitH1(filter,x)                   (filter.argH[0]=x)
#define KALMAN_InitH2(filter,x)                   (filter.argH[1]=x)

#define KALMAN_InitCovarAngle(filter,x)           (filter.argQ[0]=x)
#define KALMAN_InitCovarGyroBias(filter,x)        (filter.argQ[1]=x)
#define KALMAN_InitCovarAcceAngle(filter,x)       (filter.argR=x)


/* Exported parameters -------------------------------------------------------*/
    
/* Exported functions --------------------------------------------------------*/
/** @defgroup CANx Basic Control Function
  * @brief    Implement basic CAN functions
  * @{
  */
  
/** 
  * @brief    Init 2D Kalman filter with default value
  * @param    kf: instance pointer of  @ref Kalman2_Filter_Type
  */
void KALMAN2_DeInit(Kalman2_Filter_Type *kf);

/** 
  * @brief  2 Dimension kalman filter
  * @param  kf        Kalman filter
  *         gyro    Angular velocity measurement from gyroscope
  *         acceang    Angle solve from acceleration measurement
  * @retval Equals to state->state[0], so maybe angle or velocity.
  * @note   Modle
  *             Acce = H.[ang] + R, H = [1 0]
  *                      [off]
  *             ang = Angle solve by gyro, off = gyro bias
  *             Generally. p2=p3=0
  */
float KALMAN2_Filterate(Kalman2_Filter_Type *kf, float anglemeasure, float gyro);


/** 
  * @brief
  * @note
    (1) Use accelerations as observed data and gyroscopic data as estimated model.
    (2) The accelerometer can only get pith-roll data using for rotation matrix.
        Pitch(x) = \phi = atan2(G_{py} / \sqrt{ G_{px}^2 + G_{pz}^2 } )
        Roll(y) = \theta = atan2( -G_{px} / G_{pz} )
*/
void KALMAN2_Mul_A_X(void);
void KALMAN_Init(KALMAM_Type* kalman);
void KALMAN_Filter(
    KALMAM_Type*    kalman, 
    float32_t*      matZ );

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /*__SP_KALMAN_H */

/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/

