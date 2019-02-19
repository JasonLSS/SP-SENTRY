#ifndef __LQR_CONTROLLER_H
#define __LQR_CONTROLLER_H

#include <stdint.h>
#include <math.h>
#include "sp_conf.h"
#include "sp_math.h"

/*
    expression:

    \left\{
    \begin{array}{**lr**}
        \dot{x}=Ax+Bu \\
        y=Cx+D
    \end{array}
    \right. \\

    u=-Kx \\
    A^TP+PA+Q-PBR^{-1}B^TP=0 \\
    K=R^{-1}B^TP

*/


#define IMU_PI              3.141592653f                /*!< PI definition */
#define IMU_RAD2DEG(x)      ((x) * 57.2957795f)         /*!< Radians to degrees converter */
#define IMU_DEG2RAD(x)      ((x) * 0.0174532925f)       /*!< Radians to degrees converter */

/*
    model:

    \left\{
    \begin{array}{**lr**}
        \begin{bmatrix} \dot{\theta} \\ \dot{\omega} \end{bmatrix}
        = \begin{bmatrix} 0 & 1 \\ -\frac{k}{m} & -\frac{c}{m} \end{bmatrix} \times
          \begin{bmatrix} \theta \\ \omega \end{bmatrix} +
          \begin{bmatrix} 0 \\ k_B\Delta(\theta) \end{bmatrix} \\

        y = \begin{bmatrix} \theta \\ \omega \end{bmatrix} \\

        B=1,C=I, D=0
    \end{array}
    \right.
*/
typedef struct {
    float freq;
    struct {
        const float _P[4];
        const float _K[4];
    } _private;
    struct {
        const float A[4];
        float kB;
        float Q[4];
        float R[4];
    } param;
    struct {
        float theta;
        float omega;
    } x;
} LQR_Motor_t;


void LQR_Motor_Init(
    LQR_Motor_t* lqr, 
    const float paramA[4], 
    float kB, 
    const float paramQ[4], 
    const float paramR[4]);

void LQR_Motor_Update(
        LQR_Motor_t* lqr, 
        float target_theta,
        float current_theta, 
        float omega,
        float dt);

#endif // __LQR_CONTROLLER_H
