#include "string.h"
#include "lqr.h"

static inline void matrix_add_2x2(
    const float mata[4],
    const float matb[4],
    float matc[4]) {
    
    matc[0] = mata[0] + matb[0];
    matc[1] = mata[1] + matb[1];
    matc[2] = mata[2] + matb[2];
    matc[3] = mata[3] + matb[3];
}
    
static inline void matrix_sub_2x2(
    const float mata[4],
    const float matb[4],
    float matc[4]) {
    
    matc[0] = mata[0] - matb[0];
    matc[1] = mata[1] - matb[1];
    matc[2] = mata[2] - matb[2];
    matc[3] = mata[3] - matb[3];
}

static inline void matrix_multi_2x2(
    const float mata[4],
    const float matb[4],
    float matc[4]) {
    
    matc[0] = mata[0]*matb[0] + mata[1]*matb[2];
    matc[1] = mata[0]*matb[1] + mata[1]*matb[3];
    matc[2] = mata[1]*matb[0] + mata[2]*matb[2];
    matc[3] = mata[1]*matb[1] + mata[2]*matb[3];
}


static inline void matrix_inv_2x2(
    const float mata[4],
    float matc[4]) {
    float denomin = 1.f/(mata[0]*mata[3]-mata[1]*mata[2]);
    matc[0] = mata[1]*denomin;
    matc[1] = mata[0]*denomin;
    matc[2] = -mata[2]*denomin;
    matc[3] = -mata[3]*denomin;
}


static inline void matrix_transpose_2x2(
    const float mata[4],
    float matc[4]) {
        
    matc[0] = mata[0];
    matc[1] = mata[2];
    matc[2] = mata[1];
    matc[3] = mata[3];
}




void LQR_Motor_Init(
    LQR_Motor_t* lqr, 
    const float paramA[4], 
    float kB, 
    const float paramQ[4], 
    const float paramR[4]) {
    
    lqr->x.theta = lqr->x.omega = 0.f;
    memcpy((void*)(lqr->param.A), paramA, sizeof(lqr->param.A));
    lqr->param.kB = kB;
    memcpy((void*)(lqr->param.Q), paramQ, sizeof(lqr->param.Q));
    memcpy((void*)(lqr->param.R), paramR, sizeof(lqr->param.R));
        
//    float A_P[4],PA[4],RB[4];
//    matrix_inv_2x2(lqr->param.R, R_);
//    matrix_inv_2x2(lqr->param.R, R_);
    
}

void LQR_Motor_Update(
        LQR_Motor_t* lqr, 
        float target_theta,
        float current_theta, 
        float omega,
        float dt) {
    
    float theta_ = omega;
    float omega_ = lqr->param.A[2]*current_theta + lqr->param.A[3]*omega + \
        lqr->param.kB*(target_theta-current_theta);
    




}