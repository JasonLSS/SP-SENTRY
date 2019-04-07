/**
  ******************************************************************************
  * @file       sp_math.c
  * @author     YTom
  * @version    v0.0-alpha
  * @date       2018.Dec.10
  * @brief      Some useful math functions
  @verbatim      

  @endverbatim
  ******************************************************************************
  * @license
  *
  ******************************************************************************
  */


#include "sp_math.h"

float limit_bilateral_loop(float x, float limit) {
    if(limit==0) {
        return x;
    }
    if(limit < 0) {
        limit = fabs(limit);
    }
    return x - (int)((fabs(x)+limit)/(2*limit))*2*limit*sign(x);
}


void LPF_FirstOrder_Init(LPF_FirstOrder_type* lpf, float cutFreq, float sampleFreq ) {
    lpf->cutFreq = cutFreq;
    lpf->sampleFreq = sampleFreq;
    lpf->Vo = lpf->Vo_prev = 0.f;
}

float LPF_FirstOrder_filter(LPF_FirstOrder_type* lpf, float Vi ) {
    float RC = 1.f/(2.f*PI*lpf->cutFreq);
    float denom = (1.f+RC*lpf->sampleFreq);
    lpf->Vo = (Vi + (RC*lpf->sampleFreq)*lpf->Vo_prev)/denom;
    lpf->Vo_prev = lpf->Vo;
    return lpf->Vo;
}


void HPF_FirstOrder_Init(HPF_FirstOrder_type* lpf, float cutFreq, float sampleFreq ) {
    lpf->cutFreq = cutFreq;
    lpf->sampleFreq = sampleFreq;
    lpf->Vi_prev = lpf->Vo = lpf->Vo_prev = 0.f;
}

float HPF_FirstOrder_filter(HPF_FirstOrder_type* lpf, float Vi ) {
    float RC = 1.f/(2.f*PI*lpf->cutFreq);
    float factor = RC/(RC + 1.f/lpf->sampleFreq);
    lpf->Vo = (Vi - lpf->Vi_prev +  lpf->Vo_prev) * factor;
    lpf->Vi_prev = Vi;
    lpf->Vo_prev = lpf->Vo;
    return lpf->Vo;
}


/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/
