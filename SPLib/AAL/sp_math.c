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

/* Includes ------------------------------------------------------------------*/
#include "sp_math.h"

void memset_f32(float* array, float value, uint16_t size) {
    while(size--) {
        *(array++) = value;
    }
}

float invSqrt(float x) {
    float xhalf = 0.5f * x;
    int i = *(int*)&x;              // get bits for floating value
    i = 0x5f375a86 - (i >> 1);      // gives initial guess y0
    x = *(float*)&i;                // convert bits back to float
    x = x * (1.5f - xhalf * x * x); // Newton step, repeating increases accuracy
    return x;
}

float sign(float x) {
    return (x==0)?0:((x>0)?1:-1);
}

float limit_bilateral(float x, float limit) {
    if(fabs(x)>fabs(limit)) {
        return fabs(limit)*sign(x);
    }
    return x;
}

float limit_bilateral_loop(float x, float limit) {
    if(limit==0) {
        return x;
    }
    if(limit < 0) {
        limit = fabs(limit);
    }
    return x - (int)((fabs(x)+limit)/(2*limit))*2*limit*sign(x);
}

float limit_minmax(float x, float min, float max) {
    if(min>max) {
        return x;
    }
    return (x>max)?max:((x<min)?min:x);
}

float limit_deadzone_bilateral(float x, float deadzone) {
    if(deadzone < 0) {
        deadzone = fabs(deadzone);
    }
    return (x<deadzone && x>-deadzone)?0:x;
}

float limit_deadzone_minmax(float x, float min, float max) {
    if(min>max) {
        return x;
    }
    return (x<max && x>min)?0:x;
}


void LPF_FirstOrder_Init(LPF_FirstOrder_type* lpf, float cutFreq, float sampleFreq ) {
    lpf->cutFreq = cutFreq;
    lpf->sampleFreq = sampleFreq;
    lpf->Vi = lpf->Vo = lpf->Vo_prev = 0.f;
}

float LPF_FirstOrder_filter(LPF_FirstOrder_type* lpf, float Vi ) {
    float RC = 1.f/(2.f*PI*lpf->cutFreq);
    float denom = (1.f+RC*lpf->sampleFreq);
    lpf->Vo = lpf->Vi/denom + lpf->sampleFreq/denom*lpf->Vo_prev;
    return lpf->Vo;
}


void HPF_FirstOrder_Init(HPF_FirstOrder_type* lpf, float cutFreq, float sampleFreq ) {
    lpf->cutFreq = cutFreq;
    lpf->sampleFreq = sampleFreq;
    lpf->Vi = lpf->Vi_prev = lpf->Vo = lpf->Vo_prev = 0.f;
}

float HPF_FirstOrder_filter(HPF_FirstOrder_type* lpf, float Vi ) {
    float RC = 1.f/(2.f*PI*lpf->cutFreq);
    float factor = RC/(RC + 1.f/lpf->sampleFreq);
    lpf->Vo = (lpf->Vi - lpf->Vi_prev +  lpf->Vo_prev) * factor;
    return lpf->Vo;
}


/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/
