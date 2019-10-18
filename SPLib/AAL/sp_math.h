/**
  ******************************************************************************
  * @file       sp_math.h
  * @author     YTom
  * @version    v0.0-alpha
  * @date       2018.Dec.10
  * @brief      Some useful math functions
  @verbatim      
  Test case:
        LPF_FirstOrder_type lpf;
        HPF_FirstOrder_type hpf;
        LPF_FirstOrder_Init(&lpf, 10, 100);
        HPF_FirstOrder_Init(&hpf, 10, 100);
        ------------------------------------
        if(tick % 10 == 5) {
            extern char uart_tx_buff[256];;
            float time = TASK_GetSecond();
            float freq = 0.2f*3.1415926f*time;
            float val = sin(freq*time);
            float out = HPF_FirstOrder_filter(&hpf, val);
            uint8_t size = sprintf(uart_tx_buff, "%f,%f,%f,%f\n", val, out, freq, time);
            spDMA.controller.start(spDMA_USART6_tx_stream, (uint32_t)uart_tx_buff, (uint32_t)&USART6->DR, size);
        }
  @endverbatim
  ******************************************************************************
  * @license
  *
  ******************************************************************************
  */

#ifndef __SP_MATH_H
#define __SP_MATH_H

#ifdef __cplusplus
 extern "C" {
#endif


/** @addtogroup SP
  * @{
  */

/** @addtogroup MATH
  * @{
  */

#include <stdint.h>
#include "stm32f4xx.h"
#include <arm_math.h>
#include <math.h>




     
/** @defgroup   Common_Transformer
  * @ingroup    MATH
  * @{
  */

/**
  * @brief  Unit conversion from RPM to rad/s.
  */ 
#define spMATH_RPM2RAD(x)       ((x)*0.104719753f)

/**
  * @brief  Unit conversion from rad/s to RPM.
  */ 
#define spMATH_RAD2RPM(x)       ((x)*9.549296748f)

/**
  * @brief  Unit conversion from rad to deg.
  */ 
#define spMATH_RAD2DEG(x)       ((x)*57.29578049f)

/**
  * @brief  Unit conversion from deg to rad.
  */ 
#define spMATH_DEG2RAD(x)       ((x)*0.017453292f)



/**
  * @brief  Reset values for a float32 array
  */ 
static __inline void memset_f32(float* array, float value, uint16_t size) {
    while(size--) {
        *(array++) = value;
    }
}

/**
  * @brief  Carmack's Unusual Inverse Square Root
        Fast inverse square-root
        See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
  */
static __inline float inv_sqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    y = y * (1.5f - (halfx * y * y));
    return y;
}


/**
  * @brief  Exchange two values.
  * @param A: pointer of first value, which should be cast to uint8_t* type.
  * @param B: pointer of second value, which should be cast to uint8_t* type.
  * @param size: size of the value.
  */ 
static __inline void swap(void* A, void* B, uint16_t size) {
    uint8_t* ptr_a = (uint8_t*)A;
    uint8_t* ptr_b = (uint8_t*)B;
    
    for(uint16_t i=0; i<size; i++) {
        ptr_a[i] ^= ptr_b[i];
        ptr_b[i] ^= ptr_a[i];
        ptr_a[i] ^= ptr_b[i];
    }
}


/** 
  * @}
  */
  
  
/** @defgroup   Value_Constraint
  * @ingroup    MATH
  * @{
  */

/**
  * @brief  Get number's sign.
  * @note   sign(x) = 1  if x > 0
  *         sign(x) = 0  if x = 0
  *         sign(x) = -1 if x < 0
  */ 
static __inline float sign(float x) {
    return (x==0)?0:((x>0)?1:-1);
}

/**
  * @brief  Limit value in a bilateral range
  * @param  x: value for limiting
  * @param  limit: bilateral limitation
  * @note   x will output between [-limit, limit]
  */ 
static __inline float limit_bilateral(float x, float limit) {
    if(fabs(x)>fabs(limit)) {
        return fabs(limit)*sign(x);
    }
    return x;
}

/**
  * @brief  Make loop restriction of value in a bilateral range
  * @param  x: value for limiting
  * @param  limit: bilateral limitation
  * @note   x will output between [-limit, limit] in a loop way
  *         eg. limit_bilateral_loop(10, 3) = -2
  *         eg. limit_bilateral_loop(11.2, 1.0) = -0.8
  */ 
extern __inline float limit_bilateral_loop(float x, float limit);

/**
  * @brief  Limit value in a range
  * @param  x: value for limiting
  * @param  min: lower bound of input value
  * @param  max: upper bound of input value
  * @note   x will output between [min, max]
  */ 
static __inline float limit_minmax(float x, float min, float max) {
    if(min>max) {
        swap((uint8_t*)&min, (uint8_t*)&max, sizeof(min));
    }
    return (x>max)?max:((x<min)?min:x);
}

/**
  * @brief  Make bilateral deadzone for value
  * @param  x: value for limiting
  * @param  deadzone: bilateral deadzone
  * @note   x between (-deadzone, deadzone) will be filtered.
  */ 
static __inline float limit_deadzone_bilateral(float x, float deadzone) {
    deadzone = fabs(deadzone);
    return (x<deadzone && x>-deadzone)?0:x;
}

/**
  * @brief  Make minmax deadzone for value
  * @param  x: value for limiting
  * @param  min: lower bound of deadzone
  * @param  max: upper bound of deadzone
  * @note   x between (-min, max) will be filtered.
  */ 
static __inline float limit_deadzone_minmax(float x, float min, float max) {
    if(min>max) {
        swap((uint8_t*)&min, (uint8_t*)&max, sizeof(min));
    }
    return (x<max && x>min)?0:x;
}

/**
  * @brief  Use exp type offset sigmoid function.
  * @param  x: value for limiting
  * @param  d: half of the growth range
  * @param  M: max output value
  * @note   Excpression: \f[ M(2\frac{e^{\frac{6}{d}x}}{e^{\frac{6}{d}x} + 1}-1),|x|<M \f]
  */ 
static __inline float sigmoid_offset(float x, float d, float M) {
    d = fabs(d);
    M = fabs(M);
    if(fabs(x) < M) {
        float expval = exp(x*6.f/d);
        return (2.f*expval/(expval+1)-1.f)*M;
    }
    return x;
}

/**
  * @brief  Redefine vlaue performance in deadzone.
  * @param  x: value for limiting
  * @param  d: half of the growth range
  * @param  M: max output value
  * @note   x in (-n,n) will be 0, x in (-m,-n) wiil be -gain, x in (n,m) wiil be gain, otherwise will be x.
  */ 
static __inline float deadzone_gain(float x, float n, float m) {
    n = fabs(n);
    m = fabs(m);
    if(fabs(x) > m)
        return x;
    if(fabs(x) < n)
        return 0;
    return m*sign(x);
}

/** 
  * @}
  */



/** @defgroup   Low_Pass_Filter
  * @ingroup    MATH
  * @{
  */

/**
  * @brief  I-order RC low pass filter struct
  * @note   Excpression: \f[ V_o(k) = \frac{ V_i(k)+\frac{RC}{T_s}*V_o(k-1) }{ 1+ \frac{RC}{T_s} } \f]
  */
typedef struct {
//    float  Vi;
    float  Vo_prev;
    float  Vo;
    float  cutFreq;
    float  sampleFreq;
} LPF_FirstOrder_type;

/**
  * @brief  Init an I-order RC low pass filter
  * @param  lpf: LPF struct @ref LPF_FirstOrder_type
  * @param  cutFreq: cut-off frenquency
  * @param  sampleFreq: sample frenquency
  */
void LPF_FirstOrder_Init(LPF_FirstOrder_type* lpf, float cutFreq, float sampleFreq );

/**
  * @brief  I-order RC low pass filter
  * @param  lpf: LPF struct @ref LPF_FirstOrder_type
  * @param  Vi: Current iutput sample value
  * @retval Current output value.
  */
float LPF_FirstOrder_filter(LPF_FirstOrder_type* lpf, float Vi );

/** 
  * @}
  */


/** @defgroup   High_Pass_Filter
  * @ingroup    MATH
  * @{
  */

/**
  * @brief  I-order RC high pass filter struct
  * @note   Excpression: \f[ V_o(k) = (V_i(k) -  V_i(k-1) +  V_o(k-1)) \frac{ RC }{ RC + T_s} \f]
  */
typedef struct {
//    float  Vi;
    float  Vi_prev;
    float  Vo;
    float  Vo_prev;
    float  cutFreq;
    float  sampleFreq;
} HPF_FirstOrder_type;

/**
  * @brief  Init an I-order RC high pass filter
  * @param  lpf: LPF struct @ref HPF_FirstOrder_type
  * @param  cutFreq: cut-off frenquency
  * @param  sampleFreq: sample frenquency
  */
void HPF_FirstOrder_Init(HPF_FirstOrder_type* lpf, float cutFreq, float sampleFreq );

/**
  * @brief  I-order RC high pass filter
  * @param  lpf: LPF struct @ref HPF_FirstOrder_type
  * @param  Vi: Current iutput sample value
  * @retval Current output value.
  */
float HPF_FirstOrder_filter(HPF_FirstOrder_type* lpf, float Vi );

/** 
  * @}
  */


/**
  * @}
  */

/**
  * @}
  */


#ifdef __cplusplus
}
#endif

#endif /*__SP_MATH_H */

/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/
