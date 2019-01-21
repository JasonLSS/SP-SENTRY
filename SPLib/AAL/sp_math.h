/**
  ******************************************************************************
  * @file       sp_math.h
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

/* Define to prevent recursive inclusion -------------------------------------*/
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

     
/** @defgroup   Common Mathematical Functions
  * @ingroup    MATH
  * @{
  */

/**
  * @brief  Reset values for a float32 array
  */ 
extern __inline void memset_f32(float* array, float value, uint16_t size);

/**
  * @brief  Carmack's Unusual Inverse Square Root
  */
extern __inline float InvSqrt(float x);

/** 
  * @}
  */
  
  
/** @defgroup   Value Constraint Functions
  * @ingroup    MATH
  * @{
  */

/**
  * @brief  Get number's sign.
  * @note   sign(x) = 1  if x > 0
  *         sign(x) = 0  if x = 0
  *         sign(x) = -1 if x < 0
  */ 
extern __inline float sign(float x);

/**
  * @brief  Limit value in a bilateral range
  * @param  x: value for limiting
  * @param  limit: bilateral limitation
  * @note   x will output between [-limit, limit]
  */ 
extern __inline float limit_bilateral(float x, float limit);

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
extern __inline float limit_minmax(float x, float min, float max);

/**
  * @brief  Make bilateral deadzone for value
  * @param  x: value for limiting
  * @param  deadzone: bilateral deadzone
  * @note   x between (-deadzone, deadzone) will be filtered.
  */ 
extern __inline float limit_deadzone_bilateral(float x, float deadzone);

/**
  * @brief  Make minmax deadzone for value
  * @param  x: value for limiting
  * @param  min: lower bound of deadzone
  * @param  max: upper bound of deadzone
  * @note   x between (-min, max) will be filtered.
  */ 
extern __inline float limit_deadzone_minmax(float x, float min, float max);

/** 
  * @}
  */



/** @defgroup   LPF Singnal Low Pass Filter Functions
  * @ingroup    MATH
  * @{
  */

/**
  * @brief  I-order RC low pass filter struct
  * @note   Excpression: \f[ V_o(k) = \frac{ V_i(k)+\frac{RC}{T_s}*V_o(k-1) }{ 1+ \frac{RC}{T_s} } \f]
  */
typedef struct {
    float  Vi;
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


/** @defgroup   HPF Singnal High Pass Filter Functions
  * @ingroup    MATH
  * @{
  */

/**
  * @brief  I-order RC high pass filter struct
  * @note   Excpression: \f[ V_o(k) = (V_i(k) -  V_i(k-1) +  V_o(k-1)) \frac{ RC }{ RC + T_s} \f]
  
  */
typedef struct {
    float  Vi;
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
