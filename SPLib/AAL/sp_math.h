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


/** @addtogroup sp
  * @{
  */

/** @defgroup   Common Mathematical Functions
  * @{
  */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "stm32f4xx.h"
#include <arm_math.h>

/* Exported types ------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/**
  * @brief  Reset values for a float32 array
  */ 
static inline void memset_f32(float* array, float value, uint16_t size) {
    while(size--) {
        *(array++) = value;
    }
}


/**
  * @brief  Carmack's Unusual Inverse Square Root
  */ 
static inline float InvSqrt(float x) {
    float xhalf = 0.5f * x;
    int i = *(int*)&x;              // get bits for floating value
    i = 0x5f375a86 - (i >> 1);      // gives initial guess y0
    x = *(float*)&i;                // convert bits back to float
    x = x * (1.5f - xhalf * x * x); // Newton step, repeating increases accuracy
    return x;
}


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
