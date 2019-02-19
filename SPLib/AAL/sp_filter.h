/**
  ******************************************************************************
  * @file       sp_filter.h
  * @author     YTom
  * @version    v0.0-alpha
  * @date       2018.Nov.27
  * @brief      Some algorithmic functions of filter
  * @usage      
  ******************************************************************************
  * @license
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SP_FILTER_H
#define __SP_FILTER_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdlib.h>
#include <stdint.h>
#include <math.h>

/**
  * @brief  Using limited moving abverage on data list, fitting type:
  *         f32 @ref float; d32 @ref uint32_t; d16 @ref uint16_t 
  * @note   Smaller the index is, the value is newer. this means data[0] is the newest value.
  *         New value will be limited within @expr data[0]-rela_limit to @expr data[0]+rela_limit
  * @param  data: recent data list
  * @param  size: size of restored recent data
  * @param  newval: new value prepend to the list
  * @param  rela_limit: relative limit on new value
  * @retval Newest value calculated by moving average filter.
  */ 
float MovingAverageFilter_f32(float* data, uint8_t size, float newval, uint16_t rela_limit);
int32_t MovingAverageFilter_d32(int32_t* data, uint8_t size, int32_t newval, uint16_t rela_limit);
int16_t MovingAverageFilter_d16(int16_t* data, uint8_t size, int16_t newval, uint16_t rela_limit);


#ifdef __cplusplus
}
#endif

#endif /*__SP_FILTER_H */

/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/
