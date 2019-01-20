/**
  ******************************************************************************
  * @file       sp_filter.c
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

/* Includes ------------------------------------------------------------------*/
#include "sp_filter.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @defgroup Utility of General Algorithms
  * @{
  */
  
float MovingAverageFilter_f32(float* data, uint8_t size, float newval, uint16_t rela_limit) {
    float avg = 0;
    /* Amplitude restriction */
    if(fabs(newval - data[0]) > rela_limit) {
        newval = data[0] + ((newval>data[0])?rela_limit:-rela_limit);
    }
    /* Calc moving average */
    for(uint16_t i=size-1; i>0; i--) {
        data[i] = data[i-1];
        avg += data[i]/size;
    }
    data[0] = newval;
    avg += data[0]/size;
    return (data[0] = avg);
}

int32_t MovingAverageFilter_d32(int32_t* data, uint8_t size, int32_t newval, uint16_t rela_limit) {
    int32_t avg = 0;
    /* Amplitude restriction */
    if(abs(newval - data[0]) > rela_limit) {
        newval = data[0] + ((newval>data[0])?rela_limit:-rela_limit);
    }
    /* Calc moving average */
    for(uint16_t i=size-1; i>0; i--) {
        data[i] = data[i-1];
        avg += data[i]/size;
    }
    data[0] = newval;
    avg += data[0]/size;
    return (data[0] = avg);
}

int16_t MovingAverageFilter_d16(int16_t* data, uint8_t size, int16_t newval, uint16_t rela_limit) {
    int16_t avg = 0;
    /* Amplitude restriction */
    if(abs(newval - data[0]) > rela_limit) {
        newval = data[0] + ((newval>data[0])?rela_limit:-rela_limit);
    }
    /* Calc moving average */
    for(uint16_t i=size-1; i>0; i--) {
        data[i] = data[i-1];
        avg += data[i]/size;
    }
    data[0] = newval;
    avg += data[0]/size;
    return (data[0] = avg);
}

/**
  * @}
  */


/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/
