/**
  ******************************************************************************
  * @file       sp_type.c
  * @author     YTom
  * @version    v0.0-alpha
  * @date       2018.Nov.10
  * @brief      General type definations.
  ******************************************************************************
  * @license
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SP_TYPE_H
#define __SP_TYPE_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx.h"

/* Exported types ------------------------------------------------------------*/

//#ifndef __cplusplus
///**
// * @brief   User-implemented bool type.
// */
//typedef enum {
//    false = 0,
//    true = 1
//} bool;
//#endif


typedef struct {
    GPIO_TypeDef*       gpio;
    uint32_t            pin_source;
} spPinSet;
static const spPinSet spNullPin = {NULL};



/** @defgroup General Implement Functions Type
  * @brief    Functions for simple usages without parameters.
  * @{
  */
/**
  * @brief   Type of implement function without return values.
  */
typedef void (*tFuncImplementNoRet)(void);

/**
  * @brief   Type of implement function with bool return values.
  */
typedef bool (*tFuncImplementRetBool)(void);

/**
  * @brief   Type of implement function based on systick.
  */
typedef void (*tFuncImplementSystickBased)(uint32_t);

/**
  * @}
  */



/** @defgroup General Callback Functions Type
  * @brief    Functions for callback.
  * @{
  */
/**
 * @brief   Type of implement function with bool return values.
 */
typedef void (*tFuncCallbackVoid)(void);

/**
  * @}
  */
  

/** @defgroup Member Functions Type
  * @brief    Functions for struct member function pointer without parameters
  * @note     Member functions should start with @type void* which reserved for pointer *this*
  * @{
  */
/**
 * @brief   Type of member function of a struct.
 */
typedef void (*tFuncMemberNoParam)(void*);

/**
  * @}
  */



/** @defgroup Universal Types
  * @brief    
  * @note     
  * @{
  */
/**
 * @brief   Type of time stamp
 */
typedef struct {
    uint32_t        ms;                     /*!< Current millisecond */
    uint32_t        us;                     /*!< Current microsecond */
} spTimeStamp;


/**
  * @}
  */


/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/


#ifdef __cplusplus
}
#endif

#endif /*__SP_TYPE_H */

/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/
