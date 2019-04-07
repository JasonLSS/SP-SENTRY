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

/* __packed keyword used to decrease the data type alignment to 1-byte */
#if defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 1)
    #define packed_struct       struct __attribute__((packed))
    #define packed_union        union __attribute__((packed))
#else
    #define packed_struct       __packed struct
    #define packed_union        __packed union
#endif
    
#define spINLINE __inline
#ifndef spFORCE_INLINE
    // #define portFORCE_INLINE __forceinline
    #define spFORCE_INLINE __attribute__((always_inline))
#endif

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
 * @brief   Bit-filed data type
 */
typedef packed_union {
    uint8_t data;
    struct {
        uint8_t bit0:1;
        uint8_t bit1:1;
        uint8_t bit2:1;
        uint8_t bit3:1;
        uint8_t bit4:1;
        uint8_t bit5:1;
        uint8_t bit6:1;
        uint8_t bit7:1;
    } bits;
} spByte_t;
typedef spByte_t spInt8_t;

typedef packed_union {
    uint16_t data;
    struct {
        uint16_t bit0:1;
        uint16_t bit1:1;
        uint16_t bit2:1;
        uint16_t bit3:1;
        uint16_t bit4:1;
        uint16_t bit5:1;
        uint16_t bit6:1;
        uint16_t bit7:1;
        uint16_t bit8:1;
        uint16_t bit9:1;
        uint16_t bit10:1;
        uint16_t bit11:1;
        uint16_t bit12:1;
        uint16_t bit13:1;
        uint16_t bit14:1;
        uint16_t bit15:1;
    } bits;
} spHalfWord_t;
typedef spHalfWord_t spInt16_t;

typedef packed_union {
    uint32_t data;
    struct {
        uint32_t bit0:1;
        uint32_t bit1:1;
        uint32_t bit2:1;
        uint32_t bit3:1;
        uint32_t bit4:1;
        uint32_t bit5:1;
        uint32_t bit6:1;
        uint32_t bit7:1;
        uint32_t bit8:1;
        uint32_t bit9:1;
        uint32_t bit10:1;
        uint32_t bit11:1;
        uint32_t bit12:1;
        uint32_t bit13:1;
        uint32_t bit14:1;
        uint32_t bit15:1;
        uint32_t bit16:1;
        uint32_t bit17:1;
        uint32_t bit18:1;
        uint32_t bit19:1;
        uint32_t bit20:1;
        uint32_t bit21:1;
        uint32_t bit22:1;
        uint32_t bit23:1;
        uint32_t bit24:1;
        uint32_t bit25:1;
        uint32_t bit26:1;
        uint32_t bit27:1;
        uint32_t bit28:1;
        uint32_t bit29:1;
        uint32_t bit30:1;
        uint32_t bit31:1;
    } bits;
} spWord_t;
typedef spWord_t spInt32_t;

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
