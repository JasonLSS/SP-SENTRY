/**
  ******************************************************************************
  * @file       sp_flash.
  * @author     YTom
  * @version    v0.0-alpha
  * @date       2019.Jul.25
  * @brief      Flash storage module.
  *             Last 512kb(Sec20~Sec23) of 2MB flash is using for storage.
  * @usage      
  ******************************************************************************
  * @license
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SP_FLASH_H
#define __SP_FLASH_H

#ifdef __cplusplus
 extern "C" {
#endif

/** @addtogroup SP
  * @brief      SuperPower
  * @{
  */

/** @defgroup FLASH
  * @brief    FLASH Module
  * @{
  */
#include "sp_type.h"
#include "stm32f4xx_flash.h"

/** @defgroup FLASH_Declarations
  * @ingroup  FLASH
  * @{
  */
#define __FLASH_END_PTR     (0x08200000)
#define __FLASH_SIZE        (0x00080000)
#define __FLASH_START_PTR   (__FLASH_END_PTR-__FLASH_SIZE)

#define __AT(addr)          __attribute__((at(addr))) 

typedef packed_struct {
    char        name[16];
    size_t      size;
    uint32_t    address;        /* Returned value */
} flash_require_t;
/** @} */


/** @defgroup FLASH_APIs
  * @ingroup  FLASH
  * @{
  */
extern const struct FLASH_Controllers_Type {
    /** 
      * @brief  Initialize flash manager.
      * @param  
      * @retval If succeed
      */
    int (*init)(void);
    
    bool (*unlock)(void);
    bool (*lock)(void);
    bool (*islock)(void);
    int (*format)(void);
    
    int (*alloc)(const flash_require_t* req);
} spFLASH;
/** @} */

/**
  * @}
  */
  
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /*__SP_FLASH_H */

/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/
