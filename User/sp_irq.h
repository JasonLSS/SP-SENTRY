/**
  ******************************************************************************
  * @file       sp_irq.h
  * @author     YTom
  * @version    v0.0-alpha
  * @date       2018.Nov.27
  * @brief      General interrupt request functions
  * @usage      
  ******************************************************************************
  * @license
  *
  ******************************************************************************
  */

#ifndef __SP_IRQ_H
#define __SP_IRQ_H


#include "sp_conf.h"
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup SP
  * @brief      SuperPower
  * @{
  */

/** @defgroup IRQ
  * @brief    IRQ Module
  * @{
  */

/** @defgroup IRQ_Definations
  * @brief    IRQ Exported Macros And Definations
  * @ingroup  IRQ
  * @{
  */
typedef void (*IRQ_Callback_t)(void);
 
/**
 * @brief Callback unit type, which could be used in IRQ functions.
 */
struct __spIRQ_CbUnit {
    /**
     * @brief STM32F4XX Interrupt Number Definition, according to the selected device 
     *        in @ref Library_configuration_section 
     */
    IRQn_Type                           irq_type;
    IRQ_Callback_t                      callback;           /*!< Callback function. */
    struct __spIRQ_CbUnit*              next;               /*!< Next callback unit. */
};
typedef struct __spIRQ_CbUnit spIRQ_CbUnit_t;

/**
 * @brief The empty spIRQ_CbUnit_t.
 */
static const spIRQ_CbUnit_t             spIRQ_CbNull = {(IRQn_Type)-1, NULL, NULL};

/**
 * @brief A reserved pool for spIRQ_CbUnit_t.
 */
extern spIRQ_CbUnit_t                   spIRQ_CbPool[USING_IRQ_POOL_SIZE];

/**
 * @brief Callback entries for each IRQ function.
 */
extern spIRQ_CbUnit_t*                  spIRQ_CbEntries[91];
/** @} */



/** @defgroup IRQ_API
  * @brief    IRQ user operations
  * @ingroup  IRQ
  * @{
  */

/**
 * @brief System hosted init funtion.
 */
void IRQ_ManagerInit(void);

/**
 * @brief User interfaces
 */
extern struct __IRQ_Manager_Type {
    /**
     * @brief Module initialization
     */
    void (*init)(void);
    /**
     * @brief The empty spIRQ_CbUnit_t.
     */
    spIRQ_CbUnit_t* (*registe)(IRQn_Type, IRQ_Callback_t);
    /**
     * @brief The empty spIRQ_CbUnit_t.
     */
    void (*invoke)(IRQn_Type);
} spIRQ_Manager;
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

#endif

/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/
