/**
  ******************************************************************************
  * @file       sp_runner.h
  * @author     YTom
  * @version    v0.0-alpha
  * @date       2018.Nov.21
  * @brief      Time-based process runner
  * @usage      
  ******************************************************************************
  * @license
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SP_RUNNER_H
#define __SP_RUNNER_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "sp_conf.h"

/* Exported types ------------------------------------------------------------*/
/**
  * @brief  Returned subevent status
  */ 
typedef enum {
    Runner_Null = 0,      //没有动作
    Runner_Skip = 1,      //跳过当前步骤
    Runner_End = 2        //结束整个流程(可调用结束事件)
} Runner_RetFlag;

/**
  * @brief  
  */ 
typedef void(*pRunnerFunc)(void);
typedef Runner_RetFlag(*pAutoDetectFunc)(uint8_t);

/**
  * @brief  
  */ 
typedef struct RunnerUnit{
    uint8_t                 id;             /* Unit ID */
    uint16_t                time;           /* Time for current unit */
    uint16_t                cnt;            /* Unit timer counter */
    uint16_t                loop;           /* If unit circularly running,  0<=loop<time will circularly running */
    pRunnerFunc             func;           /* Unit event function */
    pAutoDetectFunc         detect;         /* User-defined detection, can skip this unit event */
} RunnerUnit, *pRunnerUnit;

/**
  * @brief  
  */ 
typedef struct {
    uint16_t                length:8;       /* Total size of units */
    uint16_t                enable:1;       /* If current process useful */
    uint16_t                terminate:1;    /* User-terminated flag */
    pRunnerUnit             entry;          /* Entry of units list */
    uint8_t                 currID;         /* Current unit id, should limit in 0~(length-1) */
    
    tFuncMemberNoParam      run;            /* Run this process */
    tFuncMemberNoParam      stop;           /* Stop this process with callback */
    tFuncMemberNoParam      force_stop;     /* Stop this process ignoring callback */
    
    pRunnerFunc             end_cb;         /* Callbakc on event ending */
} RunnerEvent;

/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/** @defgroup Runner Module Use Intefaces
  * @{
  */
/**
  * @brief  Register a new runner
  * @param  runner: new runner list of @ref RunnerEvent
  * @param  length: total size of units
  * @param  units: list of units of @ref RunnerUnit
  * @param  end_callback: callback function on runner ending
  * @retval If register succeed.
  */
bool Runner_RegisterEvent(
    RunnerEvent*    runner,
    uint8_t         length,
    RunnerUnit*     units,
    pRunnerFunc     end_callback);

/**
  * @brief  Start a new runner process
  * @param  runner: runner object
  */
void Runner_Start(RunnerEvent* runner);

/**
  * @brief  Stop a runner process
  * @param  runner: runner object
  * @note   This function will trigger @ref end_callback()
  */
void Runner_Stop(RunnerEvent* runner);

/**
  * @brief  Force a runner to stop
  * @param  runner: runner object
  */
void Runner_ForceStop(RunnerEvent* runner);

/**
  * @}
  */



/** @defgroup Runner Module Control Looper\
  * @note     SYSTEM OWNED
  * @{
  */

/**
  * @brief  Internal looper
  * @param  tick: timer counter value from system looper
  */
void Runner_ControlLooper(uint32_t tick);
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /*__SP_RUNNER_H */

/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/

