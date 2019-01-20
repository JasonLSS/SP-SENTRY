/**
  ******************************************************************************
  * @file       sp_runner.c
  * @author     YTom
  * @version    v0.0-alpha
  * @date       2018.Nov.21
  * @brief      Time-based process runner
  @verbatim   
 ===============================================================================
                   #####  How to use this driver #####
 ===============================================================================
  *         LED process example:
  *         RunnerEvent   LEDEvent;
  *         static __inline void LED_Auto_Step1(void)
  *         {
  *             LED_Toggle(3);
  *         }
  *         static __inline void LED_Auto_Step2(void)
  *         {
  *             LED_Toggle(3);
  *         }
  *         static __inline void LED_Auto_Step3(void)
  *         {
  *             LED_Toggle(3);
  *         }
  *         static __inline void LED_Auto_End(void)
  *         {
  *             Runner_Start(&LEDEvent);
  *         }
  *         RunnerUnit LEDUnits[] = {
  *                     {0,300,0,(uint16_t)-1,LED_Auto_Step1,NULL},
  *                     {1,600,0,(uint16_t)-1,LED_Auto_Step2,NULL},
  *                     {2,900,0,(uint16_t)-1,LED_Auto_Step3,NULL}};
  *         Runner_RegisterEvent(&LEDEvent,sizeof(LEDUnits)/sizeof(LEDUnits[0]),LEDUnits,LED_Auto_End);
  *         Runner_Start(&LEDEvent);
  @endverbatim   
  ******************************************************************************
  * @license
  *
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "sp_runner.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define RUNNER_POOLSIZE                   8

/* Private variables ---------------------------------------------------------*/
RunnerEvent*                            Runner_List[RUNNER_POOLSIZE];

/* Private function prototypes -----------------------------------------------*/
void __Runner_INNER(RunnerEvent* runner);
void __Runner_EndEvent(RunnerEvent* runner);

/* Private functions ---------------------------------------------------------*/
/** @defgroup Runner Module Internal Functions
  * @{
  */
/**
  * @brief  Stop a runner process
  * @param  runner: runner object
  */
void __Runner_EndEvent(RunnerEvent* runner) {
    runner->enable = 0;
    runner->currID = 0;
    runner->terminate = 0;
    if(runner->end_cb) (*(runner->end_cb))();               /* Invoke end_callback function */
}

/**
  * @brief  Run a runner in this timer unit
  * @param  runner: runner object
  */
void __Runner_INNER(RunnerEvent* runner) {
    static pRunnerUnit pNode;
    static Runner_RetFlag retflag; 
    if(runner->enable) {
        pNode = runner->entry + runner->currID;
        /* Execute process unit when condition fits */
        if(pNode->cnt%pNode->loop==0 || pNode->cnt==0) {
            if(pNode->func) (*pNode->func)();
        }
        /* User-defined detection */
        if(pNode->detect) {
            retflag = pNode->detect(runner->currID);
        }else{
            retflag = Runner_Null;
        }
        /* On stopping */
        if(runner->terminate || retflag==Runner_End) {
            __Runner_EndEvent(runner);
        }else{
            /* Current unit overtime or skipped */
            if(pNode->cnt==pNode->time || retflag==Runner_Skip) {
                pNode->cnt = 0;
                /* Curent process over */
                if(runner->currID == runner->length - 1) {
                    __Runner_EndEvent(runner);
                }else{
                    runner->currID ++;
                }
            }else{
                pNode->cnt ++;
            }
        }
    }
}
/**
  * @}
  */


/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/** @defgroup Runner Module Use Intefaces
  * @{
  */
/**
  * @brief  Start a new runner process
  * @param  runner: runner object
  */
void Runner_Start(RunnerEvent* runner) {
    if(!runner->enable) {
        runner->enable = 1;
        /* Clear flags */
        runner->currID = 0;
        runner->terminate = 0;
        runner->entry->cnt = 0;
    }
}

/**
  * @brief  Stop a runner process
  * @param  runner: runner object
  * @note   This function will trigger @ref end_callback()
  */
void Runner_Stop(RunnerEvent* runner) {
    if(runner->enable) {
        /* User terminated flag */
        runner->terminate = 1;
    }
}

/**
  * @brief  Force a runner to stop
  * @param  runner: runner object
  */
void Runner_ForceStop(RunnerEvent* runner) {
    if(runner->enable) {
        runner->enable = 0;
        runner->currID = 0;
        runner->terminate = 0;
    }
}

/**
  * @brief  Register a new runner
  * @param  runner: new runner list of @ref RunnerEvent
  * @param  length: total size of units
  * @param  units: list of units of @ref RunnerUnit
  * @param  end_callback: callback function on runner ending
  * @retval If register succeed.
  */
bool Runner_RegisterEvent(
    RunnerEvent* runner, 
    uint8_t length, 
    RunnerUnit* units, 
    pRunnerFunc end_callback)
{
    uint8_t id = 0, size=sizeof(Runner_List)/sizeof(*Runner_List);
    while(id < size) {
        if(!Runner_List[id]) {
            /* Init flags */
            runner->currID       = 0;
            runner->enable       = 0;
            runner->terminate    = 0;
            /* Register object and callbacks */
            runner->length       = length;
            runner->entry        = units;
            runner->end_cb       = end_callback;
            runner->run          = (tFuncMemberNoParam)Runner_Start;
            runner->stop         = (tFuncMemberNoParam)Runner_Stop;
            runner->force_stop   = (tFuncMemberNoParam)Runner_ForceStop;
            /* Append to runner pool */
            Runner_List[id] = runner;
            return true;
        }
        id++;
    }
    return false;
}

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
void Runner_ControlLooper(uint32_t tick) {
    uint8_t id = 0, size=sizeof(Runner_List)/sizeof(*Runner_List);
    while(id < size) {
        if(Runner_List[id]) __Runner_INNER(Runner_List[id]);
        id++;
    }
}

/**
  * @}
  */


/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/
