/**
  ******************************************************************************
  * @file       sp_monitor.h
  * @author     YTom
  * @version    v0.0-alpha
  * @date       2018.Nov.19
  * @brief      Monitor (protection manager) module
  * @usage      
  ******************************************************************************
  * @license
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SP_MONITOR_HPP
#define __SP_MONITOR_HPP

/* Includes ------------------------------------------------------------------*/
#include "sp_conf.h"
//#include "sp_list.hpp"

namespace sp {

/* Exported types ------------------------------------------------------------*/
/**
  * @brief  Current object status
  */
typedef enum {
    MONITOR_PREPARE=-2,
    MONITOR_ERROR=-1,
    MONITOR_NORMAL=0,
} MONITOR_StatusType;

/**
  * @brief  Monitor class
  */
class Monitor {

    friend class __MonitorManager_t;

public:
    Monitor() : __status(MONITOR_PREPARE) {
        this->__checker = NULL;
    }
public:
    /** @brief  Monitor infomation */
    typedef struct {
        uint32_t        type;
        std::string     info;
    } Monitor_Info;
    /** @brief  Monitor callback prototype */
    typedef void (*Monitor_OnRecovery)(const Monitor_Info&);
    /** @brief  Monitor callback prototype */
    typedef void (*Monitor_OnError)(const Monitor_Info&);
    /** @brief  Monitor checker prototype */
    typedef MONITOR_StatusType (*Monitor_Checker)(Monitor_Info&);
    /** @brief  Monitor status infomation */
    std::string info = "";
    /** @brief  Monitor callback types */
    typedef enum {
        onError,
        onRecovery
    } Monitor_Callback_t;
public:
    /** @brief Checking current status, implemented by user-class. */
    void setChecker(Monitor_Checker checker);
    /** @brief Get current module status. */
    MONITOR_StatusType  getStatus();
    /** @brief  Bind callback */
    void bind( Monitor::Monitor_Callback_t type, void(*fnc)(void));
private:
    std::list<Monitor::Monitor_OnError>         __on_error;
    std::list<Monitor::Monitor_OnRecovery>      __on_recovery;
    Monitor_Checker                             __checker;
protected:
    MONITOR_StatusType                          __status;
    Monitor_Info                                __info;
public:
    void operator()(void);
};

/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/** @defgroup Monitor User Interface API
  * @brief    User callable functions for minitor management
  * @{
  */

/**
  * @brief  Register module monitor to system.
  * @retval Success/Fail status
  */
bool MONITOR_Register(const Monitor& monitor);

/**
  * @}
  */



/** @defgroup System-owned Initialization Functions and Looper
  * @brief    Implement monitor basic management
  * @note     SHOULD NOT call by users.
  * @{
  */

/**
  * @brief  Init monitor module
  */ 
void MONITOR_ManagerInit(void);

/**
  * @brief  Loop to invoke morotr control
  * @note   Periodic invoke by SYSTEM.
  */
void MONITOR_ManagerLooper(void);

/**
  * @}
  */

};              /* End namepace */

#endif /*__SP_MONITOR_H */

/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/
