/**
  ******************************************************************************
  * @file       sp_monitor.c
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

/* Includes ------------------------------------------------------------------*/
#include "sp_monitor.hpp"

namespace sp {

/* Private typedef -----------------------------------------------------------*/
/**
  * @brief  Monitor manager class
  */
class __MonitorManager_t{

public:
    __MonitorManager_t() { }

public:
    bool add_monitor(const Monitor& monitor) {
        this->__monitors.push_back(monitor);
        return true;
    }

    void looper() {
        for(std::list<Monitor>::iterator ite=this->__monitors.begin();
            ite!=this->__monitors.end(); ite++) {
            MONITOR_StatusType status = ite->__status;
            (*ite)();
            if(ite->__status != status) {
                if(status == sp::MONITOR_ERROR) {
                    for(auto item : ite->__on_recovery) {
                        item(ite->__info);
                    }
                    break;
                } else if(ite->__status == sp::MONITOR_ERROR) {
                    for(auto item : ite->__on_error) {
                        item(ite->__info);
                    }
                    break;
                }
            }
        }
    }
    
private:
    std::list<Monitor> __monitors;
};


/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/** @bridef Placehold for monitor */
__MonitorManager_t              spMonitorManager;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
MONITOR_StatusType Monitor::getStatus(void) {
    return this->__status;
}

void Monitor::setChecker(Monitor_Checker checker) {
    this->__checker = checker;
}

void Monitor::bind( Monitor::Monitor_Callback_t type, void (*fnc)(void)) {
    switch(type) {
        case Monitor::onError:
            this->__on_error.push_back((Monitor::Monitor_OnError)fnc);
            break;
        case Monitor::onRecovery:
            this->__on_recovery.push_back((Monitor::Monitor_OnRecovery)fnc);
            break;
    }
}

void Monitor::operator()(void) {
    this->__status = this->__checker(this->__info);
}


/* Exported variables --------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/** @defgroup System-owned Initialization Functions and Looper
  * @brief    Implement monitor basic management
  * @{
  */

void MONITOR_ManagerInit(void) {
    
}

void MONITOR_ManagerLooper(void) {
    spMonitorManager.looper();
}

/**
  * @}
  */



/** @defgroup Monitor User Interface API
  * @brief    User callable functions for minitor management
  * @{
  */

/**
  * @brief  
  * @note   
  * @param  
  * @retval 
  */ 
bool MONITOR_Register(const Monitor& monitor) {
    spMonitorManager.add_monitor(monitor);
    return true;
}

};

/**
  * @}
  */


/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/
