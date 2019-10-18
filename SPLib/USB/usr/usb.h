/********************
  *---INFORMATION---*
  ******************************************************************************
  * \file    :
  * \author  :
  * \info    :
  * \version :
  * \date    :
  * \brief   :
  * \usage   :  USB_RegisteCallback(callback);
  ******************************************************************************
  * @title
  *   @@subtitle
  *     @@@subsubitle
  ******************************************************************************
  */
  
#ifndef _USB_H_
#define _USB_H_

/** Includes 
  * @brief
  */
#include <stdbool.h>
  
#include "usb_conf.h"
#include "usb_dcd_int.h"

#include "usbd_core.h"
#include "usbd_usr.h"

/** Defines
  * @brief
  */
typedef void(*USB_Callback_Type)(uint8_t* , uint32_t);

/** Variables
  * @brief
  */
extern USB_OTG_CORE_HANDLE             USB_OTG_dev;

/** Function
  * @brief 
  */
void        USB_TaskInit(void);
void        USB_TaskLoop(void);
uint16_t    USB_SendData(uint8_t* buf, uint32_t len);
void        USB_RegisteCallback(USB_Callback_Type cb);


/** Function
  * @brief    ºÏ≤‚USB «∑Ò¡¨Ω”
  */
static inline bool USB_IsConnected(void) {
    return USB_OTG_dev.dev.device_status != USB_OTG_SUSPENDED;
}

#endif
/**********************************END OF FILE**********************************/

