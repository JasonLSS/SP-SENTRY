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
  * 	  @@@subsubitle
  ******************************************************************************
  */
  
#ifndef _USB_H_
#define _USB_H_

/** Includes 
  * @brief
  */
#ifdef USING_OLED
	extern char USBD_Info[128];			//USB页面信息
#endif 
#include "usb_conf.h"
#include "usb_dcd_int.h"
#include "usb_hcd_int.h"
#include "usbd_core.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "usbd_cdc_core.h"
#include "usbd_cdc_if.h"
#include "usbh_core.h"
#include <string.h>

#include "sp_conf.h"

/** Defines
  * @brief
  */
typedef uint8_t                 uint8;
#define USB_MAX_DATE            256
extern char usbRetMsg[USB_MAX_DATE];

typedef void(*USB_Callback_Type)(uint8_t* , uint32_t);

/** Variables
  * @brief
  */
extern struct __USB_FLAG_REG{
	uint8_t convsend:1;		//从机连续发送信号
	
} USB_Flag;
  
extern CDC_IF_Prop_TypeDef 		CDC_fops;	// usbd_cdc_if(interface).c
extern USB_OTG_CORE_HANDLE 		USB_OTG_dev;
extern USBH_HOST               	USB_Host;
extern uint8					USB_ReadBuffer[CDC_DATA_MAX_PACKET_SIZE];
extern CDC_IF_Status_Type		CDC_State;


/** Function
  * @brief 
  */
void 		USB_TaskInit(void);
void 		USB_TaskLoop(void);
uint16_t 	USB_SendData(uint8_t* buf, uint32_t len);
void USB_RegisteCallback(USB_Callback_Type cb);

#endif
/**********************************END OF FILE**********************************/

