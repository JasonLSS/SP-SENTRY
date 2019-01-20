/**
  ******************************************************************************
  * @file    usbd_conf_template.h
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    19-March-2012
  * @brief   usb device configuration template file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_CONF__H__
#define __USBD_CONF__H__

/* Includes ------------------------------------------------------------------*/
#include "usb_conf.h"

/** @defgroup USB_CONF_Exported_Defines
  * @{
  */ 
#define USBD_CFG_MAX_NUM           1
#define USB_MAX_STR_DESC_SIZ       64 
#define USBD_EP0_MAX_PACKET_SIZE   64

/**
  * @}
  */ 


/** @defgroup USB_CONF_Exported_Types
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup USB_CONF_Exported_Macros
  * @{
  */ 
/**
  * @note	    
	USBD_CFG_MAX_NUM=1
    USBD_ITF_MAX_NUM=1
    USB_MAX_STR_DESC_SIZ=50
    CDC_IN_EP=0x81 ：EP1用于data IN
    CDC_OUT_EP=0x01 ：EP1用于data OUT
    CDC_CMD_EP=0x82 ：EP2用于CDC命令
    CDC_DATA_MAX_PACKET_SIZE=64 ：输入输出包最大大小
    CDC_CMD_PACKET_SZE=8 ：控制端点包大小
    CDC_IN_FRAME_INTERVAL=5 ：两个IN传输间最大帧数量
    APP_RX_DATA_SIZE=2048 ：IN缓冲的总大小
    APP_FOPS=cdc_fops
  */ 
#ifdef USB_USB_OTG_HS
	#define CDC_DATA_MAX_PACKET_SIZE		512
	#define CDC_IN_FRAME_INTERVAL			40
	#define APP_RX_DATA_SIZE				2048
#else
	#define CDC_DATA_MAX_PACKET_SIZE		64
	#define CDC_IN_FRAME_INTERVAL			5
	#define APP_RX_DATA_SIZE				2048
#endif
#define CDC_CMD_PACKET_SZE					8
#define CDC_OUT_EP							0x01
#define CDC_CMD_EP							0x82
#define CDC_IN_EP							0x81

#define USBD_ITF_MAX_NUM					1
#define APP_FOPS							CDC_fops

/** @defgroup USB_CONF_Exported_Variables
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup USB_CONF_Exported_FunctionsPrototype
  * @{
  */ 
/**
  * @}
  */ 

#endif //__USBD_CONF__H__

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

