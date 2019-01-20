/**
  ******************************************************************************
  * @file    CDC_if_template.h
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    19-March-2012
  * @brief   Header for dfu_mal.c file.
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
#ifndef __USBD_CDC_IF_H
#define __USBD_CDC_IF_H

/* Includes ------------------------------------------------------------------*/
#include "usb_conf.h"
#include "usbd_conf.h"
#include "usbd_cdc_core.h"

/* Exported types ------------------------------------------------------------*/
/* These are external variables imported from CDC core to be used for IN 
   transfer management. */
extern uint8_t  APP_Rx_Buffer []; /* Write CDC received data in this buffer.
                                     These data will be sent over USB IN endpoint
                                     in the CDC core functions. */
extern uint8_t 	USB_Rx_Buffer [];
extern uint32_t APP_Rx_ptr_in;    /* Increment this pointer or roll it back to
                                     start address when writing received data
                                     in the buffer APP_Rx_Buffer. */
extern uint32_t APP_Rx_ptr_out;

typedef struct {
	uint32_t 	dataReadPointer;		//当前读取位置
	uint32_t 	dataReadLength;			//接收数据长度
	uint8_t*	dataReadRef;			//外部数据存储位置
	uint8_t		dataReadFlag:1;			//接收数据完成标志
	uint8_t		dataReadReadyFlag:1;	//发送数据准备完成标志
	
	uint32_t 	dataSendLength;			//发送数据长度
	uint8_t*	dataSendRef;			//外部数据存储位置
	uint8_t		dataSentFlag:1;			//发送数据完成标志
	uint8_t		dataSendReadyFlag:1;	//发送数据准备完成标志
} CDC_IF_Status_Type;
extern CDC_IF_Status_Type	CDC_State;

/* The following structures groups all needed parameters to be configured for the 
   ComPort. These parameters can modified on the fly by the host through CDC class
   command class requests. */
typedef struct
{
  uint32_t bitrate;
  uint8_t  format;
  uint8_t  paritytype;
  uint8_t  datatype;
	uint8_t changed;
}LINE_CODING;

#define DEFAULT_CONFIG                  0
#define OTHER_CONFIG                    1

/* Exported constants --------------------------------------------------------*/

extern CDC_IF_Prop_TypeDef  USBD_CDC_fops;

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

extern uint16_t CDC_DataTx   (uint8_t* Buf, uint32_t Len);
extern uint16_t CDC_DataRx   (uint8_t* Buf, uint32_t Len);

#endif /* __USBD_CDC_IF_TEMPLATE_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
