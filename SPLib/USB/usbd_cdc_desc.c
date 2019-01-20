/**
  ******************************************************************************
  * @file    usbd_desc.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    19-March-2012
  * @brief   This file provides the USBD descriptors and string formating method.
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

/* Includes ------------------------------------------------------------------*/
#include "usbd_core.h"
#include "usbd_cdc_desc.h"
#include "usbd_req.h"
#include "usbd_conf.h"
#include "usb_regs.h"

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @{
  */
uint8_t USBD_DeviceDesc  		[USB_SIZ_DEVICE_DESC];
uint8_t USBD_OtherSpeedCfgDesc	[USB_LEN_CFG_DESC]; 
uint8_t USBD_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC];
uint8_t USBD_LangIDDesc			[USB_SIZ_STRING_LANGID];
USBD_DEVICE USR_CDC_desc; 

/** @defgroup USBD_DESC 
  * @brief USBD descriptors module
  * @{
  */ 

/** @defgroup USBD_DESC_Private_TypesDefinitions
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup USBD_DESC_Private_Defines
  * @{
  */ 

#define usbd_cdc_VID                      0x0483
#define usbd_cdc_PID                      0x5710

#define usbd_cdc_LANGID_STRING            0x409
#define usbd_cdc_MANUFACTURER_STRING      "STMicroelectronics"

#define usbd_cdc_PRODUCT_HS_STRING        "Communication in HS mode"
#define usbd_cdc_SERIALNUMBER_HS_STRING   "00000000011B"

#define usbd_cdc_PRODUCT_FS_STRING        "Communication in FS Mode"
#define usbd_cdc_SERIALNUMBER_FS_STRING   "00000000011C"

#define usbd_cdc_CONFIGURATION_HS_STRING  "CDC Config"
#define usbd_cdc_INTERFACE_HS_STRING      "CDC Interface"

#define usbd_cdc_CONFIGURATION_FS_STRING  "CDC Config"
#define usbd_cdc_INTERFACE_FS_STRING      "CDC Interface"
/**
  * @}
  */ 


/** @defgroup USBD_DESC_Private_Macros
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup USBD_DESC_Private_Variables
  * @{
  */ 

USBD_DEVICE USR_CDC_desc =
{
  usbd_cdc_DeviceDescriptor,
  usbd_cdc_LangIDStrDescriptor, 
  usbd_cdc_ManufacturerStrDescriptor,
  usbd_cdc_ProductStrDescriptor,
  usbd_cdc_SerialStrDescriptor,
  usbd_cdc_ConfigStrDescriptor,
  usbd_cdc_InterfaceStrDescriptor,
};


#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4   
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
/* USB Standard Device Descriptor */
__ALIGN_BEGIN uint8_t usbd_cdc_DeviceDesc[USB_SIZ_DEVICE_DESC] __ALIGN_END =
{
    0x12,                       /*bLength */
    USB_DEVICE_DESCRIPTOR_TYPE, /*bDescriptorType*/
    0x00,                       /*bcdUSB */
    0x02,
    0x00,                       /*bDeviceClass*/
    0x00,                       /*bDeviceSubClass*/
    0x00,                       /*bDeviceProtocol*/
    USB_OTG_MAX_EP0_SIZE,      /*bMaxPacketSize*/
    LOBYTE(usbd_cdc_VID),           /*idVendor*/
    HIBYTE(usbd_cdc_VID),           /*idVendor*/
    LOBYTE(usbd_cdc_PID),           /*idVendor*/
    HIBYTE(usbd_cdc_PID),           /*idVendor*/
    0x00,                       /*bcdDevice rel. 2.00*/
    0x02,
    USBD_IDX_MFC_STR,           /*Index of manufacturer  string*/
    USBD_IDX_PRODUCT_STR,       /*Index of product string*/
    USBD_IDX_SERIAL_STR,        /*Index of serial number string*/
    USBD_CFG_MAX_NUM            /*bNumConfigurations*/
}; /* USB_DeviceDescriptor */

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4   
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
/* USB Standard Device Descriptor */
__ALIGN_BEGIN uint8_t usbd_cdc_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  0x40,
  0x01,
  0x00,
};

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4   
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
/* USB Standard Device Descriptor */
__ALIGN_BEGIN uint8_t usbd_cdc_LangIDDesc[USB_SIZ_STRING_LANGID] __ALIGN_END =
{
     USB_SIZ_STRING_LANGID,         
     USB_DESC_TYPE_STRING,       
     LOBYTE(usbd_cdc_LANGID_STRING),
     HIBYTE(usbd_cdc_LANGID_STRING), 
};
/**
  * @}
  */ 


/** @defgroup USBD_DESC_Private_FunctionPrototypes
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup USBD_DESC_Private_Functions
  * @{
  */ 

/**
* @brief  usbd_cdc_DeviceDescriptor 
*         return the device descriptor
* @param  speed : current device speed
* @param  length : pointer to data length variable
* @retval pointer to descriptor buffer
*/
uint8_t *  usbd_cdc_DeviceDescriptor( uint8_t speed , uint16_t *length)
{
  *length = sizeof(usbd_cdc_DeviceDesc);
  return USBD_DeviceDesc;
}

/**
* @brief  usbd_cdc_LangIDStrDescriptor 
*         return the LangID string descriptor
* @param  speed : current device speed
* @param  length : pointer to data length variable
* @retval pointer to descriptor buffer
*/
uint8_t *  usbd_cdc_LangIDStrDescriptor( uint8_t speed , uint16_t *length)
{
  *length =  sizeof(usbd_cdc_LangIDDesc);  
  return USBD_LangIDDesc;
}


/**
* @brief  usbd_cdc_ProductStrDescriptor 
*         return the product string descriptor
* @param  speed : current device speed
* @param  length : pointer to data length variable
* @retval pointer to descriptor buffer
*/
uint8_t *  usbd_cdc_ProductStrDescriptor( uint8_t speed , uint16_t *length)
{
 
  
  if(speed == 0)
  {   
    USBD_GetString (usbd_cdc_PRODUCT_HS_STRING, USBD_StrDesc, length);
  }
  else
  {
    USBD_GetString (usbd_cdc_PRODUCT_FS_STRING, USBD_StrDesc, length);    
  }
  return USBD_StrDesc;
}

/**
* @brief  usbd_cdc_ManufacturerStrDescriptor 
*         return the manufacturer string descriptor
* @param  speed : current device speed
* @param  length : pointer to data length variable
* @retval pointer to descriptor buffer
*/
uint8_t *  usbd_cdc_ManufacturerStrDescriptor( uint8_t speed , uint16_t *length)
{
  USBD_GetString (usbd_cdc_MANUFACTURER_STRING, USBD_StrDesc, length);
  return USBD_StrDesc;
}

/**
* @brief  usbd_cdc_SerialStrDescriptor 
*         return the serial number string descriptor
* @param  speed : current device speed
* @param  length : pointer to data length variable
* @retval pointer to descriptor buffer
*/
uint8_t *  usbd_cdc_SerialStrDescriptor( uint8_t speed , uint16_t *length)
{
  if(speed  == USB_OTG_SPEED_HIGH)
  {    
    USBD_GetString (usbd_cdc_SERIALNUMBER_HS_STRING, USBD_StrDesc, length);
  }
  else
  {
    USBD_GetString (usbd_cdc_SERIALNUMBER_FS_STRING, USBD_StrDesc, length);    
  }
  return USBD_StrDesc;
}

/**
* @brief  usbd_cdc_ConfigStrDescriptor 
*         return the configuration string descriptor
* @param  speed : current device speed
* @param  length : pointer to data length variable
* @retval pointer to descriptor buffer
*/
uint8_t *  usbd_cdc_ConfigStrDescriptor( uint8_t speed , uint16_t *length)
{
  if(speed  == USB_OTG_SPEED_HIGH)
  {  
    USBD_GetString (usbd_cdc_CONFIGURATION_HS_STRING, USBD_StrDesc, length);
  }
  else
  {
    USBD_GetString (usbd_cdc_CONFIGURATION_FS_STRING, USBD_StrDesc, length); 
  }
  return USBD_StrDesc;  
}


/**
* @brief  usbd_cdc_InterfaceStrDescriptor 
*         return the interface string descriptor
* @param  speed : current device speed
* @param  length : pointer to data length variable
* @retval pointer to descriptor buffer
*/
uint8_t *  usbd_cdc_InterfaceStrDescriptor( uint8_t speed , uint16_t *length)
{
  if(speed == 0)
  {
    USBD_GetString (usbd_cdc_INTERFACE_HS_STRING, USBD_StrDesc, length);
  }
  else
  {
    USBD_GetString (usbd_cdc_INTERFACE_FS_STRING, USBD_StrDesc, length);
  }
  return USBD_StrDesc;  
}

/**
  * @}
  */ 


/**
  * @}
  */ 


/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

