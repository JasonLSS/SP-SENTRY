/********************
  *---INFORMATION---*
  ******************************************************************************
  * \file    :
  * \author  :
  * \info    :
  * \version :
  * \date    :
  * \brief   :
    ******************************************************************************
    * @title
    *   @@subtitle
    *       @@@subsubitle
    ******************************************************************************
  */

/** Includes 
  * @brief
  */
#include "usb.h"
#include "usbd_conf.h"
#include <string.h>
#include "sp_conf.h"

USB_OTG_CORE_HANDLE             USB_OTG_dev;
//USBH_HOST                       USB_Host;
uint8_t                         USB_ReadBuffer[1024];    // 内部数据接收缓存
USB_Callback_Type               USB_Callback_Pool[USING_USB_POOLSIZE];       // 回调函数池
extern USBD_DEVICE              USR_desc;
extern USBD_Class_cb_TypeDef    USBD_HID_cb;

/** Function
  * @brief
  * @param
  *     @arg
  * @retval
  */
void USB_TaskInit(void)
{
    // USB device inti
    USBD_Init(
    &USB_OTG_dev,
#ifdef USE_USB_OTG_HS
    USB_OTG_HS_CORE_ID,
#else
    USB_OTG_FS_CORE_ID,
#endif
    &USR_desc, 
    //&USBD_CDC_cb, 
    &USBD_HID_cb,
    &USR_cb);
    
//    CDC_State.dataReadRef = USB_ReadBuffer;
//    CDC_State.dataSendRef = NULL;
    
    memset(USB_ReadBuffer, 0x00, sizeof(USB_ReadBuffer));
    memset(USB_Callback_Pool, 0x00, sizeof(USB_Callback_Pool));
}

/** Function
  * @brief    USB控制循环
  * @param
  * @note    进行数收发处理
  */
union char2int{
    uint32_t value;
    uint8_t bytes[4];
} retLen;

void USB_TaskLoop(void)
{
//    static uint32_t i;
//    /* Check if data sent */
//    if(CDC_State.dataSendReadyFlag && APP_Rx_ptr_in==APP_Rx_ptr_out) {
//        CDC_State.dataSentFlag = 1;
//        CDC_State.dataSendReadyFlag = 0;
//    }
//    /* Copy received to outer */
//    if(CDC_State.dataReadReadyFlag){
//        for(i=0; i<CDC_State.dataReadLength; i++){
//            CDC_State.dataReadRef[i] = USB_Rx_Buffer[CDC_State.dataReadPointer+i];
//        }
//        CDC_State.dataReadRef[CDC_State.dataReadLength] = 0x00;    //数据结尾，尤其是字符串数据
//        CDC_State.dataReadReadyFlag = 0;
//        CDC_State.dataReadFlag = 1;            //已经读取标志
//        
//        for(uint16_t i=0; i<sizeof(USB_Callback_Pool)/sizeof(*USB_Callback_Pool); i++) {
//            if(USB_Callback_Pool[i]) {
//                USB_Callback_Pool[i](CDC_State.dataReadRef, CDC_State.dataReadLength);
//            }
//        }
//    }
}

/** Function
  * @brief    准备USB发送的数据
  * @param
  * @note    
  */
uint16_t USB_SendData(uint8_t* buf, uint32_t len)
{
//    return CDC_DataTx(buf, len);
    return 0;
}

/** Function
  * @brief    注册USB回调函数
  * @param
  * @note    
  */
void USB_RegisteCallback(USB_Callback_Type cb) {
    for(uint16_t i=0; i<sizeof(USB_Callback_Pool)/sizeof(*USB_Callback_Pool); i++) {
        if(USB_Callback_Pool[i] == NULL) {
            USB_Callback_Pool[i] = cb;
            return;
        } else if(USB_Callback_Pool[i] == cb) {
            return;
        }
    }
}


/** Function
  * @brief    自定义页显示函数
  * @param
  * @note    
  */
void USB_OLED(void)
{
#ifdef USING_OLED 
  #ifdef USING_USB
    if(DISP_CurrentPage==&DISP_Pages.Pages.Page_USB_Rx){
        DISP_RefreshPage(DISP_CurrentPage);
        OLED_ShowFormatNext(4,"%d",CDC_State.dataReadLength);
        OLED_SetPos(0,2);
        OLED_ShowFormatNext(CDC_State.dataReadLength,"%s",&USB_ReadBuffer[CDC_State.dataReadPointer]);
    }
  #endif
#endif
}


