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
#include "usbd_hid_core.h"
#include <string.h>
#include "sp_conf.h"

USB_OTG_CORE_HANDLE             USB_OTG_dev;
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
    USBD_Init(
        &USB_OTG_dev,
#ifdef USE_USB_OTG_HS
        USB_OTG_HS_CORE_ID,
#else
        USB_OTG_FS_CORE_ID,
#endif
        &USR_desc,
        &USBD_HID_cb,
        &USR_cb);

    memset(USB_ReadBuffer, 0x00, sizeof(USB_ReadBuffer));
    memset(USB_Callback_Pool, 0x00, sizeof(USB_Callback_Pool));
}

/** Function
  * @brief    USB控制循环
  * @param
  * @note    进行数收发处理
  */
union char2int {
    uint32_t value;
    uint8_t bytes[4];
} retLen;

void USB_TaskLoop(void)
{
}

void USB_RxInterrupt(uint8_t* buf, uint32_t size)
{
    for(uint16_t i=0; i<sizeof(USB_Callback_Pool)/sizeof(*USB_Callback_Pool); i++) {
        if(USB_Callback_Pool[i]) {
            USB_Callback_Pool[i](buf, size);
        }
    }
}

/** Function
  * @brief    准备USB发送的数据
  * @param
  * @note
  */
uint16_t USB_SendData(uint8_t* buf, uint32_t len)
{
    USBD_HID_SendReport(&USB_OTG_dev, buf, len);
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

