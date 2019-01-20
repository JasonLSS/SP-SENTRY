/**
  ******************************************************************************
  * @file    CDC_if_template.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    19-March-2012
  * @brief   Generic media access Layer.
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

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED 
#pragma     data_alignment = 4 
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


LINE_CODING linecoding = {
	115200, /* baud rate */
	0x00,   /* stop bits-1 */
	0x00,   /* parity - none */
	0x08,   /* nb. of bits 8 */
	1		/* Changed flag */
};

/* Private function prototypes -----------------------------------------------*/
static uint16_t CDC_Init     (void);
static uint16_t CDC_DeInit   (void);
static uint16_t CDC_Ctrl     (uint32_t Cmd, uint8_t* Buf, uint32_t Len);
uint16_t CDC_DataTx   (uint8_t* Buf, uint32_t Len);
uint16_t CDC_DataRx   (uint8_t* Buf, uint32_t Len);

CDC_IF_Prop_TypeDef CDC_fops = 
{
  CDC_Init,
  CDC_DeInit,
  CDC_Ctrl,
  CDC_DataTx,
  CDC_DataRx
};

CDC_IF_Status_Type CDC_State = {
	0,0,NULL,
	0,0
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  CDC_Init
  *         Initializes the CDC media low layer
  * @param  None
  * @retval Result of the opeartion: USBD_OK if all operations are OK else USBD_FAIL
  */
static uint16_t CDC_Init(void)
{
	/*
	 Add your initialization code here 
	*/
	return USBD_OK;
}

/**
  * @brief  CDC_DeInit
  *         DeInitializes the CDC media low layer
  * @param  None
  * @retval Result of the opeartion: USBD_OK if all operations are OK else USBD_FAIL
  */
static uint16_t CDC_DeInit(void)
{
	/*
	 Add your deinitialization code here 
	*/
	return USBD_OK;
}


/**
  * @brief  CDC_Ctrl
  *         Manage the CDC class requests
  * @param  Cmd: Command code            
  * @param  Buf: Buffer containing command data (request parameters)
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the opeartion: USBD_OK if all operations are OK else USBD_FAIL
  */
static uint16_t CDC_Ctrl (uint32_t Cmd, uint8_t* Buf, uint32_t Len)
{ 
	switch (Cmd)
	{
	case SEND_ENCAPSULATED_COMMAND:
		/* Add your code here */
		break;

	case GET_ENCAPSULATED_RESPONSE:
		/* Add your code here */
		break;

	case SET_COMM_FEATURE:
		/* Add your code here */
		break;

	case GET_COMM_FEATURE:
		/* Add your code here */
		break;

	case CLEAR_COMM_FEATURE:
		/* Add your code here */
		break;

	case SET_LINE_CODING:
//		linecoding.bitrate    = (uint32_t)(pbuf[0] | (pbuf[1] << 8) |\
//						(pbuf[2] << 16) | (pbuf[3] << 24));
//		linecoding.format     = pbuf[4];
//		linecoding.paritytype = pbuf[5];
//		linecoding.datatype   = pbuf[6];
		/* Add your code here */
		break;

	case GET_LINE_CODING:
//		pbuf[0] = (uint8_t)(linecoding.bitrate);
//		pbuf[1] = (uint8_t)(linecoding.bitrate >> 8);
//		pbuf[2] = (uint8_t)(linecoding.bitrate >> 16);
//		pbuf[3] = (uint8_t)(linecoding.bitrate >> 24);
//		pbuf[4] = linecoding.format;
//		pbuf[5] = linecoding.paritytype;
//		pbuf[6] = linecoding.datatype;    
		/* Add your code here */
		break;

	case SET_CONTROL_LINE_STATE:
		/* Add your code here */
		break;

	case SEND_BREAK:
		/* Add your code here */
		break;    

	default:
		break;
	}

	return USBD_OK;
}

/**
  * @brief  CDC_DataTx
  *         CDC received data to be send over USB IN endpoint are managed in 
  *         this function.
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the opeartion: USBD_OK if all operations are OK else VCP_FAIL
  */
uint16_t CDC_DataTx (uint8_t* Buf, uint32_t Len) 
{
	static uint32_t tx_counter;
	
	for(tx_counter=0; tx_counter<Len; tx_counter++) {
		/* Copy data to send */
		APP_Rx_Buffer[APP_Rx_ptr_in] = Buf[tx_counter];	
		APP_Rx_ptr_in++;
		
		/* To avoid buffer overflow */
		if (APP_Rx_ptr_in >= APP_RX_DATA_SIZE) {
			APP_Rx_ptr_in = 0;
		}
	}
	/* Set data send ready flag */
	CDC_State.dataSendReadyFlag = 1;
	CDC_State.dataSentFlag = 0;
	return USBD_OK;
}

/**
  * @brief  CDC_DataRx
  *         Data received over USB OUT endpoint are sent over CDC interface 
  *         through this function.
  *           
  *         @note
  *         This function will block any OUT packet reception on USB endpoint 
  *         untill exiting this function. If you exit this function before transfer
  *         is complete on CDC interface (ie. using DMA controller) it will result 
  *         in receiving more data while previous ones are still not sent.
  *                 
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the opeartion: USBD_OK if all operations are OK else VCP_FAIL
  */
uint16_t CDC_DataRx (uint8_t* Buf, uint32_t Len) 
{
	//static uint32_t i;
	
	// Set read ready flag
	CDC_State.dataReadPointer = 0;
	CDC_State.dataReadLength = Len;
	CDC_State.dataReadFlag = 0;
	CDC_State.dataReadReadyFlag = 1;
	return USBD_OK;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
