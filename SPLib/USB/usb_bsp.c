/**
  ******************************************************************************
  * @file    usb_bsp.c
  * @author  MCD Application Team
  * @version V2.1.0
  * @date    19-March-2012
  * @brief   This file is responsible to offer board support package and is
  *          configurable by user.
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
#include "usb_bsp.h"

/** @addtogroup USB_OTG_DRIVER
* @{
*/

/** @defgroup USB_BSP
  * @brief This file is responsible to offer board support package
  * @{
  */ 

/** @defgroup USB_BSP_Private_Defines
  * @{
  */ 
/**	
  * @brief 
	USB OTG接口中有5条线：2条用来传送数据（D+ 、D-）,1条是电源线(VBUS),1条则是接地线(GND),1条是ID线。
	ID线---以用于识别不同的电缆端点，mini-A插头(即A外设)中的ID引脚接地，mini-B插头（即B外设）中的ID引脚浮空。
	当OTG设备检测到接地的ID引脚时，表示默认的是A设备（主机），而检测到ID引脚浮空的设备则认为是B设备（外设）。
  */ 
#define USB_VCP_DISABLE_VBUS
#define USB_VCP_DISABLE_ID

/** @defgroup USB_BSP_Private_TypesDefinitions
  * @{
  */ 
/**
  * @}
  */ 
#ifndef USB_VCP_NVIC_PRIORITY
#define USB_VCP_NVIC_PRIORITY			0x00
#endif

#ifndef USB_VCP_NVIC_SUBPRIORITY
#define USB_VCP_NVIC_SUBPRIORITY		0x02
#endif

extern USB_OTG_CORE_HANDLE           	USB_OTG_dev;


/** @defgroup USB_BSP_Private_Macros
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup USBH_BSP_Private_Variables
  * @{
  */ 

/**
  * @}e
  */ 

/** @defgroup USBH_BSP_Private_FunctionPrototypes
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup USB_BSP_Private_Functions
  * @{
  */ 


/**
  * @brief  USB_OTG_BSP_Init
  *         Initilizes BSP configurations
  * @param  None
  * @retval None
  */
//USB主机电源控制口
#define USB_HOST_PWRCTRL 	PAout(15)	//PA15
void USB_OTG_BSP_Init(USB_OTG_CORE_HANDLE *pdev)
{

	GPIO_InitTypeDef  GPIO_InitStructure;
#ifdef USE_USB_OTG_HS
	// B14, B15
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA , ENABLE);  
	GPIO_InitStructure.GPIO_Pin = 	GPIO_Pin_11 | 	// OTG FS Data -
									GPIO_Pin_12;	// OTG FS Data +
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);  
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_OTG_HS);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_OTG_HS);
#else
	// A11, A12
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA , ENABLE);  
	GPIO_InitStructure.GPIO_Pin = 	GPIO_Pin_11 | 	// OTG FS Data -
									GPIO_Pin_12;	// OTG FS Data +
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);  
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_OTG_FS); 
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_OTG_FS);
#endif
	
#ifndef USB_VCP_DISABLE_VBUS
	// Configure  VBUS Pin
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);    
#endif
	
#ifndef USB_VCP_DISABLE_ID
	// Configure ID pin
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);  
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_OTG1_FS); 
#endif
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
#ifdef USE_USB_OTG_FS
	RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_OTG_FS, ENABLE); 
#else
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_OTG_HS, ENABLE); 
#endif
}
///**
//* @brief  USB_OTG_BSP_Init
//*         Initilizes BSP configurations
//* @param  None
//* @retval None
//*/
//void USB_OTG_BSP_Init(USB_OTG_CORE_HANDLE *pdev) {
//  GPIO_InitTypeDef GPIO_InitStructure;   
//#ifdef USE_USB_OTG_FS
//	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA , ENABLE);  
//	GPIO_InitStructure.GPIO_Pin = 	GPIO_Pin_11 | 	// OTG FS Data -
//									GPIO_Pin_12;	// OTG FS Data +
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);  

//	GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_OTG1_FS); 
//	GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_OTG1_FS);
//	
//	#ifndef USB_VCP_DISABLE_VBUS
//		// Configure  VBUS Pin
//		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
//		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//		GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
//		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
//		GPIO_Init(GPIOA, &GPIO_InitStructure);    
//	#endif
//	
//	#ifndef USB_VCP_DISABLE_ID
//		// Configure ID pin
//		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
//		GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
//		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;  
//		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//		GPIO_Init(GPIOA, &GPIO_InitStructure);  
//		GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_OTG1_FS); 
//	#endif
//	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
//	RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_OTG_FS, ENABLE); 
//#else
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
//	GPIO_InitStructure.GPIO_Pin = 	GPIO_Pin_14 | // Data -
//									GPIO_Pin_15;  // Data +
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_Init(GPIOB, &GPIO_InitStructure);

//	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_OTG2_FS);
//	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_OTG2_FS);
//	
//	#ifndef USB_VCP_DISABLE_VBUS
//		//Configure VBUS Pin
//		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
//		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//		GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
//		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//		GPIO_Init(GPIOB, &GPIO_InitStructure);
//		GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_OTG2_FS);
//	#endif

//	#ifndef USB_VCP_DISABLE_ID
//		//Configure ID pin
//		GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12;
//		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//		GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
//		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//		GPIO_Init(GPIOB, &GPIO_InitStructure);
//		GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_OTG2_FS);
//	#endif

//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_OTG_HS, ENABLE);
//#endif
//}

/**
* @brief  USB_OTG_BSP_EnableInterrupt
*         Enabele USB Global interrupt
* @param  None
* @retval None
*/
void USB_OTG_BSP_EnableInterrupt(USB_OTG_CORE_HANDLE *pdev) {
	NVIC_InitTypeDef NVIC_InitStructure;
#ifdef USE_USB_OTG_FS
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel = OTG_FS_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = USB_VCP_NVIC_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = USB_VCP_NVIC_SUBPRIORITY + 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#else
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel = OTG_HS_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = USB_VCP_NVIC_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = USB_VCP_NVIC_SUBPRIORITY + 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel = OTG_HS_EP1_OUT_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = USB_VCP_NVIC_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = USB_VCP_NVIC_SUBPRIORITY + 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel = OTG_HS_EP1_IN_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = USB_VCP_NVIC_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = USB_VCP_NVIC_SUBPRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure); 
#endif
}


#ifdef USE_HOST_MODE
/**
  * @brief  BSP_Drive_VBUS
  *         Drives the Vbus signal through IO
  * @param  speed : Full, Low 
  * @param  state : VBUS states
  * @retval None
  */
//void USB_OTG_BSP_DriveVBUS(uint32_t speed, uint8_t state)
void USB_OTG_BSP_DriveVBUS(USB_OTG_CORE_HANDLE *pdev,uint8_t state)
{

}

/**
  * @brief  USB_OTG_BSP_ConfigVBUS
  *         Configures the IO for the Vbus and OverCurrent
  * @param  Speed : Full, Low 
  * @retval None
  */
//void  USB_OTG_BSP_ConfigVBUS(uint32_t speed)
void USB_OTG_BSP_ConfigVBUS(USB_OTG_CORE_HANDLE *pdev)
{

}
#endif
/**
  * @brief  USB_OTG_BSP_TimeInit
  *         Initialises delay unit Systick timer /Timer2
  * @param  None
  * @retval None
  */
void USB_OTG_BSP_TimeInit ( void )
{
	
}

/**
  * @brief  USB_OTG_BSP_uDelay
  *         This function provides delay time in micro sec
  * @param  usec : Value of delay required in micro sec
  * @retval None
  */
void USB_OTG_BSP_uDelay (const uint32_t usec)
{
  uint32_t count = 0;
  const uint32_t utime = (120 * usec / 7);
  do
  {
    if ( ++count > utime )
    {
      return ;
    }
  }
  while (1);   
}


/**
  * @brief  USB_OTG_BSP_mDelay
  *          This function provides delay time in milli sec
  * @param  msec : Value of delay required in milli sec
  * @retval None
  */
void USB_OTG_BSP_mDelay (const uint32_t msec)
{
    USB_OTG_BSP_uDelay(msec * 1000);    
}


/**
  * @brief  USB_OTG_BSP_TimerIRQ
  *         Time base IRQ
  * @param  None
  * @retval None
  */

void USB_OTG_BSP_TimerIRQ (void)
{
	
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
