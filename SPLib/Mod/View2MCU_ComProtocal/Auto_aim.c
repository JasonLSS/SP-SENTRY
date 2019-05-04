/**
  ******************************************************************************
  * @file       Auto_aim.c
  * @author     LSS
  * @version    v0.5-alpha
  * @date       2019.Mar.29
  * @brief      auto aim control
  * @usage      
  ******************************************************************************
  * @license
  *
  ******************************************************************************
  */
#include "Auto_aim.h"
#include "gimbal.h"
#include "sp_utility.h"
 
frame frame_ex;//存储上一个视觉发来的结构体
frame fram;//存储视觉数据的结构体
frame frame_visual;

int     if_newframe = 0;
int     if_if_newframe = 0;

uint8_t enemy_area = 6;
uint8_t enemy_empty_area = 0;

float last_pitch = 0;
float last_yaw = 0;
float yaw_aim_limit = 2;
float pitch_aim_limit = 10;
//char uart6_buff[256];

uint8_t auto_aim_flag = 0;
uint8_t small_power_flag = 0;

void Auto_aim(u8 *rx_buf,int len)
{
		if(len==0){
				return;
		}
		rx_buf[len] = '\0';
	
	 if(unpackFrame(rx_buf,len,&fram) == 0)  //解包成功
   {
        if(fram.timestamp != frame_ex.timestamp)//如果前一帧数据和当前帧时间戳一样,目标丢失,不作为
        {
            if_newframe = 1;
        }
				else
				{
					if_newframe = 0;
				}
				frame_ex.timestamp = fram.timestamp;
    }
		else
		{
			if_newframe = 0;
		}
		//task
		if(if_newframe == 1) {
				frame_visual = fram;
				enemy_area = (int)(fram.extra[1] & 0xf0) >> 4;
				enemy_empty_area = (int)(fram.extra[1] & 0x0f);
				if(frame_visual.yaw > 10)
					frame_visual.yaw = 10;
				else if(frame_visual.yaw < -10)
					frame_visual.yaw = -10;
				if(frame_visual.pitch > 10)
					frame_visual.pitch = 10;
				else if(frame_visual.pitch < -10)
					frame_visual.pitch = -10;
				
				if(fram.extra[0] == 4){
					if_if_newframe = 0;auto_aim_flag = 0;
				}
				else if(fram.extra[0] == 5){
					if_if_newframe = 1;auto_aim_flag = 0;
				}
				else if(fram.extra[0] == 7){
					if_if_newframe = 1;auto_aim_flag = 1;
				}
				else{
					if_if_newframe = 0;auto_aim_flag = 0;
				}
				
				if(fabs(fram.yaw) < yaw_aim_limit && fabs(fram.pitch) < pitch_aim_limit)
					auto_aim_flag = 1;
				else
					auto_aim_flag = 0;

//				u8 size = sprintf(uart6_buff, "%d,%f,%d\r\n",len, fram.yaw, miss);
//				spDMA.controller.start(spDMA_UART7_tx_stream, (uint32_t)uart6_buff, (uint32_t)&UART7->DR, size);
		}
}


static uint8_t __view_buffer[128];
struct {
    UsartBuffer_t buffer;
    float stamp, stamp_ex;
    float freq;
} view_buffer = {
    .buffer = {
        .buffer = __view_buffer,
        .size = 128,
        .curr_ptr = 0,
        .last_ptr = 0
    },
    .stamp = 0.f,
    .stamp_ex = 0.f,
    .freq = 0.f
};

void Autoaim_Init(void) {
#ifdef USING_USB
		USB_RegisteCallback(USB_Cb);
#else
		USART_RX_Config(USART2, 115200);
    DMA_USART_RX_Config(USART2, (uint32_t)view_buffer.buffer.buffer, view_buffer.buffer.size, false);
    USART_TX_Config(USART2, 115200);
    DMA_USART_TX_Config(USART2);
    DMA_Cmd(spDMA_USART2_rx_stream, ENABLE);
    spIRQ.registe(USART2_IRQn, USART_IT_IDLE, Autoaim_USART_Interface);
    USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
    USART_Cmd(USART2, ENABLE);
#endif
		
}

void Autoaim_USART_Interface(void) {
    
		u8 bt = USART2->DR;
	  view_buffer.stamp_ex = view_buffer.stamp;
    view_buffer.stamp = TASK_GetSecond();
    view_buffer.freq = view_buffer.stamp - view_buffer.stamp_ex;
    
    uint16_t size = view_buffer.buffer.size - spDMA_USART2_rx_stream->NDTR;
    spDMA.controller.reset_counter(spDMA_USART2_rx_stream, view_buffer.buffer.size);
    Auto_aim(view_buffer.buffer.buffer, size);
    
}

void USB_Cb(uint8_t* buf, uint32_t len) {
    Auto_aim(buf, len);
}



