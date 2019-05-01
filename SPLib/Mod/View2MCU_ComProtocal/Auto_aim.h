#ifndef __AutoAim
#define __AutoAim	
#include "sp_conf.h"
#include "frame.h"

void Autoaim_Init(void);
void Auto_aim(uint8_t *rx_buf,int len);
void Autoaim_USART_Interface(void);
void USB_Cb(uint8_t* buf, uint32_t len);

extern int  if_newframe;
extern int 	if_if_newframe;
extern frame frame_visual;
extern uint8_t enemy_area;
extern uint8_t enemy_empty_area;
extern uint8_t auto_aim_flag;
extern uint8_t small_power_flag;

#endif
