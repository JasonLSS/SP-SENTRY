#ifndef __AutoAim
#define __AutoAim	
#include "sp_conf.h"
#include "frame.h"

void Autoaim_Init(void);
void Auto_aim(uint8_t *rx_buf,int len);
void Autoaim_USART_Interface(void);

extern double view_ch2;
extern double view_ch3;
extern uint32_t last_time_tick_1ms,last_last_time_tick_1ms;
extern int  FirstFlag;
extern int  if_newframe;
extern int  if_rotate_ok;
extern int 	if_if_newframe;

extern double  virual_ch2;
extern double  virual_ch3;

extern uint8_t auto_aim_flag;
extern uint8_t small_power_flag;

#endif
