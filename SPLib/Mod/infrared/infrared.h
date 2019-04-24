#ifndef __INFRARED_H_
#define __INFRARED_H_

#include "sp_conf.h"

void Infrared_Init(void);
void Infrared_Update(void);
uint16_t Get_Adc1(uint8_t ch);   

extern uint8_t Infrared_Flag;

#endif
