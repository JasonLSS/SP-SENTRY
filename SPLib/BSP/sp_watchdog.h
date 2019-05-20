#ifndef __SPWD_H
#define __SPWD_H
#include "stm32f4xx.h"
#include "stm32f4xx_iwdg.h"

void SP_WatchDog_Init(void);
void SP_WatchDog_FeedDog(void);

#endif
