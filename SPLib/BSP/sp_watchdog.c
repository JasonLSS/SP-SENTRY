#include "sp_watchdog.h"

void SP_WatchDog_Init(void)
{
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_SetPrescaler(IWDG_Prescaler_32);
	IWDG_SetReload(1000);
	IWDG_Enable();
}

void SP_WatchDog_FeedDog(void)
{
	IWDG_ReloadCounter();
}
