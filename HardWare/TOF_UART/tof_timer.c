#include "tof_timer.h"
#include "main.h" // For HAL_GetTick

uint32_t tim_get_count(void)
{
    return HAL_GetTick();
}

uint8_t tim_check_timeout(uint32_t start, uint32_t now, uint32_t invt)
{
	if((uint32_t)(start + invt) >= start)//未溢出
	{
		if((now >= (uint32_t)(start + invt)) || (now < start))
		{
			return 1;
		}
	}
	else//溢出
	{
		if((now < start) && (now >= (uint32_t)(start + invt)))
		{
			return 1;
		}
	}
	return 0;
}

uint8_t tim_check_timenow(uint32_t start, uint32_t invt)
{
    return tim_check_timeout(start, tim_get_count(), invt);
}
