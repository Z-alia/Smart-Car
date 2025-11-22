#ifndef __TOF_TIMER_H_
#define __TOF_TIMER_H_

#include "tof_type.h"

uint32_t tim_get_count(void);
uint8_t tim_check_timeout(uint32_t start, uint32_t now, uint32_t invt);
uint8_t tim_check_timenow(uint32_t start, uint32_t invt);

#endif // __TOF_TIMER_H_
