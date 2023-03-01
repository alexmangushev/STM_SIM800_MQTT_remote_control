/*
 * micro_delay.c
 *
 *  Created on: Jan 30, 2022
 *      Author: mango
 */

#include "micro_delay.h"

/*
 * \time - time of delay in microseconds, maximum value is
 */
void Delay_Micro(uint32_t time)
{
	SCB_DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // permission counter
	DWT_CONTROL |= DWT_CTRL_CYCCNTENA_Msk;   // start counter

	DWT->CYCCNT = 0; // clear counter
	uint32_t tacts = HAL_RCC_GetSysClockFreq() / 1000000;
	tacts *= time;
	while(DWT->CYCCNT < tacts) {}
}

