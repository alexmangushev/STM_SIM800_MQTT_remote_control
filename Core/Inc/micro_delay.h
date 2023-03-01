/*
 * micro_delay.h
 *
 *  Created on: Jan 30, 2022
 *      Author: mango
 */

#ifndef INC_MICRO_DELAY_H_
#define INC_MICRO_DELAY_H_

#include <main.h>

#define    DWT_CYCCNT    *(volatile uint32_t *)0xE0001004
#define    DWT_CONTROL   *(volatile uint32_t *)0xE0001000
#define    SCB_DEMCR     *(volatile uint32_t *)0xE000EDFC

void Delay_Micro(uint32_t time);

#endif /* INC_MICRO_DELAY_H_ */
