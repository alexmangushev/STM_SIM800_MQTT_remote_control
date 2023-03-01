/*
 * BS18B20.h
 *
 *  Created on: 20 џэт. 2022 у.
 *      Author: mango
 */

#ifndef INC_DS18B20_H_
#define INC_DS18B20_H_

#include "main.h"
#include "micro_delay.h"

//-----------------------------
//user's setting
#define DS_PIN GPIO_PIN_7
#define DS_PORT GPIOA
//timer for delay - DWT


void DS_INIT (uint8_t *ok);
void DS_START_MEASURE();
void DS_GET_TEMPERATURE(int16_t *ans, float *half);

#endif /* INC_DS18B20_H_ */
