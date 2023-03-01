/*
 * DS18B20.c
 *
 *  Created on: 20 янв. 2022 г.
 *      Author: mango
 */

#include "DS18B20.h"

void DS_WRITE (uint8_t data)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = DS_PIN;

	uint8_t d;
	while (HAL_GPIO_ReadPin(DS_PORT, DS_PIN) == 0);

	HAL_GPIO_WritePin(DS_PORT, DS_PIN, 0);

	for (uint8_t i = 0; i < 8; i++)
	{
		d = data & (1 << i);
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(DS_PORT, &GPIO_InitStruct);

		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;

		Delay_Micro((d ? 2 : 65)); //2 or 65us


		HAL_GPIO_Init(DS_PORT, &GPIO_InitStruct);

		Delay_Micro((d ? 65 : 2)); //65 or 2us

		Delay_Micro(3); //3us

	}

}

void DS_READ (uint8_t *ans)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = DS_PIN;

	while (HAL_GPIO_ReadPin(DS_PORT, DS_PIN) == 0);

	HAL_GPIO_WritePin(DS_PORT, DS_PIN, 0);

	for (uint8_t i = 0; i <= 7; i++)
	{

		uint8_t d;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(DS_PORT, &GPIO_InitStruct);

		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		Delay_Micro(2); //2us

		HAL_GPIO_Init(DS_PORT, &GPIO_InitStruct);

		Delay_Micro(15); //15us

		d = HAL_GPIO_ReadPin(DS_PORT, DS_PIN);
		*ans += d << i;

		Delay_Micro(45); //45us

	}

}

void DS_INIT (uint8_t *ok)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = DS_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(DS_PORT, &GPIO_InitStruct);

	HAL_GPIO_WritePin(DS_PORT, DS_PIN, 0);

	Delay_Micro(495); //495us

	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(DS_PORT, &GPIO_InitStruct);

	Delay_Micro(65); //65us

	if (HAL_GPIO_ReadPin(DS_PORT, DS_PIN) == 1)
	{
		*ok = 0;
	}
	else
	{
		//inicialization
		//WRITE SCRATCHPAD 0x4E
		//TH REGISTER 100 градусов 0x64
		//TL REGISTER - 30 градусов 0x9E
		//Resolution 12 bit 0x7F
		DS_WRITE(0xCC);
		DS_WRITE(0x4E);
		DS_WRITE(0x64);
		DS_WRITE(0x9E);
		DS_WRITE(0x7F);
		*ok = 1;
	}
}

void DS_START_MEASURE()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = DS_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(DS_PORT, &GPIO_InitStruct);

	HAL_GPIO_WritePin(DS_PORT, DS_PIN, 0);

	Delay_Micro(495); //495us

	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(DS_PORT, &GPIO_InitStruct);

	Delay_Micro(65); //65us
	DS_WRITE(0xCC);
	DS_WRITE(0x44);
}

// ans - integer part of temperature
// half - float part of temperature
void DS_GET_TEMPERATURE(int16_t *ans, float *half)
{
	uint8_t temp[10];
	uint16_t tmp;
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = DS_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(DS_PORT, &GPIO_InitStruct);

	HAL_GPIO_WritePin(DS_PORT, DS_PIN, 0);

	Delay_Micro(495); //495us

	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(DS_PORT, &GPIO_InitStruct);

	Delay_Micro(65); //65us

	DS_WRITE(0xCC);
	DS_WRITE(0xBE);

	temp[0] = 0;
	DS_READ(&temp[0]);
	temp[1] = 0;
	DS_READ(&temp[1]);
	temp[2] = 0;
	DS_READ(&temp[2]);
	temp[3] = 0;
	DS_READ(&temp[3]);
	temp[4] = 0;
	DS_READ(&temp[4]);
	temp[5] = 0;
	DS_READ(&temp[5]);
	temp[6] = 0;
	DS_READ(&temp[6]);
	temp[7] = 0;
	DS_READ(&temp[7]);
	temp[8] = 0;
	DS_READ(&temp[8]);


	uint8_t minus = 0;
	tmp = (temp[1] << 8) | temp[0];
	if (tmp & 0xF000 == 0xFC00)
		minus = 1;

	float mn = 0.0625;
	*half = 0;
	for (uint8_t i = 0; i < 4; i++)
	{
		*half+=((tmp & (1 << i)) ? 1 : 0)*mn;
		mn*=2;
	}
	tmp >>= 4;
	tmp &= 0x00FF;
	*ans = tmp;
	if (minus)
		*ans = -(*ans);

}


