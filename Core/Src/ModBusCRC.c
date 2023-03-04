/*
 * ModBusCRC.c
 *
 *  Created on: Mar 4, 2023
 *      Author: mango
 */

#include "ModBusCRC.h"

uint16_t usMBCRC16(uint8_t * pucFrame, uint16_t usLen)
{
	uint8_t           ucCRCHi = 0xFF;
	uint8_t           ucCRCLo = 0xFF;
    int             iIndex;

    while( usLen-- )
    {
        iIndex = ucCRCLo ^ *( pucFrame++ );
        ucCRCLo = ( uint8_t )( ucCRCHi ^ aucCRCHi[iIndex] );
        ucCRCHi = aucCRCLo[iIndex];
    }
    return ( uint16_t )( ucCRCHi << 8 | ucCRCLo );
}
