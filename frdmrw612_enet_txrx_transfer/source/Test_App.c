/*
 * Test_App.c
 *
 *  Created on: 18 feb. 2025
 *      Author: hecto
 */

#include "ETH_API.h"

int main(void)
{
	uint8_t Data[10] = "HECTOR MIG";

	ETH_API_vInit();

	(void)ETH_API_u8Send((uint8_t*)&Data, (uint16_t)10);
}


