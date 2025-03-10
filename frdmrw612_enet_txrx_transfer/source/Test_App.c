/*
 * Test_App.c
 *
 *  Created on: 18 feb. 2025
 *      Author: hecto
 */

#include "stdint.h"
#include "fsl_debug_console.h"
#include "ETH_API.h"

#define E_OK				   (1)
#define E_NOT_OK			   (0)
#define ENET_DATA_LENGTH       (1488)
#define TOTAL_MSGS			   (16)

typedef struct
{
	uint16_t u16Size;
	uint8_t u8MsgToSend[40];
}Test_App_stMsgs;

uint16_t u16GetMsgSize(uint8_t* u8MsgData)
{
	uint16_t u16Size = 0U;
	uint8_t u8Flag = 1U;

	while(u8Flag)
	{
		if(u8MsgData[u16Size] == '.')
		{
			u8Flag = 0U;
			u16Size += 2;
		}
		else if(u8MsgData[u16Size] == '?')
		{
			u8Flag = 0U;
		}
		u16Size++;
	}

	return u16Size;
}

int main(void)
{
	uint8_t u8IndexMsg = 0U;
	uint8_t u8SendState = (uint8_t)E_NOT_OK;
	uint8_t u8RxState = (uint8_t)E_NOT_OK;
	uint8_t pu8RxData[ENET_DATA_LENGTH] = {0U};

	Test_App_stMsgs stMsgData[TOTAL_MSGS] = {0, "No todo lo que es oro reluce...",
									 0, "Aún en la oscuridad...",
									 0, "¿Qué es la vida?",
									 0, "No temas a la oscuridad...",
									 0, "Hasta los más pequeños...",
									 0, "No digas que el sol se ha puesto...",
									 0, "El coraje se encuentra...",
									 0, "No todos los tesoros...",
									 0, "Es peligroso...",
									 0, "Un mago nunca llega tarde...",
									 0, "Aún hay esperanza...",
									 0, "El mundo está cambiando...",
									 0, "Las raíces profundas...",
									 0, "No se puede...",
									 0, "Y sobre todo...",
									 0, "De las cenizas, un fuego..."
	};


	for(uint8_t u8Index = 0U; u8Index < TOTAL_MSGS; u8Index++)
	{
		stMsgData[u8Index].u16Size = u16GetMsgSize((uint8_t*)(&stMsgData[u8Index].u8MsgToSend));
	}

	ETH_API_vInit();

	while(1)
	{
		for(u8IndexMsg = 0U; u8IndexMsg < TOTAL_MSGS; u8IndexMsg++)
		{
			SDK_DelayAtLeastUs(1000000, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
			u8SendState = ETH_API_u8Send((uint8_t*)&stMsgData[u8IndexMsg].u8MsgToSend, (uint16_t)stMsgData[u8IndexMsg].u16Size);

			if(u8SendState == (uint8_t)E_OK)
			{
				SDK_DelayAtLeastUs(10000, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
				u8RxState = ETH_API_u8Receive((uint8_t*)pu8RxData);

				if(u8RxState == (uint8_t)E_OK)
				{
					 PRINTF("Frame received!\r\n");
				}
			}
		}
	}

}


