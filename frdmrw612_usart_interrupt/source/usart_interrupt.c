/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "board.h"
#include "app.h"
#include "fsl_usart.h"
#include "fsl_gpio.h"
#include "fsl_debug_console.h"
#include "LIN.h"
#include "LIN_cfg.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

uint8_t MsgsTx[TOTAL_MSGS_TX] = MSG_IDS_MASTER;
static stResponseData MsgsRx[TOTAL_MSGS_RX_RESP] = MSG_RESPONSE_TABLE;
static bool SendMsg = false;
static uint8_t Index = 0U;
static uint32_t counter = 0U;
static uint32_t counter2 = 0U;
static stLinMsg BufferRxFrame = {0U};
static uint8_t BufferRxData[16] = {0U};
static bool Header_flag = false;
static bool Data_flag = false;

/*******************************************************************************
 * Code
 ******************************************************************************/
void GPIO_INTA_DriverIRQHandler(void)
{
    /* clear the interrupt status */
    GPIO_PinClearInterruptFlag(GPIO, 0, 20, 0);
    /* Change state of switch. */

    if(counter > counter2)
    {
    	if((counter - counter2) > 10)
		{
			SendMsg = true;

			Index = Index < (TOTAL_MSGS_TX - 1) ? Index+1 : 0U;

			counter2 = counter;
		}
    }
    else
    {
    	counter2 = counter;
    }

    SDK_ISR_EXIT_BARRIER;
}

void SysTick_Handler(void)
{
	if(counter < 5000)
	{
		counter++;
	}
	else
	{
		SendMsg = true;

		Index = Index < (TOTAL_MSGS_TX - 1) ? Index+1 : 0U;

		counter = 0;
	}
}

static void SysTick_Init(void)
{
	uint32_t coreClock = 0U;
	uint32_t tickRate = 0U;

	coreClock = CLOCK_GetCoreSysClkFreq();
	tickRate = coreClock / 1000;

    SysTick_Config(tickRate);
}

bool MsgFrameSent_callback(bool State)
{
	return State;
}

void MsgRx_Callback(void* data, uint8_t length)
{
	stLinMsg* Buff = (stLinMsg*)data;
	uint8_t* BuffData = (uint8_t*)data;
	uint8_t BufferIndex = 0U;

	if(length == 1)
	{
		BufferRxFrame.Break = Buff->Break;
		BufferRxFrame.SynchByte = Buff->SynchByte;
		BufferRxFrame.IDMsg = Buff->IDMsg;

		Header_flag = true;
	}
	else
	{
		for(BufferIndex = 0U; BufferIndex < length; BufferIndex++)
		{
			BufferRxData[BufferIndex] = BuffData[BufferIndex];
		}

		Data_flag = true;
	}
}


/*!
 * @brief Main function
 */
int main(void)
{
	bool CheckSum_State = false;
	uint8_t MsgId = 0U;

    BOARD_InitHardware();

    SysTick_Init();

    PRINTF("Practica: LIN-Protocol \r\n");
    PRINTF("Presione el boton para enviar un Frame... \r\n");

    LIN_vInit();

    LIN_vInstallMsgRxCB((lin_msg_callback)&MsgRx_Callback);

    while(1)
    {
        if(SendMsg == true)
        {
        	SendMsg = false;

        	LIN_vSendMsgFrame(MsgsTx[Index]);
        	PRINTF("Frame enviado \r\n");
        }

    	if(Header_flag)
    	{
    		Header_flag = false;
    		MsgId = LIN_u8GetMsgId(BufferRxFrame.IDMsg);

    		for(uint8_t u8i = 0; u8i < TOTAL_MSGS_RX_RESP; u8i++)
    		{
    			if(MsgId == MsgsRx[u8i].Id)
    			{
    				PRINTF("Respuesta enviada: %s \r\n", MsgsRx[u8i].Response);
    				LIN_vTxMsg(MsgsRx[u8i].Response, sizeof(MsgsRx[u8i].Response));
					break;
    			}
    		}
    	}

        if(Data_flag)
        {
        	Data_flag = false;

        	CheckSum_State = LIN_u8Checksum(BufferRxData, 8);

        	if(CheckSum_State)
        	{
        		PRINTF("Respuesta recibida: %s \r\n", BufferRxData);
        	}
        }
    }
}
