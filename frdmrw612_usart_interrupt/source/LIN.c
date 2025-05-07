/*
 * LIN.c}
 *
 *  Created on: 6 may. 2025
 *      Author: hecto
 */

#include "fsl_usart.h"
#include "LIN.h"


const usart_config_t FLEXCOMM0_config = {
  .baudRate_Bps = 19200UL,
  .syncMode = kUSART_SyncModeDisabled,
  .parityMode = kUSART_ParityDisabled,
  .stopBitCount = kUSART_OneStopBit,
  .bitCountPerChar = kUSART_8BitsPerChar,
  .loopback = false,
  .txWatermark = kUSART_TxFifo0,
  .rxWatermark = kUSART_RxFifo1,
  .rxTimeout = {
    .enable = false,
    .resetCounterOnEmpty = true,
    .resetCounterOnReceive = true,
    .counter = 0U,
    .prescaler = 0U
  },
  .enableRx = true,
  .enableTx = true,
  .enableHardwareFlowControl = false,
  .enableMode32k = false,
  .clockPolarity = kUSART_RxSampleOnFallingEdge,
  .enableContinuousSCLK = false,
  .enableLinMode = true
};

void USART_IRQHandler(void)
{
    uint8_t data;

    /* If new data arrived. */
//    if ((kUSART_RxFifoNotEmptyFlag | kUSART_RxError | kUSART_BreakDetectChangeFlag) & USART_GetStatusFlags(USART_BASE))
    if ((kUSART_BreakDetectChangeFlag) & USART_GetStatusFlags(USART_BASE))
    {
        data = USART_ReadByte(USART_BASE);
    }
    SDK_ISR_EXIT_BARRIER;
}

void LIN_vInit(void)
{
	USART_Init(USART_BASE, &FLEXCOMM0_config, USART_CLK_FREQ);

    /* Enable RX interrupt. */
//    USART_EnableInterrupts(USART_BASE, kUSART_RxLevelInterruptEnable | kUSART_RxErrorInterruptEnable |kUSART_RxBreakChangeInterruptEnable);
    USART_EnableInterrupts(USART_BASE, kUSART_RxBreakChangeInterruptEnable);
    EnableIRQ(USART_IRQn);
}

void LIN_vTxMsg(uint8_t *data, size_t length)
{
	USART_WriteBlocking(USART_BASE, data, length);
}

void LIN_vSendHeader(void)
{
	USART_Type* base = USART_BASE;
	uint8_t DummyChar = 0xFF;
	uint8_t SyncByte = 0x55;

//	base->CTL |= ((uint32_t)(((uint32_t)(1)) << USART_CTL_TXDIS_SHIFT));

//	while((base->STAT & USART_STAT_TXDISSTAT_MASK) == 0U)
//	{
//	}

//	USART_CTL_TXBRKEN(1);

	base->CTL |= USART_CTL_TXBRKEN_MASK;
//	base->CTL |= ((uint32_t)(((uint32_t)(1)) << USART_CTL_AUTOBAUD_SHIFT));

//	USART_WriteByte(USART_BASE, 0);

	USART_WriteBlocking(USART0, &DummyChar, 1);

//	delay();


	base->CTL &= ~USART_CTL_TXBRKEN_MASK;
//	USART_CTL_TXBRKEN(0);
//	base->CTL &= ~USART_CTL_TXDIS_MASK;
//	base->CTL &= ((uint32_t)(((uint32_t)(0)) << USART_CTL_TXDIS_SHIFT));

	USART_WriteBlocking(USART0, &SyncByte, 1);
}
