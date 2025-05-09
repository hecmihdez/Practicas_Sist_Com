/*
 * LIN_cfg.h
 *
 *  Created on: 8 may. 2025
 *      Author: hecto
 */

#ifndef LIN_CFG_H_
#define LIN_CFG_H_

#define USART_BASE            USART0
#define USART_CLK_SRC    	  kCLOCK_Flexcomm0
#define USART_CLK_FREQ   	  CLOCK_GetFlexCommClkFreq(0U)
#define USART_IRQHandler 	  FLEXCOMM0_IRQHandler
#define USART_IRQn       	  FLEXCOMM0_IRQn

#define MSG_IDS_MASTER 			{0x7, 0xB, 0xF}
#define TOTAL_MSGS_TX			(3)
#define TOTAL_MSGS_RX_RESP		(1)
#define TOTAL_MSGS_RX_STORE		(0)

#define MSG_RESPONSE_TABLE \
{	\
	{0xB, "HECTOR"},	\
}

#define MSG_STORAGE_TABLE 		{0}

#endif /* LIN_CFG_H_ */
