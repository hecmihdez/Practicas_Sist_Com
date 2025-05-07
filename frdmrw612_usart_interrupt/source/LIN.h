/*
 * LIN.h
 *
 *  Created on: 6 may. 2025
 *      Author: hecto
 */

#ifndef LIN_H_
#define LIN_H_


#define USART_BASE            USART0
#define USART_CLK_SRC    	  kCLOCK_Flexcomm0
#define USART_CLK_FREQ   	  CLOCK_GetFlexCommClkFreq(0U)
#define USART_IRQHandler 	  FLEXCOMM0_IRQHandler
#define USART_IRQn       	  FLEXCOMM0_IRQn

extern void LIN_vInit(void);
extern void LIN_vTxMsg(uint8_t *data, size_t length);
extern void LIN_vSendHeader(void);

#endif /* LIN_H_ */
