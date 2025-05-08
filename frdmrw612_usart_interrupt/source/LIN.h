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

#define MSG_IDS_MASTER 			{0x7, 0xB, 0xF}
#define TOTAL_MSGS_TX				(3)
#define TOTAL_MSGS_RX				(1)

#define MSG_RESPONSE_TABLE \
{	\
	{0xB, "HECTOR"},	\
}

typedef void (*lin_msg_callback)(void*, uint8_t);

typedef struct
{
	uint8_t Break;
	uint8_t SynchByte;
	uint8_t IDMsg;
}stLinMsg;

typedef struct
{
	uint8_t Data[8];
	uint8_t CheckSum;
}stResponseMsg;

typedef struct
{
	uint8_t Id;
	uint8_t Response[8];
}stResponseData;

extern void LIN_vInit(void);
extern void LIN_vTxMsg(uint8_t *data, size_t length);
extern void LIN_vSendMsgFrame(uint8_t MsgId);
extern void LIN_vInstallMsgRxCB(lin_msg_callback callback_user);
extern uint8_t LIN_u8GetMsgId(uint8_t MsgId);
extern bool LIN_u8Checksum(uint8_t *data, uint8_t size);

#endif /* LIN_H_ */
