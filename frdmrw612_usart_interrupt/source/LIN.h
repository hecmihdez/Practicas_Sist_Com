/*
 * LIN.h
 *
 *  Created on: 6 may. 2025
 *      Author: hecto
 */

#ifndef LIN_H_
#define LIN_H_

/*******************************************************************************
 * Definitions
 ******************************************************************************/

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


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

extern void LIN_vInit(void);
extern void LIN_vTxMsg(uint8_t *data, size_t length);
extern void LIN_vSendMsgFrame(uint8_t MsgId);
extern void LIN_vInstallMsgRxCB(lin_msg_callback callback_user);
extern uint8_t LIN_u8GetMsgId(uint8_t MsgId);
extern bool LIN_u8Checksum(uint8_t *data, uint8_t size);

#endif /* LIN_H_ */
