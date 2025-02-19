/*
 * ETH_API.h
 *
 *  Created on: 9 feb. 2025
 *      Author: hecto
 */

#ifndef ETH_API_H_
#define ETH_API_H_


extern void ETH_API_vInit(void);

extern uint8_t ETH_API_u8Send(uint8_t* pData, uint16_t u16DataSize);

extern uint8_t ETH_API_u8Receive(uint8_t* pu8DataBuff);


#endif /* ETH_API_H_ */
