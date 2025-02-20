/*
 * ETH_API.h
 *
 */

#ifndef ETH_API_H_
#define ETH_API_H_

/*!
 * @brief Initialize board hardware, the ENET and PHY modules.
 *
 * @note This must be called before ETH_API_u8Send or ETH_API_u8Receive
 * functions.
 *
 */
extern void ETH_API_vInit(void);

/*!
 * @brief Encrypt a message using AES128 algorithm and calculates CRC32,
 * then transmits the frame.
 *
 * @note This should be called when a frame transmission is required.
 *
 * @param pData  Buffer data to transmit.
 * @param u16DataSize Buffer size in bytes.
 * return The transmission status, successful (E_OK) or failure (E_NOT_OK).
 */
extern uint8_t ETH_API_u8Send(uint8_t* pData, uint16_t u16DataSize);


/*!
 * @brief Receive a message, then decrypt it using AES128 algorithm and
 * check CRC32.
 *
 * @note This should be called when a frame reception is required.
 *
 * @param pu8DataBuff  Buffer to allocate data received.
 * return The reception status, successful (E_OK) or failure (E_NOT_OK).
 */
extern uint8_t ETH_API_u8Receive(uint8_t* pu8DataBuff);

#endif /* ETH_API_H_ */





