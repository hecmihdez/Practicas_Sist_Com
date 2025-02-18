/*
 * ETH_API.c
 *
 *  Created on: 9 feb. 2025
 *      Author: hecto
 */

#include "stdint.h"
#include "fsl_debug_console.h"
#include "fsl_silicon_id.h"
#include "fsl_enet.h"
#include "fsl_phy.h"
#include "fsl_crc.h"
#include "board.h"
#include "app.h"
#include "aes.h"

#include "ETH_API_cfg.h"
#include "ETH_API.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define ENET_RXBD_NUM          (4)
#define ENET_TXBD_NUM          (4)
#define E_OK				   (1)
#define E_NOT_OK			   (0)
#define HEADER_OFFSET		   (14)
#define ENET_RXBUFF_SIZE       (ENET_FRAME_MAX_FRAMELEN)
#define ENET_TXBUFF_SIZE       (ENET_FRAME_MAX_FRAMELEN)
#define ENET_DATA_LENGTH       (1000)
#define ENET_TRANSMIT_DATA_NUM (20)
#ifndef APP_ENET_BUFF_ALIGNMENT
#define APP_ENET_BUFF_ALIGNMENT ENET_BUFF_ALIGNMENT
#endif
#ifndef PHY_AUTONEGO_TIMEOUT_COUNT
#define PHY_AUTONEGO_TIMEOUT_COUNT (300000)
#endif

#ifndef MAC_ADDRESS_SOURCE
#define MAC_ADDRESS_SOURCE                        \
    {                                      \
        0x54, 0x27, 0x8d, 0x00, 0x00, 0x00 \
    }
#endif

/*******************************************************************************
 * Variables
 ******************************************************************************/
AT_NONCACHEABLE_SECTION_ALIGN(enet_rx_bd_struct_t g_rxBuffDescrip[ENET_RXBD_NUM], ENET_BUFF_ALIGNMENT);
AT_NONCACHEABLE_SECTION_ALIGN(enet_tx_bd_struct_t g_txBuffDescrip[ENET_TXBD_NUM], ENET_BUFF_ALIGNMENT);
SDK_ALIGN(uint8_t g_rxDataBuff[ENET_RXBD_NUM][SDK_SIZEALIGN(ENET_RXBUFF_SIZE, APP_ENET_BUFF_ALIGNMENT)],
          APP_ENET_BUFF_ALIGNMENT);
SDK_ALIGN(uint8_t g_txDataBuff[ENET_TXBD_NUM][SDK_SIZEALIGN(ENET_TXBUFF_SIZE, APP_ENET_BUFF_ALIGNMENT)],
          APP_ENET_BUFF_ALIGNMENT);

static phy_handle_t phyHandle;
static enet_handle_t g_handle;
static CRC_Type *base = CRC_ENGINE;
static uint8_t g_frame[ENET_DATA_LENGTH + HEADER_OFFSET] = {0U};
uint8_t g_macAddr[6] = MAC_ADDRESS_SOURCE;
uint8_t g_macAddr_Dest[6] = ETH_API_MAC_DEST;


/*******************************************************************************
 * Private functions
 ******************************************************************************/

static void ETH_API_s_vFillDataFrame(uint8_t* pData, uint16_t u16Size, uint32_t u32CRC)
{
	uint32_t u32count  = 0U;
	uint32_t u32CRCIndex = 0U;

    memcpy(&g_frame[0], &g_macAddr_Dest[0], 6U);
    memcpy(&g_frame[6], &g_macAddr[0], 6U);

    g_frame[12] = (u16Size >> 8) & 0xFFU;
    g_frame[13] = u16Size & 0xFFU;

    u32CRCIndex = (uint32_t)(u16Size + HEADER_OFFSET);

    for (u32count = 0U; u32count < (uint32_t)u16Size; u32count++)
    {
        g_frame[u32count + HEADER_OFFSET] = pData[u32count];
    }

    g_frame[u32CRCIndex] = (u32CRC >> 24) & 0xFFu;
    g_frame[u32CRCIndex + 1] = (u32CRC >> 16) & 0xFFu;
    g_frame[u32CRCIndex + 2] = (u32CRC >> 8) & 0xFFu;
    g_frame[u32CRCIndex + 3] = u32CRC & 0xFFu;
}

static uint16_t ETH_API_u16GetActualSize(uint16_t u16DataSize)
{
	uint16_t u16Remainder = (uint16_t)(u16DataSize % 16);
	uint16_t u16ActSize = (uint16_t)0U;

	if(u16Remainder > (uint16_t)0U)
	{
		u16ActSize = u16DataSize + (16 - u16Remainder);
	}
	else
	{
		u16ActSize = u16DataSize;
	}

	return u16ActSize;
}

static void ETH_API_vFillDataPadding(uint8_t* pBufferData, uint8_t* pDataPadding, uint16_t u16ActualSize, uint16_t u16PrevSize)
{
	uint8_t u8IndexData = (uint8_t)0U;

	for(u8IndexData = 0U; u8IndexData < u16ActualSize; u8IndexData++)
	{
		if(u8IndexData < u16PrevSize)
		{
			pDataPadding[u8IndexData] = pBufferData[u8IndexData];
		}
		else
		{
			pDataPadding[u8IndexData] = (uint8_t)(u16ActualSize - u16PrevSize);
		}
	}
}

static void ETH_API_vInitCrc32(CRC_Type *base, uint32_t seed)
{
    crc_config_t config;

    config.polynomial    = kCRC_Polynomial_CRC_32;
    config.reverseIn     = true;
    config.complementIn  = false;
    config.reverseOut    = true;
    config.complementOut = true;
    config.seed          = seed;

    CRC_Init(base, &config);
}

static uint16_t ETH_API_u16GetPayloadData(uint8_t* pu8Data, uint32_t u32Length, uint8_t* pu8PayloadData, uint8_t* pu8CRCMsg)
{
	uint32_t u32Counter = 0U;
	uint32_t u32CounterCRC = 0U;
	uint16_t u16PayloadSize = 0U;

	u16PayloadSize |= ((pu8Data[12]<<8)&0xFF00U);
	u16PayloadSize |= (pu8Data[13] & 0xFFU);

	for(u32Counter = 0U; u32Counter < (uint32_t)(u16PayloadSize + 4); u32Counter++)
	{
		if(u32Counter < u16PayloadSize)
		{
			pu8PayloadData[u32Counter] = pu8Data[u32Counter + HEADER_OFFSET];
		}
		else
		{
			pu8CRCMsg[u32CounterCRC] = pu8Data[u32Counter + HEADER_OFFSET];
			u32CounterCRC++;
		}
	}

	return u16PayloadSize;
}

/*******************************************************************************
 * Public functions
 ******************************************************************************/

void ETH_API_vInit(void)
{
    phy_config_t phyConfig = {0};
    uint32_t testTxNum     = 0;
    uint32_t length        = 0;
    enet_data_error_stats_t eErrStatic;
    status_t status;
    enet_config_t config;
    volatile uint32_t count = 0;
    phy_speed_t speed;
    phy_duplex_t duplex;
    bool autonego = false;
    bool link     = false;
    bool tempLink = false;

    /* Hardware Initialization. */
    BOARD_InitHardware();

    PRINTF("\r\nENET example start.\r\n");

    /* Prepare the buffer configuration. */
    enet_buffer_config_t buffConfig[] = {{
        ENET_RXBD_NUM,
        ENET_TXBD_NUM,
        SDK_SIZEALIGN(ENET_RXBUFF_SIZE, APP_ENET_BUFF_ALIGNMENT),
        SDK_SIZEALIGN(ENET_TXBUFF_SIZE, APP_ENET_BUFF_ALIGNMENT),
        &g_rxBuffDescrip[0],
        &g_txBuffDescrip[0],
        &g_rxDataBuff[0][0],
        &g_txDataBuff[0][0],
        true,
        true,
        NULL,
    }};

    /* Get default configuration. */
    /*
     * config.miiMode = kENET_RmiiMode;
     * config.miiSpeed = kENET_MiiSpeed100M;
     * config.miiDuplex = kENET_MiiFullDuplex;
     * config.rxMaxFrameLen = ENET_FRAME_MAX_FRAMELEN;
     */
    ENET_GetDefaultConfig(&config);

    /* The miiMode should be set according to the different PHY interfaces. */
    config.miiMode = kENET_RmiiMode;
    phyConfig.phyAddr = EXAMPLE_PHY_ADDRESS;
    phyConfig.autoNeg = true;
    phyConfig.ops      = EXAMPLE_PHY_OPS;
    phyConfig.resource = EXAMPLE_PHY_RESOURCE;

    /* Initialize PHY and wait auto-negotiation over. */
    PRINTF("Wait for PHY init...\r\n");
    do
    {
        status = PHY_Init(&phyHandle, &phyConfig);
        if (status == kStatus_Success)
        {
            PRINTF("Wait for PHY link up...\r\n");
            /* Wait for auto-negotiation success and link up */
            count = PHY_AUTONEGO_TIMEOUT_COUNT;
            do
            {
                PHY_GetLinkStatus(&phyHandle, &link);
                if (link)
                {
                    PHY_GetAutoNegotiationStatus(&phyHandle, &autonego);
                    if (autonego)
                    {
                        break;
                    }
                }
            } while (--count);
            if (!autonego)
            {
                PRINTF("PHY Auto-negotiation failed. Please check the cable connection and link partner setting.\r\n");
            }
        }
    } while (!(link && autonego));

    /* Get the actual PHY link speed and set in MAC. */
    PHY_GetLinkSpeedDuplex(&phyHandle, &speed, &duplex);
    config.miiSpeed  = (enet_mii_speed_t)speed;
    config.miiDuplex = (enet_mii_duplex_t)duplex;

    /* Init the ENET. */
    ENET_Init(EXAMPLE_ENET, &g_handle, &config, &buffConfig[0], &g_macAddr[0], EXAMPLE_CLOCK_FREQ);
    ENET_ActiveRead(EXAMPLE_ENET);
}


uint8_t ETH_API_u8Send(uint8_t* pData, uint16_t u16DataSize)
{
    bool link     = false;
    static bool tempLink = false;
    uint8_t u8SendState = (uint8_t)E_NOT_OK;
    uint16_t u16ActualSize = (uint16_t)0U;
    uint32_t checksum32;
    struct AES_ctx ctx;
    uint8_t key[16] = ETH_API_AES_KEY;
    uint8_t iv[16] = ETH_API_AES_IV;
    uint32_t u32DataLength = 0U;

    PHY_GetLinkStatus(&phyHandle, &link);

    if (tempLink != link)
    {
        PRINTF("PHY link changed, link status = %u\r\n", link);
        tempLink = link;
    }

    u16ActualSize = ETH_API_u16GetActualSize(u16DataSize);

    uint8_t* pPaddingData = (uint8_t *)malloc(u16ActualSize);
    ETH_API_vFillDataPadding(pData, pPaddingData, u16ActualSize, u16DataSize);

//    PRINTF(" Msg before %c %c %c %c %c %c %c %c %c %c %c %c %c %c %c %c \r\n",
//    		pPaddingData[0], pPaddingData[1], pPaddingData[2], pPaddingData[3], pPaddingData[4], pPaddingData[5], pPaddingData[6], pPaddingData[7], pPaddingData[8], pPaddingData[9],
//			pPaddingData[10], pPaddingData[11], pPaddingData[12], pPaddingData[13], pPaddingData[14], pPaddingData[15]);

    AES_init_ctx_iv((struct AES_ctx*)&ctx, &key[0], &iv[0]);
    AES_CBC_encrypt_buffer((struct AES_ctx*)&ctx, pPaddingData, (size_t)u16ActualSize);

//    PRINTF(" Msg after %c %c %c %c %c %c %c %c %c %c %c %c %c %c %c %c \r\n",
//        		pPaddingData[0], pPaddingData[1], pPaddingData[2], pPaddingData[3], pPaddingData[4], pPaddingData[5], pPaddingData[6], pPaddingData[7], pPaddingData[8], pPaddingData[9],
//    			pPaddingData[10], pPaddingData[11], pPaddingData[12], pPaddingData[13], pPaddingData[14], pPaddingData[15]);
//
//    AES_init_ctx_iv((struct AES_ctx*)&ctx, &key[0], &iv[0]);
//    AES_CBC_decrypt_buffer((struct AES_ctx*)&ctx, pPaddingData, (size_t)u16ActualSize);
//
//    PRINTF(" Msg after %c %c %c %c %c %c %c %c %c %c %c %c %c %c %c %c \r\n",
//         		pPaddingData[0], pPaddingData[1], pPaddingData[2], pPaddingData[3], pPaddingData[4], pPaddingData[5], pPaddingData[6], pPaddingData[7], pPaddingData[8], pPaddingData[9],
//     			pPaddingData[10], pPaddingData[11], pPaddingData[12], pPaddingData[13], pPaddingData[14], pPaddingData[15]);

    ETH_API_vInitCrc32(base, 0xFFFFFFFFU);
    CRC_WriteData(base, (uint8_t *)&pPaddingData[0], (size_t)u16ActualSize);
    checksum32 = CRC_Get32bitResult(base);

    PRINTF("CRC-32: 0x%x\r\n\r\n", checksum32);

    ETH_API_s_vFillDataFrame(pPaddingData, u16ActualSize, checksum32);
    free(pPaddingData);

    u32DataLength = (uint32_t)(u16ActualSize + HEADER_OFFSET + sizeof(checksum32));

	if (link)
	{
		SDK_DelayAtLeastUs(10000, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
		if (kStatus_Success ==
			ENET_SendFrame(EXAMPLE_ENET, &g_handle, &g_frame[0], u32DataLength, 0, false, NULL))
		{
			PRINTF("Transmit frame succeeded!\r\n");
			u8SendState = (uint8_t)E_OK;
		}
		else
		{
			PRINTF(" \r\nTransmit frame failed!\r\n");
		}
	}

	return u8SendState;
}


uint8_t ETH_API_u8Receive(void)
{
    uint32_t length        = 0;
    enet_data_error_stats_t eErrStatic;
    bool link     = false;
    static bool tempLink = false;
    status_t status;
    uint8_t pu8PayloadData[ENET_DATA_LENGTH] = {0U};
    uint8_t pu8CRCMsg[4] = {0U};
    uint16_t u16DataSize = 0U;
    uint32_t checksum32;
    uint32_t u32CRCMsg;
    struct AES_ctx ctx;
    uint8_t key[16] = ETH_API_AES_KEY;
    uint8_t iv[16] = ETH_API_AES_IV;

    PHY_GetLinkStatus(&phyHandle, &link);

    if (tempLink != link)
    {
        PRINTF("PHY link changed, link status = %u\r\n", link);
        tempLink = link;
    }

    /* Get the Frame size */
    (void)ENET_GetRxFrameSize(&g_handle, &length, 0);
    /* Call ENET_ReadFrame when there is a received frame. */
    if (length != 0)
    {
        /* Received valid frame. Deliver the rx buffer with the size equal to length. */
        uint8_t *data = (uint8_t *)malloc(length);
        status        = ENET_ReadFrame(EXAMPLE_ENET, &g_handle, data, length, 0, NULL);
        if (status == kStatus_Success)
        {
//            PRINTF(" A frame received. the length %d ", length);
//            PRINTF(" Dest Address %02x:%02x:%02x:%02x:%02x:%02x Src Address %02x:%02x:%02x:%02x:%02x:%02x \r\n",
//                   data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9],
//                   data[10], data[11]);
        	u16DataSize = ETH_API_u16GetPayloadData(data, length, (uint8_t*)&pu8PayloadData[0], (uint8_t*)&pu8CRCMsg[0]);

            ETH_API_vInitCrc32(base, 0xFFFFFFFFU);
            CRC_WriteData(base, (uint8_t *)&pu8PayloadData[0], (size_t)u16DataSize);
            checksum32 = CRC_Get32bitResult(base);

            u32CRCMsg |= (pu8CRCMsg[0]<<24) & 0xFF000000U;
            u32CRCMsg |= (pu8CRCMsg[1]<<16) & 0xFF0000U;
            u32CRCMsg |= (pu8CRCMsg[2]<<8) & 0xFF00U;
            u32CRCMsg |= pu8CRCMsg[3] & 0xFFU;

            if(checksum32 == u32CRCMsg)
            {
                AES_init_ctx_iv((struct AES_ctx*)&ctx, &key[0], &iv[0]);
                AES_CBC_decrypt_buffer((struct AES_ctx*)&ctx, pu8PayloadData, (size_t)u16DataSize);
            }

        }
        free(data);
    }
    else if (status == kStatus_ENET_RxFrameError)
    {
        /* Update the received buffer when error happened. */
        /* Get the error information of the received g_frame. */
        ENET_GetRxErrBeforeReadFrame(&g_handle, &eErrStatic, 0);
        /* update the receive buffer. */
        ENET_ReadFrame(EXAMPLE_ENET, &g_handle, NULL, 0, 0, NULL);
    }
}
