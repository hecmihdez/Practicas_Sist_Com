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
#include "board.h"
#include "app.h"

#include "ETH_API_cfg.h"
#include "ETH_API.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define ENET_RXBD_NUM          (4)
#define ENET_TXBD_NUM          (4)
#define E_OK				   (1)
#define E_NOT_OK			   (0)
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

#ifndef MAC_ADDRESS
#define MAC_ADDRESS                        \
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
static uint8_t g_frame[ENET_DATA_LENGTH + 14];
uint8_t g_macAddr[6] = MAC_ADDRESS;


/*******************************************************************************
 * Private functions
 ******************************************************************************/
static void ENET_BuildBroadCastFrame(void)
{
    uint32_t count  = 0;
    uint32_t length = ENET_DATA_LENGTH - 14;

    for (count = 0; count < 6U; count++)
    {
        g_frame[count] = 0xFFU;
    }
    memcpy(&g_frame[6], &g_macAddr[0], 6U);
    g_frame[12] = (length >> 8) & 0xFFU;
    g_frame[13] = length & 0xFFU;

    for (count = 0; count < length; count++)
    {
        g_frame[count + 14] = 0U;
    }
}

static void ETH_API_s_vFillDataFrame(uint8_t* pData, uint16_t u16Size)
{
	uint32_t count  = 0;
	uint32_t length = ENET_DATA_LENGTH - 14;

    for (count = 0; count < u16Size; count++)
    {
        g_frame[count + 14] = pData[count];
    }
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

    /* Build broadcast for sending. */
    ENET_BuildBroadCastFrame();
}


uint8_t ETH_API_u8Send(uint8_t* pData, uint16_t u16DataSize)
{
    bool link     = false;
    static bool tempLink = false;
    uint8_t u8SendState = (uint8_t)E_NOK_OK;

    PHY_GetLinkStatus(&phyHandle, &link);

    if (tempLink != link)
    {
        PRINTF("PHY link changed, link status = %u\r\n", link);
        tempLink = link;
    }

    ETH_API_s_vFillDataFrame(pData, u16DataSize);

	if (link)
	{
		SDK_DelayAtLeastUs(10000, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
		if (kStatus_Success ==
			ENET_SendFrame(EXAMPLE_ENET, &g_handle, &g_frame[0], ENET_DATA_LENGTH, 0, false, NULL))
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

    PHY_GetLinkStatus(&phyHandle, &link);

    if (tempLink != link)
    {
        PRINTF("PHY link changed, link status = %u\r\n", link);
        tempLink = link;
    }

    /* Get the Frame size */
    status = ENET_GetRxFrameSize(&g_handle, &length, 0);
    /* Call ENET_ReadFrame when there is a received frame. */
    if (length != 0)
    {
        /* Received valid frame. Deliver the rx buffer with the size equal to length. */
        uint8_t *data = (uint8_t *)malloc(length);
        status        = ENET_ReadFrame(EXAMPLE_ENET, &g_handle, data, length, 0, NULL);
        if (status == kStatus_Success)
        {
            PRINTF(" A frame received. the length %d ", length);
            PRINTF(" Dest Address %02x:%02x:%02x:%02x:%02x:%02x Src Address %02x:%02x:%02x:%02x:%02x:%02x \r\n",
                   data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9],
                   data[10], data[11]);
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
