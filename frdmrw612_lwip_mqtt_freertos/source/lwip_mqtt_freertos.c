/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2024 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "board.h"
#include "app.h"
#include "fsl_phy.h"
#include "mqtt_freertos.h"
#include "fsl_rtc.h"

#include "lwip/opt.h"
#include "lwip/api.h"
#include "lwip/dhcp.h"
#include "lwip/netifapi.h"
#include "ethernetif.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Must be after include of app.h */
#ifndef configMAC_ADDR
#include "fsl_silicon_id.h"
#endif

#ifndef EXAMPLE_NETIF_INIT_FN
/*! @brief Network interface initialization function. */
#define EXAMPLE_NETIF_INIT_FN ethernetif0_init
#endif /* EXAMPLE_NETIF_INIT_FN */

/*! @brief Stack size of the temporary lwIP initialization thread. */
#define INIT_THREAD_STACKSIZE 1024

/*! @brief Priority of the temporary lwIP initialization thread. */
#define INIT_THREAD_PRIO DEFAULT_THREAD_PRIO

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

static phy_handle_t phyHandle;

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief ISR for Alarm interrupt
 *
 * This function changes the state of busyWait.
 */
//void RTC_IRQHandler(void)
//{
//    if (RTC_GetStatusFlags(RTC) & kRTC_AlarmFlag)
//    {
////        busyWait = false;
//
//    	PRINTF("\r\n Alarm occurs !!!! ");
//        /* Clear alarm flag */
//        RTC_ClearStatusFlags(RTC, kRTC_AlarmFlag);
//    }
//    SDK_ISR_EXIT_BARRIER;
//}


/*!
 * @brief Initializes lwIP stack.
 *
 * @param arg unused
 */
static void stack_init(void *arg)
{
    static struct netif netif;
    ethernetif_config_t enet_config = {
        .phyHandle   = &phyHandle,
        .phyAddr     = EXAMPLE_PHY_ADDRESS,
        .phyOps      = EXAMPLE_PHY_OPS,
        .phyResource = EXAMPLE_PHY_RESOURCE,
        .srcClockHz  = EXAMPLE_CLOCK_FREQ,
#ifdef configMAC_ADDR
        .macAddress = configMAC_ADDR,
#endif
    };

    LWIP_UNUSED_ARG(arg);

    /* Set MAC address. */
#ifndef configMAC_ADDR
    (void)SILICONID_ConvertToMacAddr(&enet_config.macAddress);
#endif

    tcpip_init(NULL, NULL);

    netifapi_netif_add(&netif, NULL, NULL, NULL, &enet_config, EXAMPLE_NETIF_INIT_FN, tcpip_input);
    netifapi_netif_set_default(&netif);
    netifapi_netif_set_up(&netif);

    netifapi_dhcp_start(&netif);

    PRINTF("\r\n************************************************\r\n");
    PRINTF(" MQTT client example\r\n");
    PRINTF("************************************************\r\n");

    while (ethernetif_wait_linkup(&netif, 5000) != ERR_OK)
    {
        PRINTF("PHY Auto-negotiation failed. Please check the cable connection and link partner setting.\r\n");
    }

    /* Wait for address from DHCP */

    PRINTF("Getting IP address from DHCP...\r\n");

    (void)ethernetif_wait_ipv4_valid(&netif, ETHERNETIF_WAIT_FOREVER);

    mqtt_freertos_run_thread(&netif);

    vTaskDelete(NULL);
}

/*!
 * @brief Main function
 */
int main(void)
{
    uint32_t currSeconds;
    rtc_datetime_t date;

    BOARD_InitHardware();

//    /* Init RTC */
//    RTC_Init(RTC);
//
//    /* Set a start date time and start RT */
//    date.year   = 2025U;
//    date.month  = 3U;
//    date.day    = 10U;
//    date.hour   = 19U;
//    date.minute = 0;
//    date.second = 0;
//
//    RTC_EnableTimer(RTC, false);
//
//    RTC_SetDatetime(RTC, &date);
//
//    EnableIRQ(RTC_IRQn);
//
//    RTC_EnableTimer(RTC, true);
//
//    /* Read the RTC seconds register to get current time in seconds */
//    currSeconds = RTC_GetSecondsTimerCount(RTC);
//
//    /* Add alarm seconds to current time */
//    currSeconds += 10;
//
//    /* Set alarm time in seconds */
//    RTC_SetSecondsTimerMatch(RTC, currSeconds);

    /* Initialize lwIP from thread */
    if (sys_thread_new("main", stack_init, NULL, INIT_THREAD_STACKSIZE, INIT_THREAD_PRIO) == NULL)
    {
        LWIP_ASSERT("main(): Task creation failed.", 0);
    }

    vTaskStartScheduler();

    /* Will not get here unless a task calls vTaskEndScheduler ()*/
    return 0;
}
