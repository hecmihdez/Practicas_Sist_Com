/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2022 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "mqtt_freertos.h"

#include "board.h"
#include "fsl_silicon_id.h"
#include "fsl_rtc.h"

#include "lwip/opt.h"
#include "lwip/api.h"
#include "lwip/apps/mqtt.h"
#include "lwip/tcpip.h"

// FIXME cleanup

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief MQTT server host name or IP address. */
#ifndef EXAMPLE_MQTT_SERVER_HOST
#define EXAMPLE_MQTT_SERVER_HOST "broker.hivemq.com"
#endif

/*! @brief MQTT server port number. */
#ifndef EXAMPLE_MQTT_SERVER_PORT
#define EXAMPLE_MQTT_SERVER_PORT 1883
#endif

/*! @brief Stack size of the temporary lwIP initialization thread. */
#define INIT_THREAD_STACKSIZE 1024

/*! @brief Priority of the temporary lwIP initialization thread. */
#define INIT_THREAD_PRIO DEFAULT_THREAD_PRIO

/*! @brief Stack size of the temporary initialization thread. */
#define APP_THREAD_STACKSIZE 1024

/*! @brief Priority of the temporary initialization thread. */
#define APP_THREAD_PRIO DEFAULT_THREAD_PRIO

/*! @brief Stack size of the temporary initialization thread. */
#define PUBLISH_THREAD_STACKSIZE 1024

/*! @brief Priority of the temporary initialization thread. */
#define PUBLISH_THREAD_PRIO DEFAULT_THREAD_PRIO

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static void connect_to_mqtt(void *ctx);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*! @brief MQTT client data. */
static mqtt_client_t *mqtt_client;

static sys_thread_t xHandleP;
static volatile bool Flag = false;

/*! @brief MQTT client ID string. */
static char client_id[(SILICONID_MAX_LENGTH * 2) + 5];

/*! @brief MQTT client information. */
static const struct mqtt_connect_client_info_t mqtt_client_info = {
    .client_id   = (const char *)&client_id[0],
    .client_user = NULL,
    .client_pass = NULL,
    .keep_alive  = 100,
    .will_topic  = NULL,
    .will_msg    = NULL,
    .will_qos    = 0,
    .will_retain = 0,
#if LWIP_ALTCP && LWIP_ALTCP_TLS
    .tls_config = NULL,
#endif
};

/*! @brief MQTT broker IP address. */
static ip_addr_t mqtt_addr;

/*! @brief Indicates connection to MQTT broker. */
static volatile bool connected = false;

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief ISR for Alarm interrupt
 *
 * This function changes the state of busyWait.
 */
void RTC_IRQHandler(void)
{
//	BaseType_t xYieldRequired;

    if (RTC_GetStatusFlags(RTC) & kRTC_AlarmFlag)
    {
//        busyWait = false;

    	PRINTF("\r\n Alarm occurs !!!! ");

    	Flag = true;

        /* Clear alarm flag */
        RTC_ClearStatusFlags(RTC, kRTC_AlarmFlag);

        // Resume the suspended task.
//        xYieldRequired = xTaskResumeFromISR( xHandleP );
//
//        // We should switch context so the ISR returns to a different task.
//        // NOTE:  How this is done depends on the port you are using.  Check
//        // the documentation and examples for your port.
//        portYIELD_FROM_ISR( xYieldRequired );
    }
    SDK_ISR_EXIT_BARRIER;
}

/*!
 * @brief Called when subscription request finishes.
 */
static void mqtt_topic_subscribed_cb(void *arg, err_t err)
{
    const char *topic = (const char *)arg;

    if (err == ERR_OK)
    {
        PRINTF("Subscribed to the topic \"%s\".\r\n", topic);
    }
    else
    {
        PRINTF("Failed to subscribe to the topic \"%s\": %d.\r\n", topic, err);
    }
}

/*!
 * @brief Called when there is a message on a subscribed topic.
 */
static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len)
{
    LWIP_UNUSED_ARG(arg);

    PRINTF("Received %u bytes from the topic \"%s\": \"", tot_len, topic);
}

/*!
 * @brief Called when recieved incoming published message fragment.
 */
static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags)
{
    int i;

    LWIP_UNUSED_ARG(arg);

    for (i = 0; i < len; i++)
    {
        if (isprint(data[i]))
        {
            PRINTF("%c", (char)data[i]);
        }
        else
        {
            PRINTF("\\x%02x", data[i]);
        }
    }

    if (flags & MQTT_DATA_FLAG_LAST)
    {
        PRINTF("\"\r\n");
    }
}

/*!
 * @brief Subscribe to MQTT topics.
 */
static void mqtt_subscribe_topics(mqtt_client_t *client)
{
    static const char *topics[] = {"lwip_topic/CAN/FIGO/Air", "lwip_topic/CAN/FIGO/Direccion", "lwip_topic/CAN/FIGO/Luces", "lwip_topic/CAN/FIGO/Ventanas"};
    int qos[]                   = {0, 0, 0, 0};
    err_t err;
    int i;

    mqtt_set_inpub_callback(client, mqtt_incoming_publish_cb, mqtt_incoming_data_cb,
                            LWIP_CONST_CAST(void *, &mqtt_client_info));

    for (i = 0; i < ARRAY_SIZE(topics); i++)
    {
        err = mqtt_subscribe(client, topics[i], qos[i], mqtt_topic_subscribed_cb, LWIP_CONST_CAST(void *, topics[i]));

        if (err == ERR_OK)
        {
            PRINTF("Subscribing to the topic \"%s\" with QoS %d...\r\n", topics[i], qos[i]);
        }
        else
        {
            PRINTF("Failed to subscribe to the topic \"%s\" with QoS %d: %d.\r\n", topics[i], qos[i], err);
        }
    }
}

/*!
 * @brief Called when connection state changes.
 */
static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status)
{
    const struct mqtt_connect_client_info_t *client_info = (const struct mqtt_connect_client_info_t *)arg;

    connected = (status == MQTT_CONNECT_ACCEPTED);

    switch (status)
    {
        case MQTT_CONNECT_ACCEPTED:
            PRINTF("MQTT client \"%s\" connected.\r\n", client_info->client_id);
            mqtt_subscribe_topics(client);
            break;

        case MQTT_CONNECT_DISCONNECTED:
            PRINTF("MQTT client \"%s\" not connected.\r\n", client_info->client_id);
            /* Try to reconnect 1 second later */
            sys_timeout(1000, connect_to_mqtt, NULL);
            break;

        case MQTT_CONNECT_TIMEOUT:
            PRINTF("MQTT client \"%s\" connection timeout.\r\n", client_info->client_id);
            /* Try again 1 second later */
            sys_timeout(1000, connect_to_mqtt, NULL);
            break;

        case MQTT_CONNECT_REFUSED_PROTOCOL_VERSION:
        case MQTT_CONNECT_REFUSED_IDENTIFIER:
        case MQTT_CONNECT_REFUSED_SERVER:
        case MQTT_CONNECT_REFUSED_USERNAME_PASS:
        case MQTT_CONNECT_REFUSED_NOT_AUTHORIZED_:
            PRINTF("MQTT client \"%s\" connection refused: %d.\r\n", client_info->client_id, (int)status);
            /* Try again 10 seconds later */
            sys_timeout(10000, connect_to_mqtt, NULL);
            break;

        default:
            PRINTF("MQTT client \"%s\" connection status: %d.\r\n", client_info->client_id, (int)status);
            /* Try again 10 seconds later */
            sys_timeout(10000, connect_to_mqtt, NULL);
            break;
    }
}

/*!
 * @brief Starts connecting to MQTT broker. To be called on tcpip_thread.
 */
static void connect_to_mqtt(void *ctx)
{
    LWIP_UNUSED_ARG(ctx);

    PRINTF("Connecting to MQTT broker at %s...\r\n", ipaddr_ntoa(&mqtt_addr));

    mqtt_client_connect(mqtt_client, &mqtt_addr, EXAMPLE_MQTT_SERVER_PORT, mqtt_connection_cb,
                        LWIP_CONST_CAST(void *, &mqtt_client_info), &mqtt_client_info);
}

/*!
 * @brief Called when publish request finishes.
 */
static void mqtt_message_published_cb(void *arg, err_t err)
{
    const char *topic = (const char *)arg;

    if (err == ERR_OK)
    {
        PRINTF("Published to the topic \"%s\".\r\n", topic);
    }
    else
    {
        PRINTF("Failed to publish to the topic \"%s\": %d.\r\n", topic, err);
    }
}

/*!
 * @brief Publishes a message. To be called on tcpip_thread.
 */
static void publish_message(void *ctx)
{
//	static const char *topics[] = {"lwip_topic/CAN/FIGO/Velocidad", "lwip_topic/CAN/FIGO/Rpm", "lwip_topic/CAN/FIGO/Acelerador", "lwip_topic/CAN/FIGO/Temperatura"};
    static const char *topic   = "lwip_topic/100";
    static const char *message = "message from board";
//	static const char *messages[] = {"200", "5000", "Acelerando", "40"};
	int i;

    LWIP_UNUSED_ARG(ctx);

//    for (i = 0; i < ARRAY_SIZE(topics); i++)
//    {
    	PRINTF("Going to publish to the topic \"%s\"...\r\n", topic);

    	mqtt_publish(mqtt_client, topic, message, strlen(message), 1, 0, mqtt_message_published_cb, (void *)topic);
//    }
}

//static void publish_msgs2(void *arg)
//{
//	err_t err;
//	int i;
//    uint32_t currSeconds;
//
//	LWIP_UNUSED_ARG(arg);
//
//	if (connected)
//	{
//		err = tcpip_callback(publish_message, NULL);
//		if (err != ERR_OK)
//		{
//			PRINTF("Failed to invoke publishing of a message on the tcpip_thread: %d.\r\n", err);
//		}
//	}
//
//	/* Read the RTC seconds register to get current time in seconds */
//	currSeconds = RTC_GetSecondsTimerCount(RTC);
//
//	/* Add alarm seconds to current time */
//	currSeconds += 10;
//
//	/* Set alarm time in seconds */
//	RTC_SetSecondsTimerMatch(RTC, currSeconds);
//
//	Flag = false;
//
//	vTaskSuspend(NULL);
//}

static void publish_msgs(void *arg)
{
	err_t err;
	int i;
    uint32_t currSeconds;

    LWIP_UNUSED_ARG(arg);

	/* Read the RTC seconds register to get current time in seconds */
	currSeconds = RTC_GetSecondsTimerCount(RTC);

	/* Add alarm seconds to current time */
	currSeconds += 5;

	/* Set alarm time in seconds */
	RTC_SetSecondsTimerMatch(RTC, currSeconds);


//    xHandleP = sys_thread_new("publish_msgs2", publish_msgs2, NULL, PUBLISH_THREAD_STACKSIZE, PUBLISH_THREAD_PRIO);

    for(;;)
    {
    	if((Flag == true))
    	{
//    		vTaskResume(xHandleP);

    		if (connected)
    		{
    			err = tcpip_callback(publish_message, NULL);
    			if (err != ERR_OK)
    			{
    				PRINTF("Failed to invoke publishing of a message on the tcpip_thread: %d.\r\n", err);
    			}
    		}

    		/* Read the RTC seconds register to get current time in seconds */
    		currSeconds = RTC_GetSecondsTimerCount(RTC);

    		/* Add alarm seconds to current time */
    		currSeconds += 5;

    		/* Set alarm time in seconds */
    		RTC_SetSecondsTimerMatch(RTC, currSeconds);

    		Flag = false;
    	}
    }

//	if (connected)
//	{
//		err = tcpip_callback(publish_message, NULL);
//		if (err != ERR_OK)
//		{
//			PRINTF("Failed to invoke publishing of a message on the tcpip_thread: %d.\r\n", err);
//		}
//	}
//
//	/* Read the RTC seconds register to get current time in seconds */
//	currSeconds = RTC_GetSecondsTimerCount(RTC);
//
//	/* Add alarm seconds to current time */
//	currSeconds += 10;
//
//	/* Set alarm time in seconds */
//	RTC_SetSecondsTimerMatch(RTC, currSeconds);

//	vTaskSuspend(NULL);
}

/*!
 * @brief Application thread.
 */
static void app_thread(void *arg)
{
    struct netif *netif = (struct netif *)arg;
    err_t err;
    int i;
    rtc_datetime_t date;

    PRINTF("\r\nIPv4 Address     : %s\r\n", ipaddr_ntoa(&netif->ip_addr));
    PRINTF("IPv4 Subnet mask : %s\r\n", ipaddr_ntoa(&netif->netmask));
    PRINTF("IPv4 Gateway     : %s\r\n\r\n", ipaddr_ntoa(&netif->gw));

    /*
     * Check if we have an IP address or host name string configured.
     * Could just call netconn_gethostbyname() on both IP address or host name,
     * but we want to print some info if goint to resolve it.
     */
    if (ipaddr_aton(EXAMPLE_MQTT_SERVER_HOST, &mqtt_addr) && IP_IS_V4(&mqtt_addr))
    {
        /* Already an IP address */
        err = ERR_OK;
    }
    else
    {
        /* Resolve MQTT broker's host name to an IP address */
        PRINTF("Resolving \"%s\"...\r\n", EXAMPLE_MQTT_SERVER_HOST);
        err = netconn_gethostbyname(EXAMPLE_MQTT_SERVER_HOST, &mqtt_addr);
    }

    if (err == ERR_OK)
    {
        /* Start connecting to MQTT broker from tcpip_thread */
        err = tcpip_callback(connect_to_mqtt, NULL);
        if (err != ERR_OK)
        {
            PRINTF("Failed to invoke broker connection on the tcpip_thread: %d.\r\n", err);
        }
    }
    else
    {
        PRINTF("Failed to obtain IP address: %d.\r\n", err);
    }


//    while(connected == false)
//    {
//    	sys_msleep(1000U);
//    }
//
//    if (connected)
//    {


		/* Publish some messages */
	    for (i = 0; i < 5;)
	    {
	        if (connected)
	        {
//	            err = tcpip_callback(publish_message, NULL);
//	            if (err != ERR_OK)
//	            {
//	                PRINTF("Failed to invoke publishing of a message on the tcpip_thread: %d.\r\n", err);
//	            }
	            i++;
	        }

	        sys_msleep(1000U);
	    }


	/* Init RTC */
	RTC_Init(RTC);

	/* Set a start date time and start RT */
	date.year   = 2025U;
	date.month  = 3U;
	date.day    = 10U;
	date.hour   = 19U;
	date.minute = 0;
	date.second = 0;

	RTC_EnableTimer(RTC, false);

	RTC_SetDatetime(RTC, &date);

//	EnableIRQWithPriority(RTC_IRQn, 2);
	EnableIRQ(RTC_IRQn);

	RTC_EnableTimer(RTC, true);

//		xHandleP = sys_thread_new("publish_msgs", publish_msgs, NULL, PUBLISH_THREAD_STACKSIZE, PUBLISH_THREAD_PRIO);

		if (sys_thread_new("publish_msgs", publish_msgs, NULL, PUBLISH_THREAD_STACKSIZE, PUBLISH_THREAD_PRIO) == NULL)
		{
			LWIP_ASSERT("main(): Task creation failed.", 0);
		}
//    }

    vTaskDelete(NULL);
}

static void generate_client_id(void)
{
    uint8_t silicon_id[SILICONID_MAX_LENGTH];
    const char *hex = "0123456789abcdef";
    status_t status;
    uint32_t id_len = sizeof(silicon_id);
    int idx         = 0;
    int i;
    bool id_is_zero = true;

    /* Get unique ID of SoC */
    status = SILICONID_GetID(&silicon_id[0], &id_len);
    assert(status == kStatus_Success);
    assert(id_len > 0U);
    (void)status;

    /* Covert unique ID to client ID string in form: nxp_hex-unique-id */

    /* Check if client_id can accomodate prefix, id and terminator */
    assert(sizeof(client_id) >= (5U + (2U * id_len)));

    /* Fill in prefix */
    client_id[idx++] = 'n';
    client_id[idx++] = 'x';
    client_id[idx++] = 'p';
    client_id[idx++] = '_';

    /* Append unique ID */
    for (i = (int)id_len - 1; i >= 0; i--)
    {
        uint8_t value    = silicon_id[i];
        client_id[idx++] = hex[value >> 4];
        client_id[idx++] = hex[value & 0xFU];

        if (value != 0)
        {
            id_is_zero = false;
        }
    }

    /* Terminate string */
    client_id[idx] = '\0';

    if (id_is_zero)
    {
        PRINTF(
            "WARNING: MQTT client id is zero. (%s)"
#ifdef OCOTP
            " This might be caused by blank OTP memory."
#endif
            "\r\n",
            client_id);
    }
}

/*!
 * @brief Create and run example thread
 *
 * @param netif  netif which example should use
 */
void mqtt_freertos_run_thread(struct netif *netif)
{
    LOCK_TCPIP_CORE();
    mqtt_client = mqtt_client_new();
    UNLOCK_TCPIP_CORE();
    if (mqtt_client == NULL)
    {
        PRINTF("mqtt_client_new() failed.\r\n");
        while (1)
        {
        }
    }

    generate_client_id();

    if (sys_thread_new("app_task", app_thread, netif, APP_THREAD_STACKSIZE, APP_THREAD_PRIO) == NULL)
    {
        LWIP_ASSERT("mqtt_freertos_start_thread(): Task creation failed.", 0);
    }
}
