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
#include "stdlib.h"
#include "time.h"

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
#define EXAMPLE_MQTT_SERVER_HOST /*"test.mosquitto.org"*/ "broker.hivemq.com"
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

#define SAMPLE_FREQ	  1

#define APP_BOARD_TEST_LED_PORT BOARD_LED_BLUE_GPIO_PORT
#define APP_BOARD_TEST_LED_PIN  BOARD_LED_BLUE_GPIO_PIN
#define APP_BOARD_TEST_LED_GREEN_PIN  12U
#define APP_BOARD_TEST_LED_RED_PIN  1U


typedef enum {
	Brake = 0,
	Accelerator,
	RPM,
	Veloc,
	Temp,
	TotalMsgs
}enPublishSMsgs;

typedef enum {
	Air = 0,
	Direction,
	Lights,
	Windows,
	TotalRxMsgs
}enReceiveSMsgs;


typedef struct {
	uint8_t u8IndexMsg;
	uint8_t MsgPayLoad[5];
}stMsgStruct;

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
static uint8_t TopicIndex;
static uint8_t RxMsgs[TotalRxMsgs][35];

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
    if (RTC_GetStatusFlags(RTC) & kRTC_AlarmFlag)
    {
    	Flag = true;

        /* Clear alarm flag */
        RTC_ClearStatusFlags(RTC, kRTC_AlarmFlag);
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

static uint8_t GetTopicIndex(const char *topic)
{
	uint8_t Index = (uint8_t)TotalRxMsgs;

	if(strcmp(topic, "HM_TeleCar/CAN/FIGO/Air") == 0)
	{
		Index = (uint8_t)Air;
	}
	else if(strcmp(topic, "HM_TeleCar/CAN/FIGO/Direccion") == 0)
	{
		Index = (uint8_t)Direction;
	}
	else if(strcmp(topic, "HM_TeleCar/CAN/FIGO/Luces") == 0)
	{
		Index = (uint8_t)Lights;
	}
	else if(strcmp(topic, "HM_TeleCar/CAN/FIGO/Ventanas") == 0)
	{
		Index = (uint8_t)Windows;
	}

	return Index;
}

/*!
 * @brief Called when there is a message on a subscribed topic.
 */
static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len)
{
    LWIP_UNUSED_ARG(arg);

    PRINTF("Received %u bytes from the topic \"%s\": \"", tot_len, topic);

    TopicIndex = GetTopicIndex(topic);
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

    if(TopicIndex < TotalRxMsgs)
    {
        for(i = 0; i < 35; i++)
        {
        	RxMsgs[TopicIndex][i] = 0;
        }

    	for(i = 0; i < len; i++)
        {
        	RxMsgs[TopicIndex][i] = data[i];
        }
    }
}

/*!
 * @brief Subscribe to MQTT topics.
 */
static void mqtt_subscribe_topics(mqtt_client_t *client)
{
    static const char *topics[] = {"HM_TeleCar/CAN/FIGO/Air", "HM_TeleCar/CAN/FIGO/Direccion", "HM_TeleCar/CAN/FIGO/Luces", "HM_TeleCar/CAN/FIGO/Ventanas"};
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
#ifdef TEST_APP
        PRINTF("Published to the topic \"%s\".\r\n", topic);
#endif
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
	static const char *topics[] = {"HM_TeleCar/CAN/FIGO/Frenos", "HM_TeleCar/CAN/FIGO/Acelerador", "HM_TeleCar/CAN/FIGO/Rpm", "HM_TeleCar/CAN/FIGO/Velocidad", "HM_TeleCar/CAN/FIGO/Temperatura"};

	stMsgStruct* pMsg;
    uint8_t msg[5];

    pMsg = (stMsgStruct*)ctx;

    for(uint8_t u8i = 0; u8i < 5; u8i++)
    {
    	msg[u8i] = pMsg->MsgPayLoad[u8i];
    }

	int i;
#ifdef TEST_APP
    PRINTF("Going to publish to the topic \"%s\"...\r\n", topics[pMsg->u8IndexMsg]);
#endif

    mqtt_publish(mqtt_client, topics[pMsg->u8IndexMsg], msg, strlen(msg), 1, 0, mqtt_message_published_cb, (void *)topics[pMsg->u8IndexMsg]);
}

static void CalcValues(uint16_t* pSensorsVal)
{
	uint8_t u8Accelerator = 0U;
	uint8_t u8Aux = 0U;
	uint8_t u8Brake = 0U;
	uint16_t u16RPM = 0U;
	static uint8_t u8Vel = 0U;
	static uint8_t u8Temp = 0U;

	u8Aux = rand() % 10;

	if((u8Vel > 100)&&(u8Vel <= 200))
	{
		u8Accelerator = (u8Aux >= 8 ? 1U : 0U);
	}
	else
	{
		u8Accelerator = (u8Aux >= 7 ? 0U : 1U);
	}

	if(u8Accelerator)
	{
		if(u8Vel < 200)
		{
			u8Vel = u8Vel + ((rand() % 9) + 1);

			u8Vel = (u8Vel >= 200 ? 200 : u8Vel);
		}

		if(u8Temp < 50)
		{
			u8Temp = u8Temp + ((rand() % 3) + 1);

			u8Temp = (u8Temp >= 50 ? 50 : u8Temp);
		}
	}
	else
	{
		if(u8Vel > 0)
		{
			u8Vel = u8Vel - ((rand() % 9) + 1);

			u8Vel = (u8Vel > 200 ? 0 : u8Vel);
		}

		if(u8Temp > 0)
		{
			u8Temp = u8Temp - ((rand() % 3) + 1);

			u8Temp = (u8Temp > 50 ? 0 : u8Temp);
		}
	}

	u16RPM = ((uint16_t)u8Vel * 30);
	u8Brake = !(u8Accelerator);

	pSensorsVal[Accelerator] = (uint16_t)u8Accelerator;
	pSensorsVal[Brake] = (uint16_t)u8Brake;
	pSensorsVal[RPM] = u16RPM;
	pSensorsVal[Veloc] = (uint16_t)u8Vel;
	pSensorsVal[Temp] = (uint16_t)u8Temp;
}

static void ConvertToString(uint16_t u16DecimalValue, uint8_t* StringMsg)
{
	uint16_t u16Aux = 0U;
	uint8_t u8Counter = 0U;
	uint8_t u8Index = 0U;
	uint8_t u8Dig = 0U;

	u16Aux = u16DecimalValue;

	if(u16Aux == 0)
	{
		u8Counter = 1;
	}
	else
	{
		while(u16Aux > 0)
		{
			u16Aux = u16Aux/10;
			u8Counter++;
		}
	}

	StringMsg[u8Counter] = '\0';
	u16Aux = u16DecimalValue;
	u8Index = u8Counter - 1;

	while((u8Index >= 0)&&(u8Index < u8Counter))
	{
		u8Dig = (uint8_t)u16Aux%10;
		u16Aux = u16Aux/10;
		StringMsg[u8Index] = u8Dig + '0';
		u8Index--;
	}
}

static void SetControl(void)
{
	if(strcmp(RxMsgs[Lights], "Encender luces altas") == 0)
	{
		GPIO_PinWrite(GPIO, APP_BOARD_TEST_LED_PORT, APP_BOARD_TEST_LED_PIN, 0U);
		GPIO_PinWrite(GPIO, APP_BOARD_TEST_LED_PORT, APP_BOARD_TEST_LED_GREEN_PIN, 1U);
		GPIO_PinWrite(GPIO, APP_BOARD_TEST_LED_PORT, APP_BOARD_TEST_LED_RED_PIN, 1U);
	}
	else if(strcmp(RxMsgs[Lights], "Encender luces bajas") == 0)
	{
		GPIO_PinWrite(GPIO, APP_BOARD_TEST_LED_PORT, APP_BOARD_TEST_LED_PIN, 1U);
		GPIO_PinWrite(GPIO, APP_BOARD_TEST_LED_PORT, APP_BOARD_TEST_LED_GREEN_PIN, 0U);
		GPIO_PinWrite(GPIO, APP_BOARD_TEST_LED_PORT, APP_BOARD_TEST_LED_RED_PIN, 0U);
	}
	else if(strcmp(RxMsgs[Lights], "Apagar luces") == 0)
	{
		GPIO_PinWrite(GPIO, APP_BOARD_TEST_LED_PORT, APP_BOARD_TEST_LED_PIN, 1U);
		GPIO_PinWrite(GPIO, APP_BOARD_TEST_LED_PORT, APP_BOARD_TEST_LED_GREEN_PIN, 1U);
		GPIO_PinWrite(GPIO, APP_BOARD_TEST_LED_PORT, APP_BOARD_TEST_LED_RED_PIN, 1U);
	}
}

static void publish_msgs(void *arg)
{
	uint16_t u16MsgsValues[TotalMsgs];
	stMsgStruct Msgs[TotalMsgs];

	err_t err;
    uint32_t currSeconds;

    LWIP_UNUSED_ARG(arg);

    srand(time(0));

	/* Read the RTC seconds register to get current time in seconds */
	currSeconds = RTC_GetSecondsTimerCount(RTC);

	/* Add alarm seconds to current time */
	currSeconds += SAMPLE_FREQ;

	/* Set alarm time in seconds */
	RTC_SetSecondsTimerMatch(RTC, currSeconds);

    for(;;)
    {
    	if(Flag == true)
    	{
    		CalcValues((uint16_t*)u16MsgsValues);

    		for(uint8_t u8i = 0U; u8i < TotalMsgs; u8i++)
    		{
    			ConvertToString(u16MsgsValues[u8i], (uint8_t*)&Msgs[u8i].MsgPayLoad);
    			Msgs[u8i].u8IndexMsg = u8i;
    		}

    		if (connected)
    		{
    			for(uint8_t u8Index = 0U; u8Index < TotalMsgs; u8Index++)
    			{
    				err = tcpip_callback(publish_message, (void*)&Msgs[u8Index]);

					if (err != ERR_OK)
					{
						PRINTF("Failed to invoke publishing of a message on the tcpip_thread: %d.\r\n", err);
					}
#ifdef TEST_APP
					sys_msleep(500U);
#endif
    			}
    		}

    		/* Read the RTC seconds register to get current time in seconds */
    		currSeconds = RTC_GetSecondsTimerCount(RTC);

    		/* Add alarm seconds to current time */
    		currSeconds += SAMPLE_FREQ;

    		/* Set alarm time in seconds */
    		RTC_SetSecondsTimerMatch(RTC, currSeconds);

    		Flag = false;
    	}

    	SetControl();
    }
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

	/* Publish some messages */
	for (i = 0; i < 5;)
	{
		if (connected)
		{
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

	EnableIRQ(RTC_IRQn);

	RTC_EnableTimer(RTC, true);

	if (sys_thread_new("publish_msgs", publish_msgs, NULL, PUBLISH_THREAD_STACKSIZE, PUBLISH_THREAD_PRIO) == NULL)
	{
		LWIP_ASSERT("main(): Task creation failed.", 0);
	}

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
