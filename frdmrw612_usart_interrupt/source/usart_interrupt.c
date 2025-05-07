/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "board.h"
#include "app.h"
#include "fsl_usart.h"
#include "LIN.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

uint8_t g_tipString[] =
    "HELLO";

/*******************************************************************************
 * Code
 ******************************************************************************/


/*!
 * @brief Main function
 */
int main(void)
{
    usart_config_t config;

    BOARD_InitHardware();

    LIN_vInit();

    LIN_vSendHeader();

//    USART_WriteByte(USART_BASE, 0x55);

    LIN_vTxMsg(g_tipString, (sizeof(g_tipString) / sizeof(g_tipString[0])) - 1);
}
