/*
 * Copyright 2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_common.h"
#include "fsl_gpio.h"
#include "pin_mux.h"
#include "fsl_io_mux.h"

/* FUNCTION ************************************************************************************************************
 * 
 * Function Name : BOARD_InitBootPins
 * Description   : Calls initialization functions.
 * 
 * END ****************************************************************************************************************/
void BOARD_InitBootPins(void) {
    BOARD_InitPins();
}

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitPins(void) {                                /*!< Function assigned for the core: Cortex-M33[cm33] */

	/* Enables the clock for the GPIO0 module */
	GPIO_PortInit(GPIO, 0);

	gpio_pin_config_t gpio0_pinD12_config = {
		.pinDirection = kGPIO_DigitalOutput,
		.outputLogic = 0U
	};
	/* Initialize GPIO functionality on pin PIO0_0 (pin D12)  */
	GPIO_PinInit(GPIO, 0U, 0U, &gpio0_pinD12_config);
	GPIO_PinInit(GPIO, 0U, 12U, &gpio0_pinD12_config);
	GPIO_PinInit(GPIO, 0U, 1U, &gpio0_pinD12_config);

   IO_MUX_SetPinMux(IO_MUX_FC3_USART_DATA);
   IO_MUX_SetPinMux(IO_MUX_ENET_CLK);
   IO_MUX_SetPinMux(IO_MUX_ENET_RX);
   IO_MUX_SetPinMux(IO_MUX_ENET_TX);
   IO_MUX_SetPinMux(IO_MUX_ENET_MDIO);
   IO_MUX_SetPinMux(IO_MUX_GPIO21);
   IO_MUX_SetPinMux(IO_MUX_GPIO55);
   /* Initialize GPIO0 functionality on pin GPIO_0 (pin D12) */
   IO_MUX_SetPinMux(IO_MUX_GPIO0);
   IO_MUX_SetPinMux(IO_MUX_GPIO1);
   IO_MUX_SetPinMux(IO_MUX_GPIO12);
}

/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
