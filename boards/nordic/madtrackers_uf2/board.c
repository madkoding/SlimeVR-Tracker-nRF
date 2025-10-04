/*
 * Copyright (c) 2018 Nordic Semiconductor ASA.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <hal/nrf_gpio.h>
#include <zephyr/devicetree.h>
#include <zephyr/init.h>

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

#define VCC_GPIO_PIN DT_GPIO_PIN(ZEPHYR_USER_NODE, vcc_gpios)
#define VCC_GPIO_PORT DT_GPIO_LABEL(ZEPHYR_USER_NODE, vcc_gpios)

#define VCC_GPIO_PORT_NUM DT_PROP(DT_GPIO_CTLR(ZEPHYR_USER_NODE, vcc_gpios), port)

static int board_madtrackers_init(void) {
	/* using vcc-gpios for sensor power defined in madtrackers_uf2.dts
	 */
	nrf_gpio_cfg(
		NRF_GPIO_PIN_MAP(VCC_GPIO_PORT_NUM, VCC_GPIO_PIN),
		NRF_GPIO_PIN_DIR_OUTPUT,
		NRF_GPIO_PIN_INPUT_DISCONNECT,
		NRF_GPIO_PIN_NOPULL,
		NRF_GPIO_PIN_D0H1,
		NRF_GPIO_PIN_NOSENSE
	);
	nrf_gpio_pin_set(NRF_GPIO_PIN_MAP(VCC_GPIO_PORT_NUM, VCC_GPIO_PIN));
	/* pull up on PO.13 for external 3V3 regulator
	 */
	nrf_gpio_cfg(
		NRF_GPIO_PIN_MAP(0, 13),
		NRF_GPIO_PIN_DIR_INPUT,
		NRF_GPIO_PIN_INPUT_DISCONNECT,
		NRF_GPIO_PIN_PULLUP,
		NRF_GPIO_PIN_S0S1,
		NRF_GPIO_PIN_NOSENSE
	);
	return 0;
}

SYS_INIT(board_madtrackers_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);