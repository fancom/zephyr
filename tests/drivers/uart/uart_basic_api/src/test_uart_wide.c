/*
 * Copyright (c) 2023 Jeroen van Dooren
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "test_uart.h"

#if DT_NODE_EXISTS(DT_NODELABEL(dut))
#define UART_NODE DT_NODELABEL(dut)
#else
#define UART_NODE DT_CHOSEN(zephyr_console)
#endif

const struct uart_config uart_cfg_wide = {
		.baudrate = 115200,
		.parity = UART_CFG_PARITY_NONE,
		.stop_bits = UART_CFG_STOP_BITS_1,
		.data_bits = UART_CFG_DATA_BITS_9,
		.flow_ctrl = UART_CFG_FLOW_CTRL_NONE
	};

static int test_configure_wide(void)
{
	const struct device *const uart_dev = DEVICE_DT_GET(UART_NODE);

	if (!device_is_ready(uart_dev)) {
		TC_PRINT("UART device not ready\n");
		return TC_FAIL;
	}

	/* Verify configure() - set device configuration using data in cfg */
	int ret = uart_configure(uart_dev, &uart_cfg_wide);

	if (ret == -ENOSYS) {
		return TC_SKIP;
	}

	/* 0 if successful, - error code otherwise */
	return (ret == 0) ? TC_PASS : TC_FAIL;

}

/* test UART configure get (retrieve configuration) */
static int test_config_get_wide(void)
{
	struct uart_config uart_cfg_check;
	const struct device *const uart_dev = DEVICE_DT_GET(UART_NODE);

	if (!device_is_ready(uart_dev)) {
		TC_PRINT("UART device not ready\n");
		return TC_FAIL;
	}

	TC_PRINT("This is a configure_get_wide test.\n");

	/* Verify configure() - set device configuration using data in cfg */
	/* 0 if successful, - error code otherwise */
	int ret = uart_configure(uart_dev, &uart_cfg_wide);

	if (ret == -ENOSYS) {
		return TC_SKIP;
	}

	zassert_true(ret == 0, "set config error");

	/* Verify config_get() - get device configuration, put in cfg */
	/* 0 if successful, - error code otherwise */
	/* so get the configurations from the device and check */
	ret = uart_config_get(uart_dev, &uart_cfg_check);
	zassert_true(ret == 0, "get config error");

	/* Confirm the values from device are the values put in cfg*/
	if (memcmp(&uart_cfg_wide, &uart_cfg_check, sizeof(uart_cfg_wide)) != 0) {
		return TC_FAIL;
	} else {
		return TC_PASS;
	}
}

#if CONFIG_SHELL
void test_uart_configure_wide(void)
#else
ZTEST(uart_basic_api, test_uart_configure_wide)
#endif
{
	int ret = test_configure_wide();

	zassert_true((ret == TC_PASS) || (ret == TC_SKIP));
}

#if CONFIG_SHELL
void test_uart_config_get_wide(void)
#else
ZTEST(uart_basic_api, test_uart_config_get_wide)
#endif
{
	int ret = test_config_get_wide();

	zassert_true((ret == TC_PASS) || (ret == TC_SKIP));
}
