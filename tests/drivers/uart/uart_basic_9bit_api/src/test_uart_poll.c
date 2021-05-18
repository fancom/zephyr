/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "test_uart.h"

static const short poll_data[] = {
	0x054, 0x068, 0x069, 0x073, 0x020, 0x069, 0x073, 0x020,
	0x061, 0x020, 0x046, 0x049, 0x046, 0x04f, 0x020, 0x074,
	0x065, 0x073, 0x074, 0x02e, 0x00D, 0x00A, 0x155, 0x1ff
};

#define DATA_SIZE	(sizeof(poll_data)/sizeof(uint16_t))

static int test_poll_in(void)
{
	unsigned short recv_data;
	const struct device *uart_dev = device_get_binding(UART_DEVICE_NAME);

	if (!uart_dev) {
		TC_PRINT("Cannot get UART device\n");
		return TC_FAIL;
	}

	TC_PRINT("Please send characters to serial console\n");

	/* Verify uart_poll_in9() */
	while (1) {
		while (uart_poll_in9(uart_dev, &recv_data) < 0) {
		}

		TC_PRINT("0x%03X", recv_data);

		if (recv_data == 0x1ff) {
			break;
		}
	}

	return TC_PASS;
}

static int test_poll_out(void)
{
	int i;
	const struct device *uart_dev = device_get_binding(UART_DEVICE_NAME);

	if (!uart_dev) {
		TC_PRINT("Cannot get UART device\n");
		return TC_FAIL;
	}

	/* Verify uart_poll_out() */
	for (i = 0; i < DATA_SIZE; i++) {
		uart_poll_out9(uart_dev, poll_data[i]);
	}

	return TC_PASS;
}

void test_uart_poll_out(void)
{
	zassert_true(test_poll_out() == TC_PASS, NULL);
}

void test_uart_poll_in(void)
{
	zassert_true(test_poll_in() == TC_PASS, NULL);
}
