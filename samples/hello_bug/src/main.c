/* Networking DHCPv4 client */

/*
 * Copyright (c) 2017 ARM Ltd.
 * Copyright (c) 2016 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(net_dhcpv4_mdns_bug, LOG_LEVEL_DBG);

#include <zephyr/kernel.h>
#include <zephyr/linker/sections.h>
#include <errno.h>
#include <stdio.h>

#include <zephyr/net/net_if.h>
#include <zephyr/net/net_core.h>
#include <zephyr/net/net_context.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/net_config.h>
#include <zephyr/drivers/gpio.h>

static struct net_mgmt_event_callback mgmt_cb;

K_SEM_DEFINE(my_sem, 0, 1);

static void handler(struct net_mgmt_event_callback *cb,
		    uint32_t mgmt_event,
		    struct net_if *iface)
{
	int i = 0;

	if (mgmt_event != NET_EVENT_IPV4_ADDR_ADD) {
		return;
	}

	for (i = 0; i < NET_IF_MAX_IPV4_ADDR; i++) {
		char buf[NET_IPV4_ADDR_LEN];

		if (iface->config.ip.ipv4->unicast[i].addr_type !=
							NET_ADDR_DHCP) {
			continue;
		}

		LOG_INF("Your address: %s",
			net_addr_ntop(AF_INET,
			    &iface->config.ip.ipv4->unicast[i].address.in_addr,
						  buf, sizeof(buf)));
		LOG_INF("Lease time: %u seconds",
			 iface->config.dhcpv4.lease_time);
		LOG_INF("Subnet: %s",
			net_addr_ntop(AF_INET,
				       &iface->config.ip.ipv4->netmask,
				       buf, sizeof(buf)));
		LOG_INF("Router: %s",
			net_addr_ntop(AF_INET,
						 &iface->config.ip.ipv4->gw,
						 buf, sizeof(buf)));
		k_sem_give(&my_sem);
	}
}

void main(void)
{
	struct net_if *iface;
	int ret;

	LOG_INF("Run hello-bug");

	/*gpio_pin_configure(
		DEVICE_DT_GET_OR_NULL(DT_GPIO_CTLR(DT_NODELABEL(ethsw_reset), gpios)),
		DT_GPIO_PIN(DT_NODELABEL(ethsw_reset), gpios),
            	GPIO_OUTPUT_INACTIVE | DT_GPIO_FLAGS(DT_NODELABEL(ethsw_reset), gpios));*/

	net_mgmt_init_event_callback(&mgmt_cb, handler,
				     NET_EVENT_IPV4_ADDR_ADD | NET_EVENT_IPV4_MCAST_JOIN);
	net_mgmt_add_event_callback(&mgmt_cb);

	iface = net_if_get_default();

	LOG_INF("Bringing the interface down");
	ret = net_if_down(iface);
        if (ret) {
		LOG_ERR("Failed to bring network interface down: %d", ret);
            	return;
        }

	LOG_INF("Bringing the interface up");
        ret = net_if_up(iface);
        if (ret) {
            LOG_ERR("Failed to bring network interface up again: %d", ret);
            return;
        }

	LOG_INF("Configuring the interface");
	ret = net_config_init_by_iface(iface, "hello-bug-if", NET_CONFIG_NEED_IPV4,
                                           CONFIG_NET_CONFIG_INIT_TIMEOUT * MSEC_PER_SEC);
        if (ret) {
            LOG_ERR("Failed to init network: %d", ret);
            return;
        }

	LOG_INF("Waiting for dhcpv4 assigned IP address");
	if (k_sem_take(&my_sem, K_MINUTES(1)) != 0) {
		LOG_ERR("Failed to receive a dhcpv4 assigned IP address: %d", ret);
	}
	else {
		LOG_INF("Success!");
	}
}
