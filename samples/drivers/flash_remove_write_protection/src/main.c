/*
 * Copyright (c) 2023 Nobleo Technology
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdlib.h>
#include <string.h>

#include <logging/log.h>
#include <drivers/flash.h>
#include <device.h>

#include <storage/flash_map.h>

LOG_MODULE_REGISTER(app);

void main(void)
{
	static const struct device *flash_device;
	int ret = 0;

	/* Get device */
	flash_device = device_get_binding(DT_CHOSEN_ZEPHYR_FLASH_CONTROLLER_LABEL);
	if (flash_device) {
		LOG_INF("Found flash controller %s.\n",
			DT_CHOSEN_ZEPHYR_FLASH_CONTROLLER_LABEL);
	} else {
		LOG_ERR("**No flash controller found!**\n");
		return;
	}

	/* Setup test data */
	const size_t size = DT_PROP(DT_NODELABEL(external_flash), size) / 8;
	LOG_INF("Flash is: %d bytes", size);
	
	size_t i;
	const size_t chunk_size = 4096;
	const size_t nr_chunks = size / chunk_size;

	uint8_t data [chunk_size];
	for (size_t i = 0; i < chunk_size; i++){
		data[i] = i % (UINT8_MAX + 1);
	}
	uint8_t check [chunk_size];
	memset(check, 0, sizeof(chunk_size));


	LOG_INF("Attempting erasing %d chucks of size %d bytes...", nr_chunks, chunk_size);
	for (i = 0; i < nr_chunks; i++){
		if (i % (nr_chunks / 10) == 0){
			LOG_INF("\t Progress %d%%", (i + 1) * 100 / nr_chunks);
		}
		ret = flash_erase(flash_device, i * chunk_size, sizeof(data));
		if (ret != 0) {
			break;
		}
	}
	if (ret != 0) {
		LOG_ERR("Failure to erase flash at chunk %u. Error %d", i, ret);
	} else {
		LOG_INF("Erase success");
	}

	LOG_INF("Attempting writing %d chucks of size %d bytes...", nr_chunks, chunk_size);
	for (i = 0; i < nr_chunks; i++){
		if (i % (nr_chunks / 10) == 0){
			LOG_INF("\t Progress %d%%", (i + 1) * 100 / nr_chunks);
		}
		ret = flash_write(flash_device, i * chunk_size, data, sizeof(data));
		if (ret != 0) {
			break;
		}
	}
	if (ret != 0) {
		LOG_ERR("Failure to write to flash at chunk %u. Error %d", i, ret);
	} else {
		LOG_INF("Write success");
	}

	LOG_INF("Attempting verifying %d chucks of size %d bytes...", nr_chunks, chunk_size);
	for (i = 0; i < nr_chunks; i++){
		if (i % (nr_chunks / 10) == 0){
			LOG_INF("\t Progress %d%%", (i + 1) * 100 / nr_chunks);
		}	
		ret = flash_read(flash_device, i * chunk_size, check, sizeof(check));
		if (ret != 0) {
			LOG_ERR("Failure to read from flash (chunk %u)! Error %d. Aborting.", i, ret);
			return;
		}

		if (memcmp(data, check, sizeof(data)) != 0) { 
			LOG_ERR("Verification FAILED at chunk %d! Aborting.", i);
			return;
		}
	}

	LOG_INF("Verification success! This flash should be safe to use. Done.");
	k_sleep(K_FOREVER);
}
