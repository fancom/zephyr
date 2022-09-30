/*
 * Copyright (c) 2016 Linaro Limited.
 * Copyright (c) 2019 Song Qiang <songqiang1304521@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Common part of BDMA drivers for stm32.
 * @note  Functions named with stm32_bdma_* are SoCs related functions
 *        implemented in bdma_stm32_v*.c
 */

#include "bdma_stm32.h"

#include <init.h>
#include <drivers/clock_control.h>
#include <drivers/dma/dma_stm32.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(bdma_stm32, CONFIG_DMA_LOG_LEVEL);

#define DT_DRV_COMPAT st_stm32_bdma

#define BDMA_STM32_0_STREAM_COUNT 8

static uint32_t table_m_size[] = {
	LL_BDMA_MDATAALIGN_BYTE,
	LL_BDMA_MDATAALIGN_HALFWORD,
	LL_BDMA_MDATAALIGN_WORD,
};

static uint32_t table_p_size[] = {
	LL_BDMA_PDATAALIGN_BYTE,
	LL_BDMA_PDATAALIGN_HALFWORD,
	LL_BDMA_PDATAALIGN_WORD,
};

static uint32_t bdma_stm32_id_to_channel(uint32_t id)
{
	static const uint32_t channel_nr[] = {
		LL_BDMA_CHANNEL_0,
		LL_BDMA_CHANNEL_1,
		LL_BDMA_CHANNEL_2,
		LL_BDMA_CHANNEL_3,
		LL_BDMA_CHANNEL_4,
		LL_BDMA_CHANNEL_5,
		LL_BDMA_CHANNEL_6,
		LL_BDMA_CHANNEL_7,
	};

	__ASSERT_NO_MSG(id < ARRAY_SIZE(channel_nr));

	return channel_nr[id];
}

#if !defined(CONFIG_DMAMUX_STM32)
uint32_t bdma_stm32_slot_to_channel(uint32_t slot)
{
	static const uint32_t channel_nr[] = {
		LL_BDMA_CHANNEL_0,
		LL_BDMA_CHANNEL_1,
		LL_BDMA_CHANNEL_2,
		LL_BDMA_CHANNEL_3,
		LL_BDMA_CHANNEL_4,
		LL_BDMA_CHANNEL_5,
		LL_BDMA_CHANNEL_6,
		LL_BDMA_CHANNEL_7,
	};

	__ASSERT_NO_MSG(slot < ARRAY_SIZE(channel_nr));

	return channel_nr[slot];
}
#endif

static void bdma_stm32_dump_stream_irq(const struct device *dev, uint32_t id)
{
}

static void bdma_stm32_clear_stream_irq(const struct device *dev, uint32_t id)
{
	const struct bdma_stm32_config *config = dev->config;
	BDMA_TypeDef *bdma = (BDMA_TypeDef *)(config->base);

	bdma_stm32_clear_gi(bdma, id);
	bdma_stm32_clear_tc(bdma, id);
	bdma_stm32_clear_ht(bdma, id);
	bdma_stm32_clear_te(bdma, id);
}

static bool stm32_bdma_is_tc_irq_active(BDMA_TypeDef *dma, uint32_t id)
{
	return LL_BDMA_IsEnabledIT_TC(dma, bdma_stm32_id_to_channel(id)) &&
	       bdma_stm32_is_tc_active(dma, id);
}

static bool stm32_bdma_is_ht_irq_active(BDMA_TypeDef *dma, uint32_t id)
{
	return LL_BDMA_IsEnabledIT_HT(dma, bdma_stm32_id_to_channel(id)) &&
	       bdma_stm32_is_ht_active(dma, id);
}

static void bdma_stm32_irq_handler(const struct device *dev, uint32_t id)
{
	const struct bdma_stm32_config *config = dev->config;
	BDMA_TypeDef *dma = (BDMA_TypeDef *)(config->base);
	struct bdma_stm32_stream *stream;
	uint32_t callback_arg;

	__ASSERT_NO_MSG(id < config->max_streams);

	stream = &config->streams[id];

#ifdef CONFIG_DMAMUX_STM32
	callback_arg = stream->mux_channel;
#else
	callback_arg = id + STREAM_OFFSET;
#endif /* CONFIG_DMAMUX_STM32 */

	if (!IS_ENABLED(CONFIG_DMAMUX_STM32)) {
		stream->busy = false;
	}

	/* the dma stream id is in range from STREAM_OFFSET..<dma-requests> */
	if (stm32_bdma_is_ht_irq_active(dma, id)) {
		/* Let HAL DMA handle flags on its own */
		if (!stream->hal_override) {
			bdma_stm32_clear_ht(dma, id);
		}
		stream->bdma_callback(dev, stream->user_data, callback_arg, 0);
	} else if (stm32_bdma_is_tc_irq_active(dma, id)) {
#ifdef CONFIG_DMAMUX_STM32
		stream->busy = false;
#endif
		/* Let HAL DMA handle flags on its own */
		if (!stream->hal_override) {
			bdma_stm32_clear_tc(dma, id);
		}
		stream->bdma_callback(dev, stream->user_data, callback_arg, 0);
	} else if (stm32_dma_is_unexpected_irq_happened(dma, id)) {
		LOG_ERR("Unexpected irq happened.");
		stream->bdma_callback(dev, stream->user_data,
				     callback_arg, -EIO);
	} else {
		LOG_ERR("Transfer Error.");
		bdma_stm32_dump_stream_irq(dev, id);
		bdma_stm32_clear_stream_irq(dev, id);
		stream->bdma_callback(dev, stream->user_data,
				     callback_arg, -EIO);
	}
}

static int bdma_stm32_get_priority(uint8_t priority, uint32_t *ll_priority)
{
	switch (priority) {
	case 0x0:
		*ll_priority = LL_BDMA_PRIORITY_LOW;
		break;
	case 0x1:
		*ll_priority = LL_BDMA_PRIORITY_MEDIUM;
		break;
	case 0x2:
		*ll_priority = LL_BDMA_PRIORITY_HIGH;
		break;
	case 0x3:
		*ll_priority = LL_BDMA_PRIORITY_VERYHIGH;
		break;
	default:
		LOG_ERR("Priority error. %d", priority);
		return -EINVAL;
	}

	return 0;
}

static int bdma_stm32_get_direction(enum dma_channel_direction direction,
				   uint32_t *ll_direction)
{
	switch (direction) {
	case MEMORY_TO_MEMORY:
		*ll_direction = LL_BDMA_DIRECTION_MEMORY_TO_MEMORY;
		break;
	case MEMORY_TO_PERIPHERAL:
		*ll_direction = LL_BDMA_DIRECTION_MEMORY_TO_PERIPH;
		break;
	case PERIPHERAL_TO_MEMORY:
		*ll_direction = LL_BDMA_DIRECTION_PERIPH_TO_MEMORY;
		break;
	default:
		LOG_ERR("Direction error. %d", direction);
		return -EINVAL;
	}

	return 0;
}

static int bdma_stm32_get_memory_increment(enum dma_addr_adj increment,
					  uint32_t *ll_increment)
{
	switch (increment) {
	case DMA_ADDR_ADJ_INCREMENT:
		*ll_increment = LL_BDMA_MEMORY_INCREMENT;
		break;
	case DMA_ADDR_ADJ_NO_CHANGE:
		*ll_increment = LL_BDMA_MEMORY_NOINCREMENT;
		break;
	case DMA_ADDR_ADJ_DECREMENT:
		return -ENOTSUP;
	default:
		LOG_ERR("Memory increment error. %d", increment);
		return -EINVAL;
	}

	return 0;
}

static int bdma_stm32_get_periph_increment(enum dma_addr_adj increment,
					  uint32_t *ll_increment)
{
	switch (increment) {
	case DMA_ADDR_ADJ_INCREMENT:
		*ll_increment = LL_BDMA_PERIPH_INCREMENT;
		break;
	case DMA_ADDR_ADJ_NO_CHANGE:
		*ll_increment = LL_BDMA_PERIPH_NOINCREMENT;
		break;
	case DMA_ADDR_ADJ_DECREMENT:
		return -ENOTSUP;
	default:
		LOG_ERR("Periph increment error. %d", increment);
		return -EINVAL;
	}

	return 0;
}

bool bdma_stm32_is_gi_active(BDMA_TypeDef *BDMAx, uint32_t id)
{
	static const bdma_stm32_check_flag_func func[] = {
		LL_BDMA_IsActiveFlag_GI0,
		LL_BDMA_IsActiveFlag_GI1,
		LL_BDMA_IsActiveFlag_GI2,
		LL_BDMA_IsActiveFlag_GI3,
		LL_BDMA_IsActiveFlag_GI4,
		LL_BDMA_IsActiveFlag_GI5,
		LL_BDMA_IsActiveFlag_GI6,
		LL_BDMA_IsActiveFlag_GI7,
	};

	__ASSERT_NO_MSG(id < ARRAY_SIZE(func));

	return func[id](BDMAx);
}

void bdma_stm32_clear_gi(BDMA_TypeDef *BDMAx, uint32_t id)
{
	static const bdma_stm32_clear_flag_func func[] = {
		LL_BDMA_ClearFlag_GI0,
		LL_BDMA_ClearFlag_GI1,
		LL_BDMA_ClearFlag_GI2,
		LL_BDMA_ClearFlag_GI3,
		LL_BDMA_ClearFlag_GI4,
		LL_BDMA_ClearFlag_GI5,
		LL_BDMA_ClearFlag_GI6,
		LL_BDMA_ClearFlag_GI7,
	};

	__ASSERT_NO_MSG(id < ARRAY_SIZE(func));

	func[id](BDMAx);
}

bool bdma_stm32_is_tc_active(BDMA_TypeDef *BDMAx, uint32_t id)
{
	static const bdma_stm32_check_flag_func func[] = {
		LL_BDMA_IsActiveFlag_TC0,
		LL_BDMA_IsActiveFlag_TC1,
		LL_BDMA_IsActiveFlag_TC2,
		LL_BDMA_IsActiveFlag_TC3,
		LL_BDMA_IsActiveFlag_TC4,
		LL_BDMA_IsActiveFlag_TC5,
		LL_BDMA_IsActiveFlag_TC6,
		LL_BDMA_IsActiveFlag_TC7,
	};

	__ASSERT_NO_MSG(id < ARRAY_SIZE(func));

	return func[id](BDMAx);
}

void bdma_stm32_clear_tc(BDMA_TypeDef *BDMAx, uint32_t id)
{
	static const bdma_stm32_clear_flag_func func[] = {
		LL_BDMA_ClearFlag_TC0,
		LL_BDMA_ClearFlag_TC1,
		LL_BDMA_ClearFlag_TC2,
		LL_BDMA_ClearFlag_TC3,
		LL_BDMA_ClearFlag_TC4,
		LL_BDMA_ClearFlag_TC5,
		LL_BDMA_ClearFlag_TC6,
		LL_BDMA_ClearFlag_TC7,
	};

	__ASSERT_NO_MSG(id < ARRAY_SIZE(func));

	func[id](BDMAx);
}

bool bdma_stm32_is_ht_active(BDMA_TypeDef *BDMAx, uint32_t id)
{
	static const bdma_stm32_check_flag_func func[] = {
		LL_BDMA_IsActiveFlag_HT0,
		LL_BDMA_IsActiveFlag_HT1,
		LL_BDMA_IsActiveFlag_HT2,
		LL_BDMA_IsActiveFlag_HT3,
		LL_BDMA_IsActiveFlag_HT4,
		LL_BDMA_IsActiveFlag_HT5,
		LL_BDMA_IsActiveFlag_HT6,
		LL_BDMA_IsActiveFlag_HT7,
	};

	__ASSERT_NO_MSG(id < ARRAY_SIZE(func));

	return func[id](BDMAx);
}

void bdma_stm32_clear_ht(BDMA_TypeDef *BDMAx, uint32_t id)
{
	static const bdma_stm32_clear_flag_func func[] = {
		LL_BDMA_ClearFlag_HT0,
		LL_BDMA_ClearFlag_HT1,
		LL_BDMA_ClearFlag_HT2,
		LL_BDMA_ClearFlag_HT3,
		LL_BDMA_ClearFlag_HT4,
		LL_BDMA_ClearFlag_HT5,
		LL_BDMA_ClearFlag_HT6,
		LL_BDMA_ClearFlag_HT7,
	};

	__ASSERT_NO_MSG(id < ARRAY_SIZE(func));

	func[id](BDMAx);
}

bool bdma_stm32_is_te_active(BDMA_TypeDef *BDMAx, uint32_t id)
{
	static const bdma_stm32_check_flag_func func[] = {
		LL_BDMA_IsActiveFlag_TE0,
		LL_BDMA_IsActiveFlag_TE1,
		LL_BDMA_IsActiveFlag_TE2,
		LL_BDMA_IsActiveFlag_TE3,
		LL_BDMA_IsActiveFlag_TE4,
		LL_BDMA_IsActiveFlag_TE5,
		LL_BDMA_IsActiveFlag_TE6,
		LL_BDMA_IsActiveFlag_TE7,
	};

	__ASSERT_NO_MSG(id < ARRAY_SIZE(func));

	return func[id](BDMAx);
}

void bdma_stm32_clear_te(BDMA_TypeDef *BDMAx, uint32_t id)
{
	static const bdma_stm32_clear_flag_func func[] = {
		LL_BDMA_ClearFlag_TE0,
		LL_BDMA_ClearFlag_TE1,
		LL_BDMA_ClearFlag_TE2,
		LL_BDMA_ClearFlag_TE3,
		LL_BDMA_ClearFlag_TE4,
		LL_BDMA_ClearFlag_TE5,
		LL_BDMA_ClearFlag_TE6,
		LL_BDMA_ClearFlag_TE7,
	};

	__ASSERT_NO_MSG(id < ARRAY_SIZE(func));

	func[id](BDMAx);
}

static bool stm32_bdma_is_te_irq_active(BDMA_TypeDef *dma, uint32_t id)
{
	return LL_BDMA_IsEnabledIT_TE(dma, bdma_stm32_id_to_channel(id)) &&
	       bdma_stm32_is_te_active(dma, id);
}

bool stm32_bdma_is_irq_active(BDMA_TypeDef *bdma, uint32_t id)
{
	return stm32_bdma_is_tc_irq_active(bdma, id) ||
	       stm32_bdma_is_ht_irq_active(bdma, id) ||
	       stm32_bdma_is_te_irq_active(bdma, id);
}

static void stm32_bdma_enable_channel(BDMA_TypeDef *bdma, uint32_t id)
{
	LL_BDMA_EnableChannel(bdma, bdma_stm32_id_to_channel(id));
}

int stm32_bdma_disable_stream(BDMA_TypeDef *bdma, uint32_t id)
{
	LL_BDMA_DisableChannel(bdma, bdma_stm32_id_to_channel(id));

	if (!LL_BDMA_IsEnabledChannel(bdma, bdma_stm32_id_to_channel(id))) {
		return 0;
	}

	return -EAGAIN;
}

static int bdma_stm32_disable_stream(BDMA_TypeDef *bdma, uint32_t id)
{
	int count = 0;

	for (;;) {
		if (stm32_bdma_disable_stream(bdma, id) == 0) {
			return 0;
		}
		/* After trying for 5 seconds, give up */
		if (count++ > (5 * 1000)) {
			return -EBUSY;
		}
		k_sleep(K_MSEC(1));
	}

	return 0;
}

BDMA_STM32_EXPORT_API int bdma_stm32_configure(const struct device *dev,
					     uint32_t id,
					     struct dma_config *config)
{
	const struct bdma_stm32_config *dev_config = dev->config;
	struct bdma_stm32_stream *stream =
				&dev_config->streams[id];
	BDMA_TypeDef *bdma = (BDMA_TypeDef *)dev_config->base;
	LL_BDMA_InitTypeDef BDMA_InitStruct;
	int ret;

	//printf("bdma_stm32_configure id: %d\n", id);

	/* Check potential BDMA override */
	if (config->linked_channel == STM32_DMA_HAL_OVERRIDE) {
		/* BDMA channel is overridden by HAL BDMA
		 * Retain that the channel is busy and proceed to the minimal
		 * configuration to properly route the IRQ
		 */
		stream->busy = true;
		stream->hal_override = true;
		stream->bdma_callback = config->dma_callback;
		stream->user_data = config->user_data;
		return 0;
	}

	if (id >= dev_config->max_streams) {
		LOG_ERR("cannot configure the bdma stream %d.", id);
		return -EINVAL;
	}

	if (stream->busy) {
		LOG_ERR("bdma stream %d is busy.", id);
		return -EBUSY;
	}

	if (bdma_stm32_disable_stream(bdma, id) != 0) {
		LOG_ERR("could not disable bdma stream %d.", id);
		return -EBUSY;
	}

	bdma_stm32_clear_stream_irq(dev, id);

	if (config->head_block->block_size > BDMA_STM32_MAX_DATA_ITEMS) {
		LOG_ERR("Data size too big: %d\n",
		       config->head_block->block_size);
		return -EINVAL;
	}

#ifdef CONFIG_BDMA_STM32_V1
	if ((config->channel_direction == MEMORY_TO_MEMORY) &&
		(!dev_config->support_m2m)) {
		LOG_ERR("Memcopy not supported for device %s",
			dev->name);
		return -ENOTSUP;
	}
#endif /* CONFIG_BDMA_STM32_V1 */

	/* support only the same data width for source and dest */
	if ((config->dest_data_size != config->source_data_size)) {
		LOG_ERR("source and dest data size differ.");
		return -EINVAL;
	}

	if (config->source_data_size != 4U &&
	    config->source_data_size != 2U &&
	    config->source_data_size != 1U) {
		LOG_ERR("source and dest unit size error, %d",
			config->source_data_size);
		return -EINVAL;
	}

	/*
	 * STM32's circular mode will auto reset both source address
	 * counter and destination address counter.
	 */
	if (config->head_block->source_reload_en !=
		config->head_block->dest_reload_en) {
		LOG_ERR("source_reload_en and dest_reload_en must "
			"be the same.");
		return -EINVAL;
	}

	stream->busy		= true;
	stream->bdma_callback	= config->dma_callback;
	stream->direction	= config->channel_direction;
	stream->user_data       = config->user_data;
	stream->src_size	= config->source_data_size;
	stream->dst_size	= config->dest_data_size;

	/* check dest or source memory address, warn if 0 */
	if ((config->head_block->source_address == 0)) {
		LOG_WRN("source_buffer address is null.");
	}

	if ((config->head_block->dest_address == 0)) {
		LOG_WRN("dest_buffer address is null.");
	}

	if (stream->direction == MEMORY_TO_PERIPHERAL) {
		BDMA_InitStruct.MemoryOrM2MDstAddress =
					config->head_block->source_address;
		BDMA_InitStruct.PeriphOrM2MSrcAddress =
					config->head_block->dest_address;
	} else {
		BDMA_InitStruct.PeriphOrM2MSrcAddress =
					config->head_block->source_address;
		BDMA_InitStruct.MemoryOrM2MDstAddress =
					config->head_block->dest_address;
	}

	uint16_t memory_addr_adj = 0, periph_addr_adj = 0;

	ret = bdma_stm32_get_priority(config->channel_priority,
				     &BDMA_InitStruct.Priority);
	if (ret < 0) {
		return ret;
	}

	ret = bdma_stm32_get_direction(config->channel_direction,
				      &BDMA_InitStruct.Direction);
	if (ret < 0) {
		return ret;
	}

	switch (config->channel_direction) {
	case MEMORY_TO_MEMORY:
	case PERIPHERAL_TO_MEMORY:
		memory_addr_adj = config->head_block->dest_addr_adj;
		periph_addr_adj = config->head_block->source_addr_adj;
		break;
	case MEMORY_TO_PERIPHERAL:
		memory_addr_adj = config->head_block->source_addr_adj;
		periph_addr_adj = config->head_block->dest_addr_adj;
		break;
	/* Direction has been asserted in bdma_stm32_get_direction. */
	default:
		LOG_ERR("Channel direction error (%d).",
				config->channel_direction);
		return -EINVAL;
	}

	ret = bdma_stm32_get_memory_increment(memory_addr_adj,
					&BDMA_InitStruct.MemoryOrM2MDstIncMode);
	if (ret < 0) {
		return ret;
	}
	ret = bdma_stm32_get_periph_increment(periph_addr_adj,
					&BDMA_InitStruct.PeriphOrM2MSrcIncMode);
	if (ret < 0) {
		return ret;
	}

	if (config->head_block->source_reload_en) {
		BDMA_InitStruct.Mode = LL_BDMA_MODE_CIRCULAR;
	} else {
		BDMA_InitStruct.Mode = LL_BDMA_MODE_NORMAL;
	}

	stream->source_periph = stream->direction == MEMORY_TO_PERIPHERAL;

	/* set the data width, when source_data_size equals dest_data_size */
	int index = find_lsb_set(config->source_data_size) - 1;
	BDMA_InitStruct.PeriphOrM2MSrcDataSize = table_p_size[index];
	index = find_lsb_set(config->dest_data_size) - 1;
	BDMA_InitStruct.MemoryOrM2MDstDataSize = table_m_size[index];

#if defined(CONFIG_BDMA_STM32_V1)

#if !defined(CONFIG_SOC_SERIES_STM32H7X)
	if (config->channel_direction != MEMORY_TO_MEMORY) {
		if (config->dma_slot >= 8) {
			LOG_ERR("dma slot error.");
			return -EINVAL;
		}
	} else {
		if (config->dma_slot >= 8) {
			LOG_ERR("dma slot is too big, using 0 as default.");
			config->dma_slot = 0;
		}
	}

	//printf("dma-slot: %d\n", config->dma_slot);
	BDMA_InitStruct.Channel = bdma_stm32_slot_to_channel(config->dma_slot);
#endif

	if (stm32_bdma_check_fifo_mburst(&BDMA_InitStruct)) {
		BDMA_InitStruct.FIFOMode = LL_BDMA_FIFOMODE_ENABLE;
	} else {
		BDMA_InitStruct.FIFOMode = LL_BDMA_FIFOMODE_DISABLE;
	}
#endif
	if (stream->source_periph) {
		BDMA_InitStruct.NbData = config->head_block->block_size /
					config->source_data_size;
	} else {
		BDMA_InitStruct.NbData = config->head_block->block_size /
					config->dest_data_size;
	}

#if defined(CONFIG_BDMA_STM32_V2) || defined(CONFIG_DMAMUX_STM32)
	/*
	 * the with bdma V2 and bdma mux,
	 * the request ID is stored in the dma_slot
	 */
#if !defined(CONFIG_SOC_SERIES_STM32F0X) || defined(CONFIG_SOC_STM32F030XC)
	BDMA_InitStruct.PeriphRequest = config->dma_slot;
#endif
#endif
	LL_BDMA_Init(bdma, bdma_stm32_id_to_channel(id), &BDMA_InitStruct);

	LL_BDMA_EnableIT_TC(bdma, bdma_stm32_id_to_channel(id));

	/* Enable Half-Transfer irq if circular mode is enabled */
	if (config->head_block->source_reload_en) {
		LL_BDMA_EnableIT_HT(bdma, bdma_stm32_id_to_channel(id));
	}

#if defined(CONFIG_BDMA_STM32_V1)
	if (BDMA_InitStruct.FIFOMode == LL_BDMA_FIFOMODE_ENABLE) {
		LL_BDMA_EnableFifoMode(bdma, bdma_stm32_id_to_channel(id));
		LL_BDMA_EnableIT_FE(bdma, bdma_stm32_id_to_channel(id));
	} else {
		LL_BDMA_DisableFifoMode(bdma, bdma_stm32_id_to_channel(id));
		LL_BDMA_DisableIT_FE(bdma, bdma_stm32_id_to_channel(id));
	}
#endif
	return ret;
}

BDMA_STM32_EXPORT_API int bdma_stm32_reload(const struct device *dev, uint32_t id,
					  uint32_t src, uint32_t dst,
					  size_t size)
{
	const struct bdma_stm32_config *config = dev->config;
	BDMA_TypeDef *bdma = (BDMA_TypeDef *)(config->base);
	struct bdma_stm32_stream *stream;

	if (id >= config->max_streams) {
		return -EINVAL;
	}

	stream = &config->streams[id];

	if (bdma_stm32_disable_stream(bdma, id) != 0) {
		return -EBUSY;
	}

	switch (stream->direction) {
	case MEMORY_TO_PERIPHERAL:
		LL_BDMA_SetMemoryAddress(bdma, bdma_stm32_id_to_channel(id), src);
		LL_BDMA_SetPeriphAddress(bdma, bdma_stm32_id_to_channel(id), dst);
		break;
	case MEMORY_TO_MEMORY:
	case PERIPHERAL_TO_MEMORY:
		LL_BDMA_SetPeriphAddress(bdma, bdma_stm32_id_to_channel(id), src);
		LL_BDMA_SetMemoryAddress(bdma, bdma_stm32_id_to_channel(id), dst);
		break;
	default:
		return -EINVAL;
	}

	if (stream->source_periph) {
		LL_BDMA_SetDataLength(bdma, bdma_stm32_id_to_channel(id),
				     size / stream->src_size);
	} else {
		LL_BDMA_SetDataLength(bdma, bdma_stm32_id_to_channel(id),
				     size / stream->dst_size);
	}

	stm32_bdma_enable_channel(bdma, id);

	return 0;
}

BDMA_STM32_EXPORT_API int bdma_stm32_start(const struct device *dev, uint32_t id)
{
	const struct bdma_stm32_config *config = dev->config;
	BDMA_TypeDef *bdma = (BDMA_TypeDef *)(config->base);

	/* Only M2P or M2M mode can be started manually. */
	if (id >= config->max_streams) {
		return -EINVAL;
	}

	bdma_stm32_clear_stream_irq(dev, id);

	stm32_bdma_enable_channel(bdma, id);

	return 0;
}

BDMA_STM32_EXPORT_API int bdma_stm32_stop(const struct device *dev, uint32_t id)
{
	const struct bdma_stm32_config *config = dev->config;
	struct bdma_stm32_stream *stream = &config->streams[id];
	BDMA_TypeDef *bdma = (BDMA_TypeDef *)(config->base);

	if (id >= config->max_streams) {
		return -EINVAL;
	}

	/* in bdma_stm32_configure, enabling is done regardless of defines */
	LL_BDMA_DisableIT_TC(bdma, bdma_stm32_id_to_channel(id));
	LL_BDMA_DisableIT_HT(bdma, bdma_stm32_id_to_channel(id));

#if defined(CONFIG_BDMA_STM32_V1)
	stm32_bdma_disable_fifo_irq(bdma, id);
#endif
	bdma_stm32_disable_stream(bdma, id);
	bdma_stm32_clear_stream_irq(dev, id);

	/* Finally, flag stream as free */
	stream->busy = false;

	return 0;
}

static int bdma_stm32_init(const struct device *dev)
{
	const struct bdma_stm32_config *config = dev->config;
	const struct device *clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);

	if (clock_control_on(clk,
		(clock_control_subsys_t *) &config->pclken) != 0) {
		LOG_ERR("clock op failed\n");
		return -EIO;
	}

	config->config_irq(dev);

	for (uint32_t i = 0; i < config->max_streams; i++) {
		config->streams[i].busy = false;
#ifdef CONFIG_DMAMUX_STM32
		/* each further stream->mux_channel is fixed here */
		config->streams[i].mux_channel = i + config->offset;
#endif /* CONFIG_DMAMUX_STM32 */
	}

	return 0;
}

BDMA_STM32_EXPORT_API int bdma_stm32_get_status(const struct device *dev,
				uint32_t id, struct dma_status *stat)
{
	const struct bdma_stm32_config *config = dev->config;
	BDMA_TypeDef *bdma = (BDMA_TypeDef *)(config->base);
	struct bdma_stm32_stream *stream;

	if (id >= config->max_streams) {
		return -EINVAL;
	}

	stream = &config->streams[id];
	stat->pending_length = LL_BDMA_GetDataLength(bdma, bdma_stm32_id_to_channel(id));
	stat->dir = stream->direction;
	stat->busy = stream->busy;

	return 0;
}

static const struct dma_driver_api dma_funcs = {
	.reload		 = bdma_stm32_reload,
	.config		 = bdma_stm32_configure,
	.start		 = bdma_stm32_start,
	.stop		 = bdma_stm32_stop,
	.get_status	 = bdma_stm32_get_status,
};

#ifdef CONFIG_DMAMUX_STM32
#define BDMA_STM32_OFFSET_INIT(index)			\
	.offset = DT_INST_PROP(index, dma_offset),
#else
#define BDMA_STM32_OFFSET_INIT(index)
#endif /* CONFIG_DMAMUX_STM32 */

#ifdef CONFIG_BDMA_STM32_V1
#define BDMA_STM32_MEM2MEM_INIT(index)					\
	.support_m2m = DT_INST_PROP(index, st_mem2mem),
#else
#define BDMA_STM32_MEM2MEM_INIT(index)
#endif /* CONFIG_BDMA_STM32_V1 */					\

#define BDMA_STM32_INIT_DEV(index)					\
static struct bdma_stm32_stream						\
	bdma_stm32_streams_##index[BDMA_STM32_##index##_STREAM_COUNT];	\
									\
const struct bdma_stm32_config bdma_stm32_config_##index = {		\
	.pclken = { .bus = DT_INST_CLOCKS_CELL(index, bus),		\
		    .enr = DT_INST_CLOCKS_CELL(index, bits) },		\
	.config_irq = bdma_stm32_config_irq_##index,			\
	.base = DT_INST_REG_ADDR(index),				\
	BDMA_STM32_MEM2MEM_INIT(index)					\
	.max_streams = BDMA_STM32_##index##_STREAM_COUNT,		\
	.streams = bdma_stm32_streams_##index,				\
	BDMA_STM32_OFFSET_INIT(index)					\
};									\
									\
static struct bdma_stm32_data bdma_stm32_data_##index = {			\
};									\
									\
DEVICE_DT_INST_DEFINE(index,						\
		    &bdma_stm32_init,					\
		    NULL,						\
		    &bdma_stm32_data_##index, &bdma_stm32_config_##index,	\
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,	\
		    &dma_funcs)

#ifdef CONFIG_BDMA_STM32_SHARED_IRQS

#define BDMA_STM32_DEFINE_IRQ_HANDLER(bdma, chan) /* nothing */

#define BDMA_STM32_IRQ_CONNECT(bdma, chan)				\
	do {								\
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(bdma, chan, irq),		\
			    DT_INST_IRQ_BY_IDX(bdma, chan, priority),	\
			    bdma_stm32_shared_irq_handler,		\
			    DEVICE_DT_INST_GET(bdma), 0);		\
		irq_enable(DT_INST_IRQ_BY_IDX(bdma, chan, irq));		\
	} while (0)


#else /* CONFIG_BDMA_STM32_SHARED_IRQS */

#define BDMA_STM32_DEFINE_IRQ_HANDLER(bdma, chan)				\
static void bdma_stm32_irq_##bdma##_##chan(const struct device *dev)	\
{									\
	bdma_stm32_irq_handler(dev, chan);				\
}


#define BDMA_STM32_IRQ_CONNECT(bdma, chan)				\
	do {								\
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(bdma, chan, irq),		\
			    DT_INST_IRQ_BY_IDX(bdma, chan, priority),	\
			    bdma_stm32_irq_##bdma##_##chan,		\
			    DEVICE_DT_INST_GET(bdma), 0);		\
		irq_enable(DT_INST_IRQ_BY_IDX(bdma, chan, irq));		\
	} while (0)

#endif /* CONFIG_BDMA_STM32_SHARED_IRQS */


#if DT_NODE_HAS_STATUS(DT_DRV_INST(0), okay)

BDMA_STM32_DEFINE_IRQ_HANDLER(0, 0);
BDMA_STM32_DEFINE_IRQ_HANDLER(0, 1);
BDMA_STM32_DEFINE_IRQ_HANDLER(0, 2);
BDMA_STM32_DEFINE_IRQ_HANDLER(0, 3);
BDMA_STM32_DEFINE_IRQ_HANDLER(0, 4);
#if DT_INST_IRQ_HAS_IDX(0, 5)
BDMA_STM32_DEFINE_IRQ_HANDLER(0, 5);
#if DT_INST_IRQ_HAS_IDX(0, 6)
BDMA_STM32_DEFINE_IRQ_HANDLER(0, 6);
#if DT_INST_IRQ_HAS_IDX(0, 7)
BDMA_STM32_DEFINE_IRQ_HANDLER(0, 7);
#endif /* DT_INST_IRQ_HAS_IDX(0, 5) */
#endif /* DT_INST_IRQ_HAS_IDX(0, 6) */
#endif /* DT_INST_IRQ_HAS_IDX(0, 7) */

static void bdma_stm32_config_irq_0(const struct device *dev)
{
	ARG_UNUSED(dev);

	BDMA_STM32_IRQ_CONNECT(0, 0);
	BDMA_STM32_IRQ_CONNECT(0, 1);
#ifndef CONFIG_BDMA_STM32_SHARED_IRQS
	BDMA_STM32_IRQ_CONNECT(0, 2);
#endif /* CONFIG_BDMA_STM32_SHARED_IRQS */
	BDMA_STM32_IRQ_CONNECT(0, 3);
#ifndef CONFIG_BDMA_STM32_SHARED_IRQS
	BDMA_STM32_IRQ_CONNECT(0, 4);
#if DT_INST_IRQ_HAS_IDX(0, 5)
	BDMA_STM32_IRQ_CONNECT(0, 5);
#if DT_INST_IRQ_HAS_IDX(0, 6)
	BDMA_STM32_IRQ_CONNECT(0, 6);
#if DT_INST_IRQ_HAS_IDX(0, 7)
	BDMA_STM32_IRQ_CONNECT(0, 7);
#endif /* DT_INST_IRQ_HAS_IDX(0, 5) */
#endif /* DT_INST_IRQ_HAS_IDX(0, 6) */
#endif /* DT_INST_IRQ_HAS_IDX(0, 7) */
#endif /* CONFIG_BDMA_STM32_SHARED_IRQS */
/* Either 5 or 6 or 7 or 8 channels for BDMA across all stm32 series. */
}

BDMA_STM32_INIT_DEV(0);

#endif /* DT_NODE_HAS_STATUS(DT_DRV_INST(0), okay) */
