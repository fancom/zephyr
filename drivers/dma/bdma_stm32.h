/*
 * Copyright (c) 2019 Song Qiang <songqiang1304521@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BDMA_STM32_H_
#define BDMA_STM32_H_

#include <soc.h>
#include <stm32_ll_bdma.h>
#include <drivers/dma.h>
#include <drivers/clock_control/stm32_clock_control.h>

/* Maximum data sent in single transfer (Bytes) */
#define BDMA_STM32_MAX_DATA_ITEMS	0xffff

struct bdma_stm32_stream {
	uint32_t direction;
#ifdef CONFIG_DMAMUX_STM32
	int mux_channel; /* stores the dmamux channel */
#endif /* CONFIG_DMAMUX_STM32 */
	bool source_periph;
	bool hal_override;
	volatile bool busy;
	uint32_t src_size;
	uint32_t dst_size;
	void *user_data; /* holds the client data */
	dma_callback_t bdma_callback;
};

struct bdma_stm32_data {
};

struct bdma_stm32_config {
	struct stm32_pclken pclken;
	void (*config_irq)(const struct device *dev);
	bool support_m2m;
	uint32_t base;
	uint32_t max_streams;
#ifdef CONFIG_BDMAMUX_STM32
	uint8_t offset; /* position in the list of bdmamux channel list */
#endif
	struct bdma_stm32_stream *streams;
};

#if !defined(CONFIG_BDMAMUX_STM32)
uint32_t bdma_stm32_slot_to_channel(uint32_t id);
#endif

typedef void (*bdma_stm32_clear_flag_func)(BDMA_TypeDef *BDMAx);
typedef uint32_t (*bdma_stm32_check_flag_func)(BDMA_TypeDef *BDMAx);

bool bdma_stm32_is_gi_active(BDMA_TypeDef *BDMAx, uint32_t id);
void bdma_stm32_clear_gi(BDMA_TypeDef *BDMAx, uint32_t id);
bool bdma_stm32_is_tc_active(BDMA_TypeDef *BDMAx, uint32_t id);
void bdma_stm32_clear_tc(BDMA_TypeDef *BDMAx, uint32_t id);
bool bdma_stm32_is_ht_active(BDMA_TypeDef *BDMAx, uint32_t id);
void bdma_stm32_clear_ht(BDMA_TypeDef *BDMAx, uint32_t id);
bool bdma_stm32_is_te_active(BDMA_TypeDef *BDMAx, uint32_t id);
void bdma_stm32_clear_te(BDMA_TypeDef *BDMAx, uint32_t id);

bool stm32_bdma_is_irq_active(BDMA_TypeDef *bdma, uint32_t id);
bool stm32_bdma_is_irq_happened(BDMA_TypeDef *bdma, uint32_t id);
bool stm32_bdma_is_unexpected_irq_happened(BDMA_TypeDef *bdma, uint32_t id);

#if !defined(CONFIG_BDMAMUX_STM32)
void stm32_dma_config_channel_function(BDMA_TypeDef *bdma, uint32_t id,
						uint32_t slot);
#endif

#ifdef CONFIG_BDMAMUX_STM32
/* bdma_stm32_ api functions are exported to the bdmamux_stm32 */
#define BDMA_STM32_EXPORT_API
int bdma_stm32_configure(const struct device *dev, uint32_t id,
				struct dma_config *config);
int bdma_stm32_reload(const struct device *dev, uint32_t id,
			uint32_t src, uint32_t dst, size_t size);
int bdma_stm32_start(const struct device *dev, uint32_t id);
int bdma_stm32_stop(const struct device *dev, uint32_t id);
int bdma_stm32_get_status(const struct device *dev, uint32_t id,
				struct bdma_status *stat);
#else
#define BDMA_STM32_EXPORT_API static
#endif /* CONFIG_BDMAMUX_STM32 */

#endif /* BDMA_STM32_H_*/
