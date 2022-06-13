/*
 * Copyright (c) 2018 Kokoon Technology Limited
 * Copyright (c) 2019 Song Qiang <songqiang1304521@gmail.com>
 * Copyright (c) 2019 Endre Karlson
 * Copyright (c) 2020 Teslabs Engineering S.L.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_stm32_adc

#include <errno.h>

#include <drivers/adc.h>
#include <device.h>
#include <kernel.h>
#include <init.h>
#include <soc.h>
#include <stm32_ll_adc.h>

#ifdef CONFIG_ADC_STM32_DMA
#include <dt-bindings/dma/stm32_dma.h>
#include <drivers/dma.h>
#include <stm32_ll_dma.h>
#endif

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"

#define LOG_LEVEL CONFIG_ADC_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(adc_stm32);

#include <drivers/clock_control/stm32_clock_control.h>
#include <pinmux/stm32/pinmux_stm32.h>

#if !defined(CONFIG_SOC_SERIES_STM32F0X) && \
	!defined(CONFIG_SOC_SERIES_STM32G0X) && \
	!defined(CONFIG_SOC_SERIES_STM32L0X)
#define RANK(n)		LL_ADC_REG_RANK_##n
static const uint32_t table_rank[] = {
	RANK(1),
	RANK(2),
	RANK(3),
	RANK(4),
	RANK(5),
	RANK(6),
	RANK(7),
	RANK(8),
	RANK(9),
	RANK(10),
	RANK(11),
	RANK(12),
	RANK(13),
	RANK(14),
	RANK(15),
	RANK(16),
};

#define SEQ_LEN(n)	LL_ADC_REG_SEQ_SCAN_ENABLE_##n##RANKS
static const uint32_t table_seq_len[] = {
	LL_ADC_REG_SEQ_SCAN_DISABLE,
	SEQ_LEN(2),
	SEQ_LEN(3),
	SEQ_LEN(4),
	SEQ_LEN(5),
	SEQ_LEN(6),
	SEQ_LEN(7),
	SEQ_LEN(8),
	SEQ_LEN(9),
	SEQ_LEN(10),
	SEQ_LEN(11),
	SEQ_LEN(12),
	SEQ_LEN(13),
	SEQ_LEN(14),
	SEQ_LEN(15),
	SEQ_LEN(16),
};
#endif

#define RES(n)		LL_ADC_RESOLUTION_##n##B
static const uint32_t table_resolution[] = {
#if defined(CONFIG_SOC_SERIES_STM32F1X)
	RES(12),
#elif !defined(CONFIG_SOC_SERIES_STM32H7X)
	RES(6),
	RES(8),
	RES(10),
	RES(12),
#else
	RES(8),
	RES(10),
	RES(12),
	RES(14),
	RES(16),
#endif
};

#define SMP_TIME(x, y)		LL_ADC_SAMPLINGTIME_##x##CYCLE##y

/*
 * Conversion time in ADC cycles. Many values should have been 0.5 less,
 * but the adc api system currently does not support describing 'half cycles'.
 * So all half cycles are counted as one.
 */
#if defined(CONFIG_SOC_SERIES_STM32F0X) || defined(CONFIG_SOC_SERIES_STM32F1X)
static const uint16_t acq_time_tbl[8] = {2, 8, 14, 29, 42, 56, 72, 240};
static const uint32_t table_samp_time[] = {
	SMP_TIME(1,   _5),
	SMP_TIME(7,   S_5),
	SMP_TIME(13,  S_5),
	SMP_TIME(28,  S_5),
	SMP_TIME(41,  S_5),
	SMP_TIME(55,  S_5),
	SMP_TIME(71,  S_5),
	SMP_TIME(239, S_5),
};
#elif defined(CONFIG_SOC_SERIES_STM32F2X) || \
	defined(CONFIG_SOC_SERIES_STM32F4X) || \
	defined(CONFIG_SOC_SERIES_STM32F7X)
static const uint16_t acq_time_tbl[8] = {3, 15, 28, 56, 84, 112, 144, 480};
static const uint32_t table_samp_time[] = {
	SMP_TIME(3,   S),
	SMP_TIME(15,  S),
	SMP_TIME(28,  S),
	SMP_TIME(56,  S),
	SMP_TIME(84,  S),
	SMP_TIME(112, S),
	SMP_TIME(144, S),
	SMP_TIME(480, S),
};
#elif defined(CONFIG_SOC_SERIES_STM32F3X)
#ifdef ADC5_V1_1
static const uint16_t acq_time_tbl[8] = {2, 3, 5, 8, 20, 62, 182, 602};
static const uint32_t table_samp_time[] = {
	SMP_TIME(1,   _5),
	SMP_TIME(2,   S_5),
	SMP_TIME(4,   S_5),
	SMP_TIME(7,   S_5),
	SMP_TIME(19,  S_5),
	SMP_TIME(61,  S_5),
	SMP_TIME(181, S_5),
	SMP_TIME(601, S_5),
};
#else
static const uint16_t acq_time_tbl[8] = {2, 8, 14, 29, 42, 56, 72, 240};
static const uint32_t table_samp_time[] = {
	SMP_TIME(1,   _5),
	SMP_TIME(7,   S_5),
	SMP_TIME(13,  S_5),
	SMP_TIME(28,  S_5),
	SMP_TIME(41,  S_5),
	SMP_TIME(55,  S_5),
	SMP_TIME(71,  S_5),
	SMP_TIME(239, S_5),
};
#endif /* ADC5_V1_1 */
#elif defined(CONFIG_SOC_SERIES_STM32L0X) || \
	defined(CONFIG_SOC_SERIES_STM32G0X)
static const uint16_t acq_time_tbl[8] = {2, 4, 8, 13, 20, 40, 80, 161};
static const uint32_t table_samp_time[] = {
	SMP_TIME(1,   _5),
	SMP_TIME(3,   S_5),
	SMP_TIME(7,   S_5),
	SMP_TIME(12,  S_5),
	SMP_TIME(19,  S_5),
	SMP_TIME(39,  S_5),
	SMP_TIME(79,  S_5),
	SMP_TIME(160, S_5),
};
#elif defined(CONFIG_SOC_SERIES_STM32L4X) || \
	defined(CONFIG_SOC_SERIES_STM32WBX) || \
	defined(CONFIG_SOC_SERIES_STM32G4X)
static const uint16_t acq_time_tbl[8] = {3, 7, 13, 25, 48, 93, 248, 641};
static const uint32_t table_samp_time[] = {
	SMP_TIME(2,   S_5),
	SMP_TIME(6,   S_5),
	SMP_TIME(12,  S_5),
	SMP_TIME(24,  S_5),
	SMP_TIME(47,  S_5),
	SMP_TIME(92,  S_5),
	SMP_TIME(247, S_5),
	SMP_TIME(640, S_5),
};
#elif defined(CONFIG_SOC_SERIES_STM32L1X)
static const uint16_t acq_time_tbl[8] = {5, 10, 17, 25, 49, 97, 193, 385};
static const uint32_t table_samp_time[] = {
	SMP_TIME(4,   S),
	SMP_TIME(9,   S),
	SMP_TIME(16,  S),
	SMP_TIME(24,  S),
	SMP_TIME(48,  S),
	SMP_TIME(96,  S),
	SMP_TIME(192, S),
	SMP_TIME(384, S),
};
#elif defined(CONFIG_SOC_SERIES_STM32H7X)
static const uint16_t acq_time_tbl[8] = {2, 3, 9, 17, 33, 65, 388, 811};
static const uint32_t table_samp_time[] = {
	SMP_TIME(1,   _5),
	SMP_TIME(2,   S_5),
	SMP_TIME(8,   S_5),
	SMP_TIME(16,  S_5),
	SMP_TIME(32,  S_5),
	SMP_TIME(64,  S_5),
	SMP_TIME(387, S_5),
	SMP_TIME(810, S_5),
};
#endif

/* Bugfix for STM32G4 HAL */
#if !defined(ADC_CHANNEL_TEMPSENSOR)
#define ADC_CHANNEL_TEMPSENSOR ADC_CHANNEL_TEMPSENSOR_ADC1
#endif

/* External channels (maximum). */
#define STM32_CHANNEL_COUNT		20
#define STM32_ADC_SEQUENCE_MAX_LEN	16
#define STM32_CACHE_GRANULARITY		32

#ifdef CONFIG_ADC_STM32_DMA
struct stream {
	const struct device *dma_dev;
	uint32_t channel; /* stores the channel for dma or mux */
	struct dma_config dma_cfg;
	struct dma_block_config dma_blk_cfg;
	uint8_t priority;
	bool src_addr_increment;
	bool dst_addr_increment;
	int fifo_threshold;
};
#endif /* CONFIG_ADC_STM32_DMA */

struct adc_stm32_data {
	struct adc_context ctx;
	const struct device *dev;
	uint16_t *buffer;
	uint16_t *repeat_buffer;

	uint8_t resolution;
	uint8_t channel_count;
	uint8_t samples_count;
#if defined(CONFIG_SOC_SERIES_STM32F0X) || \
	defined(CONFIG_SOC_SERIES_STM32G0X) || \
	defined(CONFIG_SOC_SERIES_STM32L0X)
	int8_t acq_time_index;
#endif

#ifdef CONFIG_ADC_STM32_DMA
	size_t buffer_size;
	volatile int dma_error;
	struct stream dma;
#endif
};

struct adc_stm32_cfg {
	ADC_TypeDef *base;
	void (*irq_cfg_func)(void);
	struct stm32_pclken pclken;
	const struct soc_gpio_pinctrl *pinctrl;
	size_t pinctrl_len;
};

#ifdef CONFIG_ADC_STM32_DMA
static void dma_callback(const struct device *dev, void *user_data,
			 uint32_t channel, int status)
{
	/* user_data directly holds the adc device */
	struct adc_stm32_data *data = user_data;
	const struct adc_stm32_cfg *config = dev->config;
	ADC_TypeDef *adc = (ADC_TypeDef *)config->base;

	LOG_DBG("dma_complete");

	if (channel == data->dma.channel) {
		/* invalidate cache to make sure to get the adc data*/
		/* Note: Buffer address must to be 32 bytes aligned using __aligned(32) */
		SCB_InvalidateDCache_by_Addr(data->buffer,
			(data->buffer_size + STM32_CACHE_GRANULARITY) & ~(STM32_CACHE_GRANULARITY));

		if (LL_ADC_IsActiveFlag_OVR(adc) || (status == 0)) {
			data->samples_count = data->channel_count;
			/* Stop the DMA engine, only to start it again when the callback returns
			 *  ADC_ACTION_REPEAT or ADC_ACTION_CONTINUE
			 *  and the number of samples haven't been reached
			 *  Starting the DMA engine is done within adc_context_start_sampling
			 */
			dma_stop(data->dma.dma_dev, data->dma.channel);
			LL_ADC_ClearFlag_OVR(adc);
			adc_context_on_sampling_done(&data->ctx, dev);
		} else {
			LOG_ERR("DMA sampling complete, but DMA reported error %d", status);
			data->dma_error = status;
			LL_ADC_REG_StopConversion(adc);
			dma_stop(data->dma.dma_dev, data->dma.channel);
			adc_context_complete(&data->ctx, status);
		}
	}
}

static int adc_stm32_dma_start(const struct device *dev,
			       void *buffer, size_t buffer_size)
{
	const struct adc_stm32_cfg *config = dev->config;
	ADC_TypeDef *adc = (ADC_TypeDef *)config->base;
	struct adc_stm32_data *data = dev->data;
	struct dma_block_config *blk_cfg;
	int ret;

	struct stream *stream = &data->dma;

	blk_cfg = &stream->dma_blk_cfg;

	/* prepare the block */
	memset(blk_cfg, 0, sizeof(struct dma_block_config));
	blk_cfg->block_size = buffer_size;

	/* Source and destination */
	blk_cfg->source_address = (uint32_t)LL_ADC_DMA_GetRegAddr(adc, LL_ADC_DMA_REG_REGULAR_DATA);
	blk_cfg->source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	blk_cfg->source_reload_en = 0;

	blk_cfg->dest_address = (uint32_t)buffer;
	blk_cfg->dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	blk_cfg->dest_reload_en = 0;

	/* give the fifo mode from the DT */
	blk_cfg->fifo_mode_control = data->dma.fifo_threshold;

	/* direction is given by the DT */
	stream->dma_cfg.head_block = blk_cfg;
	stream->dma_cfg.user_data = data;

	/* pass our client origin to the dma: data->dma.channel */
	ret = dma_config(data->dma.dma_dev, data->dma.channel,
			 &stream->dma_cfg);

	/* the channel is the actual stream from 0 */
	if (ret != 0) {
		LOG_ERR("Problem setting up DMA %d", ret);
		return ret;
	}

	ret = dma_start(data->dma.dma_dev, data->dma.channel);
	if (ret != 0) {
		LOG_ERR("Problem starting DMA %d", ret);
		return ret;
	}

	/* Allow ADC to create DMA request and set to one-shot mode */
	/* Copied from the HAL ADC drivers */
#if defined(ADC_VER_V5_V90)
	if (adc == ADC3) {
		LL_ADC_REG_SetDMATransferMode(adc,
			ADC3_CFGR_DMACONTREQ(LL_ADC_REG_DMA_TRANSFER_LIMITED));
		LL_ADC_EnableDMAReq(adc);
	} else {
		LL_ADC_REG_SetDataTransferMode(adc,
			ADC_CFGR_DMACONTREQ(LL_ADC_REG_DMA_TRANSFER_LIMITED));
	}
#else
	LL_ADC_REG_SetDataTransferMode(adc, LL_ADC_REG_DMA_TRANSFER_LIMITED);
#endif


	return ret;
}
#endif /* CONFIG_ADC_STM32_DMA */

static int check_buffer_size(const struct adc_sequence *sequence,
			     uint8_t active_channels)
{
	size_t needed_buffer_size;

	needed_buffer_size = active_channels * sizeof(uint16_t);

	if (sequence->options) {
		needed_buffer_size *= (1 + sequence->options->extra_samplings);
	}

	if (sequence->buffer_size < needed_buffer_size) {
		LOG_ERR("Provided buffer is too small (%u/%u)",
				sequence->buffer_size, needed_buffer_size);
		return -ENOMEM;
	}

	return 0;
}

static void adc_stm32_disable_adc(const ADC_TypeDef *adc)
{
	if (LL_ADC_IsEnabled(adc) == 1UL) {
		LL_ADC_Disable(adc);
		while (LL_ADC_IsEnabled(adc) == 1UL) {
		}
	}
}

static void adc_stm32_wait_after_calibration(const struct device *dev)
{
#if defined(CONFIG_SOC_SERIES_STM32F0X) || \
	defined(CONFIG_SOC_SERIES_STM32F3X) || \
	defined(CONFIG_SOC_SERIES_STM32L0X) || \
	defined(CONFIG_SOC_SERIES_STM32L4X) || \
	defined(CONFIG_SOC_SERIES_STM32WBX) || \
	defined(CONFIG_SOC_SERIES_STM32G0X) || \
	defined(CONFIG_SOC_SERIES_STM32G4X) || \
	defined(CONFIG_SOC_SERIES_STM32H7X)
	/*
	 * ADC modules on these series have to wait for some cycles to be
	 * enabled after doing a calibration.
	 */
	const struct adc_stm32_cfg *config = dev->config;
	uint32_t adc_rate, wait_cycles;
	const struct device *clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);

	if (clock_control_get_rate(clk,
		(clock_control_subsys_t *) &config->pclken, &adc_rate) < 0) {
		LOG_ERR("ADC clock rate get error.");
	}

	wait_cycles = SystemCoreClock / adc_rate *
		LL_ADC_DELAY_CALIB_ENABLE_ADC_CYCLES;

	for (int i = wait_cycles; i >= 0; i--) {
	}
#endif
}

static void adc_stm32_wait_until_enabled(const ADC_TypeDef *adc)
{
#if defined(CONFIG_SOC_SERIES_STM32L4X) || \
	defined(CONFIG_SOC_SERIES_STM32WBX) || \
	defined(CONFIG_SOC_SERIES_STM32G0X) || \
	defined(CONFIG_SOC_SERIES_STM32G4X) || \
	defined(CONFIG_SOC_SERIES_STM32H7X)
	/*
	 * Enabling ADC modules in L4, WB, G0 and G4 series may fail if they are
	 * still not stabilized, this will wait for a short time to ensure ADC
	 * modules are properly enabled.
	 */
	uint32_t countTimeout = 0;

	while (LL_ADC_IsActiveFlag_ADRDY(adc) == 0) {
		if (LL_ADC_IsEnabled(adc) == 0UL) {
			LL_ADC_Enable(adc);
			countTimeout++;
			if (countTimeout == 10) {
				return; /* -ETIMEDOUT */
			}
		}
	}
#endif
}

static void adc_stm32_start_conversion(const struct device *dev)
{
	const struct adc_stm32_cfg *config = dev->config;
	ADC_TypeDef *adc = (ADC_TypeDef *)config->base;

	LOG_DBG("Starting conversion");

#if defined(CONFIG_SOC_SERIES_STM32F0X) || \
	defined(CONFIG_SOC_SERIES_STM32F3X) || \
	defined(CONFIG_SOC_SERIES_STM32L0X) || \
	defined(CONFIG_SOC_SERIES_STM32L4X) || \
	defined(CONFIG_SOC_SERIES_STM32WBX) || \
	defined(CONFIG_SOC_SERIES_STM32G0X) || \
	defined(CONFIG_SOC_SERIES_STM32G4X) || \
	defined(CONFIG_SOC_SERIES_STM32H7X)
	LL_ADC_REG_StartConversion(adc);
#else
	LL_ADC_REG_StartConversionSWStart(adc);
#endif
}

#if !defined(CONFIG_SOC_SERIES_STM32F2X) && \
	!defined(CONFIG_SOC_SERIES_STM32F4X) && \
	!defined(CONFIG_SOC_SERIES_STM32F7X) && \
	!defined(CONFIG_SOC_SERIES_STM32F1X) && \
	!defined(CONFIG_SOC_SERIES_STM32L1X)
static void adc_stm32_calib(const struct device *dev)
{
	const struct adc_stm32_cfg *config =
		(const struct adc_stm32_cfg *)dev->config;
	ADC_TypeDef *adc = config->base;

#if defined(CONFIG_SOC_SERIES_STM32F3X) || \
	defined(CONFIG_SOC_SERIES_STM32L4X) || \
	defined(CONFIG_SOC_SERIES_STM32WBX) || \
	defined(CONFIG_SOC_SERIES_STM32G4X)
	LL_ADC_StartCalibration(adc, LL_ADC_SINGLE_ENDED);
#elif defined(CONFIG_SOC_SERIES_STM32F0X) || \
	defined(CONFIG_SOC_SERIES_STM32G0X) || \
	defined(CONFIG_SOC_SERIES_STM32L0X)
	LL_ADC_StartCalibration(adc);
#elif defined(CONFIG_SOC_SERIES_STM32H7X)
	LL_ADC_StartCalibration(adc, LL_ADC_CALIB_OFFSET, LL_ADC_SINGLE_ENDED);
#endif
	while (LL_ADC_IsCalibrationOnGoing(adc)) {
	}
}
#endif

#if defined(CONFIG_SOC_SERIES_STM32G0X) || \
	defined(CONFIG_SOC_SERIES_STM32G4X) || \
	defined(CONFIG_SOC_SERIES_STM32H7X) || \
	defined(CONFIG_SOC_SERIES_STM32L0X) || \
	defined(CONFIG_SOC_SERIES_STM32L4X) || \
	defined(CONFIG_SOC_SERIES_STM32L5X) || \
	defined(CONFIG_SOC_SERIES_STM32WBX) || \
	defined(CONFIG_SOC_SERIES_STM32WLX)

#ifdef LL_ADC_OVS_RATIO_2
/* table for shifting oversampling mostly for ADC3 != ADC_VER_V5_V90 */
	static const uint32_t stm32_adc_ratio_table[] = {
		0,
		LL_ADC_OVS_RATIO_2,
		LL_ADC_OVS_RATIO_4,
		LL_ADC_OVS_RATIO_8,
		LL_ADC_OVS_RATIO_16,
		LL_ADC_OVS_RATIO_32,
		LL_ADC_OVS_RATIO_64,
		LL_ADC_OVS_RATIO_128,
		LL_ADC_OVS_RATIO_256,
	};
#endif /* ! ADC_VER_V5_V90 */

	/*
	 * Function to configure the oversampling ratio and shit using stm32 LL
	 * ratio is directly the sequence->oversampling (a 2^n value)
	 * shift is the corresponding LL_ADC_OVS_SHIFT_RIGHT_x constant
	 */
static void adc_stm32_oversampling(ADC_TypeDef *adc, uint8_t ratio, uint32_t shift)
{
	LL_ADC_SetOverSamplingScope(adc, LL_ADC_OVS_GRP_REGULAR_CONTINUED);
#if defined(CONFIG_SOC_SERIES_STM32H7X)
	/*
	 * Set bits manually to circumvent bug in LL Libraries
	 * https://github.com/STMicroelectronics/STM32CubeH7/issues/177
	 */
#if defined(ADC_VER_V5_V90)
	if (adc == ADC3) {
		MODIFY_REG(adc->CFGR2, (ADC_CFGR2_OVSS | ADC3_CFGR2_OVSR),
			(shift | stm32_adc_ratio_table[ratio]));
	} else {
		MODIFY_REG(adc->CFGR2, (ADC_CFGR2_OVSS | ADC_CFGR2_OVSR),
			(shift | (((1UL << ratio) - 1) << ADC_CFGR2_OVSR_Pos)));
	}
#endif /* ADC_VER_V5_V90*/
	MODIFY_REG(adc->CFGR2, (ADC_CFGR2_OVSS | ADC_CFGR2_OVSR),
		(shift | (((1UL << ratio) - 1) << ADC_CFGR2_OVSR_Pos)));
#else /* CONFIG_SOC_SERIES_STM32H7X */
	LL_ADC_ConfigOverSamplingRatioShift(adc, stm32_adc_ratio_table[ratio], shift);
#endif /* CONFIG_SOC_SERIES_STM32H7X */
}
#endif /* CONFIG_SOC_SERIES_STM32xxx */

static int start_read(const struct device *dev,
		      const struct adc_sequence *sequence)
{
	const struct adc_stm32_cfg *config = dev->config;
	struct adc_stm32_data *data = dev->data;
	ADC_TypeDef *adc = (ADC_TypeDef *)config->base;
	uint8_t resolution;
	int err;

	switch (sequence->resolution) {
#if defined(CONFIG_SOC_SERIES_STM32F1X)
	case 12:
		resolution = table_resolution[0];
		break;
#elif !defined(CONFIG_SOC_SERIES_STM32H7X)
	case 6:
		resolution = table_resolution[0];
		break;
	case 8:
		resolution = table_resolution[1];
		break;
	case 10:
		resolution = table_resolution[2];
		break;
	case 12:
		resolution = table_resolution[3];
		break;
#else
	case 8:
		resolution = table_resolution[0];
		break;
	case 10:
		resolution = table_resolution[1];
		break;
	case 12:
		resolution = table_resolution[2];
		break;
	case 14:
		resolution = table_resolution[3];
		break;
	case 16:
		resolution = table_resolution[4];
		break;
#endif
	default:
		LOG_ERR("Invalid resolution");
		return -EINVAL;
	}

	data->buffer = sequence->buffer;
#ifdef CONFIG_ADC_STM32_DMA
	data->buffer_size = sequence->buffer_size;
#endif
	data->channel_count = 0;
	data->samples_count = 0;

	for (uint8_t chSel = 0; chSel < STM32_CHANNEL_COUNT; chSel++) {
		if (sequence->channels & BIT(chSel)) {
			uint32_t channel = __LL_ADC_DECIMAL_NB_TO_CHANNEL(chSel);

#if defined(CONFIG_SOC_SERIES_STM32H7X)
	/*
	 * Each channel in the sequence must be previously enabled in PCSEL.
	 * This register controls the analog switch integrated in the IO level.
	 */
	LL_ADC_SetChannelPreSelection(adc, channel);
#endif

#if defined(CONFIG_SOC_SERIES_STM32F0X) || \
	defined(CONFIG_SOC_SERIES_STM32L0X)
			LL_ADC_REG_SetSequencerChannels(adc, channel);
#elif defined(CONFIG_SOC_SERIES_STM32G0X)
	/* STM32G0 in "not fully configurable" sequencer mode */
			LL_ADC_REG_SetSequencerChannels(adc, channel);
			while (LL_ADC_IsActiveFlag_CCRDY(adc) == 0) {
			}
#else
			LL_ADC_REG_SetSequencerRanks(adc, table_rank[data->channel_count], channel);
			LL_ADC_REG_SetSequencerLength(adc, table_seq_len[data->channel_count]);
#endif
			data->channel_count++;
			if (data->channel_count >= STM32_ADC_SEQUENCE_MAX_LEN) {
				break;
			}
		}
	}

	if (data->channel_count == 0) {
		LOG_ERR("No channels selected");
		return -EINVAL;
	}

	err = check_buffer_size(sequence, data->channel_count);
	if (err) {
		return err;
	}

#if defined(CONFIG_SOC_SERIES_STM32G0X)
	/*
	 * Errata: Writing ADC_CFGR1 register while ADEN bit is set
	 * resets RES[1:0] bitfield. We need to disable and enable adc.
	 */
	adc_stm32_disable_adc(adc);
	LL_ADC_SetResolution(adc, resolution);
	LL_ADC_Enable(adc);
	while (LL_ADC_IsActiveFlag_ADRDY(adc) != 1UL) {
	}
#elif !defined(CONFIG_SOC_SERIES_STM32F1X)
	LL_ADC_SetResolution(adc, resolution);
#endif

#if defined(CONFIG_SOC_SERIES_STM32G0X) || \
	defined(CONFIG_SOC_SERIES_STM32G4X) || \
	defined(CONFIG_SOC_SERIES_STM32H7X) || \
	defined(CONFIG_SOC_SERIES_STM32L0X) || \
	defined(CONFIG_SOC_SERIES_STM32L4X) || \
	defined(CONFIG_SOC_SERIES_STM32WBX) || \
	defined(CONFIG_SOC_SERIES_STM32WLX)

	switch (sequence->oversampling) {
	case 0:
		LL_ADC_SetOverSamplingScope(adc, LL_ADC_OVS_DISABLE);
		break;
	case 1:
		adc_stm32_oversampling(adc, 1, LL_ADC_OVS_SHIFT_RIGHT_1);
		break;
	case 2:
		adc_stm32_oversampling(adc, 2, LL_ADC_OVS_SHIFT_RIGHT_2);
		break;
	case 3:
		adc_stm32_oversampling(adc, 3, LL_ADC_OVS_SHIFT_RIGHT_3);
		break;
	case 4:
		adc_stm32_oversampling(adc, 4, LL_ADC_OVS_SHIFT_RIGHT_4);
		break;
	case 5:
		adc_stm32_oversampling(adc, 5, LL_ADC_OVS_SHIFT_RIGHT_5);
		break;
	case 6:
		adc_stm32_oversampling(adc, 6, LL_ADC_OVS_SHIFT_RIGHT_6);
		break;
	case 7:
		adc_stm32_oversampling(adc, 7, LL_ADC_OVS_SHIFT_RIGHT_7);
		break;
	case 8:
		adc_stm32_oversampling(adc, 8, LL_ADC_OVS_SHIFT_RIGHT_8);
		break;
#if defined(CONFIG_SOC_SERIES_STM32H7X)
	/* stm32 H7 ADC1 & 2 have oversampling ratio from 1..1024 */
	case 9:
		adc_stm32_oversampling(adc, 9, LL_ADC_OVS_SHIFT_RIGHT_9);
		break;
	case 10:
		adc_stm32_oversampling(adc, 10, LL_ADC_OVS_SHIFT_RIGHT_10);
		break;
#endif /* CONFIG_SOC_SERIES_STM32H7X */
	default:
		LOG_ERR("Invalid oversampling");
		return -EINVAL;
	}
#else
	if (sequence->oversampling) {
		LOG_ERR("Oversampling not supported");
		return -ENOTSUP;
	}
#endif

	if (sequence->calibrate) {
#if !defined(CONFIG_SOC_SERIES_STM32F2X) && \
	!defined(CONFIG_SOC_SERIES_STM32F4X) && \
	!defined(CONFIG_SOC_SERIES_STM32F7X) && \
	!defined(CONFIG_SOC_SERIES_STM32F1X) && \
	!defined(STM32F3X_ADC_V2_5) && \
	!defined(CONFIG_SOC_SERIES_STM32L1X)
		adc_stm32_disable_adc(adc);
		adc_stm32_calib(dev);

		LL_ADC_ClearFlag_ADRDY(adc);

		adc_stm32_wait_after_calibration(dev);

		LL_ADC_Enable(adc);

		adc_stm32_wait_until_enabled(adc);
#else
		LOG_ERR("Calibration not supported");
		return -ENOTSUP;
#endif
	}

	LL_ADC_ClearFlag_OVR(adc);

#if defined(CONFIG_SOC_SERIES_STM32F0X) || \
	defined(CONFIG_SOC_SERIES_STM32F3X) || \
	defined(CONFIG_SOC_SERIES_STM32L0X) || \
	defined(CONFIG_SOC_SERIES_STM32L4X) || \
	defined(CONFIG_SOC_SERIES_STM32WBX) || \
	defined(CONFIG_SOC_SERIES_STM32G0X) || \
	defined(CONFIG_SOC_SERIES_STM32G4X) || \
	defined(CONFIG_SOC_SERIES_STM32H7X)
	if (data->channel_count > 1) {
		LL_ADC_EnableIT_EOS(adc);
	} else {
		LL_ADC_EnableIT_EOC(adc);
	}
#elif defined(CONFIG_SOC_SERIES_STM32F1X)
	LL_ADC_EnableIT_EOS(adc);
#else
	LL_ADC_EnableIT_EOCS(adc);
#endif

	adc_context_start_read(&data->ctx, sequence);

	int result = adc_context_wait_for_completion(&data->ctx);

#ifdef CONFIG_ADC_STM32_DMA
	/* check if there's anything wrong with dma start */
	result = (data->dma_error ? data->dma_error : result);
#endif

	return result;
}

static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct adc_stm32_data *data =
		CONTAINER_OF(ctx, struct adc_stm32_data, ctx);

	data->repeat_buffer = data->buffer;

#ifdef CONFIG_ADC_STM32_DMA
	data->dma_error = 0;
	adc_stm32_dma_start(data->dev, data->buffer, data->buffer_size);
#endif
	adc_stm32_start_conversion(data->dev);
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx,
					      bool repeat_sampling)
{
	struct adc_stm32_data *data =
		CONTAINER_OF(ctx, struct adc_stm32_data, ctx);

	if (repeat_sampling) {
		data->buffer = data->repeat_buffer;
	}
}

static void adc_stm32_isr(const struct device *dev)
{
	struct adc_stm32_data *data = (struct adc_stm32_data *)dev->data;
	const struct adc_stm32_cfg *config =
		(const struct adc_stm32_cfg *)dev->config;
	ADC_TypeDef *adc = config->base;

	*data->buffer++ = LL_ADC_REG_ReadConversionData32(adc);
	data->samples_count++;
	if (data->samples_count == data->channel_count) {
		adc_context_on_sampling_done(&data->ctx, dev);
	}

	LOG_DBG("ISR triggered.");
}

static int adc_stm32_read(const struct device *dev,
			  const struct adc_sequence *sequence)
{
	struct adc_stm32_data *data = dev->data;
	int error;

	adc_context_lock(&data->ctx, false, NULL);
	error = start_read(dev, sequence);
	adc_context_release(&data->ctx, error);

	return error;
}

#ifdef CONFIG_ADC_ASYNC
static int adc_stm32_read_async(const struct device *dev,
				 const struct adc_sequence *sequence,
				 struct k_poll_signal *async)
{
	struct adc_stm32_data *data = dev->data;
	int error;

	adc_context_lock(&data->ctx, true, async);
	error = start_read(dev, sequence);
	adc_context_release(&data->ctx, error);

	return error;
}
#endif

static int adc_stm32_check_acq_time(uint16_t acq_time)
{
	if (acq_time == ADC_ACQ_TIME_MAX) {
		return ARRAY_SIZE(acq_time_tbl) - 1;
	}

	for (int i = 0; i < 8; i++) {
		if (acq_time == ADC_ACQ_TIME(ADC_ACQ_TIME_TICKS,
					     acq_time_tbl[i])) {
			return i;
		}
	}

	if (acq_time == ADC_ACQ_TIME_DEFAULT) {
		return 0;
	}

	LOG_ERR("Conversion time not supported.");
	return -EINVAL;
}

/*
 * Enable internal voltage reference source
 */
static void adc_stm32_set_common_path(const struct device *dev)
{
	const struct adc_stm32_cfg *config =
		(const struct adc_stm32_cfg *)dev->config;
	ADC_TypeDef *adc = (ADC_TypeDef *)config->base;
	(void) adc; /* Avoid 'unused variable' warning for some families */

	LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(adc),
				LL_ADC_PATH_INTERNAL_TEMPSENSOR);
}

static void adc_stm32_setup_speed(const struct device *dev, uint8_t id,
				  uint8_t acq_time_index)
{
	const struct adc_stm32_cfg *config =
		(const struct adc_stm32_cfg *)dev->config;
	ADC_TypeDef *adc = config->base;

#if defined(CONFIG_SOC_SERIES_STM32F0X) || defined(CONFIG_SOC_SERIES_STM32L0X)
	LL_ADC_SetSamplingTimeCommonChannels(adc,
		table_samp_time[acq_time_index]);
#elif defined(CONFIG_SOC_SERIES_STM32G0X)
	LL_ADC_SetSamplingTimeCommonChannels(adc, LL_ADC_SAMPLINGTIME_COMMON_1,
		table_samp_time[acq_time_index]);
#else
	LL_ADC_SetChannelSamplingTime(adc,
		__LL_ADC_DECIMAL_NB_TO_CHANNEL(id),
		table_samp_time[acq_time_index]);
#endif
}

static int adc_stm32_channel_setup(const struct device *dev,
				   const struct adc_channel_cfg *channel_cfg)
{
#if defined(CONFIG_SOC_SERIES_STM32F0X) || \
	defined(CONFIG_SOC_SERIES_STM32G0X) || \
	defined(CONFIG_SOC_SERIES_STM32L0X)
	struct adc_stm32_data *data = dev->data;
#endif
	int acq_time_index;

	if (channel_cfg->channel_id >= STM32_CHANNEL_COUNT) {
		LOG_ERR("Channel %d is not valid", channel_cfg->channel_id);
		return -EINVAL;
	}

	acq_time_index = adc_stm32_check_acq_time(channel_cfg->acquisition_time);
	if (acq_time_index < 0) {
		return acq_time_index;
	}
#if defined(CONFIG_SOC_SERIES_STM32F0X) || \
	defined(CONFIG_SOC_SERIES_STM32G0X) || \
	defined(CONFIG_SOC_SERIES_STM32L0X)
	if (data->acq_time_index == -1) {
		data->acq_time_index = acq_time_index;
	} else {
		/* All channels of F0/L0 must have identical acquisition time.*/
		if (acq_time_index != data->acq_time_index) {
			return -EINVAL;
		}
	}
#endif

	if (channel_cfg->differential) {
		LOG_ERR("Differential channels are not supported");
		return -EINVAL;
	}

	if (channel_cfg->gain != ADC_GAIN_1) {
		LOG_ERR("Invalid channel gain");
		return -EINVAL;
	}

	if (channel_cfg->reference != ADC_REF_INTERNAL) {
		LOG_ERR("Invalid channel reference");
		return -EINVAL;
	}

	if ((__LL_ADC_CHANNEL_TO_DECIMAL_NB(ADC_CHANNEL_TEMPSENSOR) == channel_cfg->channel_id) ||
		(__LL_ADC_CHANNEL_TO_DECIMAL_NB(ADC_CHANNEL_VREFINT) == channel_cfg->channel_id)) {
		adc_stm32_set_common_path(dev);
	}

	adc_stm32_setup_speed(dev, channel_cfg->channel_id, acq_time_index);

	LOG_DBG("Channel %d setup succeeded!", channel_cfg->channel_id);

	return 0;
}

static int adc_stm32_init(const struct device *dev)
{
	struct adc_stm32_data *data = dev->data;
	const struct adc_stm32_cfg *config = dev->config;
	const struct device *clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);
	ADC_TypeDef *adc = (ADC_TypeDef *)config->base;
	int err;

	LOG_DBG("Initializing....");

	data->dev = dev;
#if defined(CONFIG_SOC_SERIES_STM32F0X) || \
	defined(CONFIG_SOC_SERIES_STM32G0X) || \
	defined(CONFIG_SOC_SERIES_STM32L0X)
	/*
	 * All conversion time for all channels on one ADC instance for F0 and
	 * L0 series chips has to be the same. For STM32G0 currently only one
	 * of the two available common channel conversion times is used.
	 * This additional variable is for checking if the conversion time
	 * selection of all channels on one ADC instance is the same.
	 */
	data->acq_time_index = -1;
#endif

	if (clock_control_on(clk,
		(clock_control_subsys_t *) &config->pclken) != 0) {
		return -EIO;
	}

	/* Configure dt provided device signals when available */
	err = stm32_dt_pinctrl_configure(config->pinctrl,
					 config->pinctrl_len,
					 (uint32_t)config->base);
	if (err < 0) {
		LOG_ERR("ADC pinctrl setup failed (%d)", err);
		return err;
	}

#ifdef CONFIG_ADC_STM32_DMA
	if ((data->dma.dma_dev != NULL) &&
	    !device_is_ready(data->dma.dma_dev)) {
		LOG_ERR("%s device not ready", data->dma.dma_dev->name);
		return -ENODEV;
	}
#endif

#if defined(CONFIG_SOC_SERIES_STM32L4X) || \
	defined(CONFIG_SOC_SERIES_STM32WBX) || \
	defined(CONFIG_SOC_SERIES_STM32G4X) || \
	defined(CONFIG_SOC_SERIES_STM32H7X)
	/*
	 * L4, WB, G4 and H7 series STM32 needs to be awaken from deep sleep
	 * mode, and restore its calibration parameters if there are some
	 * previously stored calibration parameters.
	 */
	LL_ADC_DisableDeepPowerDown(adc);
#endif
	/*
	 * F3, L4, WB, G0 and G4 ADC modules need some time
	 * to be stabilized before performing any enable or calibration actions.
	 */
#if defined(CONFIG_SOC_SERIES_STM32F3X) || \
	defined(CONFIG_SOC_SERIES_STM32L4X) || \
	defined(CONFIG_SOC_SERIES_STM32WBX) || \
	defined(CONFIG_SOC_SERIES_STM32G0X) || \
	defined(CONFIG_SOC_SERIES_STM32G4X) || \
	defined(CONFIG_SOC_SERIES_STM32H7X)
	LL_ADC_EnableInternalRegulator(adc);
	k_busy_wait(LL_ADC_DELAY_INTERNAL_REGUL_STAB_US);
#endif

#if defined(CONFIG_SOC_SERIES_STM32F0X) || \
	defined(CONFIG_SOC_SERIES_STM32L0X)
	LL_ADC_SetClock(adc, LL_ADC_CLOCK_SYNC_PCLK_DIV4);
#elif defined(CONFIG_SOC_SERIES_STM32F3X) || \
	defined(CONFIG_SOC_SERIES_STM32L4X) || \
	defined(CONFIG_SOC_SERIES_STM32WBX) || \
	defined(CONFIG_SOC_SERIES_STM32G0X) || \
	defined(CONFIG_SOC_SERIES_STM32G4X) || \
	defined(CONFIG_SOC_SERIES_STM32H7X)
	LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(adc),
			      LL_ADC_CLOCK_SYNC_PCLK_DIV4);
#elif defined(CONFIG_SOC_SERIES_STM32L1X)
	LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(adc),
			LL_ADC_CLOCK_ASYNC_DIV4);
#endif

#if !defined(CONFIG_SOC_SERIES_STM32F2X) && \
	!defined(CONFIG_SOC_SERIES_STM32F4X) && \
	!defined(CONFIG_SOC_SERIES_STM32F7X) && \
	!defined(CONFIG_SOC_SERIES_STM32F1X) && \
	!defined(CONFIG_SOC_SERIES_STM32L1X)
	/*
	 * Calibration of F1 series has to be started after ADC Module is
	 * enabled.
	 */
	adc_stm32_calib(dev);
#endif

#if defined(CONFIG_SOC_SERIES_STM32F0X) || \
	defined(CONFIG_SOC_SERIES_STM32L0X) || \
	defined(CONFIG_SOC_SERIES_STM32L4X) || \
	defined(CONFIG_SOC_SERIES_STM32WBX) || \
	defined(CONFIG_SOC_SERIES_STM32G0X) || \
	defined(CONFIG_SOC_SERIES_STM32G4X) || \
	defined(CONFIG_SOC_SERIES_STM32H7X)
	LL_ADC_ClearFlag_ADRDY(adc);

	/*
	 * These STM32 series has one internal voltage reference source
	 * to be enabled.
	 */
	LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(adc),
				       LL_ADC_PATH_INTERNAL_VREFINT);
#endif

	adc_stm32_wait_after_calibration(dev);

	LL_ADC_Enable(adc);

	adc_stm32_wait_until_enabled(adc);

	config->irq_cfg_func();

#ifdef CONFIG_SOC_SERIES_STM32F1X
	/* Calibration of F1 must starts after two cycles after ADON is set. */
	LL_ADC_StartCalibration(adc);
	LL_ADC_REG_SetTriggerSource(adc, LL_ADC_REG_TRIG_SOFTWARE);
#endif
	adc_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static const struct adc_driver_api api_stm32_driver_api = {
	.channel_setup = adc_stm32_channel_setup,
	.read = adc_stm32_read,
#ifdef CONFIG_ADC_ASYNC
	.read_async = adc_stm32_read_async,
#endif
};

#ifdef CONFIG_ADC_STM32_DMA

#define DMA_CHANNEL_CONFIG(id, name) \
	DT_INST_DMAS_CELL_BY_NAME(id, name, channel_config)
#define DMA_FEATURES(id, name) \
	DT_INST_DMAS_CELL_BY_NAME(id, name, features)
#define DMA_CTLR(id, name) \
	DT_INST_DMAS_CTLR_BY_NAME(id, name)

#define ADC_DMA_CHANNEL_INIT(index, name, dir_cap, src_dev, dest_dev)		    \
	.dma = {								    \
		.dma_dev = DEVICE_DT_GET(DMA_CTLR(index, name)),		    \
		.channel = DT_INST_DMAS_CELL_BY_NAME(index, name, channel),	    \
		.dma_cfg = {							    \
			.dma_slot = DT_INST_DMAS_CELL_BY_NAME(index, name, slot),   \
			.channel_direction = STM32_DMA_CONFIG_DIRECTION(	    \
				DMA_CHANNEL_CONFIG(index, name)),		    \
			.source_data_size = STM32_DMA_CONFIG_##src_dev##_DATA_SIZE( \
				DMA_CHANNEL_CONFIG(index, name)),		    \
			.dest_data_size = STM32_DMA_CONFIG_##dest_dev##_DATA_SIZE(  \
				DMA_CHANNEL_CONFIG(index, name)),		    \
			.source_burst_length = 1,       /* SINGLE transfer */	    \
			.dest_burst_length = 1,         /* SINGLE transfer */	    \
			.channel_priority = STM32_DMA_CONFIG_PRIORITY(		    \
				DMA_CHANNEL_CONFIG(index, name)),		    \
			.dma_callback = dma_callback,				    \
			.block_count = 2,					    \
		},								    \
		.src_addr_increment = STM32_DMA_CONFIG_##src_dev##_ADDR_INC(	    \
			DMA_CHANNEL_CONFIG(index, name)),			    \
		.dst_addr_increment = STM32_DMA_CONFIG_##dest_dev##_ADDR_INC(	    \
			DMA_CHANNEL_CONFIG(index, name)),			    \
		.fifo_threshold = STM32_DMA_FEATURES_FIFO_THRESHOLD(		    \
			DMA_FEATURES(index, name)),				    \
	}

#define ADC_DMA_CHANNEL(id, dir, DIR, src, dest)		     \
	COND_CODE_1(DT_INST_DMAS_HAS_NAME(id, dir),		     \
		    (ADC_DMA_CHANNEL_INIT(id, dir, DIR, src, dest)), \
		    (NULL))					     \

#define IRQ_SET_STATE irq_disable

#else

#define IRQ_SET_STATE irq_enable
#define ADC_DMA_CHANNEL_INIT(index, dmamux, NULL, PERIPHERAL, MEMORY)

#endif


#define STM32_ADC_INIT(index)							\
										\
	static void adc_stm32_cfg_func_##index(void);				\
										\
	static const struct soc_gpio_pinctrl adc_pins_##index[] =		\
		ST_STM32_DT_INST_PINCTRL(index, 0);				\
										\
	static const struct adc_stm32_cfg adc_stm32_cfg_##index = {		\
		.base = (ADC_TypeDef *)DT_INST_REG_ADDR(index),			\
		.irq_cfg_func = adc_stm32_cfg_func_##index,			\
		.pclken = {							\
			.enr = DT_INST_CLOCKS_CELL(index, bits),		\
			.bus = DT_INST_CLOCKS_CELL(index, bus),			\
		},								\
		.pinctrl = adc_pins_##index,					\
		.pinctrl_len = ARRAY_SIZE(adc_pins_##index),			\
	};									\
	static struct adc_stm32_data adc_stm32_data_##index = {			\
		ADC_CONTEXT_INIT_TIMER(adc_stm32_data_##index, ctx),		\
		ADC_CONTEXT_INIT_LOCK(adc_stm32_data_##index, ctx),		\
		ADC_CONTEXT_INIT_SYNC(adc_stm32_data_##index, ctx),		\
		ADC_DMA_CHANNEL_INIT(index, dmamux, NULL, PERIPHERAL, MEMORY)	\
	};									\
										\
	DEVICE_DT_INST_DEFINE(index,						\
			      &adc_stm32_init, NULL,				\
			      &adc_stm32_data_##index, &adc_stm32_cfg_##index,	\
			      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,	\
			      &api_stm32_driver_api);				\
										\
	static void adc_stm32_cfg_func_##index(void)				\
	{									\
		IRQ_CONNECT(DT_INST_IRQN(index),				\
			    DT_INST_IRQ(index, priority),			\
			    adc_stm32_isr, DEVICE_DT_INST_GET(index), 0);	\
		IRQ_SET_STATE(DT_INST_IRQN(index));				\
	}

DT_INST_FOREACH_STATUS_OKAY(STM32_ADC_INIT)
