/*
 * Copyright (c) 2020 Libre Solar Technologies GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <kernel.h>
#include <sys/printk.h>
#include <drivers/adc.h>
#include <logging/log.h>
LOG_MODULE_REGISTER(adc_sample);

#define ADC_NUM_CHANNELS	4

/* Common settings supported by most ADCs */
#define ADC_RESOLUTION		12
#define ADC_GAIN		ADC_GAIN_1
#define ADC_REFERENCE		ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME	ADC_ACQ_TIME_DEFAULT

enum adc_action adc_callback(const struct device *dev,
						 const struct adc_sequence *sequence,
						 uint16_t sampling_index);

/* Get the numbers of channels */
static uint8_t channel_ids[ADC_NUM_CHANNELS] = { 1, 2, 3, 4 };
static int16_t sample_buffer[ADC_NUM_CHANNELS];

struct adc_channel_cfg channel_cfg = {
	.gain = ADC_GAIN,
	.reference = ADC_REFERENCE,
	.acquisition_time = ADC_ACQUISITION_TIME,
	/* channel ID will be overwritten below */
	.channel_id = 0,
	.differential = 0
};

struct adc_sequence_options options = {
	.callback = adc_callback,
};

struct adc_sequence sequence = {
	.options = &options,
	/* individual channels will be added below */
	.channels    = 0,
	.buffer      = sample_buffer,
	/* buffer size in bytes, not number of samples */
	.buffer_size = sizeof(sample_buffer),
	.resolution  = ADC_RESOLUTION,
};

enum adc_action adc_callback(const struct device *dev,
						 const struct adc_sequence *sequence,
						 uint16_t sampling_index) {

		for (uint8_t i = 0; i < ADC_NUM_CHANNELS; i++) {
			uint16_t raw_value = ((uint16_t *)sequence->buffer)[i];

				/*
				 * Convert raw reading to millivolts if driver
				 * supports reading of ADC reference voltage
				 */
				int32_t mv_value = raw_value;

				adc_raw_to_millivolts(adc_ref_internal(dev), ADC_GAIN,
					ADC_RESOLUTION, &mv_value);
				LOG_ERR("ADC ch%d reading %d mV", i, mv_value);
		}

	return ADC_ACTION_FINISH;
}

void main(void)
{
	int err;
	const struct device *dev_adc = device_get_binding(DT_LABEL(DT_NODELABEL(adc1)));

	if (!device_is_ready(dev_adc)) {
		printk("ADC device not found\n");
		return;
	}

	/*
	 * Configure channels individually prior to sampling
	 */
	for (uint8_t i = 0; i < ADC_NUM_CHANNELS; i++) {
		channel_cfg.channel_id = channel_ids[i];
#ifdef CONFIG_ADC_NRFX_SAADC
		channel_cfg.input_positive = SAADC_CH_PSELP_PSELP_AnalogInput0
					     + channel_ids[i];
#endif

		adc_channel_setup(dev_adc, &channel_cfg);

		sequence.channels |= BIT(channel_ids[i]);
	}

		/*
		 * Read sequence of channels (fails if not supported by MCU)
		 */
	while (1) {

		/* printk("sample_buffer 0x%X\n", sample_buffer); */
		err = adc_read(dev_adc, &sequence);
		if (err != 0) {
			printk("ADC reading failed with error %d.\n", err);
		}

		k_sleep(K_MSEC(1000));
		/* break; */
	}
}
