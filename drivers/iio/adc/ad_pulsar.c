// SPDX-License-Identifier: (GPL-2.0-only OR BSD-3-Clause)
/*
 * Analog Devices PulSAR ADC family driver
 *
 * Copyright 2022 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/err.h>
#include <linux/iio/buffer.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/pwm.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi-engine.h>
#include <linux/regulator/consumer.h>

#define AD_PULSAR_REG_CONFIG		0x00

#define AD4003_READ_COMMAND		0x54
#define AD4003_WRITE_COMMAND		0x14
#define AD4003_RESERVED_MSK		0xE0
#define AD4003_REG_CONFIG		AD_PULSAR_REG_CONFIG
#define  AD4003_TURBO_MODE		BIT(1)
#define  AD4003_HIGH_Z_MODE		BIT(2)

#define AD7682_NUM_TEMP_CHANNELS	0x01

#define AD7682_REG_CONFIG		AD_PULSAR_REG_CONFIG
#define AD7682_CONFIG_RESET		GENMASK(15, 0)
#define AD7682_CFG_MSK			BIT(13)
#define AD7682_POLARITY_MSK		BIT(12)
#define AD7682_PAIR_MSK			BIT(11)
#define AD7682_REF_MSK			BIT(10)
#define AD7682_SEL_MSK			GENMASK(9, 7)
#define AD7682_FILTER_MSK		BIT(6)
#define AD7682_REFBUF_MSK		GENMASK(5, 3)
#define AD7682_SEQ_MSK			GENMASK(2, 1)
#define AD7682_READBACK_MSK		BIT(0)
#define AD7682_UPDATE_CFG		AD7682_CFG_MSK
#define AD7682_NO_READBACK		AD7682_READBACK_MSK
#define AD7682_CH_POLARITY(x)		FIELD_PREP(AD7682_POLARITY_MSK, x)
#define AD7682_CH_TYPE(x)		FIELD_PREP(AD7682_PAIR_MSK, x)
#define AD7682_CH_REF(x)		FIELD_PREP(AD7682_REF_MSK, x)
#define AD7682_SEQ_SCAN(x)		FIELD_PREP(AD7682_SEQ_MSK, x)
#define AD7682_REFBUF_SEL(x)		FIELD_PREP(AD7682_REFBUF_MSK, x)
#define AD7682_BW_SEL(x)		FIELD_PREP(AD7682_FILTER_MSK, x)
#define AD7682_SEL_CH(x)		FIELD_PREP(AD7682_SEL_MSK, x)
#define AD7682_GET_FILTER(x)		FIELD_GET(AD7682_FILTER_MSK, x)
#define AD7682_SET_FILTER(x)		FIELD_PREP(AD7682_FILTER_MSK, x)

#define AD7862_SET_TYPE(reg, type)	(reg = (reg & ~AD7682_PAIR_MSK) |      \
					AD7682_CH_TYPE(type))
#define AD7682_SET_POLARITY(reg, pol)	(reg = (reg & ~AD7682_POLARITY_MSK) |  \
					AD7682_CH_POLARITY(pol))

#define AD7682_CH_TEMP_SENSOR		(AD7682_REFBUF_SEL(INT_REF_4096) |     \
	AD7682_UPDATE_CFG | AD7682_CH_TYPE(SINGLE_ENDED) | AD7682_CH_REF(GND) |\
	AD7682_CH_POLARITY(BIPOLAR))

#define AD7682_SEQ_EN_CHANNEL(i)	(AD7682_UPDATE_CFG | AD7682_SEL_CH(i) |\
	AD7682_NO_READBACK | AD7682_CH_REF(COM) | AD7682_SEQ_SCAN(DISABLED) |  \
	AD7682_CH_POLARITY(UNIPOLAR) | AD7682_CH_TYPE(SINGLE_ENDED) |	       \
	AD7682_REFBUF_SEL(INT_REF_4096) | AD7682_BW_SEL(FULL_BW))

#define AD7682_DISABLE_SEQ		(AD7682_UPDATE_CFG |		       \
	AD7682_CH_POLARITY(UNIPOLAR) | AD7682_CH_TYPE(SINGLE_ENDED) |	       \
	AD7682_CH_REF(GND) | AD7682_SEL_CH(7) | AD7682_BW_SEL(FULL_BW) |       \
	AD7682_REFBUF_SEL(EXT_REF_4) | AD7682_SEQ_SCAN(DISABLED) |	       \
	AD7682_NO_READBACK)

enum {
	ID_AD7988_5,
	ID_AD7988_1,
	ID_AD7984,
	ID_AD7983,
	ID_AD7982,
	ID_AD7980,
	ID_AD7949,
	ID_AD7946,
	ID_AD7942,
	ID_AD7699,
	ID_AD7693,
	ID_AD7691,
	ID_AD7690,
	ID_AD7689,
	ID_AD7688,
	ID_AD7687,
	ID_AD7686,
	ID_AD7685,
	ID_AD7682,
	ID_AD4022,
	ID_AD4021,
	ID_AD4020,
	ID_AD4011,
	ID_AD4007,
	ID_AD4003,
};

enum ad_pulsar_input_type {
	DIFFERENTIAL,
	SINGLE_ENDED,
};

enum ad_pulsar_input_polarity {
	BIPOLAR,
	UNIPOLAR
};

enum ad_pulsar_ch_ref {
	COM,
	GND
};

enum ad_pulsar_refbuf {
	INT_REF_2500,
	INT_REF_4096,
	EXT_REF_1,
	EXT_REF_2,
	RESERVED1,
	RESERVED2,
	EXT_REF_3,
	EXT_REF_4
};

enum ad_pulsar_sequencer_scan {
	DISABLED,
	UPDATE,
	ALL_CHANNELS_AND_TEMP,
	ALL_CHANNELS
};

enum ad_pulsar_filter_bw {
	QUARTER_BW,
	FULL_BW
};

unsigned int ad_pulsar_filter_freq[] = {
	[QUARTER_BW] = 425000,
	[FULL_BW] = 1700000
};

struct ad_pulsar_chip_info {
	enum ad_pulsar_input_type input_type;
	const char *name;
	int num_channels;
	int resolution;
	bool sequencer;
	int sclk_rate;
	int max_rate;
};

static const struct ad_pulsar_chip_info ad_pulsar_chip_infos[] = {
	[ID_AD7988_5] = {
		.name = "ad7988-5",
		.input_type = SINGLE_ENDED,
		.max_rate = 500000,
		.resolution = 16,
		.num_channels = 1,
		.sclk_rate = 80000000
	},
	[ID_AD7988_1] = {
		.name = "ad7988-1",
		.input_type = SINGLE_ENDED,
		.max_rate = 100000,
		.resolution = 16,
		.num_channels = 1,
		.sclk_rate = 80000000
	},
	[ID_AD7984] = {
		.name = "ad7984",
		.input_type = DIFFERENTIAL,
		.max_rate = 1333333,
		.resolution = 18,
		.num_channels = 1,
		.sclk_rate = 80000000
	},
	[ID_AD7983] = {
		.name = "ad7983",
		.input_type = SINGLE_ENDED,
		.max_rate = 1333333,
		.resolution = 16,
		.num_channels = 1,
		.sclk_rate = 80000000
	},
	[ID_AD7982] = {
		.name = "ad7982",
		.input_type = DIFFERENTIAL,
		.max_rate = 1000000,
		.resolution = 18,
		.num_channels = 1,
		.sclk_rate = 80000000
	},
	[ID_AD7980] = {
		.name = "ad7980",
		.input_type = SINGLE_ENDED,
		.max_rate = 1000000,
		.resolution = 16,
		.num_channels = 1,
		.sclk_rate = 80000000
	},
	[ID_AD7949] = {
		.name = "ad7949",
		.input_type = SINGLE_ENDED,
		.max_rate = 250000,
		.resolution = 14,
		.num_channels = 8 + AD7682_NUM_TEMP_CHANNELS,
		.sclk_rate = 40000000,
		.sequencer = true,
	},
	[ID_AD7946] = {
		.name = "ad7946",
		.input_type = SINGLE_ENDED,
		.max_rate = 500000,
		.resolution = 14,
		.num_channels = 1,
		.sclk_rate = 40000000
	},
	[ID_AD7942] = {
		.name = "ad7942",
		.input_type = SINGLE_ENDED,
		.max_rate = 250000,
		.resolution = 14,
		.num_channels = 1,
		.sclk_rate = 40000000
	},
	[ID_AD7699] = {
		.name = "ad7699",
		.input_type = SINGLE_ENDED,
		.max_rate = 500000,
		.resolution = 16,
		.num_channels = 8 + AD7682_NUM_TEMP_CHANNELS,
		.sclk_rate = 40000000,
		.sequencer = true,
	},
	[ID_AD7693] = {
		.name = "ad7693",
		.input_type = DIFFERENTIAL,
		.max_rate = 500000,
		.resolution = 16,
		.num_channels = 1,
		.sclk_rate = 40000000
	},
	[ID_AD7691] = {
		.name = "ad7691",
		.input_type = DIFFERENTIAL,
		.max_rate = 250000,
		.resolution = 18,
		.num_channels = 1,
		.sclk_rate = 40000000
	},
	[ID_AD7690] = {
		.name = "ad7690",
		.input_type = DIFFERENTIAL,
		.max_rate = 400000,
		.resolution = 18,
		.num_channels = 1,
		.sclk_rate = 40000000
	},
	[ID_AD7689] = {
		.name = "ad7689",
		.input_type = SINGLE_ENDED,
		.max_rate = 250000,
		.resolution = 16,
		.num_channels = 8 + AD7682_NUM_TEMP_CHANNELS,
		.sclk_rate = 40000000,
		.sequencer = true,
	},
	[ID_AD7688] = {
		.name = "ad7688",
		.input_type = DIFFERENTIAL,
		.max_rate = 500000,
		.resolution = 16,
		.num_channels = 1,
		.sclk_rate = 40000000
	},
	[ID_AD7687] = {
		.name = "ad7687",
		.input_type = DIFFERENTIAL,
		.max_rate = 250000,
		.resolution = 16,
		.num_channels = 1,
		.sclk_rate = 40000000
	},
	[ID_AD7686] = {
		.name = "ad7686",
		.input_type = SINGLE_ENDED,
		.max_rate = 500000,
		.resolution = 16,
		.num_channels = 1,
		.sclk_rate = 40000000
	},
	[ID_AD7685] = {
		.name = "ad7685",
		.input_type = SINGLE_ENDED,
		.max_rate = 250000,
		.resolution = 16,
		.num_channels = 1,
		.sclk_rate = 40000000
	},
	[ID_AD7682] = {
		.name = "ad7682",
		.input_type = SINGLE_ENDED,
		.max_rate = 250000,
		.resolution = 16,
		.num_channels = 4 + AD7682_NUM_TEMP_CHANNELS,
		.sclk_rate = 40000000,
		.sequencer = true,
	},
	[ID_AD4022] = {
		.name = "ad4022",
		.input_type = DIFFERENTIAL,
		.max_rate = 500000,
		.resolution = 20,
		.num_channels = 1,
		.sclk_rate = 80000000,
	},
	[ID_AD4021] = {
		.name = "ad4021",
		.input_type = DIFFERENTIAL,
		.max_rate = 1000000,
		.resolution = 20,
		.num_channels = 1,
		.sclk_rate = 80000000,
	},
	[ID_AD4020] = {
		.name = "ad4020",
		.input_type = DIFFERENTIAL,
		.max_rate = 1800000,
		.resolution = 20,
		.num_channels = 1,
		.sclk_rate = 80000000,
	},
	[ID_AD4011] = {
		.name = "ad4011",
		.input_type = DIFFERENTIAL,
		.max_rate = 500000,
		.resolution = 18,
		.num_channels = 1,
		.sclk_rate = 80000000,
	},
	[ID_AD4007] = {
		.name = "ad4007",
		.input_type = DIFFERENTIAL,
		.max_rate = 1000000,
		.resolution = 18,
		.num_channels = 1,
		.sclk_rate = 80000000,
	},
	[ID_AD4003] = {
		.name = "ad4003",
		.input_type = DIFFERENTIAL,
		// .max_rate = 2000000,
		// HDL does not support maximum rate
		.max_rate = 1839080,
		.resolution = 18,
		.num_channels = 1,
		.sclk_rate = 80000000,
	}
};

struct ad_pulsar_adc {
	const struct ad_pulsar_chip_info *info;
	struct iio_chan_spec *channels;
	struct spi_transfer *seq_xfer;
	unsigned long ref_clk_rate;
	struct pwm_device *cnv;
	struct spi_device *spi;
	struct regulator *vref;
	unsigned int *seq_buf;
	int spi_speed_hz;
	int samp_freq;
	int device_id;
};

static int ad_pulsar_reg_write(struct ad_pulsar_adc *adc, unsigned int reg,
			       unsigned int val)
{
	struct spi_transfer xfer = {
		.bits_per_word = adc->info->resolution,
		.speed_hz = adc->info->sclk_rate,
		.len = 4,
	};
	unsigned int tx;

	tx = val << 2;
	xfer.tx_buf = &tx;

	return spi_sync_transfer(adc->spi, &xfer, 1);
}

static int ad_pulsar_reg_read(struct ad_pulsar_adc *adc, unsigned int reg,
			      unsigned int *val)
{
	struct spi_transfer xfer = {
		.bits_per_word = adc->info->resolution,
		.speed_hz = adc->info->sclk_rate,
		.len = 4,
	};
	unsigned int rx, tx;
	int ret;

	tx = reg << 2;
	xfer.tx_buf = &tx;
	xfer.rx_buf = &rx;

	ret = spi_sync_transfer(adc->spi, &xfer, 1);
	if (ret < 0)
		return ret;

	*val = rx & GENMASK(15, 0);

	return ret;
}

static int ad_pulsar_reg_access(struct iio_dev *indio_dev, unsigned int reg,
				unsigned int writeval, unsigned int *readval)
{
	struct ad_pulsar_adc *adc = iio_priv(indio_dev);

	if (readval)
		return ad_pulsar_reg_read(adc, reg, readval);

	return ad_pulsar_reg_write(adc, reg, writeval);
}

static int ad_pulsar_set_samp_freq(struct ad_pulsar_adc *adc, int freq)
{
	unsigned long long target, ref_clk_period_ps;
	struct pwm_state cnv_state;
	int ret;

	freq = clamp(freq, 0, adc->info->max_rate);
	target = DIV_ROUND_CLOSEST_ULL(adc->ref_clk_rate, freq);
	ref_clk_period_ps = DIV_ROUND_CLOSEST_ULL(1000000000000,
						  adc->ref_clk_rate);
	cnv_state.period = ref_clk_period_ps * target;
	cnv_state.duty_cycle = ref_clk_period_ps;
	cnv_state.phase = ref_clk_period_ps;
	cnv_state.time_unit = PWM_UNIT_PSEC;
	cnv_state.enabled = true;
	ret = pwm_apply_state(adc->cnv, &cnv_state);
	if (ret < 0)
		return ret;

	adc->samp_freq = DIV_ROUND_CLOSEST_ULL(adc->ref_clk_rate, target);

	return ret;
}

static int ad_pulsar_set_lpf(struct ad_pulsar_adc *adc, int index,
			     enum ad_pulsar_filter_bw filter)
{
	switch (adc->device_id) {
	case ID_AD7949:
	case ID_AD7699:
	case ID_AD7682:
	case ID_AD7689:
		adc->seq_buf[index] &= ~AD7682_FILTER_MSK;
		adc->seq_buf[index] |= AD7682_SET_FILTER(filter);

		return 0;
	default:
		return -EINVAL;
	}
}

static int ad_pulsar_get_lpf(struct ad_pulsar_adc *adc, int index, int *val)
{
	if (!adc->info->sequencer)
		return -EINVAL;

	*val = ad_pulsar_filter_freq[AD7682_GET_FILTER(adc->seq_buf[index])];

	return 0;
}

static int ad_pulsar_read_channel(struct iio_dev *indio_dev,
				  const struct iio_chan_spec *chan, int *val)
{
	struct ad_pulsar_adc *adc = iio_priv(indio_dev);
	unsigned int temp;
	int ret, i;

	temp = chan->address;

	for (i = 0; i < 4; i++) {
		ret = ad_pulsar_reg_read(adc, temp, val);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int ad_pulsar_read_raw(struct iio_dev *indio_dev,
			      const struct iio_chan_spec *chan,
			      int *val, int *val2, long info)
{
	struct ad_pulsar_adc *adc = iio_priv(indio_dev);
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		ret = ad_pulsar_read_channel(indio_dev, chan, val);
		if (ret < 0)
			return ret;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = adc->samp_freq;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_VOLTAGE:
			ret = regulator_get_voltage(adc->vref);
			if (ret < 0)
				return ret;
			*val = ret / 1000;
			*val2 = adc->info->resolution;

			return IIO_VAL_FRACTIONAL_LOG2;
		case IIO_TEMP:
			*val = 8933;
			*val2 = 1000;

			return IIO_VAL_FRACTIONAL;
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:
		ret = ad_pulsar_get_lpf(adc, chan->address, val);
		if (ret < 0)
			return ret;

		return IIO_VAL_INT;

	default:
		return -EINVAL;
	}
}

static int ad_pulsar_write_raw(struct iio_dev *indio_dev,
			       const struct iio_chan_spec *chan,
			       int val, int val2, long info)
{
	struct ad_pulsar_adc *adc = iio_priv(indio_dev);

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		return ad_pulsar_set_samp_freq(adc, val);
	case IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:
		return ad_pulsar_set_lpf(adc, chan->scan_index, val);
	default:
		return -EINVAL;
	}
}

static int ad_pulsar_read_avail(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				const int **vals, int *type, int *length,
				long info)
{
	switch (info) {
	case IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:
		*vals = ad_pulsar_filter_freq;
		*length = ARRAY_SIZE(ad_pulsar_filter_freq);
		*type = IIO_VAL_INT;
		return IIO_AVAIL_LIST;
	default:
		return -EINVAL;
	}
}

static int ad_pulsar_buffer_preenable(struct iio_dev *indio_dev)
{
	struct ad_pulsar_adc *adc = iio_priv(indio_dev);
	unsigned int spi_tx_data = 0xFFFFFFFF;
	int ret, ch, first, second, last, max_freq;
	unsigned int freq, num_en_ch;
	unsigned int spi_rx_data;
	struct spi_transfer xfer = {
		.tx_buf = &spi_tx_data,
		.rx_buf = &spi_rx_data,
		.len = 3,
		.bits_per_word = adc->info->resolution,
		.speed_hz = adc->info->sclk_rate,
	};
	struct spi_message msg;

	if (adc->info->sequencer) {
		num_en_ch = hweight_long(*indio_dev->active_scan_mask);

		last = find_last_bit(indio_dev->active_scan_mask,
				     indio_dev->masklength);
		if (num_en_ch > 2) {
			first = find_first_bit(indio_dev->active_scan_mask,
					       indio_dev->masklength);
			second = find_next_bit(indio_dev->active_scan_mask,
					       indio_dev->masklength,
					       first + 1);
		}
		spi_message_init(&msg);

		spi_tx_data = 0;
		for_each_set_bit(ch, indio_dev->active_scan_mask,
				 indio_dev->masklength) {
			adc->seq_xfer[ch].cs_change = 1;
			adc->seq_xfer[ch].word_delay.value = 2;
			adc->seq_xfer[ch].word_delay.unit = SPI_DELAY_UNIT_USECS;
			if (num_en_ch > 2 && (ch == first || ch == second))
				continue;
			if (ch == last && num_en_ch <= 2) {
				adc->seq_xfer[ch].cs_change = 0;
				adc->seq_xfer[ch].word_delay.value = 0;
			}
			spi_message_add_tail(&adc->seq_xfer[ch], &msg);
		}
		if (num_en_ch > 2) {
			adc->seq_xfer[second].cs_change = 0;
			adc->seq_xfer[second].word_delay.value = 0;
			spi_message_add_tail(&adc->seq_xfer[first], &msg);
			spi_message_add_tail(&adc->seq_xfer[second], &msg);
		}
		max_freq = adc->info->max_rate / num_en_ch;
		freq = clamp(adc->samp_freq, 0, max_freq);
		ad_pulsar_set_samp_freq(adc, freq);
		ad_pulsar_reg_write(adc, AD7682_REG_CONFIG,
				    adc->seq_buf[first]);
		if (num_en_ch > 1) {
			ret = ad_pulsar_reg_write(adc, AD7682_REG_CONFIG,
						  adc->seq_buf[second]);
			if (ret)
				return ret;
		} else {
			ret = ad_pulsar_reg_write(adc, AD7682_REG_CONFIG,
						  adc->seq_buf[first]);
			if (ret)
				return ret;
		}
	} else {
		spi_message_init_with_transfers(&msg, &xfer, 1);
	}

	ret = spi_engine_offload_load_msg(adc->spi, &msg);
	if (ret < 0)
		return ret;
	spi_engine_offload_enable(adc->spi, true);

	return ret;
}

static int ad_pulsar_buffer_postdisable(struct iio_dev *indio_dev)
{
	struct ad_pulsar_adc *adc = iio_priv(indio_dev);
	int ret;

	spi_engine_offload_enable(adc->spi, false);
	spi_bus_unlock(adc->spi->master);

	ret = ad_pulsar_reg_write(adc, AD7682_REG_CONFIG, AD7682_DISABLE_SEQ);
	if (ret)
		return ret;

	return ad_pulsar_reg_write(adc, AD7682_REG_CONFIG, AD7682_DISABLE_SEQ);
}

static int ad_pulsar_dma_submit(struct iio_dma_buffer_queue *queue,
				struct iio_dma_buffer_block *block)
{
	return iio_dmaengine_buffer_submit_block(queue, block, DMA_DEV_TO_MEM);
}

static const struct iio_dma_buffer_ops ad_pulsar_dma_buffer_ops = {
	.submit = ad_pulsar_dma_submit,
	.abort = iio_dmaengine_buffer_abort,
};

static const struct iio_buffer_setup_ops ad_pulsar_buffer_ops = {
	.preenable = &ad_pulsar_buffer_preenable,
	.postdisable = &ad_pulsar_buffer_postdisable,
};

static const struct iio_info ad_pulsar_iio_info = {
	.read_raw = &ad_pulsar_read_raw,
	.write_raw = &ad_pulsar_write_raw,
	.read_avail = &ad_pulsar_read_avail,
	.debugfs_reg_access = &ad_pulsar_reg_access,
};

static int ad_pulsar_spi_init(struct ad_pulsar_adc *adc)
{
	int ret = 0;

	switch (adc->device_id) {
	case ID_AD4022:
	case ID_AD4021:
	case ID_AD4020:
	case ID_AD4011:
	case ID_AD4007:
	case ID_AD4003:
		ret = ad_pulsar_reg_write(adc, AD4003_REG_CONFIG,
					  AD4003_TURBO_MODE);
		break;
	case ID_AD7949:
	case ID_AD7699:
	case ID_AD7682:
	case ID_AD7689:
		ret = ad_pulsar_reg_write(adc, AD7682_REG_CONFIG,
					  AD7682_CONFIG_RESET);
		if (ret < 0)
			return ret;

		ret = ad_pulsar_reg_write(adc, AD7682_REG_CONFIG,
					  AD7682_CONFIG_RESET);
		break;
	case ID_AD7988_5:
	case ID_AD7988_1:
	case ID_AD7984:
	case ID_AD7983:
	case ID_AD7982:
	case ID_AD7980:
	case ID_AD7946:
	case ID_AD7942:
	case ID_AD7693:
	case ID_AD7691:
	case ID_AD7690:
	case ID_AD7688:
	case ID_AD7687:
	case ID_AD7686:
	case ID_AD7685:
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static const struct of_device_id ad_pulsar_of_match[] = {
	{ .compatible = "adi,pulsar,ad7988-5", .data = (void *)ID_AD7988_5 },
	{ .compatible = "adi,pulsar,ad7988-1", .data = (void *)ID_AD7988_1 },
	{ .compatible = "adi,pulsar,ad7984", .data = (void *)ID_AD7984   },
	{ .compatible = "adi,pulsar,ad7983", .data = (void *)ID_AD7983   },
	{ .compatible = "adi,pulsar,ad7982", .data = (void *)ID_AD7982   },
	{ .compatible = "adi,pulsar,ad7980", .data = (void *)ID_AD7980   },
	{ .compatible = "adi,pulsar,ad7946", .data = (void *)ID_AD7946   },
	{ .compatible = "adi,pulsar,ad7942", .data = (void *)ID_AD7942   },
	{ .compatible = "adi,pulsar,ad7693", .data = (void *)ID_AD7693   },
	{ .compatible = "adi,pulsar,ad7691", .data = (void *)ID_AD7691   },
	{ .compatible = "adi,pulsar,ad7690", .data = (void *)ID_AD7690   },
	{ .compatible = "adi,pulsar,ad7689", .data = (void *)ID_AD7689   },
	{ .compatible = "adi,pulsar,ad7688", .data = (void *)ID_AD7688   },
	{ .compatible = "adi,pulsar,ad7687", .data = (void *)ID_AD7687   },
	{ .compatible = "adi,pulsar,ad7686", .data = (void *)ID_AD7686   },
	{ .compatible = "adi,pulsar,ad7685", .data = (void *)ID_AD7685   },
	{ .compatible = "adi,pulsar,ad4022", .data = (void *)ID_AD4022   },
	{ .compatible = "adi,pulsar,ad4021", .data = (void *)ID_AD4021   },
	{ .compatible = "adi,pulsar,ad4020", .data = (void *)ID_AD4020   },
	{ .compatible = "adi,pulsar,ad4011", .data = (void *)ID_AD4011   },
	{ .compatible = "adi,pulsar,ad4007", .data = (void *)ID_AD4007   },
	{ .compatible = "adi,pulsar,ad4003", .data = (void *)ID_AD4003   },
	{ .compatible = "adi,pulsar,adaq4003", .data = (void *)ID_AD4003   },
	{ },
};
MODULE_DEVICE_TABLE(of, ad_pulsar_of_match);

static const struct spi_device_id ad_pulsar_spi_id[] = {
	{ "adi,pulsar,ad7988-5", ID_AD7988_5 },
	{ "adi,pulsar,ad7988-1", ID_AD7988_1 },
	{ "adi,pulsar,ad7984", ID_AD7984 },
	{ "adi,pulsar,ad7983", ID_AD7983 },
	{ "adi,pulsar,ad7982", ID_AD7982 },
	{ "adi,pulsar,ad7980", ID_AD7980 },
	{ "adi,pulsar,ad7946", ID_AD7946 },
	{ "adi,pulsar,ad7942", ID_AD7942 },
	{ "adi,pulsar,ad7693", ID_AD7693 },
	{ "adi,pulsar,ad7691", ID_AD7691 },
	{ "adi,pulsar,ad7690", ID_AD7690 },
	{ "adi,pulsar,ad7689", ID_AD7689 },
	{ "adi,pulsar,ad7688", ID_AD7688 },
	{ "adi,pulsar,ad7687", ID_AD7687 },
	{ "adi,pulsar,ad7686", ID_AD7686 },
	{ "adi,pulsar,ad7685", ID_AD7685 },
	{ "adi,pulsar,ad4022", ID_AD4022 },
	{ "adi,pulsar,ad4021", ID_AD4021 },
	{ "adi,pulsar,ad4020", ID_AD4020 },
	{ "adi,pulsar,ad4011", ID_AD4011 },
	{ "adi,pulsar,ad4007", ID_AD4007 },
	{ "adi,pulsar,ad4003", ID_AD4003 },
	{ "adi,pulsar,adaq4003", ID_AD4003 },
	{ }

};
MODULE_DEVICE_TABLE(spi, ad_pulsar_spi_id);

static void ad_pulsar_set_channel(struct iio_chan_spec *ch, int i, int num_ch,
				  const struct ad_pulsar_chip_info *info)
{
	ch[i].type = IIO_VOLTAGE;
	ch[i].indexed = 1;
	ch[i].channel = i;
	ch[i].scan_index = i;
	ch[i].info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ);
	ch[i].info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				   BIT(IIO_CHAN_INFO_SCALE) |
				   BIT(IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY);
	ch[i].info_mask_shared_by_all_available = BIT(IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY);
	ch[i].scan_type.sign = 'u';
	ch[i].scan_type.storagebits = 32;
	ch[i].scan_type.realbits = info->resolution;
}

static int ad_pulsar_parse_channels(struct iio_dev *indio_dev)
{
	struct ad_pulsar_adc *adc = iio_priv(indio_dev);
	struct device *dev = indio_dev->dev.parent;
	struct fwnode_handle *child;
	unsigned int dummy;
	unsigned int in[2];
	int num_ch, ret, i;

	num_ch = device_get_child_node_count(dev);
	if (num_ch > adc->info->num_channels)
		return -EINVAL;

	if (!num_ch)
		num_ch = adc->info->num_channels;

	adc->channels = devm_kzalloc(dev,
				     num_ch * sizeof(struct iio_chan_spec),
				     GFP_KERNEL);
	if (!adc->channels)
		return -ENOMEM;

	for (i = 0; i < num_ch; i++)
		ad_pulsar_set_channel(adc->channels, i, num_ch, adc->info);

	indio_dev->channels = adc->channels;
	indio_dev->num_channels = num_ch;

	if (!adc->info->sequencer)
		return 0;

	adc->seq_buf = devm_kzalloc(dev,
				    num_ch * sizeof(unsigned short),
				    GFP_KERNEL);
	if (!adc->seq_buf)
		return -ENOMEM;

	adc->seq_xfer = devm_kzalloc(dev,
				     num_ch * sizeof(struct spi_transfer),
				     GFP_KERNEL);

	for (i = 0; i < num_ch; i++) {
		adc->seq_xfer[i].tx_buf = &adc->seq_buf[i];
		adc->seq_xfer[i].rx_buf = &dummy;
		adc->seq_xfer[i].len = 1;
		adc->seq_xfer[i].bits_per_word = adc->info->resolution;
		adc->seq_xfer[i].speed_hz = adc->info->sclk_rate;
	}

	i = 0;
	device_for_each_child_node(dev, child) {
		in[0] = 0;
		in[1] = 0;
		if (fwnode_property_present(child, "diff-channels")) {
			ret = fwnode_property_read_u32_array(child,
							     "diff-channels",
							     in, 2);
			if (ret < 0)
				return ret;

			if (in[0] > 7 || in[1] > 7 || in[0] % 2 != 0 ||
			    (in[0] % 2 == 0 && in[1] != in[0] + 1))
				return -EINVAL;

			adc->seq_buf[i] = AD7682_SEQ_EN_CHANNEL(in[0]);
			AD7862_SET_TYPE(adc->seq_buf[i], DIFFERENTIAL);
			adc->channels[i].differential = 1;
			adc->channels[i].channel2 = in[1];
		} else if (fwnode_property_read_bool(child, "temp-sensor")) {
			adc->seq_buf[i] = AD7682_CH_TEMP_SENSOR;
			adc->channels[i].type = IIO_TEMP;
			adc->channels[i].indexed = 0;
		} else {
			ret = fwnode_property_read_u32(child, "reg", &in[0]);
			if (ret < 0)
				return ret;

			adc->seq_buf[i] = AD7682_SEQ_EN_CHANNEL(in[0]);
			if (in[0] > adc->info->num_channels)
				return -EINVAL;
		}

		if (fwnode_property_read_bool(child, "bipolar")) {
			adc->channels[i].scan_type.sign = 's';
			AD7682_SET_POLARITY(adc->seq_buf[i], BIPOLAR);
		}
		adc->channels[i].channel = in[0];
		adc->channels[i].scan_index = i;
		adc->channels[i].address = adc->seq_buf[i];
		i++;
	}
	for (i = 0; i < num_ch; i++)
		adc->seq_buf[i] = adc->seq_buf[i] << 2;

	indio_dev->num_channels = i;

	return 0;
}

static void ad_pulsar_reg_disable(void *data)
{
	regulator_disable(data);
}

static void ad_pulsar_pwm_diasble(void *data)
{
	pwm_disable(data);
}

static void ad_pulsar_clk_disable(void *data)
{
	clk_disable_unprepare(data);
}

static int ad_pulsar_probe(struct spi_device *spi)
{
	struct ad_pulsar_adc *adc;
	struct iio_buffer *buffer;
	struct iio_dev *indio_dev;
	struct clk *ref_clk;
	const void *id;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev,
					  sizeof(struct ad_pulsar_adc));
	if (!indio_dev)
		return -ENOMEM;

	adc = iio_priv(indio_dev);
	adc->spi = spi;

	adc->vref = devm_regulator_get(&spi->dev, "vref");
	if (IS_ERR(adc->vref))
		return PTR_ERR(adc->vref);

	ret = regulator_enable(adc->vref);
	if (ret) {
		dev_err(&spi->dev, "Failed to enable VREF regulator");
		return ret;
	}

	ret = devm_add_action_or_reset(&spi->dev, ad_pulsar_reg_disable,
				       adc->vref);
	if (ret)
		return ret;

	ref_clk = devm_clk_get(&spi->dev, "ref_clk");
	if (IS_ERR(ref_clk))
		return PTR_ERR(ref_clk);

	ret = clk_prepare_enable(ref_clk);
	if (ret < 0)
		return ret;

	ret = devm_add_action_or_reset(&spi->dev, ad_pulsar_clk_disable,
				       ref_clk);
	if (ret < 0)
		return ret;

	adc->ref_clk_rate = clk_get_rate(ref_clk);

	adc->cnv = devm_pwm_get(&spi->dev, "cnv");
	if (IS_ERR(adc->cnv))
		return PTR_ERR(adc->cnv);

	ret = devm_add_action_or_reset(&spi->dev, ad_pulsar_pwm_diasble,
				       adc->cnv);
	if (ret < 0)
		return ret;

	id = device_get_match_data(&spi->dev);
	if (!id)
		return -EINVAL;
	adc->device_id = (int)id;

	adc->info = &ad_pulsar_chip_infos[adc->device_id];

	ret = ad_pulsar_parse_channels(indio_dev);
	if (ret < 0)
		return ret;

	ret = ad_pulsar_spi_init(adc);
	if (ret < 0)
		return ret;

	indio_dev->name = adc->info->name;
	indio_dev->info = &ad_pulsar_iio_info;
	indio_dev->modes = INDIO_BUFFER_HARDWARE | INDIO_DIRECT_MODE;
	indio_dev->setup_ops = &ad_pulsar_buffer_ops;

	buffer = devm_iio_dmaengine_buffer_alloc(indio_dev->dev.parent,
						 "rx",
						 &ad_pulsar_dma_buffer_ops,
						 indio_dev);
	if (IS_ERR(buffer))
		return PTR_ERR(buffer);

	iio_device_attach_buffer(indio_dev, buffer);

	ret = ad_pulsar_set_samp_freq(adc, adc->info->max_rate);
	if (ret < 0)
		return ret;

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static struct spi_driver ad_pulsar_driver = {
	.driver = {
		.name = "pulsar_adc",
		.of_match_table = ad_pulsar_of_match,
	},
	.probe = ad_pulsar_probe,
	.id_table = ad_pulsar_spi_id,
};
module_spi_driver(ad_pulsar_driver);

MODULE_AUTHOR("Sergiu Cuciurean <sergiu.cuciurean@analog.com>");
MODULE_DESCRIPTION("Analog Devices PulSAR ADC family driver");
MODULE_LICENSE("GPL v2");
