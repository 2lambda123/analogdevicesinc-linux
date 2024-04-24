// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2024 Analog Devices, Inc.
 * Author: Radu Sabau <radu.sabau@analog.com>
 */

#include <asm/unaligned.h>
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/component.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
// #include <linux/spi/spi-engine.h>
#include <linux/util_macros.h>
#include <linux/units.h>
#include <linux/types.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>

#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

#define AD4692_NUM_REGULATORS			1
#define AD4692_MAX_ADC_MODE			4
#define AD4692_GP_MODE_NUM			4

#define AD4692_VREF_MIN				2400000
#define AD4692_VREF_MAX				5250000

#define AD4692_CONV_PERIOD_NS			380
#define AD4692_OSC_PERIOD_NS(x)			DIV_ROUND_CLOSEST_ULL(NANO, x)						

//define
#define AD4692_IS_STATUS_REG(reg)		((reg) & BIT(0))

//define
#define AD4692_ADC_MODE_MASK			GENMASK(1, 0)
#define AD4692_CONV_START_MASK			BIT(0)
#define AD4692_STOP_STATE_MASK			BIT(0)
#define AD4692_MANUAL_MODE_MASK			BIT(0)

#define AD4692_COMMAND_MASK			GENMASK(7, 3)
#define AD4692_ADC_DATA_14_LSB_MASK		GENMASK(7, 2)
#define AD4692_ADC_DATA_18_LSB_MASK		GENMASK(7, 6)
#define AD4692_CHAN_MASK			GENMASK(3, 0)
#define AD4692_OV_ERR_MASK			BIT(4)

#define AD4692_NOOP				0x00
#define AD4692_EXIT_COMMAND			0x0A
#define AD4692_TEMPERATURE_SENSOR		0x0F
#define AD4692_ADC_CHAN(ch)			(0x10 + (ch))

//define
#define AD4692_CFG_REG				0x50
#define AD4692_IN_ACC(chan)			(0x40 + (chan))

//gpio
#define AD4692_GPIO_STATE(offset)		BIT(offset)
#define AD4692_GPIO_STATE_REG			0x30
// #define AD4692_GPIO_MODE_REG			0x00
#define AD4692_GPIO_STATE_REG			0x20
#define AD4692_GPIO_STATE_EN_REG		0x60
#define AD4692_GPIO_STATE_EN(offset)		BIT(offset)
#define AD4692_GPIO_DIR_REG			0x70
#define AD4692_GPIO_DIR(offset)			BIT(offset)

#define AD4692_CHANNEL(index, real_bits, storage_bits)			\
	{								\
		.type = IIO_VOLTAGE,					\
		.indexed = 1,						\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ)	\
					   | BIT(IIO_CHAN_INFO_SCALE),	\
		.channel = index,					\
		.scan_index = index,					\
		.scan_type = {						\
			.sign = 's',					\
			.realbits = real_bits,				\
			.storagebits = storage_bits,			\
		},							\
	}

#define AD4694_CHANNELS(real_bits, storage_bits)	\
	AD4692_CHANNEL(0, real_bits, storage_bits),	\
	AD4692_CHANNEL(1, real_bits, storage_bits),	\
	AD4692_CHANNEL(2, real_bits, storage_bits),	\
	AD4692_CHANNEL(3, real_bits, storage_bits),	\
	AD4692_CHANNEL(4, real_bits, storage_bits),	\
	AD4692_CHANNEL(5, real_bits, storage_bits),	\
	AD4692_CHANNEL(6, real_bits, storage_bits),	\
	AD4692_CHANNEL(7, real_bits, storage_bits)

#define AD4692_CHANNELS(real_bits, storage_bits)	\
	AD4694_CHANNELS(real_bits, storage_bits),	\
	AD4692_CHANNEL(8, real_bits, storage_bits),	\
	AD4692_CHANNEL(9, real_bits, storage_bits),	\
	AD4692_CHANNEL(10, real_bits, storage_bits),	\
	AD4692_CHANNEL(11, real_bits, storage_bits),	\
	AD4692_CHANNEL(12, real_bits, storage_bits),	\
	AD4692_CHANNEL(13, real_bits, storage_bits),	\
	AD4692_CHANNEL(14, real_bits, storage_bits),	\
	AD4692_CHANNEL(15, real_bits, storage_bits)

enum ad4692_ids {
	ID_AD4692,
	ID_AD4691,
	ID_AD4694,
	ID_AD4693
};

enum ad4692_adc_mode {
	AD4692_CNV_CLOCK_MODE,
	AD4692_CNV_BURST_MODE,
	AD4692_AUTONOMOUS_MODE,
	AD4692_SPI_BURST_MODE,
	AD4692_MANUAL_MODE,
};

enum ad4692_gpio_mode {
	AD4692_HIGH_Z,
	AD4692_DIGITAL_OUTPUT_LOW,
	AD4692_DIGITAL_OUTPUT_LOGIC_HIGH,
	AD4692_DIGITAL_INPUT,
	AD4692_ADC_BUSY,
	AD4692_SEQ_DONE,
	AD4692_DATA_READY,
	AD4692_ACC_OVR_ERROR,
	AD4692_ACC_SAT_ERROR,
};

static int ad4692_resolutions[4] = {
	14, 16, 18, 20
};

struct ad4692_chip_info {
	struct iio_chan_spec *channels;
	const char *name;
	unsigned int num_channels;
	unsigned int resolution;
	unsigned int max_rate;
};

static const struct iio_chan_spec ad4691_channels[] = {
	AD4692_CHANNELS(18, 32)
};

static const struct iio_chan_spec ad4692_channels[] = {
	AD4692_CHANNELS(16, 16)
};

static const struct iio_chan_spec ad4693_channels[] = {
	AD4694_CHANNELS(14, 16)
};

static const struct iio_chan_spec ad4694_channels[] = {
	AD4694_CHANNELS(20, 32)
};

static const struct ad4692_chip_info ad4692_chips[] =  {
	[ID_AD4692] = {
		.channels = ad4692_channels,
		.name = "ad4692",
		.num_channels = ARRAY_SIZE(ad4692_channels),
		.resolution = 16,
		.max_rate = 1000000,
	},
	[ID_AD4691] = {
		.channels = ad4691_channels,
		.name = "ad4691",
		.num_channels = ARRAY_SIZE(ad4692_channels),
		.resolution = 18,
		.max_rate = 500000,
	},
	[ID_AD4694] = {
		.channels = ad4694_channels,
		.name = "ad4694",
		.num_channels = ARRAY_SIZE(ad4694_channels),
		.resolution = 14,
		.max_rate = 1000000,
	},
	[ID_AD4693] = {
		.channels = ad4693_channels,
		.name = "ad4693",
		.num_channels = ARRAY_SIZE(ad4694_channels),
		.resolution = 20,
		.max_rate = 500000,
	},
};

struct ad4692_state {
	const struct ad4692_chip_info	*chip;
	struct gpio_chip		gpiochip;
	struct regmap			*regmap;
	struct spi_device		*spi;
	struct pwm_device		*conv_trigger;
	struct gpio_desc		*reset_gpio;
	struct gpio_desc		*gpio_0;
	struct gpio_descs		*gpio_mode;
	struct regulator_bulk_data	regulators[AD4692_NUM_REGULATORS];

	struct iio_trigger		*trig;

	enum ad4692_adc_mode		adc_mode;

	int vio;
	int vref;
	/*
	 * Synchronize access to members of the driver state, and ensure
	 * atomicity of consecutive regmap operations.
	 */
	struct mutex			lock;

	struct spi_message		msg;
	struct spi_transfer		xfer;
	int max_rate;

	u8 tx_data[5];
	u8 rx_data[5];
};

static void ad4692_disable_regulators(void *data)
{
	struct ad4692_state *st = data;

	regulator_bulk_disable(AD4692_NUM_REGULATORS, st->regulators);
}

static void ad4692_disable_pwm(void *data)
{
	struct ad4692_state *st = data;

	pwm_disable(st->conv_trigger);
}

static int ad4692_regulators_get(struct ad4692_state *st)
{
	struct device *dev = &st->spi->dev;
	struct regulator *ref;
	int ret;

	st->regulators[0].supply = "vio";

	ret = devm_regulator_bulk_get(dev, AD4692_NUM_REGULATORS,
				      st->regulators);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get VIO regulator\n");

	ret = regulator_bulk_enable(AD4692_NUM_REGULATORS, st->regulators);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to enable regulators\n");

	ret = devm_add_action_or_reset(dev, ad4692_disable_regulators, st);
	if (ret)
		return ret;

	st->vio = regulator_get_voltage(st->regulators[0].consumer);

	ref = devm_regulator_get_optional(dev, "vref");
	if (IS_ERR(ref)) {
		if (PTR_ERR(ref) != -ENODEV)
			return dev_err_probe(dev, PTR_ERR(ref),
					     "Failed to get vref regulator");

		/* Internal REFIN must be used if optional REF isn't used. */
		ref = devm_regulator_get(dev, "vrefin");
		if (IS_ERR(ref))
			return dev_err_probe(dev, PTR_ERR(ref),
					     "Failed to get vrefin regulator");
	}

	ret = regulator_enable(ref);
	if (ret) {
		dev_err_probe(dev, ret, "Failed to enable specified ref supply\n");
		return ret;
	}

	ret = devm_add_action_or_reset(dev, ad4692_disable_regulators, ref);
	if (ret)
		return ret;

	st->vref = regulator_get_voltage(ref);
	if (st->vref < AD4692_VREF_MIN || st->vref > AD4692_VREF_MAX)
		return dev_err_probe(dev, -EINVAL, "vref(%d) must be under [%lu %lu]\n",
				     st->vref, AD4692_VREF_MIN, AD4692_VREF_MAX);

	return 0;
}

static int ad4692_spi_read(void *context, const void *reg, size_t reg_size,
			   void *val, size_t val_size)
{
	const struct ad4692_state *st = context;
	struct spi_transfer xfer = {
		.speed_hz = 100000,
		.tx_buf = st->tx_data,
		.rx_buf = st->rx_data,
		.len = reg_size + val_size,
	};
	int ret;

	memcpy(st->tx_data, reg, reg_size);

	ret = spi_sync_transfer(st->spi, &xfer, 1);
	if (ret)
		return ret;

	memcpy(val, &st->rx_data[1], val_size);
	return 0;
}

static int ad4692_spi_write(void *context, const void *data, size_t count)
{
	const struct ad4692_state *st = context;

	return spi_write(st->spi, data, count);
}

static int ad4692_transfer(struct iio_dev *indio_dev, int command, int *val)
{
	const struct ad4692_state *st = iio_priv(indio_dev);
	struct spi_transfer xfer = {
		.speed_hz = 100000,
		.tx_buf = st->tx_data,
		.rx_buf = st->rx_data,
		.len = 3,
	};
	int ret;

	memcpy(st->tx_data, &command, 3);

	ret = spi_sync_transfer(st->spi, &xfer, 1);
	if (ret)
		return ret;

	memcpy(val, st->rx_data, 3);

	return 0;
}

static int ad4692_pwm_get(struct ad4692_state *st)
{
	struct device *dev = &st->spi->dev;
	struct pwm_state conv_state = {
		.duty_cycle = 10000,
		.time_unit = PWM_UNIT_NSEC,
	};
	int ret;

	st->conv_trigger = devm_pwm_get(dev, "cnv");
	if (IS_ERR(st->conv_trigger))
		return dev_err_probe(dev, PTR_ERR(st->conv_trigger),
				     "Failed to get cnv pwm\n");

	ret = devm_add_action_or_reset(dev, ad4692_disable_pwm, st);
	if (ret)
		return ret;

	/* CNV pwm signal must have the ADC's sampling rate. */
	conv_state.period = DIV_ROUND_CLOSEST_ULL(NANO, st->chip->max_rate);

	return pwm_apply_state(st->conv_trigger, &conv_state);
}

static int ad4692_sampling_enable(const struct ad4692_state *st, bool enable)
{
	struct pwm_state conv_state;
	int ret;

	pwm_get_state(st->conv_trigger, &conv_state);
	conv_state.enabled = enable;

	return pwm_apply_state(st->conv_trigger, &conv_state);
}

static int ad4692_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan, int *val,
			   int *val2, long info)
{
	struct ad4692_state *st = iio_priv(indio_dev);
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			return ret;

		mutex_lock(&st->lock);

		switch (st->adc_mode) {
		case AD4692_CNV_CLOCK_MODE:
			ret = ad4692_sampling_enable(st, true);
			if (ret)
				return ret;

			/* For only one channel read after enabling CNV Pulse
			 * bit, there are two CNV pulse periods needed for
			 * reading the result from ADC, then placing it in the
			 * Accumulator Readback Registers plus one more CONV
			 * time period for DATA_READYb flag to be pulled low.
			 */
			fsleep(DIV_ROUND_UP(2 * AD4692_OSC_PERIOD_NS(st->chip->max_rate) + AD4692_CONV_PERIOD_NS, 1000));

			ret = ad4692_sampling_enable(st, false);
			if (ret)
				return ret;

			ret = regmap_read(st->regmap, AD4692_IN_ACC(chan->channel), val);
			break;
		case AD4692_AUTONOMOUS_MODE:
			ret = regmap_update_bits(st->regmap, AD4692_CFG_REG,
						 AD4692_CONV_START_MASK, 1);
			if (ret)
				return ret;

			/* For only one channel read after enabling CONV_START
			 * bit, there are two internal OSC periods needed for
			 * reading the result from ADC, then placing it in the
			 * Accumulator Readback Registers plus one more CONV
			 * time period for DATA_READYb flag to be pulled low.
			 */
			fsleep(DIV_ROUND_UP(2 * AD4692_OSC_PERIOD_NS(st->chip->max_rate) + AD4692_CONV_PERIOD_NS, 1000));

			ret = regmap_update_bits(st->regmap, AD4692_CFG_REG,
						 AD4692_CONV_START_MASK, 0);
			if (ret)
				return ret;

			ret = regmap_read(st->regmap, AD4692_IN_ACC(chan->channel), val);
			break;
		case AD4692_MANUAL_MODE:
			ret = ad4692_sampling_enable(st, true);
			if (ret)
				return ret;

			ret = ad4692_transfer(indio_dev, AD4692_ADC_CHAN(chan->channel), val);
			break;
		default:
			return -EINVAL;
		}

		mutex_unlock(&st->lock);

		iio_device_release_direct_mode(indio_dev);

		if (ret)
			return ret;

		if (st->adc_mode == AD4692_MANUAL_MODE) {
			ret = ad4692_sampling_enable(st, false);
			if (ret)
				return ret;
		}

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = st->chip->max_rate;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = st->vref / 1000;
		*val2 = chan->scan_type.realbits;
		return IIO_VAL_FRACTIONAL_LOG2;
	default:
		return -EINVAL;
	}
}

static int ad4692_reg_access(struct iio_dev *indio_dev, unsigned int reg,
			     unsigned int writeval, unsigned int *readval)
{
	const struct ad4692_state *st = iio_priv(indio_dev);

	if (readval)
		return regmap_read(st->regmap, reg, readval);

	return regmap_write(st->regmap, reg, writeval);
}

static int ad4692_config(struct ad4692_state *st)
{
	struct device *dev = &st->spi->dev;
	u32 mode;
	int ret;

	ret = device_property_read_u32(dev, "adi,spi-mode", &mode);
	if (!ret) {
		if (mode > AD4692_MAX_ADC_MODE)
			return dev_err_probe(dev, -EINVAL, "Invalid SPI mode(%u)\n", mode);

		st->adc_mode = mode;
	}

	return ret;
}

static int ad4692_buffer_preenable(struct iio_dev *indio_dev)
{
	struct ad4692_state *st = iio_priv(indio_dev);
	int ret;

	switch (st->adc_mode) {
	case AD4692_MANUAL_MODE:
		return ad4692_sampling_enable(st, true);
	default:
		return -EOPNOTSUPP;
	}
}

static int ad4692_buffer_postdisable(struct iio_dev *indio_dev)
{
	struct ad4692_state *st = iio_priv(indio_dev);
	int ret;

	switch (st->adc_mode) {
	case AD4692_MANUAL_MODE:
		return ad4692_sampling_enable(st, false);
	default:
		return -EOPNOTSUPP;
	}
}

static irqreturn_t ad4692_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct ad4692_state *st = iio_priv(indio_dev);
	int ret;
	int val;

	switch (st->adc_mode) {
	case AD4692_MANUAL_MODE:
		ret = spi_sync(st->spi, &st->msg);
		if (ret)
			goto done;

	break;
	case AD4692_CNV_CLOCK_MODE:
		ret = ad4692_sampling_enable(st, true);
		if (ret)
			goto done;

		/* After enabling the CNV pulse, a number consisting
		 * of the active channels of CNV pulse periods pluse
		 * one more are needed for reading the input channels
		 * values from the ADC and one extra CONV period for
		 * placing the values in the Accumulator Readback
		 * Registers.
		 */
		fsleep(DIV_ROUND_UP((st->chip->num_channels + 1) * AD4692_OSC_PERIOD_NS(st->chip->max_rate) + AD4692_CONV_PERIOD_NS));

		ret = ad4692_sampling_enable(st, false);
		if (ret)
			goto done;
		ret = ad4692_spi_read(st, AD4692_IN_ACC(st->chip->channels->channel),
				      32, &val, 16);
		if (ret)
			goto done;

		put_unaligned_be32(val, &st->rx_data[1]);
		break;
	case AD4692_AUTONOMOUS_MODE:
		ret = regmap_update_bits(st->regmap, AD4692_CFG_REG, AD4692_CONV_START_MASK, 1);
		if (ret)
			goto done;

		/* After enabling the CONV_START bit, a number consisting
		 * of the active channels of internal OSC pulse periods
		 * plus one more are needed for reading the input channels
		 * values from the ADC and one extra CONV period for
		 * placing the values in the Accumulator Readback
		 * Registers.
		 */
		fsleep(DIV_ROUND_UP((st->chip->num_channels + 1) * AD4692_OSC_PERIOD_NS(st->chip->max_rate) + AD4692_CONV_PERIOD_NS));

		ret = regmap_update_bits(st->regmap, AD4692_CFG_REG, AD4692_CONV_START_MASK, 0);
		if (ret)
			goto done;

		ret = ad4692_spi_read(st, AD4692_IN_ACC(st->chip->channels->channel),
				      32, &val, 16);
		if (ret)
			goto done;

		put_unaligned_be32(val, &st->rx_data[1]);
	break;
	default:
		return -EINVAL;
	}

	iio_push_to_buffers(indio_dev, &st->rx_data[1]);
done:
	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

static const struct iio_buffer_setup_ops ad4692_buffer_setup_ops = {
	.preenable = &ad4692_buffer_preenable,
	.postdisable = &ad4692_buffer_postdisable,
};

static irqreturn_t ad4692_irq(int irq, void *private)
{
	iio_trigger_poll(private);
	return IRQ_HANDLED;
}

static int ad4692_set_trigger_state(struct iio_trigger *trig, bool enable)
{
	struct ad4692_state *st = iio_trigger_get_drvdata(trig);

	if (enable)
		enable_irq(st->spi->irq);
	else
		disable_irq(st->spi->irq);

	return 0;
}

static const struct iio_trigger_ops ad4692_trigger_ops = {
	.set_trigger_state = ad4692_set_trigger_state,
	.validate_device = iio_trigger_validate_own_device,
};

static const struct iio_info ad4692_info = {
	.read_raw = &ad4692_read_raw,
	.debugfs_reg_access = &ad4692_reg_access,
};

static const struct regmap_bus ad4692_regmap_bus = {
	.read = ad4692_spi_read,
	.write = ad4692_spi_write,
	.reg_format_endian_default = REGMAP_ENDIAN_BIG,
};

static const struct regmap_config ad4692_regmap_config = {
	.reg_bits = 8,
	.val_bits = 32,
};

static const struct spi_device_id ad4692_id[] = {
	{ "ad4692", (kernel_ulong_t)&ad4692_chips[ID_AD4692] },
	{ "ad4691", (kernel_ulong_t)&ad4692_chips[ID_AD4691] },
	{ "ad4694", (kernel_ulong_t)&ad4692_chips[ID_AD4694] },
	{ "ad4693", (kernel_ulong_t)&ad4692_chips[ID_AD4693] },
	{}
};
MODULE_DEVICE_TABLE(spi, ad4692_id);

int ad4692_gpio_direction_input(struct gpio_chip *chip, unsigned int offset)
{
	struct ad4692_state *st = gpiochip_get_data(chip);
	int ret;

	mutex_lock(&st->lock);

	ret = regmap_update_bits(st->regmap, AD4692_GPIO_STATE_REG,
				 AD4692_GPIO_STATE(offset), 0);

	mutex_unlock(&st->lock);

	return ret;
}

int ad4692_gpio_direction_output(struct gpio_chip *chip,
				 unsigned int offset, int value)
{
	struct ad4692_state *st = gpiochip_get_data(chip);
	int ret;

	mutex_lock(&st->lock);

	ret = regmap_update_bits(st->regmap, AD4692_GPIO_STATE_REG,
				 AD4692_GPIO_STATE(offset), 1);

	mutex_unlock(&st->lock);

	return ret;
}

int ad4692_get_gpio(struct gpio_chip *chip, unsigned int offset)
{
	struct ad4692_state *st = gpiochip_get_data(chip);
	int ret;

	mutex_lock(&st->lock);

	ret = regmap_update_bits(st->regmap, AD4692_GPIO_STATE_EN_REG,
				 AD4692_GPIO_STATE_EN(offset), 1);

	mutex_unlock(&st->lock);

	return ret;
}

void ad4692_set_gpio(struct gpio_chip *chip, unsigned int offset, int value)
{
	struct ad4692_state *st = gpiochip_get_data(chip);
	int ret;

	mutex_lock(&st->lock);

	ret = regmap_update_bits(st->regmap, AD4692_GPIO_DIR_REG,
				 AD4692_GPIO_DIR(offset), value);

	mutex_unlock(&st->lock);

	return ret;
}

int ad4692_gpio_setup(struct ad4692_state *st)
{
	int ret;

	st->gpiochip.label = "ad4692";
	st->gpiochip.base = -1;
	st->gpiochip.ngpio = 5;
	st->gpiochip.parent = &st->spi->dev;
	st->gpiochip.can_sleep = true;
	st->gpiochip.get = ad4692_get_gpio;
	st->gpiochip.set = ad4692_set_gpio;
	st->gpiochip.direction_input = ad4692_gpio_direction_input;
	st->gpiochip.direction_output = ad4692_gpio_direction_output;

	return devm_gpiochip_add_data(&st->spi->dev, &st->gpiochip, st);
}

static int ad4692_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct iio_dev *indio_dev;
	struct ad4692_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->spi = spi;
	spi_set_drvdata(spi, indio_dev);
	st->chip = device_get_match_data(dev);
	if (!st->chip) {
		st->chip = (void *)spi_get_device_id(spi)->driver_data;
		if (!st->chip)
			return dev_err_probe(dev, -ENODEV,
					     "Could not find chip info data\n");
	}

	ret = ad4692_config(st);
	if (ret)
		return ret;

	st->regmap = devm_regmap_init(&spi->dev, &ad4692_regmap_bus, st,
				&ad4692_regmap_config);
	if (IS_ERR(st->regmap))
		dev_err_probe(&spi->dev,  PTR_ERR(st->regmap),
			      "Failed to initialize regmap\n");

	switch (st->adc_mode) {
	case AD4692_MANUAL_MODE:
		ret = regmap_update_bits(st->regmap, AD4692_CFG_REG, AD4692_MANUAL_MODE_MASK, 1);
		if (ret)
		return ret;

		break;
	case AD4692_CNV_CLOCK_MODE:
	case AD4692_CNV_BURST_MODE:
	case AD4692_AUTONOMOUS_MODE:
		ret = regmap_update_bits(st->regmap, AD4692_CFG_REG, AD4692_ADC_MODE_MASK, st->adc_mode);
		if (ret)
			return ret;

		break;
	default:
		return -EINVAL;
	}

	ret = ad4692_regulators_get(st);
	if (ret)
		return ret;

	st->reset_gpio = devm_gpiod_get_optional(&spi->dev, "reset",
						 GPIOD_OUT_HIGH);
	if (IS_ERR(st->reset_gpio))
		return PTR_ERR(st->reset_gpio);

	/* Recommended hardware reset after power-on Reset. */
	gpiod_set_value(st->reset_gpio, GPIOD_OUT_LOW);

	/* Reset delay required from the datasheet of 3.2ms. */
	fsleep(3200);

	gpiod_set_value(st->reset_gpio, GPIOD_OUT_HIGH);

	st->gpio_0 = devm_gpiod_get_optional(&spi->dev, "gpio_0",
					     GPIOD_OUT_LOW);
	if (IS_ERR(st->gpio_0))
		return PTR_ERR(st->gpio_0);

	st->gpio_mode = devm_gpiod_get_array_optional(&spi->dev, "gpio_mode",
						      GPIOD_OUT_LOW);
	if (IS_ERR(st->gpio_mode))
		return PTR_ERR(st->gpio_mode);

	ret = ad4692_gpio_setup(st);
	if (ret)
		return ret;
\
	// ret = ad4692_pwm_get(st);
	// if (ret)
	// 	return ret;

	indio_dev->name = st->chip->name;
	indio_dev->info = &ad4692_info;
	indio_dev->channels = st->chip->channels;
	indio_dev->num_channels = st->chip->num_channels;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->available_scan_masks = GENMASK(st->chip->num_channels - 1, 0);
	indio_dev->setup_ops = &ad4692_buffer_setup_ops;

	if (spi->irq) {
		st->trig = devm_iio_trigger_alloc(&spi->dev, "%s-trigger",
						  indio_dev->name);
		if (!st->trig) {
			ret = -ENOMEM;
			dev_err(&indio_dev->dev,
				"Failed to allocate iio trigger\n");
			return ret;
		}

		st->trig->ops = &ad4692_trigger_ops;
		iio_trigger_set_drvdata(st->trig, indio_dev);
		ret = devm_iio_trigger_register(&indio_dev->dev,
						st->trig);
		if (ret < 0) {
			dev_err(&indio_dev->dev,
				"Failed to register iio trigger\n");
			return ret;
		}

		ret = devm_request_irq(&spi->dev, spi->irq,
				       ad4692_trigger_handler,
				       IRQF_TRIGGER_FALLING,
				       spi->dev.driver->name, indio_dev);
		if (ret < 0) {
			dev_err(&indio_dev->dev, "Failed to allocate IRQ.\n");
			return ret;
		}
	}

	st->xfer.rx_buf = st->rx_data;
	if (st->chip->resolution > 16)
		st->xfer.len = 5;
	else
		st->xfer.len = 4;

	spi_message_init(&st->msg);
	spi_message_add_tail(&st->xfer, &st->msg);

	ret = devm_iio_triggered_buffer_setup(&spi->dev, indio_dev,
					      &iio_pollfunc_store_time,
					      &ad4692_trigger_handler,
					      &ad4692_buffer_setup_ops);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to get triggered buffer\n");

	return devm_iio_device_register(dev, indio_dev);
}

static const struct of_device_id ad4692_of_match[] = {
	{ .compatible = "adi,ad4692", .data = (struct ad4692_chip_info *)&ad4692_chips[ID_AD4692]},
	{ .compatible = "adi,ad4691", .data = (struct ad4692_chip_info *)&ad4692_chips[ID_AD4691]},
	{ .compatible = "adi,ad4694", .data = (struct ad4692_chip_info *)&ad4692_chips[ID_AD4694]},
	{ .compatible = "adi,ad4693", .data = (struct ad4692_chip_info *)&ad4692_chips[ID_AD4693]},
	{},
};
MODULE_DEVICE_TABLE(of, ad4692_of_match);

static struct spi_driver ad4692_driver = {
	.driver = {
		.name = "ad4692",
		.of_match_table = ad4692_of_match,
	},
	.probe = ad4692_probe,
	.id_table = ad4692_id,
};
module_spi_driver(ad4692_driver);

MODULE_AUTHOR("Radu Sabau <radu.sabau@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD4692 ADC Driver");
MODULE_LICENSE("GPL v2");
