#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/iio/iio.h>
#include <linux/regmap.h>

/* The CONFIGURATION register's bitmasks */
#define MAX31827_CONFIGURATION_1SHOT        BIT(0)
#define MAX31827_CONFIGURATION_CNV_RATE     GENMASK(3,1)
#define MAX31827_CONFIGURATION_PEC_EN       BIT(4)
#define MAX31827_CONFIGURATION_TIMEOUT      BIT(5)
#define MAX31827_CONFIGURATION_RESOL        GENMASK(7,6)
#define MAX31827_CONFIGURATION_ALRM_POL     BIT(8)
#define MAX31827_CONFIGURATION_COMP_INT     BIT(9)
#define MAX31827_CONFIGURATION_FLT_Q        GENMASK(11,10)
#define MAX31827_CONFIGURATION_PEC_ERR      BIT(13)
#define MAX31827_CONFIGURATION_U_TEMP_STAT  BIT(14)
#define MAX31827_CONFIGURATION_O_TEMP_STAT  BIT(15)

/* The MAX31827 registers */
#define MAX31827_T                          0x0
#define MAX31827_CONFIGURATION              0x2
#define MAX31827_TH                         0x4
#define MAX31827_TL                         0x6
#define MAX31827_TH_HYST                    0x8
#define MAX31827_TL_HYST                    0xA

/* Masks */
#define WRITEABLE_CONFIG_MASK               0x0FFF

struct max31827_data {
    struct regmap *regmap;
};

// check this
// might have to change the endian 
static const struct regmap_config max31827_regmap = {           
        .reg_bits = 8,
        .val_bits = 16,
        .max_register = 0xA,
};

static int max31827_read_raw(struct iio_dev *indio_dev,
                struct iio_chan_spec const *chan,
                int *val,
                int *val2,
                long mask)
{
    struct max31827_data *data = iio_priv(indio_dev);
    int ret;
    int cfg;

    switch (mask) {
    case IIO_CHAN_INFO_ENABLE:
        ret = regmap_read(data->regmap, MAX31827_CONFIGURATION, val);
        if (ret < 0)
            return ret;

        *val = *val & MAX31827_CONFIGURATION_1SHOT;
        return IIO_VAL_INT;
    case IIO_CHAN_INFO_RAW:
        ret = regmap_read(data->regmap, MAX31827_T, val);
        if (ret < 0)
            return ret;
        
        return IIO_VAL_INT;

    case IIO_CHAN_INFO_SCALE:
        ret = regmap_read(data->regmap, MAX31827_CONFIGURATION, &cfg);
        if (ret < 0)
            return ret;

        *val = 1;
        *val2 = 16;
        // cfg = (cfg & MAX31827_CONFIGURATION_RESOL) >> 6;
        // switch(cfg) {
        // case 0b00:
        //     *val2 = 1;
        //     break;
        // case 0b01:
        //     *val2 = 2;
        //     break;
        // case 0b10:
        //     *val2 = 4;
        //     break;
        // case 0b11:
        //     *val2 = 16;
        //     break;
        // default:
        //     return -EINVAL;
        // }

        return IIO_VAL_FRACTIONAL;

    case IIO_CHAN_INFO_SAMP_FREQ:
        ret = regmap_read(data->regmap, MAX31827_CONFIGURATION, &cfg);
        if (ret < 0)
            return ret;

        cfg = (cfg & MAX31827_CONFIGURATION_CNV_RATE) >> 1;
        switch(cfg) {
        case 0b000:
            *val = 0;
            break;
        case 0b001:
            *val = 1;
            *val2 = 64;
            break;
        case 0b010:
            *val = 1;
            *val2 = 32;
            break;
        case 0b011:
            *val = 1;
            *val2 = 16;
            break;
        case 0b100:
            *val = 1;
            *val2 = 4;
            break;
        case 0b101:
            *val = 1;
            *val2 = 1;
            break;
        case 0b110:
            *val = 4;
            *val2 = 1;
            break;
        case 0b111:
            *val = 8;
            *val2 = 1;
            break;
        default:
            return -EINVAL;
        }

        return IIO_VAL_FRACTIONAL;
    }
 
    return -EINVAL;
}

static int max31827_write_raw(struct iio_dev *indio_dev,
                 struct iio_chan_spec const *chan,
                 int val,
                 int val2,
                 long mask)
{
    struct max31827_data *data = iio_priv(indio_dev);
    int ret;
    unsigned int value;

    switch (mask) {
    /* One-shot = return a single conversion */
    case IIO_CHAN_INFO_ENABLE:
        if(val) {
            ret = regmap_update_bits(data->regmap, MAX31827_CONFIGURATION, MAX31827_CONFIGURATION_1SHOT |
                        MAX31827_CONFIGURATION_CNV_RATE, 0b0001);
            if (ret < 0)
                return ret;
        }
        return 0;
   
    case IIO_CHAN_INFO_SAMP_FREQ:
        val <<= 1;

        ret = regmap_update_bits(data->regmap, MAX31827_CONFIGURATION, MAX31827_CONFIGURATION_CNV_RATE, val);
        if (ret < 0)
            return ret;

        return 0;

    case IIO_CHAN_INFO_SCALE:
        switch(val) {
        case 1:
            value = 0b00;
            break;
        case 2:
            value = 0b01;
            break;
        case 4:
            value = 0b10;
            break;
        case 16:
            value = 0b11;
            break;
        default:
            return -EINVAL;
        }
        value <<= 6;

        ret = regmap_update_bits(data->regmap, MAX31827_CONFIGURATION, MAX31827_CONFIGURATION_RESOL, value);
        if (ret < 0)
            return ret;

        return 0;
    }
 
    return -EINVAL;
}

static int max31827_reg_access(struct iio_dev *indio_dev,
                        unsigned reg, unsigned writeval,
                        unsigned *readval)
{
    struct max31827_data *data = iio_priv(indio_dev);

    if (readval)
        return regmap_read(data->regmap, reg, readval);

    return regmap_write(data->regmap, reg, writeval);
}

static const struct iio_info max31827_info = {
    .read_raw = &max31827_read_raw,
    .write_raw = &max31827_write_raw,
    .debugfs_reg_access = &max31827_reg_access,
};

// check this
static const struct iio_chan_spec max31827_channels[] = {
    {
        .type = IIO_TEMP,
        .info_mask_shared_by_all = 
            BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SAMP_FREQ) |
            BIT(IIO_CHAN_INFO_ENABLE) | BIT(IIO_CHAN_INFO_SCALE), 
        .output = 0,
    }, 
};

static const struct i2c_device_id max31827_i2c_ids[] = {
	{ .name = "max31827", },
    {}
};
MODULE_DEVICE_TABLE(i2c, max31827_i2c_ids);

static const struct of_device_id max31827_of_match[] = {
	{ .compatible = "max31827", },
    {}
};
MODULE_DEVICE_TABLE(of, max31827_of_match);

static int max31827_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
    struct iio_dev *indio_dev;
    struct max31827_data *data;
    struct regmap *regmap;
    int ret;

	dev_info(&client->dev, "Entered probe function of max31827\n");
    // check this
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_WORD_DATA))
		return -EOPNOTSUPP;
 
    indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
    if (!indio_dev)
        return -ENOMEM;
    
    data = iio_priv(indio_dev);
    regmap = devm_regmap_init_i2c(client, &max31827_regmap);
    if (IS_ERR(regmap)) {
        ret = PTR_ERR(regmap);
		dev_err(&client->dev, "Failed to allocate regmap: %d\n", ret);
		return ret;
	}

    data->regmap = regmap;

    indio_dev->name = "max31827";
    indio_dev->info = &max31827_info;

    indio_dev->channels = max31827_channels;
    indio_dev->num_channels = ARRAY_SIZE(max31827_channels);
 
    return iio_device_register(indio_dev);
}

static struct i2c_driver max31827_driver = {
    .driver = {
        .name = "max31827",
		.of_match_table = max31827_of_match,
    },
    .probe = max31827_probe,
	.id_table = max31827_i2c_ids,
};
module_i2c_driver(max31827_driver);

MODULE_AUTHOR("Daniel Matyas <daniel.matyas@analog.com>");
MODULE_DESCRIPTION("Maxim MAX31827 low-power temperature switch driver");
MODULE_LICENSE("GPL");
