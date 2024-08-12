/*
 * tlv320aic3xxx-i2c.c  -- driver for TLV320AIC3XXX TODO
 *
 * Author:	Mukund Navada <navada@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>

#include "tlv320aic3xxx-core.h"

static bool aic3xxx_volatile(struct device *dev,
	unsigned int reg)
{
	return true;
}

static bool aic3xxx_writeable(struct device *dev,
	unsigned int reg)
{
	return true;
}

struct regmap_config aicxxx_i2c_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.writeable_reg = aic3xxx_writeable,
	.volatile_reg = aic3xxx_volatile,
	.cache_type = REGCACHE_NONE,
	.max_register = 0xff,
};

#ifdef CONFIG_PM
static int aic3xxx_suspend(struct device *dev)
{
	struct aic3xxx *aic3xxx = dev_get_drvdata(dev);

	aic3xxx->suspended = true;

	return 0;
}

static int aic3xxx_resume(struct device *dev)
{
	struct aic3xxx *aic3xxx = dev_get_drvdata(dev);

	aic3xxx->suspended = false;

	return 0;
}
#endif

static int aic3xxx_i2c_probe(struct i2c_client *i2c,
					  const struct i2c_device_id *id)
{
	struct aic3xxx *aicxxx;
	const struct regmap_config *regmap_config;
	int ret;

	switch (id->driver_data) {
	case TLV320AIC3262:
	case TLV320AIC3268:
		regmap_config = &aicxxx_i2c_regmap;
		break;
	default:
		dev_err(&i2c->dev, "Unknown device type %ld\n",
			id->driver_data);
		return -EINVAL;
	}

	aicxxx = devm_kzalloc(&i2c->dev, sizeof(*aicxxx), GFP_KERNEL);
	if (aicxxx == NULL)
		return -ENOMEM;

	aicxxx->regmap = devm_regmap_init_i2c(i2c, regmap_config);

	if (IS_ERR(aicxxx->regmap)) {
		ret = PTR_ERR(aicxxx->regmap);
		dev_err(&i2c->dev, "Failed to allocate register map: %d\n",
			ret);
		return ret;
	}

	aicxxx->type = id->driver_data;
	aicxxx->dev = &i2c->dev;
	aicxxx->irq = i2c->irq;
	i2c_set_clientdata(i2c, aicxxx);

	ret = aic3xxx_device_init(aicxxx, aicxxx->irq);
	if(ret) {
		dev_err(&i2c->dev, "%s:%u: Failed to init device: %d\n",
			__func__, __LINE__, ret);
		return ret;
	}

	return aic326x_register_codec(aicxxx);
}

static int aic3xxx_i2c_remove(struct i2c_client *i2c)
{
	struct aic3xxx *aicxxx = dev_get_drvdata(&i2c->dev);
	aic326x_deregister_codec(aicxxx);
	aic3xxx_device_exit(aicxxx);
	return 0;
}

static const struct of_device_id aic3262_of_match[] = {
		{ .compatible = "ti,aic3262", },
		{ .compatible = "ti,aic3268", },
		{ },
};
MODULE_DEVICE_TABLE(of, aic3262_of_match);

static const struct i2c_device_id aic3262_i2c_id[] = {
		{ "aic3262", TLV320AIC3262 },
		{ "aic3268", TLV320AIC3268 },
		{ }
};
MODULE_DEVICE_TABLE(i2c, aic3262_i2c_id);


static UNIVERSAL_DEV_PM_OPS(aic3xxx_pm_ops, aic3xxx_suspend, aic3xxx_resume,
				NULL);

static struct i2c_driver aic3xxx_i2c_driver = {
	.driver = {
		.name	= "tlv320aic3262-codec",
		.owner	= THIS_MODULE,
		.pm	= &aic3xxx_pm_ops,
		.of_match_table = aic3262_of_match,
	},
	.probe		= aic3xxx_i2c_probe,
	.remove		= aic3xxx_i2c_remove,
	.id_table	= aic3262_i2c_id,
};

module_i2c_driver(aic3xxx_i2c_driver);

MODULE_DESCRIPTION("TLV320AIC3XXX I2C bus interface");
MODULE_AUTHOR("Mukund Navada <navada@ti.com>");
MODULE_LICENSE("GPL");
