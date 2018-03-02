/*
 * P1 DSV  MFD Driver
 *
 * Copyright 2014 LG Electronics Inc,
 *
 * Author:
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>

#include <linux/mfd/dw8768.h>

static struct dw8768 *dw8768_base;
static struct mfd_cell dw8768_devs[] = {
	{ .name = "dw8768_dev" },
};


int dw8768_register_set(u8 address, u8 value)
{
	struct i2c_client *cl;
	int ret = 0;

	if (dw8768_base == NULL) {
		pr_err("%s: invalid dw8768 address \n", __func__);
		return -ENODEV;
	}

	cl = container_of(dw8768_base->dev, struct i2c_client, dev);
	if (cl == NULL) {
		pr_err("%s: invalid i2c client address \n", __func__);
		return -EINVAL;
	}

	ret = i2c_smbus_write_byte_data(cl, address, value);
	if(ret < 0)
		pr_err("%s: failed to set address %d\n", __func__, address);

	return ret;
}
EXPORT_SYMBOL_GPL(dw8768_register_set);

int dw8768_set_output_voltage(uint8_t val)
{
	int ret = 0;
	struct i2c_client *cl;

	if (dw8768_base == NULL) {
		pr_err("%s: dw8768 was not probed\n", __func__);
		return -ENODEV;
	}

	cl = container_of(dw8768_base->dev, struct i2c_client, dev);
	if (cl == NULL)
		return -EINVAL;

	ret = i2c_smbus_write_byte_data(cl, DW8768_VPOS_VOLTAGE_REG, val);
	if (ret == 0) {
		ret = i2c_smbus_read_byte_data(cl, DW8768_VPOS_VOLTAGE_REG);
		if (ret < 0) {
			pr_err("%s: DW8768_VPOS_VOLTAGE_REG read failed\n", __func__);
			return ret;
		} else {
			pr_info("%s: DW8768_VPOS_VOLTAGE_REG = 0x%02X\n", __func__, ret);
			ret = 0;
		}
	} else {
		pr_err("%s: DW8768_VPOS_VOLTAGE_REG write failed\n", __func__);
		return ret;
	}

	ret = i2c_smbus_write_byte_data(cl, DW8768_VNEG_VOLTAGE_REG, val);
	if (ret == 0) {
		ret = i2c_smbus_read_byte_data(cl, DW8768_VNEG_VOLTAGE_REG);
		if (ret < 0) {
			pr_err("%s: DW8768_VNEG_VOLTAGE_REG read failed\n", __func__);
			return ret;
		} else {
			pr_info("%s: DW8768_VNEG_VOLTAGE_REG = 0x%02X\n", __func__, ret);
			ret = 0;
		}
	} else {
		pr_err("%s: DW8768_VNEG_VOLTAGE_REG write failed\n", __func__);
		return ret;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(dw8768_set_output_voltage);

static struct regmap_config dw8768_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = DW8768_MAX_REGISTERS,
};

static int dw8768_probe(struct i2c_client *cl, const struct i2c_device_id *id)
{
	struct dw8768 *dw8768;
	struct device *dev = &cl->dev;
	struct dw8768_platform_data *pdata = dev_get_platdata(dev);
	int rc = 0;

	dw8768 = devm_kzalloc(dev, sizeof(*dw8768), GFP_KERNEL);
	if (!dw8768) {
		pr_err("%s: mem alloc failed \n", __func__);
		return -ENOMEM;
	}

	dw8768->pdata = pdata;

	dw8768->regmap = devm_regmap_init_i2c(cl, &dw8768_regmap_config);
	if (IS_ERR(dw8768->regmap)) {
		pr_err("%s: Failed to allocate register map\n", __func__);
		devm_kfree(dev, dw8768);
		return PTR_ERR(dw8768->regmap);
	}

	dw8768->dev = &cl->dev;
	i2c_set_clientdata(cl, dw8768);
	dw8768_base = dw8768;

	rc = mfd_add_devices(dev, -1, dw8768_devs, ARRAY_SIZE(dw8768_devs),
			       NULL, 0, NULL);
	if (rc) {
		pr_err("%s: Failed to add dw8768 subdevice ret=%d\n", __func__, rc);
		return -ENODEV;
	}

	pr_info("%s: done \n", __func__);

	return rc;
}

static int dw8768_remove(struct i2c_client *cl)
{
	struct dw8768 *dw8768 = i2c_get_clientdata(cl);

	mfd_remove_devices(dw8768->dev);

	return 0;
}

static const struct i2c_device_id dw8768_ids[] = {
	{ "dw8768", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, dw8768_ids);

#ifdef CONFIG_OF
static const struct of_device_id dw8768_of_match[] = {
	{ .compatible = "dw8768", },
	{ }
};
MODULE_DEVICE_TABLE(of, dw8768_of_match);
#endif

static struct i2c_driver dw8768_driver = {
	.driver = {
		.name = "dw8768",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(dw8768_of_match),
#endif
	},
	.id_table = dw8768_ids,
	.probe = dw8768_probe,
	.remove = dw8768_remove,
};
module_i2c_driver(dw8768_driver);

MODULE_DESCRIPTION("dw8768 MFD Core");
