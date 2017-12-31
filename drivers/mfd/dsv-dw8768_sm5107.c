/*
 * External DSV  MFD Driver
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
#include <linux/string.h>

#include <linux/mfd/dsv-dw8768_sm5107.h>


static struct ext_dsv *ext_dsv_base;
static struct mfd_cell ext_dsv_devs[] = {
	{ .name = "external_dev" },
};


int ext_dsv_register_set(u8 address, u8 value)
{
	struct i2c_client *cl;
	int ret = 0;

	if (ext_dsv_base == NULL) {
		pr_err("%s: invalid dw8768 address \n", __func__);
		return -ENODEV;
	}

	cl = container_of(ext_dsv_base->dev, struct i2c_client, dev);
	if (cl == NULL) {
		pr_err("%s: invalid i2c client address \n", __func__);
		return -EINVAL;
	}

	ret = i2c_smbus_write_byte_data(cl, address, value);
	if(ret < 0)
		pr_err("%s: failed to set address %d\n", __func__, address);

	return ret;
}
EXPORT_SYMBOL_GPL(ext_dsv_register_set);

static struct regmap_config ext_dsv_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = MAX_REGISTERS,
};

static int ext_dsv_probe(struct i2c_client *cl, const struct i2c_device_id *id)
{
	struct ext_dsv *ext_dsv;
	struct device *dev = &cl->dev;
	struct ext_dsv_platform_data *pdata = dev_get_platdata(dev);
	int rc = 0;

	ext_dsv = devm_kzalloc(dev, sizeof(*ext_dsv), GFP_KERNEL);
	if (!ext_dsv)
		return -ENOMEM;

	ext_dsv->pdata = pdata;

	ext_dsv->regmap = devm_regmap_init_i2c(cl, &ext_dsv_regmap_config);
	if (IS_ERR(ext_dsv->regmap)) {
		pr_err("Failed to allocate register map\n");
		devm_kfree(dev, ext_dsv);
		return PTR_ERR(ext_dsv->regmap);
	}

	ext_dsv->dev = &cl->dev;
	i2c_set_clientdata(cl, ext_dsv);
	ext_dsv_base = ext_dsv;

	rc = mfd_add_devices(dev, -1, ext_dsv_devs, ARRAY_SIZE(ext_dsv_devs),
			       NULL, 0, NULL);
	if (rc) {
		pr_err("Failed to add ext_dsv_dw8768_sm5107 subdevice ret=%d\n", rc);
		return -ENODEV;
	}

	pr_info("%s: done \n", __func__);

	return rc;
}

static int ext_dsv_remove(struct i2c_client *cl)
{
	struct ext_dsv *ext_dsv = i2c_get_clientdata(cl);

	mfd_remove_devices(ext_dsv->dev);

	return 0;
}

static const struct i2c_device_id ext_dsv_ids[] = {
	{ "ext_dsv_dw_sm", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ext_dsv_ids);

#ifdef CONFIG_OF
static const struct of_device_id ext_dsv_of_match[] = {
	{ .compatible = "ext_dsv_dw_sm", },
	{ }
};
MODULE_DEVICE_TABLE(of, ext_dsv_of_match);
#endif

static struct i2c_driver ext_dsv_driver = {
	.driver = {
		.name = "ext_dsv_dw_sm",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(ext_dsv_of_match),
#endif
	},
	.id_table = ext_dsv_ids,
	.probe = ext_dsv_probe,
	.remove = ext_dsv_remove,
};
module_i2c_driver(ext_dsv_driver);

MODULE_DESCRIPTION("ext_dsv dw8707_sm5107 MFD Core");
