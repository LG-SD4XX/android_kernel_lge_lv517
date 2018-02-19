/*
 * EXTERNAL DSV MFD Driver
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

#ifndef __MFD_EXT_DSV_DW8768_SM5107_H__
#define __MFD_EXT_DSV_DW8768_SM5107_H__

#include <linux/gpio.h>
#include <linux/pwm.h>
#include <linux/regmap.h>
#include <linux/regulator/machine.h>


#define MAX_REGISTERS 0x04

#define DW8768_POSITIVE_OUTPUT_CONTROL_REG	0X00
#define DW8768_NEGATIVE_OUTPUT_CONTROL_REG	0X01
#define DW8768_DISCHARGE_STATUS_CONTROL_REG 0x03
#define DW8768_ENABLE_REG 					0x05
#define DW8768_KNOCK_ON_CONTROL_REG 		0x07

#define SM5107_POSCNTL 	0x00
#define SM5107_NEGCNTL 	0x01
#define SM5107_CONTROL 	0x03
#define SM5107_CTRL_SET 0xFF

struct ext_dsv_platform_data {
	const char *name;
};

struct ext_dsv {
	struct device *dev;
	struct regmap *regmap;
	struct ext_dsv_platform_data *pdata;
};

int ext_dsv_register_set(u8 address, u8 value);
#endif
