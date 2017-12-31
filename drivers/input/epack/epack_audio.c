/*
 * Copyright(c) 2016, LG Electronics. All rights reserved.
 *
 * e-pack i2c device driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>

#include <linux/input/epack_core.h>

void epack_audio_work_func(struct work_struct *audio_work)
{
	struct epack_dev_data *epack = container_of(to_delayed_work(audio_work), struct epack_dev_data, audio_work);
	static int count = 0;

	pr_err("[%s] status : %d count : %d\n",__func__,get_epack_status(),count++);

	schedule_delayed_work(&epack->audio_work, msecs_to_jiffies(13000));

	return;
	}
