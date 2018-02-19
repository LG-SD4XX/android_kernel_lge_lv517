/*
 * Interval DFPS: Dynamic FPS based on frame interval.
 *
 * Copyright (C) 2016 LGE, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kallsyms.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/device.h>
#include "lge_interval_dfps.h"
#include "mdss_mdp.h"
#include "mdss_panel.h"

static struct lge_interval_dfps idfps;

static int update_fps(int new_fps) {
	int panel_fps, rc = 0;
	struct dynamic_fps_data data = {0};

	data.fps = new_fps;
	if (idfps.pdata->panel_info.type == DTV_PANEL)
		panel_fps = idfps.pdata->panel_info.lcdc.frame_rate;
	else
		panel_fps = mdss_panel_get_framerate(&idfps.pdata->panel_info);

	if (data.fps == panel_fps) {
		pr_err("%s: FPS is already %d\n",
			__func__, data.fps);
		return rc;
	}

	rc = mdss_mdp_dfps_update_params(idfps.mfd, idfps.pdata, &data);
	if (rc) {
		pr_err("failed to set dfps params\n");
		return rc;
	}

	return rc;
}

struct lge_interval_dfps *lge_get_idfps_data(void)
{
	return &idfps;
}

static void dfps_idle_work_fn(struct work_struct *work)
{
	struct device *dev;

        if (!idfps.enable)
                return;

	/* Get interval time between two successive commits */
	mutex_lock(&idfps.rw_lock);
	idfps.cur_ts = ktime_to_us(ktime_get());
	idfps.interval = idfps.cur_ts - idfps.pre_ts;
	idfps.pre_ts = idfps.cur_ts;
	if (idfps.interval < MIN_INTERVAL)
		idfps.interval = MIN_INTERVAL;

	/* Get moving average of interval */
	if (idfps.interval_w[idfps.w_idx] >= idfps.interval) {
		idfps.interval_avg -=
		(idfps.interval_w[idfps.w_idx] - idfps.interval) / idfps.w_sz;
	} else {
		idfps.interval_avg +=
		(idfps.interval - idfps.interval_w[idfps.w_idx]) / idfps.w_sz;
	}

	/* Queue the FPS */
	idfps.interval_w[idfps.w_idx] = idfps.interval;
	if (idfps.w_idx >= idfps.w_sz - 1)
		idfps.w_idx = 0;
	else
		idfps.w_idx++;

	/* Notify over threshold */
	if (idfps.downthresh < idfps.interval) {
		/* Get right framebuffer device */
		dev = idfps.mfd->fbi->dev;
		if (dev) {
			sysfs_notify(&dev->kobj, NULL, "dfps_interval");
		} else {
			pr_warn("mtkfb device is null\n");
		}
	}
	mutex_unlock(&idfps.rw_lock);
}

int lge_dfps_interval_notify(u32 cur_us)
{
	struct device *dev;

	cancel_delayed_work_sync(&idfps.idle_work);
	if (!idfps.enable)
		return 0;

	/* Get interval time between two successive commits */
	mutex_lock(&idfps.rw_lock);
	idfps.cur_ts = cur_us;
	idfps.interval = idfps.cur_ts - idfps.pre_ts;
	idfps.pre_ts = idfps.cur_ts;
	if (idfps.interval < MIN_INTERVAL)
		idfps.interval = MIN_INTERVAL;

	/* Get moving average of interval */
	if (idfps.interval_w[idfps.w_idx] >= idfps.interval) {
		idfps.interval_avg -=
		(idfps.interval_w[idfps.w_idx] - idfps.interval) / idfps.w_sz;
	} else {
		idfps.interval_avg +=
		(idfps.interval - idfps.interval_w[idfps.w_idx]) / idfps.w_sz;
	}

	/* Queue the FPS */
	idfps.interval_w[idfps.w_idx] = idfps.interval;
	if (idfps.w_idx >= idfps.w_sz - 1)
		idfps.w_idx = 0;
	else
		idfps.w_idx++;

	/* Notify over threshold */
	if (idfps.upthresh > idfps.interval ||
	    idfps.downthresh < idfps.interval) {
		/* Get right framebuffer device */
		dev = idfps.mfd->fbi->dev;
		if (dev) {
		sysfs_notify(&dev->kobj, NULL, "dfps_interval");
		} else {
			pr_warn("fb device is null\n");
		}
	}
	mutex_unlock(&idfps.rw_lock);
	schedule_delayed_work(&idfps.idle_work, usecs_to_jiffies(33000));

	return 0;
}

int interval_window_init(void)
{
	int i;

	if (idfps.interval_w)
		kfree(idfps.interval_w);

	if (!idfps.w_sz)
		idfps.w_sz = 4;

	idfps.w_idx = 0;
	idfps.interval_w = kmalloc(sizeof(u32) * idfps.w_sz, GFP_KERNEL);
	if (!idfps.interval_w) {
		pr_warn("not enough memory for interval window\n");
		return -ENOMEM;
	}

	for (i = 0; i < idfps.w_sz; i++)
		idfps.interval_w[i] = MIN_INTERVAL;

	idfps.interval_avg = MIN_INTERVAL;
	return 0;
}

int lge_dfps_init(void)
{
	/* Initialize needed data for DFPS */
	idfps.fbi_list = (struct fb_info **)kallsyms_lookup_name("fbi_list");
	if (idfps.fbi_list[0] == NULL)
		return -EFAULT;

	idfps.mfd = idfps.fbi_list[0]->par;
	if (idfps.mfd == NULL)
		return -EFAULT;

	idfps.pdata = dev_get_platdata(&idfps.mfd->pdev->dev);
	idfps.mdp5_data = mfd_to_mdp5_data(idfps.mfd);
	if (idfps.mdp5_data == NULL)
		return -EFAULT;

	idfps.ctl = idfps.mdp5_data->ctl;
	if (idfps.ctl == NULL)
		return -EFAULT;

	mutex_init(&idfps.rw_lock);
	if (interval_window_init() < 0)
		return -ENOMEM;

	INIT_DELAYED_WORK(&idfps.idle_work, dfps_idle_work_fn);

	return 0;
}

ssize_t dfps_en_store(struct device *dev,
		      struct device_attribute *attr, const char *buf,
		      size_t count)
{
	int ret;
	int enable;

	if (sscanf(buf, "%u", &enable) < 1) {
		pr_warn("Failed to store enable\n");
		return -EINVAL;
	}

	if (enable != idfps.enable) {
		if (enable) {
			if (!idfps.inited) {
				ret = lge_dfps_init();
				if (ret < 0) {
					pr_err("Failed to initialize DFPSv2");
					return ret;
				} else {
					idfps.inited = 1;
				}
			}
			mutex_lock(&idfps.rw_lock);
			idfps.enable = 1;
			mutex_unlock(&idfps.rw_lock);
		} else {
			mutex_lock(&idfps.rw_lock);
			idfps.enable = 0;
			idfps.fps = 60;
			mutex_unlock(&idfps.rw_lock);
			/* Use the VFP setting function for Refresh-Rate */
			idfps.pdata->panel_info.new_fps = idfps.fps;
			update_fps(idfps.fps);
		}
	}

	return count;
}

ssize_t dfps_en_show(struct device *dev,
		     struct device_attribute *attr, char *buf)
{
	int ret = 0;
	ret = snprintf(buf, PAGE_SIZE, "%u\n", idfps.enable);
	return ret;
}

ssize_t dfps_show(struct device *dev,
		  struct device_attribute *attr, char *buf)
{
	int ret = 0;
	ret = snprintf(buf, PAGE_SIZE, "%u %u %u\n",
		       idfps.fps, idfps.upthresh, idfps.downthresh);
	return ret;
}

ssize_t dfps_store(struct device *dev,
		   struct device_attribute *attr, const char *buf,
		   size_t count)
{
	u32 fps;
	u64 upthresh, downthresh;

        if (!idfps.enable)
		return -EFAULT;

	if (!count)
		return -EINVAL;

	/* Check Panel is Off */
	if (!(idfps.ctl->power_state) || !(idfps.mfd->panel_power_state)) {
		pr_err("Panel is off, No FPS changed\n");
		return count;
	}

	if (idfps.mfd->panel_info->type != MIPI_VIDEO_PANEL) {
		pr_err("No Video panel, No FPS changed\n");
		return -EPERM;
	}

	if (!(idfps.ctl->play_cnt)) {
		pr_err("Vsync is not enabled, No FPS changed\n");
		return -EPERM;
	}

	if (sscanf(buf, "%u %llu %llu", &fps, &upthresh, &downthresh) < 3) {
		pr_err("Failed to store FPS\n");
		return -EINVAL;
	}

	if (fps == idfps.fps) {
		pr_debug("%s: FPS is already %d\n", __func__, fps);
		return count;
	}

	mutex_lock(&idfps.rw_lock);
	if (fps == 0) {
		idfps.fps = 60;
		idfps.upthresh = 10000;
		idfps.downthresh = 3000000;
	} else {
		if (fps > 60)
			fps = 60;
		else if (fps < 30)
			fps = 30;
		idfps.fps = fps;
		idfps.upthresh = upthresh;
		idfps.downthresh = downthresh;
	}

	if (idfps.enable) {
		pr_debug("%s: update dynamic fps=%d\n", __func__, idfps.fps);
		/* Use the VFP setting function for changing Refresh-Rate */
		idfps.pdata->panel_info.new_fps = idfps.fps;
		update_fps(idfps.fps);
	}
	mutex_unlock(&idfps.rw_lock);

	return count;
}

ssize_t dfps_interval_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	int ret = 0;
	ret = snprintf(buf, PAGE_SIZE, "%u\n", idfps.interval_avg);
	return ret;
}

ssize_t dfps_wsize_store(struct device *dev,
			 struct device_attribute *attr, const char *buf,
			 size_t count)
{
	int w_sz;
	int enable = idfps.enable;

	if (sscanf(buf, "%u", &w_sz) < 1) {
		pr_warn("Failed to store window size\n");
		return -EINVAL;
	}

	if (w_sz != idfps.w_sz) {
		if (w_sz) {
			idfps.enable = 0;
			idfps.w_sz = w_sz;
			if (interval_window_init() < 0) {
				return -ENOMEM;
			}
			idfps.enable = enable;
		} else {
			pr_warn("Invalid window size\n");
			return -EFAULT;
		}
	}

	return count;
}

ssize_t dfps_wsize_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = 0;
	ret = snprintf(buf, PAGE_SIZE, "%u\n", idfps.w_sz);
	return ret;
}
