/*
 * max98927.c -- ALSA SoC Stereo MAX98927 driver
 * Copyright 2013-15 Maxim Integrated Products
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <sound/tlv.h>
#include <linux/regulator/consumer.h>
#include "max98927.h"
#ifdef CONFIG_SND_SOC_MAXIM_DSM_CAL
#include <sound/maxim_dsm_cal.h>
#endif /* CONFIG_SND_SOC_MAXIM_DSM_CAL */

#define DEBUG_MAX98927
#ifdef DEBUG_MAX98927
#define msg_maxim(format, args...) \
pr_err("[MAX98927] %s: " format "\n", __func__, ## args)
#else
#define msg_maxim(format, args...)
#endif /* DEBUG_MAX98927 */

static struct reg_default max98927_reg_map[] = {
	{0x0014, 0x78}, /* Meas ADC Thermal Warning Threshold */
	{0x0015, 0xFF}, /* Meas ADC Thermal Shutdown Threshold */
	{0x0016, 0x00}, /* Meas ADC Thermal Hysteresis */
	{0x0017, 0x55}, /* Pin Config */
	/* For mono driver we are just enabling one channel*/
	{0x0018, 0x03}, /* PCM Rx Enables A */
	{0x0019, 0x00}, /* PCM Rx Enables B */
	{0x001A, 0x03}, /* PCM Tx Enables A */
	{0x001B, 0x00}, /* PCM Tx Enables B */
	{0x001C, 0xFC}, /* PCM Tx HiZ Control A */
	{0x001D, 0xFF}, /* PCM Tx HiZ Control B */
	{0x001E, 0x10}, /* PCM Tx Channel Sources A */
	{0x001F, 0x00}, /* PCM Tx Channel Sources B */
    {0x0020, 0x41},
    {0x0022, 0x22},
    {0x0023, 0x08},
    {0x0024, 0x88},
    {0x0025, 0x80},
    {0x0026, 0x01},
	{0x0035, 0x00}, /* PDM Rx Enable */
	{0x0036, 0x40}, /* AMP Volume Control */
	{0x0037, 0x03}, /* AMP DSP Config */
	{0x0039, 0x01}, /* DRE Control */
	{0x003A, 0x00}, /* AMP Enable */
	{0x003B, 0x00}, /* Speaker Source Select */
	{0x003C, 0x05}, /* Speaker Gain */
	{0x003D, 0x8D}, /* SSM Configuration */
	{0x003E, 0x03}, /* Measurement enables */
	{0x003F, 0x07}, /* Measurement DSP Config */
	{0x0040, 0x1C}, /* Boost Control 0 */
	{0x0041, 0x01}, /* Boost Control 3 */
	{0x0042, 0x3F}, /* Boost Control 1 */
	{0x0043, 0x04}, /* Meas ADC Config */
	{0x0044, 0x00}, /* Meas ADC Base Divide MSByte */
	{0x0045, 0x00}, /* Meas ADC Base Divide LSByte */
	{0x007F, 0x0E}, /* Browout level 8 amp 1 control 1 */
	{0x0082, 0x08}, /* Env Tracker Vout Headroom */
	{0x0086, 0x01}, /* Env Tracker Control */
	{0x00FF, 0x00}, /* Global Enable */
};

int max98927_regmap_read(struct regmap *map, unsigned int reg,
                    unsigned int *val)
{
    int ret = -ENXIO;

    if (map)
        ret = regmap_read(map, reg, val);

    return ret;
}

int max98927_regmap_write(struct regmap *map, unsigned int reg,
                    unsigned int val)
{
    int ret = -ENXIO;

    if (map)
        ret = regmap_write(map, reg, val);

    return ret;
}

int max98927_wrapper_read(struct max98927_priv *max98927, int speaker,
		unsigned int reg, unsigned int *val)
{
	int ret = -EIO;
    int count = 0;

    while (count++ < MAX_TRY_COUNT && ret != 0) {
        switch (speaker) {
        case MAX98927L:
            ret = max98927_regmap_read(max98927->regmap_l, reg, val);
            break;
        case MAX98927R:
            if (max98927->mono_stereo)
                ret = max98927_regmap_read(max98927->regmap_r, reg, val);
            break;
        case MAX98927B:
        default:
            msg_maxim("Unknown type %d", speaker);
            break;
        }
        if (!ret)
            return ret;
    }

    msg_maxim("Failed to read [0x%04x][%d]", reg, ret);

	return ret;
}

int max98927_wrapper_write(struct max98927_priv *max98927, int speaker,
		unsigned int reg, unsigned int val)
{
    int ret = -EIO;
    int count = 0;

    while (count++ < MAX_TRY_COUNT && ret != 0) {
        switch (speaker) {
        case MAX98927L:
            ret = max98927_regmap_write(max98927->regmap_l, reg, val);
            break;
        case MAX98927R:
            if (max98927->mono_stereo)
                ret = max98927_regmap_write(max98927->regmap_r, reg, val);
            break;
        case MAX98927B:
            ret = max98927_regmap_write(max98927->regmap_l, reg, val);
            if (max98927->mono_stereo)
                ret = max98927_regmap_write(max98927->regmap_r, reg, val);
            break;
        default:
            msg_maxim("Unknown type %d", speaker);
            break;
        }
        if (!ret)
            return ret;
    }

    msg_maxim("Failed to write [0x%04x, 0x%02x][%d]", reg, val, ret);

    return ret;
}

int max98927_wrapper_update(struct max98927_priv *max98927, int speaker,
		unsigned int reg, unsigned int mask, unsigned int val)
{
    int ret = -EIO;
    int count = 0;

    while (count++ < MAX_TRY_COUNT && ret != 0) {
        switch (speaker) {
        case MAX98927L:
            ret = regmap_update_bits(max98927->regmap_l,
                    reg, mask, val);
            break;
        case MAX98927R:
            ret = regmap_update_bits(max98927->regmap_r,
                    reg, mask, val);
            break;
        case MAX98927B:
            ret = regmap_update_bits(max98927->regmap_l,
                    reg, mask, val);
            if (max98927->mono_stereo)
                ret = regmap_update_bits(max98927->regmap_r,
                        reg, mask, val);
            break;
        default:
            msg_maxim("Unknown type %d", speaker);
        }
        if (!ret)
            return ret;
    }

    msg_maxim("Failed to update [0x%04x, 0x%02x][%d]", reg, val, ret);

    return ret;
}

static int max98927_reg_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol, unsigned int reg,
		unsigned int mask, unsigned int shift)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);
	int data = 0;

	max98927_wrapper_read(max98927, MAX98927L, reg, &data);
	ucontrol->value.integer.value[0] =
		(data & mask) >> shift;

	return 0;
}

static int max98927_reg_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol, unsigned int reg,
		unsigned int mask, unsigned int shift)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);
	unsigned int sel = ucontrol->value.integer.value[0];

	max98927_wrapper_update(max98927, MAX98927B,
        reg, mask, sel << shift);
	msg_maxim("register 0x%02X, value 0x%02X", reg, sel);

	return 0;
}

static bool max98927_readable_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case 0x0001 ... 0x0028:
	case 0x002B ... 0x004E:
	case 0x0051 ... 0x0055:
	case 0x005A ... 0x0061:
	case 0x0072 ... 0x0087:
	case 0x00FF:
	case 0x0100:
	case 0x01FF:
		return true;
	}
	return false;
};

#ifdef CONFIG_SND_SOC_MAXIM_DSM
#ifdef USE_DSM_LOG
static int max98927_get_dump_status(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = maxdsm_get_dump_status();
	return 0;
}
static int max98927_set_dump_status(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);

	int val = 0;

	max98927_wrapper_read(max98927, MAX98927L,
			MAX98927_GLOBAL_ENABLE, &val);
	msg_maxim("val: %d", val);

	if (val != 0)
		maxdsm_update_param();

	return 0;
}
static ssize_t max98927_log_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return maxdsm_log_prepare(buf);
}

static DEVICE_ATTR(dsm_log, S_IRUGO, max98927_log_show, NULL);
#endif /* USE_DSM_LOG */

#ifdef USE_DSM_UPDATE_CAL
static int max98927_get_dsm_param(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = maxdsm_cal_avail();
	return 0;
}

static int max98927_set_dsm_param(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	maxdsm_update_caldata(ucontrol->value.integer.value[0]);
	return 0;
}
static ssize_t max98927_cal_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return maxdsm_cal_prepare(buf);
}
static DEVICE_ATTR(dsm_cal, S_IRUGO, max98927_cal_show, NULL);
#endif /* USE_DSM_UPDATE_CAL */

#if defined(USE_DSM_LOG) || defined(USE_DSM_UPDATE_CAL)
#define DEFAULT_LOG_CLASS_NAME "dsm"
static const char *class_name_log = DEFAULT_LOG_CLASS_NAME;

static struct attribute *max98927_attributes[] = {
#ifdef USE_DSM_LOG
	&dev_attr_dsm_log.attr,
#endif /* USE_DSM_LOG */
#ifdef USE_DSM_UPDATE_CAL
	&dev_attr_dsm_cal.attr,
#endif /* USE_DSM_UPDATE_CAL */
	NULL
};

static struct attribute_group max98927_attribute_group = {
	.attrs = max98927_attributes
};
#endif /* USE_DSM_LOG || USE_DSM_UPDATE_CAL */
#endif /* CONFIG_SND_SOC_MAXIM_DSM */

static int max98927_dai_set_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);
	unsigned int invert = 0;

	msg_maxim("fmt 0x%08X", fmt);
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		max98927_wrapper_update(max98927, MAX98927B,
				MAX98927_PCM_MASTER_MODE,
				MAX98927_PCM_MASTER_MODE_MSTR_MODE_MASK,
				MAX98927_PCM_MASTER_MODE_MSTR_MODE_SLAVE);
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		max98927->master = true;
		max98927_wrapper_update(max98927, MAX98927B,
				MAX98927_PCM_MASTER_MODE,
				MAX98927_PCM_MASTER_MODE_MSTR_MODE_MASK,
				MAX98927_PCM_MASTER_MODE_MSTR_MODE_MASTER);
		break;
	case SND_SOC_DAIFMT_CBS_CFM:
		max98927_wrapper_update(max98927, MAX98927B,
				MAX98927_PCM_MASTER_MODE,
				MAX98927_PCM_MASTER_MODE_MSTR_MODE_MASK,
				MAX98927_PCM_MASTER_MODE_MSTR_MODE_HYBRID);
	default:
		dev_err(codec->dev, "DAI clock mode unsupported");
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_IB_NF:
		invert = MAX98927_PCM_MODE_CFG_PCM_BCLKEDGE;
		break;
	default:
		dev_err(codec->dev, "DAI invert mode unsupported");
		return -EINVAL;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		max98927->iface |= SND_SOC_DAIFMT_I2S;
		max98927_wrapper_update(max98927, MAX98927B,
				MAX98927_PCM_MODE_CFG,
				max98927->iface, max98927->iface);
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		max98927->iface |= SND_SOC_DAIFMT_LEFT_J;
		max98927_wrapper_update(max98927, MAX98927B,
				MAX98927_PCM_MODE_CFG,
				max98927->iface, max98927->iface);
		break;
	default:
		return -EINVAL;
	}

	max98927_wrapper_update(max98927, MAX98927B,
			MAX98927_PCM_MODE_CFG,
			MAX98927_PCM_MODE_CFG_PCM_BCLKEDGE, invert);

	return 0;
}

/* codec MCLK rate in master mode */
static const int rate_table[] = {
	5644800, 6000000, 6144000, 6500000,
	9600000, 11289600, 12000000, 12288000,
	13000000, 19200000,
};

static int max98927_set_clock(struct max98927_priv *max98927,
		struct snd_pcm_hw_params *params)
{
	/* BCLK/LRCLK ratio calculation */
	int blr_clk_ratio = (params_channels(params) == 1 ? 2 : params_channels(params)) * max98927->ch_size;
	int value = 0;

	if (max98927->master) {
		int i;
		/* match rate to closest value */
		for (i = 0; i < ARRAY_SIZE(rate_table); i++) {
			if (rate_table[i] >= max98927->sysclk)
				break;
		}
		if (i == ARRAY_SIZE(rate_table)) {
			msg_maxim("couldn't get the MCLK to match codec");
			return -EINVAL;
		}
		max98927_wrapper_update(max98927, MAX98927B,
				MAX98927_PCM_MASTER_MODE,
				MAX98927_PCM_MASTER_MODE_MCLK_RATE_MASK,
				i << MAX98927_PCM_MASTER_MODE_MCLK_RATE_SHIFT);
	}

	switch (blr_clk_ratio) {
	case 32:
		value = 2;
		break;
	case 48:
		value = 3;
		break;
	case 64:
		value = 4;
		break;
	default:
		return -EINVAL;
	}
	max98927_wrapper_update(max98927, MAX98927B,
			MAX98927_PCM_CLOCK_SETUP,
			MAX98927_PCM_CLOCK_SETUP_PCM_BSEL_MASK, value);

	return 0;
}

static int max98927_dai_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params,
		struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);
	int sampling_rate = 0;

	msg_maxim("width %d rate %d",
			snd_pcm_format_width(params_format(params)),
			params_rate(params));

	switch (snd_pcm_format_width(params_format(params))) {
	case 16:
		max98927_wrapper_update(max98927, MAX98927B,
				MAX98927_PCM_MODE_CFG,
				MAX98927_PCM_MODE_CFG_PCM_CHANSZ_16,
				MAX98927_PCM_MODE_CFG_PCM_CHANSZ_16);
		max98927->ch_size = 16;
		break;
	case 24:
		max98927_wrapper_update(max98927, MAX98927B,
				MAX98927_PCM_MODE_CFG,
				MAX98927_PCM_MODE_CFG_PCM_CHANSZ_24,
				MAX98927_PCM_MODE_CFG_PCM_CHANSZ_24);
		max98927->ch_size = 24;
		break;
	case 32:
		max98927_wrapper_update(max98927, MAX98927B,
				MAX98927_PCM_MODE_CFG,
				MAX98927_PCM_MODE_CFG_PCM_CHANSZ_32,
				MAX98927_PCM_MODE_CFG_PCM_CHANSZ_32);
		max98927->ch_size = 32;
		break;
	default:
		dev_err(codec->dev, "%s: format unsupported %d",
				__func__, params_format(params));
		goto err;
	}
	msg_maxim("format supported %d", params_format(params));

	switch (params_rate(params)) {
	case 8000:
		sampling_rate |=
			MAX98927_PCM_SR_SETUP_1_DIG_IF_SR_8000;
		break;
	case 11025:
		sampling_rate |=
			MAX98927_PCM_SR_SETUP_1_DIG_IF_SR_11025;
		break;
	case 12000:
		sampling_rate |=
			MAX98927_PCM_SR_SETUP_1_DIG_IF_SR_12000;
		break;
	case 16000:
		sampling_rate |=
			MAX98927_PCM_SR_SETUP_1_DIG_IF_SR_16000;
		break;
	case 22050:
		sampling_rate |=
			MAX98927_PCM_SR_SETUP_1_DIG_IF_SR_22050;
		break;
	case 24000:
		sampling_rate |=
			MAX98927_PCM_SR_SETUP_1_DIG_IF_SR_24000;
		break;
	case 32000:
		sampling_rate |=
			MAX98927_PCM_SR_SETUP_1_DIG_IF_SR_32000;
		break;
	case 44100:
		sampling_rate |=
			MAX98927_PCM_SR_SETUP_1_DIG_IF_SR_44100;
		break;
	case 48000:
		sampling_rate |=
			MAX98927_PCM_SR_SETUP_1_DIG_IF_SR_48000;
		break;
	case 192000:
		sampling_rate |=
			MAX98927_PCM_SR_SETUP_1_DIG_IF_SR_192000;
		break;
	default:
		dev_err(codec->dev, "%s rate %d not supported\n",
				__func__, params_rate(params));
		goto err;
	}

	/* set DAI_SR to correct LRCLK frequency */
	max98927_wrapper_update(max98927, MAX98927B,
			MAX98927_PCM_SR_SETUP_1,
			MAX98927_PCM_SR_SETUP_1_DIG_IF_SR_MASK,
			sampling_rate);
	max98927_wrapper_update(max98927, MAX98927B,
			MAX98927_PCM_SR_SETUP_2,
			MAX98927_PCM_SR_SETUP_2_SPK_SR_MASK,
			sampling_rate<<4);
	max98927_wrapper_update(max98927, MAX98927B,
			MAX98927_PCM_SR_SETUP_2,
			MAX98927_PCM_SR_SETUP_2_IVADC_SR_MASK,
			sampling_rate);

	return max98927_set_clock(max98927, params);

err:
	return -EINVAL;
}

static int max98927_dai_startup(struct snd_pcm_substream *substream,
        struct snd_soc_dai *dai)
{
    struct snd_soc_pcm_runtime *rtd = substream->private_data;
    struct snd_soc_codec *codec = rtd->codec;
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);
	struct max98927_pdata *pdata = max98927->pdata;
    int rdc = 0, temp = 0, ret = 0;

    if (pdata->nodsm || substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
        return 0;

    msg_maxim("substream %s, stream %d", substream->name, substream->stream);

#ifdef CONFIG_SND_SOC_MAXIM_DSM_CAL
    ret = maxdsm_cal_get_rdc(&rdc);
#endif /* CONFIG_SND_SOC_MAXIM_DSM_CAL */
    if (ret || rdc <= 0) {
        msg_maxim("Failed to get rdc (0x%08x)", rdc);
        goto error;
    }

#ifdef CONFIG_SND_SOC_MAXIM_DSM_CAL
    ret = maxdsm_cal_get_temp(&temp);
#endif /* CONFIG_SND_SOC_MAXIM_DSM_CAL */
    if (ret || temp <= 0) {
        msg_maxim("Failed to get temp (%d)", temp);
        goto error;
    }

#ifdef CONFIG_SND_SOC_MAXIM_DSM
	maxdsm_set_rdc_temp(rdc, (int)(temp / 10));
#endif /* CONFIG_SND_SOC_MAXIM_DSM */

error:
    return 0;
}

#define MAX98927_RATES SNDRV_PCM_RATE_8000_48000

#define MAX98927_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | \
		SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

static int max98927_dai_set_sysclk(struct snd_soc_dai *dai,
		int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = dai->codec;
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);

	msg_maxim("clk_id %d, freq %d, dir %d",
			clk_id, freq, dir);

	max98927->sysclk = freq;

	return 0;
}

static int __max98927_spk_enable(struct max98927_priv *max98927)
{
	struct max98927_pdata *pdata = max98927->pdata;
	struct max98927_volume_step_info *vstep = &max98927->vstep;
	unsigned int gain_l, gain_r;
	unsigned int enable_l, enable_r;
	unsigned int vimon = 0;
#ifdef CONFIG_SND_SOC_MAXIM_DSM
	unsigned int smode_table[MAX98927_OSM_MAX] = {
		2, /* MAX98927_OSM_MONO_L */
		1, /* MAX98927_OSM_MONO_R */
		2, /* MAX98927_OSM_RCV_L */
		1, /* MAX98927_OSM_RCV_R */
		0, /* MAX98927_OSM_STEREO */
		3, /* MAX98927_OSM_STEREO_MODE2 */
	};
#endif /* CONFIG_SND_SOC_MAXIM_DSM */

	max98927_wrapper_read(max98927, MAX98927L,
			MAX98927_BST_CTRL_0, &pdata->boostv);

	gain_l = gain_r = max98927->spk_gain;
	enable_l = enable_r = 0x00;

	if (vstep->adc_status && !pdata->nodsm)
	    vimon = MAX98927_MEAS_ENABLES_IVADC_VI_EN;

	switch (pdata->osm) {
	case MAX98927_OSM_STEREO_MODE2:
	case MAX98927_OSM_STEREO:
		enable_l = enable_r = 1;
		break;
	case MAX98927_OSM_RCV_L:
		gain_l = 0x00; /* mute */
		vimon = 0; /* turn off VIMON */
		pdata->boostv &= MAX98927_BST_CTRL_0_EXT_PVDD_EN; /* 6.5V */
	case MAX98927_OSM_MONO_L:
		enable_l = 1;
		break;
	case MAX98927_OSM_RCV_R:
		gain_r = 0x00; /* mute */
		vimon = 0; /* turn off VIMON */
		pdata->boostv &= MAX98927_BST_CTRL_0_EXT_PVDD_EN; /* 6.5V */
	case MAX98927_OSM_MONO_R:
		enable_r = 1;
		break;
	default:
		msg_maxim("Invalid one_stop_mode");
		return -EINVAL;
	}

#ifdef CONFIG_SND_SOC_MAXIM_DSM
	maxdsm_set_stereo_mode_configuration(smode_table[pdata->osm]);
#endif /* CONFIG_SND_SOC_MAXIM_DSM */

	max98927_wrapper_update(max98927, MAX98927L,
			MAX98927_SPK_GAIN,
			MAX98927_SPK_GAIN_SPK_PCM_GAIN_MASK,
			gain_l);
	max98927_wrapper_update(max98927, MAX98927L,
			MAX98927_AMP_VOLUME_CTRL,
			MAX98927_AMP_VOLUME_CTRL_AMP_VOL_MASK,
			max98927->digital_gain);
	if (max98927->mono_stereo) {
		max98927_wrapper_update(max98927, MAX98927R,
				MAX98927_SPK_GAIN,
				MAX98927_SPK_GAIN_SPK_PCM_GAIN_MASK,
				gain_r);
		max98927_wrapper_update(max98927, MAX98927R,
				MAX98927_AMP_VOLUME_CTRL,
				MAX98927_AMP_VOLUME_CTRL_AMP_VOL_MASK,
				max98927->digital_gain);
	}

	max98927_wrapper_write(max98927, MAX98927B,
			MAX98927_BST_CTRL_0,
			pdata->boostv);

	max98927_wrapper_update(max98927, MAX98927B,
			MAX98927_MEAS_ENABLES,
			MAX98927_MEAS_ENABLES_IVADC_VI_EN,
			vimon);
	vstep->adc_status = !!vimon;

	max98927_wrapper_update(max98927, MAX98927L,
			MAX98927_AMP_ENABLES,
			MAX98927_AMP_ENABLES_SPK_EN, enable_l);
	max98927_wrapper_update(max98927, MAX98927L,
			MAX98927_GLOBAL_ENABLE,
			MAX98927_GLOBAL_ENABLE_EN,
			enable_l);
	if (max98927->mono_stereo) {
		max98927_wrapper_update(max98927, MAX98927R,
				MAX98927_AMP_ENABLES,
				MAX98927_AMP_ENABLES_SPK_EN, enable_r);
		max98927_wrapper_update(max98927, MAX98927R,
				MAX98927_GLOBAL_ENABLE,
				MAX98927_GLOBAL_ENABLE_EN,
				enable_r);
	}

	return 0;
}

static void max98927_spk_enable(struct max98927_priv *max98927,
		int mute)
{
	if (!mute)
		__max98927_spk_enable(max98927);
	else {
		max98927_wrapper_update(max98927, MAX98927B,
				MAX98927_GLOBAL_ENABLE, 1, 0);
		max98927_wrapper_update(max98927, MAX98927B,
				MAX98927_AMP_ENABLES, 1, 0);
		/* disable the v and i for vi feedback */
		max98927_wrapper_update(max98927, MAX98927B,
				MAX98927_MEAS_ENABLES,
				MAX98927_MEAS_ENABLES_IVADC_VI_EN,
				0);
	}

#ifdef CONFIG_SND_SOC_MAXIM_DSM
	maxdsm_set_spk_state(!mute);
#endif /* CONFIG_SND_SOC_MAXIM_DSM */
}

static int max98927_dai_digital_mute(struct snd_soc_dai *codec_dai, int mute)
{
	struct max98927_priv *max98927
		= snd_soc_codec_get_drvdata(codec_dai->codec);
	bool action = 1;

	if (max98927->pca.capture_active != codec_dai->capture_active) {
		max98927->pca.capture_active = codec_dai->capture_active;
		action = 0;
	}

	if (max98927->pca.playback_active != codec_dai->playback_active) {
		max98927->pca.playback_active = codec_dai->playback_active;
		action = 1;
	}

	if ((mute && codec_dai->playback_active) && action)
		action = 0;

	msg_maxim("mute=%d playback_active=%d capture_active=%d action=%d",
			mute, max98927->pca.playback_active,
			max98927->pca.capture_active, action);

	if (action)
		max98927_spk_enable(max98927, mute);

	return 0;
}

static const struct snd_soc_dai_ops max98927_dai_ops = {
    .startup = max98927_dai_startup,
	.set_sysclk = max98927_dai_set_sysclk,
	.set_fmt = max98927_dai_set_fmt,
	.hw_params = max98927_dai_hw_params,
	.digital_mute = max98927_dai_digital_mute,
};

static DECLARE_TLV_DB_SCALE(max98927_spk_tlv, 300, 300, 0);
static DECLARE_TLV_DB_SCALE(max98927_digital_tlv, -1600, 25, 0);
static DECLARE_TLV_DB_SCALE(max98927_pdm_tlv, 300, 300, 0);
static DECLARE_TLV_DB_SCALE(max98927_hold_tlv, 0, 100, 0);

static int pdm_l_zero;
static int pdm_l_one;

int max98927_common_pdm(struct max98927_priv *max98927, int channel, int source)
{
	int ret = 0;
	/* enable the pdm receive interface */
	max98927_wrapper_update(max98927, MAX98927B,
			MAX98927_PDM_RX_ENABLE,
			MAX98927_PDM_RX_ENABLE_PDM_RX_EN,
			MAX98927_PDM_RX_ENABLE_PDM_RX_EN);

	/* enable the pdm transmit interface */
	max98927_wrapper_update(max98927, MAX98927B,
			MAX98927_PDM_TX_ENABLES,
			MAX98927_PDM_TX_ENABLES_PDM_TX_EN,
			MAX98927_PDM_TX_ENABLES_PDM_TX_EN);

	/* enable channel */
	if (channel == 0) {
		max98927_wrapper_update(max98927, MAX98927B,
				MAX98927_PDM_RX_ENABLE,
				MAX98927_PDM_RX_ENABLE_PDM_RX_CH_SEL, 0);
	} else {
		max98927_wrapper_update(max98927, MAX98927B,
				MAX98927_PDM_RX_ENABLE,
				MAX98927_PDM_RX_ENABLE_PDM_RX_CH_SEL,
				MAX98927_PDM_RX_ENABLE_PDM_RX_CH_SEL);
	}
	switch (source) {
	case 0:
		/* enable source*/
		if (channel == 1) {
			max98927_wrapper_update(max98927, MAX98927B,
					MAX98927_PDM_TX_CTRL,
					MAX98927_PDM_TX_CTRL_PDM_TX_CH1_SRC,
					MAX98927_PDM_TX_CTRL_PDM_TX_CH1_SRC);
		} else {
			max98927_wrapper_update(max98927, MAX98927B,
					MAX98927_PDM_TX_CTRL,
					MAX98927_PDM_TX_CTRL_PDM_TX_CH0_SRC,
					MAX98927_PDM_TX_CTRL_PDM_TX_CH0_SRC);
		}
		break;
	case 1:
		/* enable source*/
		if (channel == 1) {
			max98927_wrapper_update(max98927, MAX98927B,
					MAX98927_PDM_TX_CTRL,
					MAX98927_PDM_TX_CTRL_PDM_TX_CH1_SRC, 0);
		} else {
			max98927_wrapper_update(max98927, MAX98927B,
					MAX98927_PDM_TX_CTRL,
					MAX98927_PDM_TX_CTRL_PDM_TX_CH0_SRC, 0);
		}
		break;
	default:
		dev_err(max98927->i2c_dev,
				"%s no source found %d\n", __func__, source);
		return -EINVAL;
	}

	return ret;
}

static int max98927_get_pdm_l_zero(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = pdm_l_zero;

	return 0;
}

static int max98927_put_pdm_l_zero(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);
	int ret;

	pdm_l_zero = ucontrol->value.integer.value[0];
	ret = max98927_common_pdm(max98927, 0, pdm_l_zero);
	if (ret < 0)
		return -EINVAL;

	return 0;
}

static int max98927_get_pdm_l_one(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = pdm_l_one;

	return 0;
}

static int max98927_put_pdm_l_one(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);
	int ret;

	pdm_l_one = ucontrol->value.integer.value[0];
	ret = max98927_common_pdm(max98927, 1, pdm_l_one);
	if (ret < 0)
		return -EINVAL;

	return 0;
}

static const char *const pdm_1_text[] = {
	"Current", "Voltage",
};

static const char *const pdm_0_text[] = {
	"Current", "Voltage",
};

static const char * const max98927_boost_voltage_text[] = {
	"6.5V", "6.625V", "6.75V", "6.875V", "7V",
	"7.125V", "7.25V", "7.375V", "7.5V", "7.625V",
	"7.75V", "7.875V", "8V", "8.125V", "8.25V",
	"8.375V", "8.5V", "8.625V", "8.75V", "8.875V",
	"9V", "9.125V", "9.25V", "9.375V", "9.5V",
	"9.625V", "9.75V", "9.875V", "10V"
};

static const char * const max98927_speaker_source_text[] = {
	"i2s", "reserved", "tone", "pdm"
};

static const char * const max98927_monomix_output_text[] = {
	"ch_0", "ch_1", "ch_1_2_div"
};

static const char * const max98927_one_stop_mode_text[] = {
	"Mono Left", "Mono Right",
	"Receiver Left", "Receiver Right",
	"Stereo",
	"Stereo II",
};

static const struct soc_enum max98927_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(pdm_0_text), pdm_0_text),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(pdm_1_text), pdm_1_text),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(max98927_monomix_output_text),
			max98927_monomix_output_text),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(max98927_speaker_source_text),
			max98927_speaker_source_text),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(max98927_boost_voltage_text),
			max98927_boost_voltage_text),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(max98927_one_stop_mode_text),
			max98927_one_stop_mode_text),
};

static int max98927_spk_gain_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = max98927->spk_gain;
	msg_maxim("spk_gain setting returned %d",
			(int) ucontrol->value.integer.value[0]);

	return 0;
}

static int max98927_spk_gain_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);
	unsigned int sel = ucontrol->value.integer.value[0];

	if (sel < ((1 << MAX98927_SPK_GAIN_WIDTH) - 1)) {
		max98927_wrapper_update(max98927, MAX98927B,
				MAX98927_SPK_GAIN,
				MAX98927_SPK_GAIN_SPK_PCM_GAIN_MASK,
				sel);
		max98927->spk_gain = sel;
	}

	msg_maxim("sel %d, set spk_gain to %d", sel, max98927->spk_gain);

	return 0;
}

static int max98927_bde_thres_hyste_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = max98927->thres_hyste;

	return 0;
}

static int max98927_thres_hyste_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);
	unsigned int sel = ucontrol->value.integer.value[0];

	max98927_wrapper_update(max98927, MAX98927B,
			MAX98927_BROWNOUT_THRES_HYST,
			MAX98927_BROWNOUT_THRES_HYST_MASK,
			sel);
	max98927->thres_hyste = sel;

	return 0;
}

static int max98927_bde_l5_amp1_c1_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = max98927->amp1_level;

	return 0;
}

static int max98927_bde_l5_amp1_c1_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);
	unsigned int sel = ucontrol->value.integer.value[0];

	max98927_wrapper_update(max98927, MAX98927B,
			MAX98927_BROWNOUT_LVL_5_AMP_1_CTRL_1,
			MAX98927_BROWNOUT_LVL_5_AMP_1_CTRL_1_MASK,
			sel);
	max98927->amp1_level = sel;

	return 0;
}

static int max98927_bde_l5_amp1_c2_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = max98927->amp2_level;

	return 0;
}

static int max98927_bde_l5_amp1_c2_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);
	unsigned int sel = ucontrol->value.integer.value[0];

	max98927_wrapper_update(max98927, MAX98927B,
			MAX98927_BROWNOUT_LVL_5_AMP_1_CTRL_2,
			MAX98927_BROWNOUT_LVL_5_AMP_1_CTRL_2_MASK,
			sel);
	max98927->amp2_level = sel;

	return 0;
}

static int max98927_bde_l5_amp1_c3_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = max98927->amp3_level;

	return 0;
}

static int max98927_bde_l5_amp1_c3_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);
	unsigned int sel = ucontrol->value.integer.value[0];

	max98927_wrapper_update(max98927, MAX98927B,
			MAX98927_BROWNOUT_LVL_5_AMP_1_CTRL_3,
			MAX98927_BROWNOUT_LVL_5_AMP_1_CTRL_3_MASK,
			sel);
	max98927->amp3_level = sel;

	return 0;
}

static int max98927_bde_l6_amp1_c1_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = max98927->amp1_level6;

	return 0;
}

static int max98927_bde_l6_amp1_c1_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);
	unsigned int sel = ucontrol->value.integer.value[0];

	max98927_wrapper_update(max98927, MAX98927B,
			MAX98927_BROWNOUT_LVL_6_AMP_1_CTRL_1,
			MAX98927_BROWNOUT_LVL_6_AMP_1_CTRL_1_MASK,
			sel);
	max98927->amp1_level6 = sel;

	return 0;
}

static int max98927_bde_l6_amp1_c2_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = max98927->amp2_level6;

	return 0;
}

static int max98927_bde_l6_amp1_c2_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);
	unsigned int sel = ucontrol->value.integer.value[0];

	max98927_wrapper_update(max98927, MAX98927B,
			MAX98927_BROWNOUT_LVL_6_AMP_1_CTRL_2,
			MAX98927_BROWNOUT_LVL_6_AMP_1_CTRL_2_MASK,
			sel);
	max98927->amp2_level6 = sel;

	return 0;
}

static int max98927_bde_l6_amp1_c3_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = max98927->amp3_level6;

	return 0;
}

static int max98927_bde_l6_amp1_c3_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);
	unsigned int sel = ucontrol->value.integer.value[0];

	max98927_wrapper_update(max98927, MAX98927B,
			MAX98927_BROWNOUT_LVL_6_AMP_1_CTRL_3,
			MAX98927_BROWNOUT_LVL_6_AMP_1_CTRL_3_MASK,
			sel);
	max98927->amp3_level6 = sel;

	return 0;
}

static int max98927_bde_l7_amp1_c1_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = max98927->amp1_level7;

	return 0;
}

static int max98927_bde_l7_amp1_c1_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);
	unsigned int sel = ucontrol->value.integer.value[0];

	max98927_wrapper_update(max98927, MAX98927B,
			MAX98927_BROWNOUT_LVL_7_AMP_1_CTRL_1,
			MAX98927_BROWNOUT_LVL_7_AMP_1_CTRL_1_MASK,
			sel);
	max98927->amp1_level7 = sel;

	return 0;
}

static int max98927_bde_l7_amp1_c2_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = max98927->amp2_level7;

	return 0;
}

static int max98927_bde_l7_amp1_c2_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);
	unsigned int sel = ucontrol->value.integer.value[0];

	max98927_wrapper_update(max98927, MAX98927B,
			MAX98927_BROWNOUT_LVL_7_AMP_1_CTRL_2,
			MAX98927_BROWNOUT_LVL_7_AMP_1_CTRL_2_MASK,
			sel);
	max98927->amp2_level7 = sel;

	return 0;
}

static int max98927_bde_l7_amp1_c3_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = max98927->amp3_level7;

	return 0;
}

static int max98927_bde_l7_amp1_c3_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);
	unsigned int sel = ucontrol->value.integer.value[0];

	max98927_wrapper_update(max98927, MAX98927B,
			MAX98927_BROWNOUT_LVL_7_AMP_1_CTRL_3,
			MAX98927_BROWNOUT_LVL_7_AMP_1_CTRL_3_MASK,
			sel);
	max98927->amp3_level7 = sel;

	return 0;
}

static int max98927_bde_l8_amp1_c1_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = max98927->amp1_level8;

	return 0;
}

static int max98927_bde_l8_amp1_c1_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);
	unsigned int sel = ucontrol->value.integer.value[0];

	max98927_wrapper_update(max98927, MAX98927B,
			MAX98927_BROWNOUT_LVL_8_AMP_1_CTRL_1,
			MAX98927_BROWNOUT_LVL_8_AMP_1_CTRL_1_MASK,
			sel);
	max98927->amp1_level8 = sel;

	return 0;
}

static int max98927_bde_l8_amp1_c2_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = max98927->amp2_level8;

	return 0;
}

static int max98927_bde_l8_amp1_c2_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);
	unsigned int sel = ucontrol->value.integer.value[0];

	max98927_wrapper_update(max98927, MAX98927B,
			MAX98927_BROWNOUT_LVL_8_AMP_1_CTRL_2,
			MAX98927_BROWNOUT_LVL_8_AMP_1_CTRL_2_MASK,
			sel);
	max98927->amp2_level8 = sel;

	return 0;
}

static int max98927_bde_l8_amp1_c3_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = max98927->amp3_level8;

	return 0;
}

static int max98927_bde_l8_amp1_c3_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);
	unsigned int sel = ucontrol->value.integer.value[0];

	max98927_wrapper_update(max98927, MAX98927B,
			MAX98927_BROWNOUT_LVL_8_AMP_1_CTRL_3,
			MAX98927_BROWNOUT_LVL_8_AMP_1_CTRL_3_MASK,
			sel);
	max98927->amp3_level8 = sel;

	return 0;
}

static int max98927_bde_amp_limit_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = max98927->amp_limit_rel;

	return 0;
}

static int max98927_bde_amp_limit_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);
	unsigned int sel = ucontrol->value.integer.value[0];

	max98927_wrapper_update(max98927, MAX98927B,
			MAX98927_BROWNOUT_AMP_LIM_ATK_REL,
			MAX98927_BROWNOUT_AMP_LIM_ATK_MASK,
			sel<<4);
	max98927->amp_limit_rel = sel;

	return 0;
}

static int max98927_amp_limit_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = max98927->amp_limit;

	return 0;
}

static int max98927_amp_limit_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);
	unsigned int sel = ucontrol->value.integer.value[0];

	max98927_wrapper_update(max98927, MAX98927B,
			MAX98927_BROWNOUT_AMP_LIM_ATK_REL,
			MAX98927_BROWNOUT_AMP_LIM_RLS_MASK,
			sel);
	max98927->amp_limit = sel;

	return 0;
}

static int max98927_bde_l8_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = max98927->level8_hold;

	return 0;
}

static int max98927_bde_l8_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);
	unsigned int sel = ucontrol->value.integer.value[0];

	max98927_wrapper_update(max98927, MAX98927B,
			MAX98927_BROWNOUT_LVL_8_THRES,
			MAX98927_BROWNOUT_LVL_8_THRES_MASK,
			sel);
	max98927->level8_hold = sel;

	return 0;
}

static int max98927_bde_l7_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = max98927->level7_hold;

	return 0;
}

static int max98927_bde_l7_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);
	unsigned int sel = ucontrol->value.integer.value[0];

	max98927_wrapper_update(max98927, MAX98927B,
			MAX98927_BROWNOUT_LVL_7_THRES,
			MAX98927_BROWNOUT_LVL_7_THRES_MASK,
			sel);
	max98927->level7_hold = sel;

	return 0;
}

static int max98927_bde_l6_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = max98927->level6_hold;

	return 0;
}

static int max98927_bde_l6_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);
	unsigned int sel = ucontrol->value.integer.value[0];

	max98927_wrapper_update(max98927, MAX98927B,
			MAX98927_BROWNOUT_LVL_6_THRES,
			MAX98927_BROWNOUT_LVL_6_THRES_MASK,
			sel);
	max98927->level6_hold = sel;

	return 0;
}

static int max98927_bde_l5_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = max98927->level5_hold;

	return 0;
}

static int max98927_bde_l5_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);
	unsigned int sel = ucontrol->value.integer.value[0];

	max98927_wrapper_update(max98927, MAX98927B,
			MAX98927_BROWNOUT_LVL_5_THRES,
			MAX98927_BROWNOUT_LVL_5_THRES_MASK,
			sel);
	max98927->level5_hold = sel;

	return 0;
}

static int max98927_bde_hold_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = max98927->level_hold;

	return 0;
}

static int max98927_bde_hold_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);
	unsigned int sel = ucontrol->value.integer.value[0];

	max98927_wrapper_update(max98927, MAX98927B,
			MAX98927_BROWNOUT_LVL_HOLD,
			MAX98927_BROWNOUT_LVL_HOLD_MASK,
			sel);
	max98927->level_hold = sel;

	return 0;
}

static int max98927_pdm_gain_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = max98927->pdm_gain;
	msg_maxim("pdm_gain setting returned %d",
			(int) ucontrol->value.integer.value[0]);

	return 0;
}

static int max98927_pdm_gain_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);
	unsigned int sel = ucontrol->value.integer.value[0];

	if (sel < ((1 << MAX98927_PDM_GAIN_WIDTH) - 1)) {
		max98927_wrapper_update(max98927, MAX98927B,
				MAX98927_SPK_GAIN,
				MAX98927_SPK_GAIN_SPK_PDM_GAIN_MASK,
				sel << MAX98927_SPK_GAIN_SPK_PDM_GAIN_SHIFT);
		max98927->pdm_gain = sel;
	}

	return 0;
}

static int max98927_digital_gain_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = max98927->digital_gain;
	msg_maxim("digital_gain setting returned %d",
			(int) ucontrol->value.integer.value[0]);

	return 0;
}

static int max98927_digital_gain_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);
	unsigned int sel = ucontrol->value.integer.value[0];

	if (sel < ((1 << MAX98927_AMP_VOL_WIDTH) - 1)) {
		max98927_wrapper_update(max98927, MAX98927B,
				MAX98927_AMP_VOLUME_CTRL,
				MAX98927_AMP_VOLUME_CTRL_AMP_VOL_MASK,
				sel);
		max98927->digital_gain = sel;
	}

	return 0;
}

static int max98927_bde_amp1_clip_mode_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	return max98927_reg_get(kcontrol, ucontrol,
			MAX98927_BROWNOUT_AMP1_CLIP_MODE,
			MAX98927_BROWNOUT_AMP1_CLIP_MODE_MODE, 0);
}

static int max98927_bde_amp1_clip_mode_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	return max98927_reg_put(kcontrol, ucontrol,
			MAX98927_BROWNOUT_AMP1_CLIP_MODE,
			MAX98927_BROWNOUT_AMP1_CLIP_MODE_MODE, 0);
}

static int max98927_bde_infinite_hold_clear_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	return max98927_reg_get(kcontrol, ucontrol,
			MAX98927_BROWNOUT_LVL_INF_HOLD_CLEAR,
			MAX98927_BROWNOUT_LVL_INF_HOLD_CLEAR_L4, 1);
}

static int max98927_bde_infinite_hold_clear_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	return max98927_reg_put(kcontrol, ucontrol,
			MAX98927_BROWNOUT_LVL_INF_HOLD_CLEAR,
			MAX98927_BROWNOUT_LVL_INF_HOLD_CLEAR_L4, 1);
}

static int max98927_bde_infinite_hold_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	return max98927_reg_get(kcontrol, ucontrol,
			MAX98927_BROWNOUT_LVL_INF_HOLD,
			MAX98927_BROWNOUT_LVL_INF_HOLD_L4, 1);
}

static int max98927_bde_infinite_hold_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	return max98927_reg_put(kcontrol, ucontrol,
			MAX98927_BROWNOUT_LVL_INF_HOLD,
			MAX98927_BROWNOUT_LVL_INF_HOLD_L4, 1);
}

static int max98927_boost_voltage_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	return max98927_reg_get(kcontrol, ucontrol,
			MAX98927_BST_CTRL_0,
			MAX98927_BST_CTRL_0_BST_VOUT_MASK, 0);
}

static int max98927_boost_voltage_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	return max98927_reg_put(kcontrol, ucontrol,
			MAX98927_BST_CTRL_0,
			MAX98927_BST_CTRL_0_BST_VOUT_MASK, 0);
}

static int max98927_amp_vol_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	return max98927_reg_get(kcontrol, ucontrol,
			MAX98927_AMP_VOLUME_CTRL,
			MAX98927_AMP_VOLUME_CTRL_AMP_VOL_SEL,
			MAX98927_AMP_VOL_LOCATION_SHIFT);
}

static int max98927_bde_enable_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	return max98927_reg_put(kcontrol, ucontrol,
			MAX98927_BROWNOUT_ENABLES,
			MAX98927_BROWNOUT_ENABLES_BDE_EN, 0);
}

static int max98927_bde_enable_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	return max98927_reg_get(kcontrol, ucontrol,
			MAX98927_BROWNOUT_ENABLES,
			MAX98927_BROWNOUT_ENABLES_BDE_EN, 0);
}

static int max98927_bde_amp_enable_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	return max98927_reg_put(kcontrol, ucontrol,
			MAX98927_BROWNOUT_ENABLES,
			MAX98927_BROWNOUT_ENABLES_BDE_AMP_EN,
			MAX98927_BDE_AMP_SHIFT);
}

static int max98927_bde_amp_enable_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	return max98927_reg_get(kcontrol, ucontrol,
			MAX98927_BROWNOUT_ENABLES,
			MAX98927_BROWNOUT_ENABLES_BDE_AMP_EN,
			MAX98927_BDE_AMP_SHIFT);
}

static int max98927_amp_dsp_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	return max98927_reg_put(kcontrol, ucontrol,
			MAX98927_BROWNOUT_ENABLES,
			MAX98927_BROWNOUT_ENABLES_AMP_DSP_EN,
			MAX98927_BDE_DSP_SHIFT);
}

static int max98927_amp_dsp_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	return max98927_reg_get(kcontrol, ucontrol,
			MAX98927_BROWNOUT_ENABLES,
			MAX98927_BROWNOUT_ENABLES_AMP_DSP_EN,
			MAX98927_BDE_DSP_SHIFT);
}

static int max98927_ramp_switch_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	return max98927_reg_put(kcontrol, ucontrol,
			MAX98927_AMP_DSP_CFG,
			MAX98927_AMP_DSP_CFG_AMP_VOL_RMP_BYPASS,
			MAX98927_SPK_RMP_EN_SHIFT);
}
static int max98927_ramp_switch_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	return max98927_reg_get(kcontrol, ucontrol,
			MAX98927_AMP_DSP_CFG,
			MAX98927_AMP_DSP_CFG_AMP_VOL_RMP_BYPASS,
			MAX98927_SPK_RMP_EN_SHIFT);
}

static int max98927_dre_en_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	return max98927_reg_put(kcontrol, ucontrol,
			MAX98927_DRE_CTRL,
			MAX98927_DRE_CTRL_DRE_EN, 0);
}
static int max98927_dre_en_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	return max98927_reg_get(kcontrol, ucontrol,
			MAX98927_DRE_CTRL,
			MAX98927_DRE_CTRL_DRE_EN, 0);
}
static int max98927_amp_vol_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	return max98927_reg_put(kcontrol, ucontrol,
			MAX98927_AMP_VOLUME_CTRL,
			MAX98927_AMP_VOLUME_CTRL_AMP_VOL_SEL,
			MAX98927_AMP_VOL_LOCATION_SHIFT);
}
static int max98927_spk_src_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	return max98927_reg_get(kcontrol, ucontrol,
			MAX98927_SPK_SRC_SELECT,
			MAX98927_SPK_SRC_SELECT_SPK_SRC_MASK, 0);
}

static int max98927_spk_src_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	return max98927_reg_put(kcontrol, ucontrol,
			MAX98927_SPK_SRC_SELECT,
			MAX98927_SPK_SRC_SELECT_SPK_SRC_MASK, 0);
}

static int max98927_mono_out_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	return max98927_reg_get(kcontrol, ucontrol,
			MAX98927_PCM_TO_SPK_MONOMIX_A,
			MAX98927_PCM_TO_SPK_MONOMIX_A_CH0_SRC_MASK, 0);
}

static int max98927_mono_out_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	return max98927_reg_put(kcontrol, ucontrol,
			MAX98927_PCM_TO_SPK_MONOMIX_A,
			MAX98927_PCM_TO_SPK_MONOMIX_A_CH0_SRC_MASK, 0);
}

static int max98927_adc_en_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);
	int data = 0;

	max98927_wrapper_read(max98927, MAX98927L,
			MAX98927_MEAS_ENABLES, &data);

	if (data & MAX98927_MEAS_ENABLES_IVADC_VI_EN)
		ucontrol->value.integer.value[0] = 1;
	else
		ucontrol->value.integer.value[0] = 0;

	return 0;
}

static int max98927_adc_en_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);
	struct max98927_pdata *pdata = max98927->pdata;
	struct max98927_volume_step_info *vstep = &max98927->vstep;
	int sel = (int)ucontrol->value.integer.value[0];

	if (!pdata->nodsm) {
		sel = sel > 0 ? MAX98927_MEAS_ENABLES_IVADC_VI_EN : 0;
		max98927_wrapper_update(max98927, MAX98927B,
				MAX98927_MEAS_ENABLES,
				MAX98927_MEAS_ENABLES_IVADC_VI_EN,
				sel);
		vstep->adc_status = !!sel;
	}

	return 0;
}

static int max98927_adc_thres_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);
	struct max98927_volume_step_info *vstep = &max98927->vstep;

	ucontrol->value.integer.value[0] = vstep->adc_thres;

	return 0;
}

static int max98927_adc_thres_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);
	struct max98927_volume_step_info *vstep = &max98927->vstep;
	int ret = 0;

	if (ucontrol->value.integer.value[0] >= MAX98927_VSTEP_0 &&
			ucontrol->value.integer.value[0] <= MAX98927_VSTEP_15)
		vstep->adc_thres = (int)ucontrol->value.integer.value[0];
	else
		ret = -EINVAL;

	return ret;
}

static int max98927_volume_step_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);
	struct max98927_volume_step_info *vstep = &max98927->vstep;

	ucontrol->value.integer.value[0] = vstep->vol_step;

	return 0;
}

static int max98927_volume_step_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);
	struct max98927_pdata *pdata = max98927->pdata;
	struct max98927_volume_step_info *vstep = &max98927->vstep;

	int sel = (int)ucontrol->value.integer.value[0];
	bool adc_status = vstep->adc_status;

	/*
	 * ADC status will be updated according to the volume.
	 * Under step 7 : Disable
	 * Over step 7  : Enable
	 */
	if (!pdata->nodsm) {
		if (sel <= vstep->adc_thres
				&& vstep->adc_status) {
			max98927_wrapper_update(max98927, MAX98927B,
					MAX98927_MEAS_ENABLES,
					MAX98927_MEAS_ENABLES_IVADC_VI_EN,
					0);
			adc_status = !vstep->adc_status;
			pdata->boostv |= MAX98927_BST_CTRL_0_EXT_PVDD_EN;
		} else if (sel > vstep->adc_thres
				&& !vstep->adc_status) {
			max98927_wrapper_update(max98927, MAX98927B,
					MAX98927_MEAS_ENABLES,
					MAX98927_MEAS_ENABLES_IVADC_VI_EN,
					MAX98927_MEAS_ENABLES_IVADC_VI_EN);
			adc_status = !vstep->adc_status;
			pdata->boostv &= ~MAX98927_BST_CTRL_0_EXT_PVDD_EN;
		} else if (sel > MAX98927_VSTEP_MAX) {
			msg_maxim("Unknown value %d", sel);
			return -EINVAL;
		}

		if (adc_status != vstep->adc_status)
			vstep->adc_status = adc_status;
	}

	/*
	 * Boost voltage will be updated according to the volume.
	 * Step 0 ~ Step 13 : 6.5V
	 * Step 14			: 8.0V
	 * Over step 15		: 10V
	 */
	pdata->boostv &= MAX98927_BST_CTRL_0_EXT_PVDD_EN;
	pdata->boostv |= vstep->boost_step[sel];
	max98927_wrapper_write(max98927, MAX98927B,
			MAX98927_BST_CTRL_0,
			pdata->boostv);

	/* Set volume step to ... */
	vstep->vol_step = sel;

	return 0;
}

static int max98927_one_stop_mode_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);
	struct max98927_pdata *pdata = max98927->pdata;

	ucontrol->value.integer.value[0] = pdata->osm;

	return 0;
}

static int max98927_one_stop_mode_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);
	struct max98927_pdata *pdata = max98927->pdata;
	int osm = (int)ucontrol->value.integer.value[0];

	osm = osm < 0 ? 0 : osm;
	if (osm < MAX98927_OSM_MAX &&
			pdata->osm != osm) {
		pdata->osm = osm;
		__max98927_spk_enable(max98927);
	}

	return osm >= MAX98927_OSM_MAX ? -EINVAL : 0;
}

static const struct snd_kcontrol_new max98927_snd_controls[] = {
	SOC_SINGLE_EXT_TLV("Speaker Gain",
			MAX98927_SPK_GAIN,
			0, (1<<MAX98927_SPK_GAIN_WIDTH)-1, 0,
			max98927_spk_gain_get,
			max98927_spk_gain_put,
			max98927_spk_tlv),
	SOC_SINGLE_EXT_TLV("Digital Gain",
			MAX98927_AMP_VOLUME_CTRL,
			0, (1<<MAX98927_AMP_VOL_WIDTH)-1, 0,
			max98927_digital_gain_get,
			max98927_digital_gain_put,
			max98927_digital_tlv),
	SOC_SINGLE_EXT_TLV("Pdm Gain",
			MAX98927_SPK_GAIN,
			MAX98927_PDM_GAIN_SHIFT,
			(1<<MAX98927_PDM_GAIN_WIDTH)-1, 0,
			max98927_pdm_gain_get,
			max98927_pdm_gain_put,
			max98927_pdm_tlv),
	SOC_SINGLE_EXT_TLV("BDE Level Hold",
			MAX98927_BROWNOUT_LVL_HOLD,
			0, 0xff, 0,
			max98927_bde_hold_get,
			max98927_bde_hold_put,
			max98927_hold_tlv),
	SOC_SINGLE_EXT_TLV("BDE Level_5 Threshold",
			MAX98927_BROWNOUT_LVL_5_THRES,
			0, 0xff, 0,
			max98927_bde_l5_get,
			max98927_bde_l5_put,
			max98927_hold_tlv),
	SOC_SINGLE_EXT_TLV("BDE Level_6 Threshold",
			MAX98927_BROWNOUT_LVL_6_THRES,
			0, 0xff, 0,
			max98927_bde_l6_get,
			max98927_bde_l6_put,
			max98927_hold_tlv),
	SOC_SINGLE_EXT_TLV("BDE Level_7 Threshold",
			MAX98927_BROWNOUT_LVL_7_THRES,
			0, 0xff, 0,
			max98927_bde_l7_get,
			max98927_bde_l7_put,
			max98927_hold_tlv),
	SOC_SINGLE_EXT_TLV("BDE Level_8 Threshold",
			MAX98927_BROWNOUT_LVL_8_THRES,
			0, 0xff, 0,
			max98927_bde_l8_get,
			max98927_bde_l8_put,
			max98927_hold_tlv),
	SOC_SINGLE_EXT_TLV("BDE Amp Limiter Release",
			MAX98927_BROWNOUT_AMP_LIM_ATK_REL,
			0, 0xf, 0,
			max98927_amp_limit_get,
			max98927_amp_limit_put,
			max98927_hold_tlv),
	SOC_SINGLE_EXT_TLV("BDE Level_5 Amp_1 Control_1",
			MAX98927_BROWNOUT_LVL_5_AMP_1_CTRL_1,
			0, 0xf, 0,
			max98927_bde_l5_amp1_c1_get,
			max98927_bde_l5_amp1_c1_put,
			max98927_hold_tlv),
	SOC_SINGLE_EXT_TLV("BDE Level_5 Amp_1 Control_2",
			MAX98927_BROWNOUT_LVL_5_AMP_1_CTRL_2,
			0, 0x3f, 0,
			max98927_bde_l5_amp1_c2_get,
			max98927_bde_l5_amp1_c2_put,
			max98927_hold_tlv),
	SOC_SINGLE_EXT_TLV("BDE Level_5 Amp_1 Control_3",
			MAX98927_BROWNOUT_LVL_5_AMP_1_CTRL_3,
			0, 0x3f, 0,
			max98927_bde_l5_amp1_c3_get,
			max98927_bde_l5_amp1_c3_put,
			max98927_hold_tlv),
	SOC_SINGLE_EXT_TLV("BDE Level_6 Amp_1 Control_1",
			MAX98927_BROWNOUT_LVL_6_AMP_1_CTRL_1,
			0, 0xf, 0,
			max98927_bde_l6_amp1_c1_get,
			max98927_bde_l6_amp1_c1_put,
			max98927_hold_tlv),
	SOC_SINGLE_EXT_TLV("BDE Level_6 Amp_1 Control_2",
			MAX98927_BROWNOUT_LVL_6_AMP_1_CTRL_2,
			0, 0x3f, 0,
			max98927_bde_l6_amp1_c2_get,
			max98927_bde_l6_amp1_c2_put,
			max98927_hold_tlv),
	SOC_SINGLE_EXT_TLV("BDE Level_6 Amp_1 Control_3",
			MAX98927_BROWNOUT_LVL_6_AMP_1_CTRL_3,
			0, 0x3f, 0,
			max98927_bde_l6_amp1_c3_get,
			max98927_bde_l6_amp1_c3_put,
			max98927_hold_tlv),
	SOC_SINGLE_EXT_TLV("BDE Level_7 Amp_1 Control_1",
			MAX98927_BROWNOUT_LVL_7_AMP_1_CTRL_1,
			0, 0xf, 0,
			max98927_bde_l7_amp1_c1_get,
			max98927_bde_l7_amp1_c1_put,
			max98927_hold_tlv),
	SOC_SINGLE_EXT_TLV("BDE Level_7 Amp_1 Control_2",
			MAX98927_BROWNOUT_LVL_7_AMP_1_CTRL_2,
			0, 0x3f, 0,
			max98927_bde_l7_amp1_c2_get,
			max98927_bde_l7_amp1_c2_put,
			max98927_hold_tlv),
	SOC_SINGLE_EXT_TLV("BDE Level_7 Amp_1 Control_3",
			MAX98927_BROWNOUT_LVL_7_AMP_1_CTRL_3,
			0, 0x3f, 0,
			max98927_bde_l7_amp1_c3_get,
			max98927_bde_l7_amp1_c3_put,
			max98927_hold_tlv),
	SOC_SINGLE_EXT_TLV("BDE Level_8 Amp_1 Control_1",
			MAX98927_BROWNOUT_LVL_8_AMP_1_CTRL_1,
			0, 0xf, 0,
			max98927_bde_l8_amp1_c1_get,
			max98927_bde_l8_amp1_c1_put,
			max98927_hold_tlv),
	SOC_SINGLE_EXT_TLV("BDE Level_8 Amp_1 Control_2",
			MAX98927_BROWNOUT_LVL_8_AMP_1_CTRL_2,
			0, 0x3f, 0,
			max98927_bde_l8_amp1_c2_get,
			max98927_bde_l8_amp1_c2_put,
			max98927_hold_tlv),
	SOC_SINGLE_EXT_TLV("BDE Level_8 Amp_1 Control_3",
			MAX98927_BROWNOUT_LVL_8_AMP_1_CTRL_3,
			0, 0x3f, 0,
			max98927_bde_l8_amp1_c3_get,
			max98927_bde_l8_amp1_c3_put,
			max98927_hold_tlv),
	SOC_SINGLE_EXT_TLV("BDE Amp Limiter Attack",
			MAX98927_BROWNOUT_AMP_LIM_ATK_REL,
			MAX98927_BROWNOUT_AMP_LIM_ATK_SHIFT,
			0xff, 0,
			max98927_bde_amp_limit_get,
			max98927_bde_amp_limit_put,
			max98927_hold_tlv),
	SOC_SINGLE_EXT_TLV("BDE Threshold Hystersis",
			MAX98927_BROWNOUT_THRES_HYST,
			0, 0xf, 0,
			max98927_bde_thres_hyste_get,
			max98927_thres_hyste_put,
			max98927_hold_tlv),

	SOC_ENUM_EXT("PDM_L_CH_0", max98927_enum[0],
			max98927_get_pdm_l_zero, max98927_put_pdm_l_zero),
	SOC_ENUM_EXT("PDM_L_CH_1", max98927_enum[1],
			max98927_get_pdm_l_one, max98927_put_pdm_l_one),

	SOC_SINGLE_EXT("BDE Infite Hold",
			MAX98927_BROWNOUT_LVL_INF_HOLD,
			1, 1, 0,
			max98927_bde_infinite_hold_get,
			max98927_bde_infinite_hold_put),
	SOC_SINGLE_EXT("BDE Infite Hold Clear",
			MAX98927_BROWNOUT_LVL_INF_HOLD_CLEAR,
			1, 1, 0,
			max98927_bde_infinite_hold_clear_get,
			max98927_bde_infinite_hold_clear_put),
	SOC_SINGLE_EXT("BDE Amp_1 Clip Mode",
			MAX98927_BROWNOUT_AMP1_CLIP_MODE,
			0, 1, 0,
			max98927_bde_amp1_clip_mode_get,
			max98927_bde_amp1_clip_mode_put),
	SOC_SINGLE_EXT("BDE Enable",
			MAX98927_BROWNOUT_ENABLES,
			0, 1, 0,
			max98927_bde_enable_get,
			max98927_bde_enable_put),
	SOC_SINGLE_EXT("BDE Amp Enable",
			MAX98927_BROWNOUT_ENABLES,
			MAX98927_BDE_AMP_SHIFT, 1, 0,
			max98927_bde_amp_enable_get,
			max98927_bde_amp_enable_put),
	SOC_SINGLE_EXT("Amp DSP Enable",
			MAX98927_BROWNOUT_ENABLES,
			MAX98927_BDE_DSP_SHIFT, 1, 0,
			max98927_amp_dsp_get,
			max98927_amp_dsp_put),
	SOC_SINGLE_EXT("Ramp Switch",
			MAX98927_AMP_DSP_CFG,
			MAX98927_SPK_RMP_EN_SHIFT, 1, 1,
			max98927_ramp_switch_get,
			max98927_ramp_switch_put),
	SOC_SINGLE_EXT("DRE EN",
			MAX98927_DRE_CTRL,
			MAX98927_DRE_CTRL_DRE_SHIFT, 1, 0,
			max98927_dre_en_get,
			max98927_dre_en_put),
	SOC_SINGLE_EXT("Amp Volume Location",
			MAX98927_AMP_VOLUME_CTRL,
			MAX98927_AMP_VOL_LOCATION_SHIFT, 1, 0,
			max98927_amp_vol_get,
			max98927_amp_vol_put),

	SOC_ENUM_EXT("Boost Output Voltage", max98927_enum[4],
			max98927_boost_voltage_get, max98927_boost_voltage_put),
	SOC_ENUM_EXT("Speaker Source", max98927_enum[3],
			max98927_spk_src_get, max98927_spk_src_put),
	SOC_ENUM_EXT("Monomix Output", max98927_enum[2],
			max98927_mono_out_get, max98927_mono_out_put),

	SOC_SINGLE_EXT("ADC Enable", 0, 0, 1, 0,
			max98927_adc_en_get, max98927_adc_en_put),
	SOC_SINGLE_EXT("ADC Threshold", SND_SOC_NOPM, 0, 15, 0,
			max98927_adc_thres_get, max98927_adc_thres_put),
	SOC_SINGLE_EXT("Volume Step", SND_SOC_NOPM, 0, 15, 0,
			max98927_volume_step_get, max98927_volume_step_put),
	SOC_ENUM_EXT("One Stop Mode", max98927_enum[5],
			max98927_one_stop_mode_get, max98927_one_stop_mode_put),

#ifdef USE_DSM_LOG
	SOC_SINGLE_EXT("DSM LOG", SND_SOC_NOPM, 0, 3, 0,
		max98927_get_dump_status, max98927_set_dump_status),
#endif /* USE_DSM_LOG */
#ifdef USE_DSM_UPDATE_CAL
	SOC_SINGLE_EXT("DSM SetParam", SND_SOC_NOPM, 0, 1, 0,
		max98927_get_dsm_param, max98927_set_dsm_param),
#endif /* USE_DSM_UPDATE_CAL */
};

static struct snd_soc_dai_driver max98927_dai[] = {
	{
		.name = "max98927-aif1",
		.playback = {
			.stream_name = "HiFi Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MAX98927_RATES,
			.formats = MAX98927_FORMATS,
		},
		.capture = {
			.stream_name = "HiFi Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MAX98927_RATES,
			.formats = MAX98927_FORMATS,
		},
		.ops = &max98927_dai_ops,
	}
};

static void max98927_handle_pdata(struct snd_soc_codec *codec)
{
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);
	struct max98927_pdata *pdata = max98927->pdata;
	struct reg_default *reg_chg;
	int loop = 0;
	int len = 0;
    int val, reg, ret = 0;

	if (!pdata) {
		dev_dbg(codec->dev, "No platform data\n");
		return;
	}

	if (pdata->reg_arr != NULL) {
		len = pdata->reg_arr_len / sizeof(uint32_t);
		for (loop = 0; loop < len; loop += 2) {
			reg_chg = (struct reg_default *)&pdata->reg_arr[loop];
            reg = be32_to_cpu(reg_chg->reg);
            val = be32_to_cpu(reg_chg->def);
			/*msg_maxim("[W][0x%04X, 0x%02X]", reg, val);*/
			max98927_wrapper_write(max98927, MAX98927B, reg, val);
            val = 0;
            ret = max98927_wrapper_read(max98927, MAX98927L, reg, &val);
            /*msg_maxim("[R][0x%04X, 0x%02X] ret %d", reg, val, ret);*/
		}
	}
}

static int max98927_probe(struct snd_soc_codec *codec)
{
	struct max98927_priv *max98927 = snd_soc_codec_get_drvdata(codec);
	struct max98927_pdata *pdata = max98927->pdata;
	struct max98927_volume_step_info *vstep = &max98927->vstep;
	int ret = 0, reg = 0, i;
	unsigned int vimon = 0;

	msg_maxim("Start");

	max98927->codec = codec;
	codec->control_data = max98927->regmap_l;
	codec->cache_bypass = 1;

	ret = max98927_wrapper_read(max98927, MAX98927L,
			MAX98927_REV_ID, &reg);
	msg_maxim("L device version 0x%02X", reg);

	reg = 0;
	if (max98927->mono_stereo) {
		ret = max98927_wrapper_read(max98927, MAX98927R,
				MAX98927_REV_ID, &reg);
		msg_maxim("R device version 0x%02X", reg);
	}

	for (i = 0; i < ARRAY_SIZE(max98927_reg_map); i++)
		max98927_wrapper_write(max98927, MAX98927B,
				max98927_reg_map[i].reg,
				max98927_reg_map[i].def);

	max98927_handle_pdata(codec);

	max98927_wrapper_read(max98927, MAX98927L,
			MAX98927_BST_CTRL_0, &pdata->boostv);

	max98927_wrapper_read(max98927, MAX98927L,
			MAX98927_MEAS_ENABLES, &vimon);
	vstep->adc_status = !!vimon;

#ifdef CONFIG_SND_SOC_MAXIM_DSM
	/* If maxdsm module was already registerd, will be ignored. */
	maxdsm_init();
	if (pdata->pinfo)
		maxdsm_update_info(pdata->pinfo);
#endif /* CONFIG_SND_SOC_MAXIM_DSM */

#if defined(USE_DSM_LOG) || defined(USE_DSM_UPDATE_CAL)
	if (!g_class)
		g_class = class_create(THIS_MODULE, class_name_log);
	max98927->class = g_class;
	if (max98927->class) {
		max98927->dev =
			device_create(max98927->class,
					NULL, 1, NULL, "max98927");
		if (IS_ERR(max98927->dev)) {
			ret = sysfs_create_group(&codec->dev->kobj,
				&max98927_attribute_group);
			if (ret)
				msg_maxim(
				"failed to create sysfs group [%d]", ret);
		} else {
			ret = sysfs_create_group(&max98927->dev->kobj,
				&max98927_attribute_group);
			if (ret)
				msg_maxim(
				"failed to create sysfs group [%d]", ret);
		}
	}
	msg_maxim("g_class=%p %p", g_class, max98927->class);
#endif /* USE_DSM_LOG */

	msg_maxim("End");

	return ret;
}

static const struct snd_soc_codec_driver soc_codec_dev_max98927 = {
	.probe				= max98927_probe,
	.controls			= max98927_snd_controls,
	.num_controls		= ARRAY_SIZE(max98927_snd_controls),
};

static const struct regmap_config max98927_regmap = {
	.reg_bits           = 16,
	.val_bits           = 8,
	.cache_type         = REGCACHE_RBTREE,
	.reg_defaults       = max98927_reg_map,
	.num_reg_defaults   = ARRAY_SIZE(max98927_reg_map),
	.max_register       = MAX98927_REV_ID,
	.readable_reg	    = max98927_readable_register,
	.reg_format_endian  = REGMAP_ENDIAN_DEFAULT,
	.val_format_endian  = REGMAP_ENDIAN_DEFAULT,
};

static int reg_set_optimum_mode_check(struct regulator *reg, int load_ua)
{
	return (regulator_count_voltages(reg) > 0) ?
		regulator_set_optimum_mode(reg, load_ua) : 0;
}

#define VCC_I2C_MIN_UV	1800000
#define VCC_I2C_MAX_UV	1800000
#define I2C_LOAD_UA	300000
static int max98927_regulator_config(struct device *dev)
{
	struct regulator *max98927_vcc_i2c;
	int n_voltages = 0;
	int ret = 0;

	max98927_vcc_i2c = regulator_get(dev, "vcc_i2c");
	if (IS_ERR(max98927_vcc_i2c)) {
		ret = PTR_ERR(max98927_vcc_i2c);
		dev_err(dev, "%s: regulator get failed ret=%d\n",
				__func__, ret);
		goto err_get_vtg_i2c;
	}

	n_voltages = regulator_count_voltages(max98927_vcc_i2c);
	if (n_voltages > 0) {
		ret = regulator_set_voltage(max98927_vcc_i2c,
				VCC_I2C_MIN_UV, VCC_I2C_MAX_UV);
		if (ret) {
			dev_err(dev, "%s: regulator set_vtg failed ret=%d\n",
					__func__, ret);
			goto err_set_vtg_i2c;
		}
		ret = reg_set_optimum_mode_check(max98927_vcc_i2c, I2C_LOAD_UA);
		if (ret < 0) {
			dev_err(dev, "%s: regulator vcc_i2c set_opt failed ret=%d\n",
					__func__, ret);
			goto err_reg_opt_i2c;
		}
	}

	ret = regulator_enable(max98927_vcc_i2c);
	if (ret) {
		dev_err(dev, "%s: regulator vcc_i2c enable failed ret=%d\n",
				__func__, ret);
		goto err_reg_en_vcc_i2c;
	}

	msg_maxim("min_uv:%d max_uv:%d load_ua:%d",
		VCC_I2C_MIN_UV, VCC_I2C_MAX_UV, I2C_LOAD_UA);

	return 0;

err_reg_en_vcc_i2c:
	if (n_voltages > 0)
		regulator_set_optimum_mode(max98927_vcc_i2c, 0);

err_reg_opt_i2c:
	if (n_voltages > 0)
		regulator_set_voltage(max98927_vcc_i2c, 0, VCC_I2C_MAX_UV);

err_set_vtg_i2c:
	regulator_put(max98927_vcc_i2c);

err_get_vtg_i2c:

	return ret;
}

int probe_common(struct i2c_client *i2c, struct max98927_priv *max98927)
{
	int ret = 0, value;

	if (!of_property_read_u32(i2c->dev.of_node,
				"maxim,vmon-l-slot", &value))
		max98927->v_l_slot = value;
	if (!of_property_read_u32(i2c->dev.of_node,
				"maxim,imon-l-slot", &value))
		max98927->i_l_slot = value;
	if (!of_property_read_u32(i2c->dev.of_node,
				"maxim,vmon-r-slot", &value))
		max98927->v_r_slot = value;
	if (!of_property_read_u32(i2c->dev.of_node,
				"maxim,imon-r-slot", &value))
		max98927->i_r_slot = value;

	ret = snd_soc_register_codec(&i2c->dev, &soc_codec_dev_max98927,
			max98927_dai, ARRAY_SIZE(max98927_dai));
	if (ret < 0)
		dev_err(&i2c->dev,
				"Failed to register codec: %d\n", ret);

	return ret;
}

static int max98927_i2c_probe(struct i2c_client *i2c,
		const struct i2c_device_id *id)
{
    struct max98927_priv *max98927;
	struct max98927_pdata *pdata;
	struct max98927_volume_step_info *vstep;
	int ret = 0, value;

	msg_maxim("Start. driver_data %ld", id->driver_data);

    if (id->driver_data == MAX98927R) {
        msg_maxim("We will not support stereo mode");
        return 0;
    }

    max98927 = devm_kzalloc(&i2c->dev,
            sizeof(struct max98927_priv), GFP_KERNEL);
    if (!max98927)
        return -ENOMEM;
    max98927->pdata = devm_kzalloc(&i2c->dev,
            sizeof(struct max98927_pdata), GFP_KERNEL);
    if (!max98927->pdata)
        return -ENOMEM;

	i2c_set_clientdata(i2c, max98927);
	max98927->i2c_dev = &i2c->dev;
	pdata = max98927->pdata;
	vstep = &max98927->vstep;

	switch (id->driver_data) {
	case MAX98927L:
		if (!i2c->dev.of_node) {
			max98927->sysclk = 12288000;
			for (ret = 0; ret < MAX98927_VSTEP_14; ret++)
				vstep->boost_step[ret] = 0x00;
			vstep->boost_step[MAX98927_VSTEP_14] = 0x0C; /* 8V */
			vstep->boost_step[MAX98927_VSTEP_15] = 0x10; /* 8.5V */
			max98927->spk_gain = 0x05; /* +15db for PCM */
			max98927->digital_gain = 0x40; /* 0db */
			max98927->mono_stereo = 0;
			max98927->interleave = 0;
			vstep->adc_thres = MAX98927_VSTEP_8;
			pdata->nodsm = 0;
			break;
		}

		if (of_property_read_bool(
				i2c->dev.of_node, "maxim,i2c-pull-up"))
			max98927_regulator_config(&i2c->dev);

		if (of_property_read_u32_array(i2c->dev.of_node,
				"maxim,platform_info",
				(u32 *)&pdata->pinfo,
				sizeof(pdata->pinfo)/sizeof(uint32_t)))
			dev_warn(&i2c->dev, "set platform_info by default.\n");

		if (of_property_read_u32_array(i2c->dev.of_node,
				"maxim,boost_step",
				(uint32_t *) &vstep->boost_step,
				sizeof(vstep->boost_step)/sizeof(uint32_t))) {
			dev_warn(&i2c->dev, "set boost_step by default.\n");
			for (ret = 0; ret < MAX98927_VSTEP_14; ret++)
				vstep->boost_step[ret] = 0x00;
			vstep->boost_step[MAX98927_VSTEP_14] = 0x0C; /* 8V */
			vstep->boost_step[MAX98927_VSTEP_15] = 0x10; /* 8.5V */
		}

		if (of_property_read_u32(i2c->dev.of_node,
				"maxim,spk-gain",
				&max98927->spk_gain)) {
			dev_warn(&i2c->dev, "set spk_gain by default.\n");
			max98927->spk_gain = 0x05; /* +15db for PCM */
		}

		if (of_property_read_u32(i2c->dev.of_node,
				"maxim,digital-gain",
				&max98927->digital_gain)) {
			dev_warn(&i2c->dev, "set digital_gain by default.\n");
			max98927->digital_gain = 0x40; /* 0db */
		}

		if (of_property_read_u32(i2c->dev.of_node,
				"maxim,sysclk", &max98927->sysclk)) {
			dev_warn(&i2c->dev, "set sysclk by default value.\n");
			max98927->sysclk = 12288000;
		}

		if (!of_property_read_u32(i2c->dev.of_node,
					"maxim,mono_stereo", &value)) {
			max98927->mono_stereo = value;
		} else
			max98927->mono_stereo = 0;
		dev_info(&i2c->dev, "mono_stereo %d\n", max98927->mono_stereo);

		if (!of_property_read_u32(i2c->dev.of_node,
					"maxim,interleave", &value)) {
			if (value > 1)
				dev_warn(&i2c->dev, "interleave number is wrong:\n");
			max98927->interleave = value;
		} else
			max98927->interleave = 0;
		dev_info(&i2c->dev, "interleave %d\n", max98927->interleave);

		if (of_property_read_u32(i2c->dev.of_node,
				"maxim,adc_threshold", &vstep->adc_thres)) {
			dev_warn(&i2c->dev, "set adc_threshold by default.\n");
			vstep->adc_thres = MAX98927_VSTEP_8;
		}

		pdata->reg_arr = of_get_property(i2c->dev.of_node,
				"maxim,registers-of-amp", &pdata->reg_arr_len);

		pdata->nodsm = of_property_read_bool(
				i2c->dev.of_node, "maxim,nodsm");
		msg_maxim("use DSM(%d)", pdata->nodsm);

#ifdef USE_DSM_LOG
		if (of_property_read_string(i2c->dev.of_node,
				"maxim,log_class", &class_name_log)) {
			dev_warn(&i2c->dev, "There is no log_class property.\n");
			class_name_log = DEFAULT_LOG_CLASS_NAME;
		}
#endif /* USE_DSM_LOG */

		max98927->regmap_l =
			devm_regmap_init_i2c(i2c, &max98927_regmap);
		if (IS_ERR(max98927->regmap_l)) {
			ret = PTR_ERR(max98927->regmap_l);
			dev_err(&i2c->dev,
					"Failed to allocate regmap_l: %d\n",
					ret);
		} else
            regcache_cache_bypass(max98927->regmap_l, true);
		break;
	case MAX98927R:
		if (!i2c->dev.of_node) {
			max98927->mono_stereo = 0;
			max98927->interleave = 0;
			break;
		}

		/* Check for second MAX98927 */
		max98927->regmap_r =
			devm_regmap_init_i2c(i2c, &max98927_regmap);
		if (IS_ERR(max98927->regmap_r)) {
			ret = PTR_ERR(max98927->regmap_r);
			dev_err(&i2c->dev,
					"Failed to allocate regmap_r: %d\n",
					ret);
		} else
            regcache_cache_bypass(max98927->regmap_r, true);
		break;
	default:
		dev_err(&i2c->dev, "%s: Wrong driver_data %ld\n",
				__func__, id->driver_data);
		goto error;
	}

	if (max98927->mono_stereo) {
		if (max98927->regmap_r && max98927->regmap_l)
			ret = probe_common(i2c, max98927);
	} else if (max98927->regmap_l && !max98927->mono_stereo) {
		msg_maxim("probe_common call !!");
		ret = probe_common(i2c, max98927);
	}

error:
	msg_maxim("End. driver_data %ld", id->driver_data);

	return ret;
}

static int max98927_i2c_remove(struct i2c_client *client)
{
	struct max98927_priv *max98927 = i2c_get_clientdata(client);
	struct max98927_pdata *pdata = max98927->pdata;

#ifdef CONFIG_SND_SOC_MAXIM_DSM
	maxdsm_deinit();
#endif /* CONFIG_SND_SOC_MAXIM_DSM */
	snd_soc_unregister_codec(&client->dev);
	if (max98927->regmap_l)
		regmap_exit(max98927->regmap_l);
	if (max98927->regmap_r)
		regmap_exit(max98927->regmap_r);
	devm_kfree(&client->dev, pdata);
	devm_kfree(&client->dev, max98927);

	return 0;
}

static const struct i2c_device_id max98927_i2c_id[] = {
	{ "max98927L", MAX98927L },
	{ },
};
MODULE_DEVICE_TABLE(i2c, max98927_i2c_id);

#ifdef CONFIG_OF
static const struct of_device_id max98927_of_match[] = {
	{ .compatible = "maxim,max98927L", },
	{ }
};
#else
#define max98927_of_match NULL
#endif /* CONFIG_OF */
MODULE_DEVICE_TABLE(of, max98927_of_match);

static struct i2c_driver max98927_i2c_driver = {
	.driver = {
		.name = "max98927",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(max98927_of_match),
	},
	.probe  = max98927_i2c_probe,
	.remove = max98927_i2c_remove,
	.id_table = max98927_i2c_id,
};

module_i2c_driver(max98927_i2c_driver)

MODULE_DESCRIPTION("ALSA SoC MAX98927 driver");
MODULE_AUTHOR("Anish kumar <anish.kumar@maximintegrated.com>");
MODULE_LICENSE("GPL");
