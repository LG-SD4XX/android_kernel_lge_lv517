/*****************************************************************************
	Copyright(c) 2014 FCI Inc. All Rights Reserved

	File name : fci_tun.c

	Description : source of tuner control driver

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program; if not, write to the Free Software
	Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

	History :
	----------------------------------------------------------------------
*******************************************************************************/
#include "fci_types.h"
#include "fc8180_regs.h"
#include "fci_hal.h"
#include "fci_tun.h"
#include "fci_hpi.h"
#include "fc8180_bb.h"
#include "fc8180_tun.h"

#ifdef BBM_FPGA
#include "fci_i2c.h"
#include "fci_bypass.h"
#include "fc2582_tun.h"
#endif

#ifdef BBM_FPGA
#define FC2582_TUNER_ADDR       0x56
#endif

#define FC8180_TUNER_ADDR       0xaa

struct I2C_DRV {
	s32 (*init)(HANDLE handle, s32 speed, s32 slaveaddr);
	s32 (*read)(HANDLE handle, u8 chip, u8 addr, u8 alen, u8 *data,
							u8 len);
	s32 (*write)(HANDLE handle, u8 chip, u8 addr, u8 alen, u8 *data,
							u8 len);
	s32 (*deinit)(HANDLE handle);
};

static struct I2C_DRV fcihpi = {
	&fci_hpi_init,
	&fci_hpi_read,
	&fci_hpi_write,
	&fci_hpi_deinit
};

#ifdef BBM_FPGA
static struct I2C_DRV fcii2c = {
	&fci_i2c_init,
	&fci_i2c_read,
	&fci_i2c_write,
	&fci_i2c_deinit
};

static struct I2C_DRV fcibypass = {
	&fci_bypass_init,
	&fci_bypass_read,
	&fci_bypass_write,
	&fci_bypass_deinit
};
#endif

struct TUNER_DRV {
	s32 (*init)(HANDLE handle, enum BAND_TYPE band);
	s32 (*set_freq)(HANDLE handle, u32 freq);
	s32 (*get_rssi)(HANDLE handle, s32 *rssi);
	s32 (*deinit)(HANDLE handle);
};

static struct TUNER_DRV fc8180_tuner = {
	&fc8180_tuner_init,
	&fc8180_set_freq,
	&fc8180_get_rssi,
	&fc8180_tuner_deinit
};

#ifdef BBM_FPGA
static struct TUNER_DRV fc2582_tuner = {
	&fc2582_tuner_init,
	&fc2582_set_freq,
	&fc2582_get_rssi,
	&fc2582_tuner_deinit
};
#endif

static u8 tuner_addr = FC8180_TUNER_ADDR;
static enum BAND_TYPE tuner_band = ISDBT_1_SEG_TYPE;
static enum I2C_TYPE tuner_i2c = FCI_HPI_TYPE;
static struct I2C_DRV *tuner_ctrl = &fcihpi;
static struct TUNER_DRV *tuner = &fc8180_tuner;

s32 tuner_ctrl_select(HANDLE handle, enum I2C_TYPE type)
{
	switch (type) {
	case FCI_HPI_TYPE:
		tuner_ctrl = &fcihpi;
		break;
#ifdef BBM_FPGA
	case FCI_I2C_TYPE:
		tuner_ctrl = &fcii2c;
		break;
	case FCI_BYPASS_TYPE:
		tuner_ctrl = &fcibypass;
		break;
#endif
	default:
		return BBM_E_TN_CTRL_SELECT;
	}

	if (tuner_ctrl->init(handle, 400, 0))
		return BBM_E_TN_CTRL_INIT;

	tuner_i2c = type;

	return BBM_OK;
}

s32 tuner_ctrl_deselect(HANDLE handle)
{
	if (tuner_ctrl == NULL)
		return BBM_E_TN_CTRL_SELECT;

	tuner_ctrl->deinit(handle);

	tuner_i2c = FCI_HPI_TYPE;
	tuner_ctrl = &fcihpi;

	return BBM_OK;
}

s32 tuner_i2c_read(HANDLE handle, u8 addr, u8 alen, u8 *data, u8 len)
{
	if (tuner_ctrl == NULL)
		return BBM_E_TN_CTRL_SELECT;

	if (tuner_ctrl->read(handle, tuner_addr, addr, alen, data, len))
		return BBM_E_TN_READ;

	return BBM_OK;
}

s32 tuner_i2c_write(HANDLE handle, u8 addr, u8 alen, u8 *data, u8 len)
{
	if (tuner_ctrl == NULL)
		return BBM_E_TN_CTRL_SELECT;

	if (tuner_ctrl->write(handle, tuner_addr, addr, alen, data, len))
		return BBM_E_TN_WRITE;

	return BBM_OK;
}

s32 tuner_set_freq(HANDLE handle, u32 freq)
{
	if (tuner == NULL)
		return BBM_E_TN_SELECT;

#if (BBM_BAND_WIDTH == 8)
	freq -= 460;
#else
	freq -= 380;
#endif

	if (tuner->set_freq(handle, freq))
		return BBM_E_TN_SET_FREQ;

	fc8180_reset(handle);

	return BBM_OK;
}

s32 tuner_select(HANDLE handle, enum PRODUCT_TYPE product, enum BAND_TYPE band)
{
	switch (product) {
	case FC8180_TUNER:
		tuner = &fc8180_tuner;
		tuner_addr = FC8180_TUNER_ADDR;
		tuner_band = band;
		break;
#ifdef BBM_FPGA
	case FC2582_TUNER:
		bbm_write(handle, 0x1000, 0x25);

		tuner = &fc2582_tuner;
		tuner_addr = FC2582_TUNER_ADDR;
		tuner_band = band;
		break;
#endif
	default:
		return BBM_E_TN_SELECT;
	}

	if (tuner == NULL)
		return BBM_E_TN_SELECT;

#ifdef BBM_FPGA
	if (tuner_i2c == FCI_BYPASS_TYPE)
		bbm_write(handle, BBM_RF_DEVID, tuner_addr);
#endif

	if (tuner->init(handle, tuner_band))
		return BBM_E_TN_INIT;

	return BBM_OK;
}

s32 tuner_deselect(HANDLE handle)
{
	if (tuner == NULL)
		return BBM_E_TN_SELECT;

	if (tuner->deinit(handle))
		return BBM_NOK;

	tuner = NULL;

	return BBM_OK;
}

s32 tuner_get_rssi(HANDLE handle, s32 *rssi)
{
	if (tuner == NULL)
		return BBM_E_TN_SELECT;

	if (tuner->get_rssi(handle, rssi))
		return BBM_E_TN_RSSI;

	return BBM_OK;
}

