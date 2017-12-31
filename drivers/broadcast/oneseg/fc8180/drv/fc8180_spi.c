/*****************************************************************************
	Copyright(c) 2014 FCI Inc. All Rights Reserved

	File name : fc8180_spi.c

	Description : source of SPI interface

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
#include <linux/spi/spi.h>
#include <linux/slab.h>

#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/module.h>


#include "broadcast_fc8180.h"
#include "fci_types.h"
#include "fc8180_regs.h"
#include "fci_oal.h"

#define SPI_BMODE           0x00
#define SPI_WMODE           0x10
#define SPI_LMODE           0x20
#define SPI_READ            0x40
#define SPI_WRITE           0x00
#define SPI_AINC            0x80
#define CHIPID              (0 << 3)

#define DRIVER_NAME "isdbt"

struct spi_device *fc8180_spi = NULL;

static u8 tx_data[10];

#ifdef QUP_GSBI_SPI_USE_DMOV
#define TX_RX_DATA_BUF_SIZE (8192)

static u8 *wdata_buf;
static u8 *rdata_buf;
static u8 wx_data_buf[TX_RX_DATA_BUF_SIZE+32] = {0,};
static u8 rx_data_buf[TX_RX_DATA_BUF_SIZE+32] = {0,};
#else
static u8 rdata_buf[8192] = {0};
static u8 wdata_buf[8192] = {0};
#endif

#define BUFFER_SIZE (4 << 10)
struct spi_message spi_msg;

struct spi_transfer spi_xfer;
u8 *tx_buf;

static DEFINE_MUTEX(lock);

static int fc8180_spi_write_then_read(struct spi_device *spi
	, u8 *txbuf, u16 tx_length, u8 *rxbuf, u16 rx_length)
{
    int res = 0;

    struct spi_message    message;
    struct spi_transfer    x;

    spi_message_init(&message);
    memset(&x, 0, sizeof x);

    spi_message_add_tail(&x, &message);

    memcpy(&wdata_buf[0], txbuf, tx_length);

    x.tx_buf = &wdata_buf[0];
    x.rx_buf = &rdata_buf[0];
    x.len = tx_length + rx_length;
    x.cs_change = 0;
    x.bits_per_word = 8;
    res = spi_sync(spi, &message);

    memcpy(rxbuf, x.rx_buf + tx_length, rx_length);

    return res;
}

static s32 spi_bulkread(HANDLE handle, u16 addr, u8 command, u8 *data,
							u32 length)
{
    int res;

	tx_data[0] = addr & 0xff;
	tx_data[1] = (addr >> 8) & 0xff;
	tx_data[2] = (command & 0xf0) | CHIPID | ((length >> 16) & 0x07);
	tx_data[3] = (length >> 8) & 0xff;
	tx_data[4] = length & 0xff;

	res = fc8180_spi_write_then_read(fc8180_spi
		, &tx_data[0], 5, data, length);

	if (res) {
		print_log(0, "fc8180_spi_bulkread fail : %d\n", res);
		return BBM_NOK;
	}

	return BBM_OK;
}

static s32 spi_bulkwrite(HANDLE handle, u16 addr, u8 command, u8 *data,
							u32 length)
{
	int i;
	int res;

	tx_data[0] = addr & 0xff;
	tx_data[1] = (addr >> 8) & 0xff;
	tx_data[2] = (command & 0xf0) | CHIPID | ((length >> 16) & 0x07);
	tx_data[3] = (length >> 8) & 0xff;
	tx_data[4] = length & 0xff;

	for (i = 0; i < length; i++)
	{
		tx_data[5+i] = data[i];
	}

	res = fc8180_spi_write_then_read(fc8180_spi
		, &tx_data[0], length+5, NULL, 0);

	if (res) {
		print_log(0, "fc8180_spi_bulkwrite fail : %d\n", res);
		return BBM_NOK;
	}

	return BBM_OK;
}


static s32 spi_dataread(HANDLE handle, u16 addr, u8 command, u8 *data,
							u32 length)
{
	int res;

	tx_data[0] = addr & 0xff;
	tx_data[1] = (addr >> 8) & 0xff;
	tx_data[2] = (command & 0xf0) | CHIPID | ((length >> 16) & 0x07);
	tx_data[3] = (length >> 8) & 0xff;
	tx_data[4] = length & 0xff;

	res = fc8180_spi_write_then_read(fc8180_spi
		, &tx_data[0], 5, data, length);

	if (res) {
		print_log(0, "fc8180_spi_dataread fail : %d\n", res);
		return BBM_NOK;
	}

	return BBM_OK;
}


s32 fc8180_spi_init(HANDLE handle, u16 param1, u16 param2)
{

	fc8180_spi = FCI_GET_SPI_DRIVER();

	print_log(0, "fc8180_spi_init \n");

#ifdef QUP_GSBI_SPI_USE_DMOV
	wdata_buf = (u8 *)(((u32)wx_data_buf + 31) & ~31);
	rdata_buf = (u8 *)(((u32)rx_data_buf + 31) & ~31);
#endif

	return BBM_OK;
}

s32 fc8180_spi_byteread(HANDLE handle, u16 addr, u8 *data)
{
	s32 res;
	u8 command = SPI_READ;

	mutex_lock(&lock);
	res = spi_bulkread(handle, addr, command, data, 1);
	mutex_unlock(&lock);
	return res;
}

s32 fc8180_spi_wordread(HANDLE handle, u16 addr, u16 *data)
{
	s32 res;
	u8 command = SPI_READ | SPI_AINC;

	mutex_lock(&lock);
	res = spi_bulkread(handle, addr, command, (u8 *) data, 2);
	mutex_unlock(&lock);
	return res;
}

s32 fc8180_spi_longread(HANDLE handle, u16 addr, u32 *data)
{
	s32 res;
	u8 command = SPI_READ | SPI_AINC;

	mutex_lock(&lock);
	res = spi_bulkread(handle, addr, command, (u8 *) data, 4);
	mutex_unlock(&lock);
	return res;
}

s32 fc8180_spi_bulkread(HANDLE handle, u16 addr, u8 *data, u16 length)
{
	s32 res;
	u8 command = SPI_READ | SPI_AINC;

	mutex_lock(&lock);
	res = spi_bulkread(handle, addr, command, data, length);
	mutex_unlock(&lock);
	return res;
}

s32 fc8180_spi_bytewrite(HANDLE handle, u16 addr, u8 data)
{
	s32 res;
	u8 command = SPI_WRITE;

	mutex_lock(&lock);

#ifdef BBM_SPI_IF
	if (addr == BBM_DM_DATA) {
#ifdef BBM_SPI_PHA_1
		u8 ifcommand = 0xff;
		u16 ifaddr = 0xffff;
		u8 ifdata = 0xff;
#else
		u8 ifcommand = 0x00;
		u16 ifaddr = 0x0000;
		u8 ifdata = 0x00;
#endif
		res = spi_bulkwrite(handle, ifaddr, ifcommand, &ifdata, 1);
    } else
#endif
	res = spi_bulkwrite(handle, addr, command, (u8 *) &data, 1);
	mutex_unlock(&lock);
	return res;
}

s32 fc8180_spi_wordwrite(HANDLE handle, u16 addr, u16 data)
{
	s32 res;
	u8 command = SPI_WRITE | SPI_AINC;

	mutex_lock(&lock);
	res = spi_bulkwrite(handle, addr, command, (u8 *) &data, 2);
	mutex_unlock(&lock);
	return res;
}

s32 fc8180_spi_longwrite(HANDLE handle, u16 addr, u32 data)
{
	s32 res;
	u8 command = SPI_WRITE | SPI_AINC;

	mutex_lock(&lock);
	res = spi_bulkwrite(handle, addr, command, (u8 *) &data, 4);
	mutex_unlock(&lock);
	return res;
}

s32 fc8180_spi_bulkwrite(HANDLE handle, u16 addr, u8 *data, u16 length)
{
	s32 res;
	u8 command = SPI_WRITE | SPI_AINC;

	mutex_lock(&lock);
	res = spi_bulkwrite(handle, addr, command, data, length);
	mutex_unlock(&lock);
	return res;
}

s32 fc8180_spi_dataread(HANDLE handle, u16 addr, u8 *data, u32 length)
{
    s32 res = 0;
    u8 command = SPI_READ;
#if defined(CONFIG_ARCH_MT6582) || defined(CONFIG_ARCH_MT6753)
    u32 rx_len = 0;
    u32 remain = length;

    mutex_lock(&lock);
    if ((length + 5) > 1024) {
        rx_len = 1024 * (length / 1024) - 5;
        res |= spi_dataread(handle, addr, command, data, rx_len);
        remain = length - rx_len;

        if ((remain + 5) > 1024) {
            res |= spi_dataread(handle, addr, command, &data[rx_len], 1024 - 5);
            remain -= (1024 - 5);
            rx_len += (1024 - 5);
        }
    }
    res |= spi_dataread(handle, addr, command, &data[rx_len], remain);

    mutex_unlock(&lock);
#else
    mutex_lock(&lock);
	res = spi_dataread(handle, addr, command, data, length);
    mutex_unlock(&lock);
#endif

    return res;
}

s32 fc8180_spi_deinit(HANDLE handle)
{
	return BBM_OK;
}
