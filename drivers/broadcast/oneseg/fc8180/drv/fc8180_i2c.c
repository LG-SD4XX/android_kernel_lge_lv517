/*****************************************************************************
	Copyright(c) 2014 FCI Inc. All Rights Reserved

	File name : fc8180_i2c.c

	Description : source of I2C interface

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
#include "fci_oal.h"
#include "fc8180_spi.h"
#include "fci_hal.h"

#define I2C_READ   0x01 /* read command */
#define I2C_WRITE  0x02 /* write command */
#define I2C_AINC   0x04 /* address increment */

#define CHIP_ADDR           0x58

static s32 i2c_bulkread(HANDLE handle, u8 chip, u8 addr, u8 *data, u16 length)
{
	/* Write your own i2c driver code here for read operation. */

	return BBM_OK;
}

static s32 i2c_bulkwrite(HANDLE handle, u8 chip, u8 addr, u8 *data, u16 length)
{

	/* Write your own i2c driver code here for Write operation. */

	return BBM_OK;
}

static s32 i2c_dataread(HANDLE handle, u8 chip, u8 addr, u8 *data, u32 length)
{
	return i2c_bulkread(handle, chip, addr, data, length);
}

s32 fc8180_bypass_read(HANDLE handle, u8 chip, u8 addr, u8 *data, u16 length)
{
	s32 res;
	u8 bypass_addr = 0x03;
	u8 bypass_data = 1;
	u8 bypass_len  = 1;

	OAL_OBTAIN_SEMAPHORE();
	res = i2c_bulkwrite(handle, CHIP_ADDR, bypass_addr, &bypass_data,
							bypass_len);
	res |= i2c_bulkread(handle, chip, addr, data, length);
	OAL_RELEASE_SEMAPHORE();

	return res;
}

s32 fc8180_bypass_write(HANDLE handle, u8 chip, u8 addr, u8 *data, u16 length)
{
	s32 res;
	u8 bypass_addr = 0x03;
	u8 bypass_data = 1;
	u8 bypass_len  = 1;

	OAL_OBTAIN_SEMAPHORE();
	res = i2c_bulkwrite(handle, CHIP_ADDR, bypass_addr, &bypass_data,
							bypass_len);
	res |= i2c_bulkwrite(handle, chip, addr, data, length);
	OAL_RELEASE_SEMAPHORE();

	return res;
}

s32 fc8180_i2c_init(HANDLE handle, u16 param1, u16 param2)
{
	OAL_CREATE_SEMAPHORE();

#ifdef BBM_I2C_SPI
	fc8180_spi_init(handle, 0, 0);
#else
	/* ts_initialize(); */
#endif

	return BBM_OK;
}

s32 fc8180_i2c_byteread(HANDLE handle, u16 addr, u8 *data)
{
	s32 res;
	u8 command = I2C_READ;

	OAL_OBTAIN_SEMAPHORE();
	res = i2c_bulkwrite(handle, CHIP_ADDR, BBM_ADDRESS_REG,
						(u8 *) &addr, 2);
	res |= i2c_bulkwrite(handle, CHIP_ADDR, BBM_COMMAND_REG, &command, 1);
	res |= i2c_bulkread(handle, CHIP_ADDR, BBM_DATA_REG, data, 1);
	OAL_RELEASE_SEMAPHORE();

	return res;
}

s32 fc8180_i2c_wordread(HANDLE handle, u16 addr, u16 *data)
{
	s32 res;
	u8 command = I2C_READ | I2C_AINC;

	OAL_OBTAIN_SEMAPHORE();
	res = i2c_bulkwrite(handle, CHIP_ADDR, BBM_ADDRESS_REG,
						(u8 *) &addr, 2);
	res |= i2c_bulkwrite(handle, CHIP_ADDR, BBM_COMMAND_REG, &command, 1);
	res |= i2c_bulkread(handle, CHIP_ADDR, BBM_DATA_REG, (u8 *)data, 2);
	OAL_RELEASE_SEMAPHORE();

	return res;
}

s32 fc8180_i2c_longread(HANDLE handle, u16 addr, u32 *data)
{
	s32 res;
	u8 command = I2C_READ | I2C_AINC;

	OAL_OBTAIN_SEMAPHORE();
	res = i2c_bulkwrite(handle, CHIP_ADDR, BBM_ADDRESS_REG,
						(u8 *) &addr, 2);
	res |= i2c_bulkwrite(handle, CHIP_ADDR, BBM_COMMAND_REG, &command, 1);
	res |= i2c_bulkread(handle, CHIP_ADDR, BBM_DATA_REG, (u8 *)data, 4);
	OAL_RELEASE_SEMAPHORE();

	return res;
}

s32 fc8180_i2c_bulkread(HANDLE handle, u16 addr, u8 *data, u16 length)
{
	s32 res;
	u8 command = I2C_READ | I2C_AINC;

	OAL_OBTAIN_SEMAPHORE();
	res = i2c_bulkwrite(handle, CHIP_ADDR, BBM_ADDRESS_REG,
						(u8 *) &addr, 2);
	res |= i2c_bulkwrite(handle, CHIP_ADDR, BBM_COMMAND_REG, &command, 1);
	res |= i2c_bulkread(handle, CHIP_ADDR, BBM_DATA_REG, data, length);
	OAL_RELEASE_SEMAPHORE();

	return res;
}

s32 fc8180_i2c_bytewrite(HANDLE handle, u16 addr, u8 data)
{
	s32 res;
	u8 command = I2C_WRITE;

	OAL_OBTAIN_SEMAPHORE();
	res = i2c_bulkwrite(handle, CHIP_ADDR, BBM_ADDRESS_REG,
						(u8 *) &addr, 2);
	res |= i2c_bulkwrite(handle, CHIP_ADDR, BBM_COMMAND_REG, &command, 1);
	res |= i2c_bulkwrite(handle, CHIP_ADDR, BBM_DATA_REG, (u8 *)&data, 1);
	OAL_RELEASE_SEMAPHORE();

	return res;
}

s32 fc8180_i2c_wordwrite(HANDLE handle, u16 addr, u16 data)
{
	s32 res;
	u8 command = I2C_WRITE | I2C_AINC;

	OAL_OBTAIN_SEMAPHORE();
	res = i2c_bulkwrite(handle, CHIP_ADDR, BBM_ADDRESS_REG,
						(u8 *) &addr, 2);
	res |= i2c_bulkwrite(handle, CHIP_ADDR, BBM_COMMAND_REG, &command, 1);
	res |= i2c_bulkwrite(handle, CHIP_ADDR, BBM_DATA_REG, (u8 *)&data, 2);
	OAL_RELEASE_SEMAPHORE();

	return res;
}

s32 fc8180_i2c_longwrite(HANDLE handle, u16 addr, u32 data)
{
	s32 res;
	u8 command = I2C_WRITE | I2C_AINC;

	OAL_OBTAIN_SEMAPHORE();
	res = i2c_bulkwrite(handle, CHIP_ADDR, BBM_ADDRESS_REG,
						(u8 *) &addr, 2);
	res |= i2c_bulkwrite(handle, CHIP_ADDR, BBM_COMMAND_REG, &command, 1);
	res |= i2c_bulkwrite(handle, CHIP_ADDR, BBM_DATA_REG, (u8 *)&data, 4);
	OAL_RELEASE_SEMAPHORE();

	return res;
}

s32 fc8180_i2c_bulkwrite(HANDLE handle, u16 addr, u8 *data, u16 length)
{
	s32 res;
	u8 command = I2C_WRITE | I2C_AINC;

	OAL_OBTAIN_SEMAPHORE();
	res = i2c_bulkwrite(handle, CHIP_ADDR, BBM_ADDRESS_REG,
						(u8 *) &addr, 2);
	res |= i2c_bulkwrite(handle, CHIP_ADDR, BBM_COMMAND_REG, &command, 1);
	res |= i2c_bulkwrite(handle, CHIP_ADDR, BBM_DATA_REG, data, length);
	OAL_RELEASE_SEMAPHORE();

	return res;
}

s32 fc8180_i2c_dataread(HANDLE handle, u16 addr, u8 *data, u32 length)
{
	s32 res;
	u8 command = I2C_READ;

#ifdef BBM_I2C_SPI
	res = fc8180_spi_dataread(handle, addr, data, length);
#else
	OAL_OBTAIN_SEMAPHORE();
	res = i2c_bulkwrite(handle, CHIP_ADDR, BBM_ADDRESS_REG,
						(u8 *) &addr, 2);
	res |= i2c_bulkwrite(handle, CHIP_ADDR, BBM_COMMAND_REG, &command, 1);
	res |= i2c_dataread(handle, CHIP_ADDR, BBM_DATA_REG, data, length);
	OAL_RELEASE_SEMAPHORE();
#endif

	return res;
}

s32 fc8180_i2c_deinit(HANDLE handle)
{
#ifdef BBM_I2C_SPI
	fc8180_spi_deinit(handle);
#else
	bbm_write(handle, BBM_TS_SEL, 0x00);
	/* ts_receiver_disable(); */
#endif

	OAL_DELETE_SEMAPHORE();

	return BBM_OK;
}

