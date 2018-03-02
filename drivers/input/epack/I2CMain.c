#include "cmd.h"
#include "I2CMain.h"

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

#define USE_INTERRUPT	1
#define HEXA 0x100

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C (Master) Callback Function									                                   */
/*---------------------------------------------------------------------------------------------------------*/

#if USE_INTERRUPT	  // Use interrupt.
bool I2C_MasterSendData(struct i2c_client *client)
{
	bool result;
	struct i2c_msg msgs = {
		
	.addr = client->addr,
	.flags = client->flags,
	.len = 64,
	.buf = (u8 *)sendbuf,
	};

	epack_log("%s sendbuf : \n", __func__);
 	print_array(sendbuf,PACKET_SIZE);

	result = i2c_transfer(client->adapter, &msgs, 1);

	return result;
}

bool I2C_MasterRcvData(struct i2c_client *client)
{
	bool result;
	

	struct i2c_msg msgs = {
	
	.addr = client->addr,
	.flags = client->flags | I2C_M_RD, 
	.len = 64,
	.buf = rcvbuf,
	};

	result = i2c_transfer(client->adapter, &msgs, 1);

	epack_log("%s rcvbuf : \n", __func__);
 	print_array(rcvbuf,PACKET_SIZE);

	return result;
}
#endif



bool send_data(struct i2c_client *client)
{
	bool result;

	gcksum = check_sum(sendbuf, PACKET_SIZE);

	result = I2C_MasterSendData(client);

	return result;
}

bool RcvData(struct i2c_client *client)
{
	bool result;
	unsigned short lcksum;
	int rcv_g_packno;
	u8 *buf;

	msleep(50);//50ms

	result = I2C_MasterRcvData(client);

	if(result < 0)
		return result;

	buf = rcvbuf;
	memcpy(&lcksum, buf, 2);
	buf += 4;

	rcv_g_packno = buf[0]+ (HEXA)*buf[1] + (HEXA)*(HEXA)*buf[2] + (HEXA)*(HEXA)*(HEXA)*buf[3];
	epack_log("%s   buf[0]: %x buf[1]: %x buf[2]: %x buf[3]: %x\n", __func__,buf[0],buf[1],buf[2],buf[3]);

	//if((unsigned short)(buf[0]) != g_packno)
	if(rcv_g_packno != g_packno)
	{
		epack_log("%s g_packno=%d rcv %d\n", __func__, g_packno, rcv_g_packno );
		result = 0;
	}
	else
	{
		if(lcksum != gcksum)
		{
			epack_log("%s gcksum=%x lcksum=%x\n", __func__, gcksum, lcksum);
			result = 0;
		}
		g_packno++;
	}
	return result;
}