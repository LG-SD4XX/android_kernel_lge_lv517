#ifndef __CMD_H__
#define __CMD_H__


#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>

#include <linux/input/epack_core.h>
#define DEFAULT_FW_PATH "epack/tf8/EM-P100_0102_0401_AP.bin"
#define PAGE_SIZE_EPACK                      0x00000200     /* Page size */

#define PACKET_SIZE	64
#define FILE_BUFFER	128
#define MAX_FWPATH_SIZE 255

#define CMD_UPDATE_APROM	0x000000A0
#define CMD_UPDATE_CONFIG	0x000000A1
#define CMD_READ_CONFIG		0x000000A2
#define CMD_ERASE_ALL		0x000000A3
#define CMD_SYNC_PACKNO		0x000000A4
#define CMD_GET_FWVER		0x000000A6
#define CMD_APROM_SIZE		0x000000AA
#define CMD_RUN_APROM		0x000000AB
#define CMD_RUN_LDROM		0x000000AC
#define CMD_RESET			0x000000AD

#define CMD_GET_DEVICEID	0x000000B1

#define CMD_PROGRAM_WOERASE 	0x000000C2
#define CMD_PROGRAM_WERASE 	 	0x000000C3
#define CMD_READ_CHECKSUM 	 	0x000000C8
#define CMD_WRITE_CHECKSUM 	 	0x000000C9
#define CMD_GET_FLASHMODE 	 	0x000000CA

#define APROM_MODE	1
#define LDROM_MODE	2

//extern u8 imageBegin, imageEnd;
extern u8 rcvbuf[PACKET_SIZE];
extern u8 sendbuf[PACKET_SIZE];
extern unsigned int g_packno;
extern unsigned short gcksum;

bool send_data(struct i2c_client *client);
bool rcv_data(struct i2c_client *client);
//void SysTimerDelay(int us);//unit=0.5us
int check_sum (unsigned char *buf, int len);
void WordsCpy(void *dest, void *src, int size);

void print_array(u8* buf,int size);
bool cmd_sync_packno(struct i2c_client *client);
bool cmd_get_checksum(struct i2c_client *client, int flag, int start, int len, unsigned short *cksum);
bool cmd_get_deviceID(struct i2c_client *client, int flag, unsigned int *devid);
bool cmd_get_config(struct i2c_client *client, int flag, unsigned int *config);
bool cmd_put_aprom_size(struct i2c_client *client, int flag, unsigned int apsize);
bool cmd_erase_all_chip(struct i2c_client *client, int flag);
bool cmd_update_aprom(struct i2c_client *client, int flag, const char *filename,struct device *device);
bool cmd_fw_version(struct i2c_client *client, int flag, unsigned int *fwver);
bool cmd_run_cmd(struct i2c_client *client,unsigned int  cmd, int *data);

#endif//__CMD_H__


#define CMD_GET_FWVER		0x000000A6
#define CMD_GET_DEVICEID	0x000000B1
