/* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 Copyright ?2012 Synaptics Incorporated. All rights reserved.

 The information in this file is confidential under the terms
 of a non-disclosure agreement with Synaptics and is provided
 AS IS.

 The information in this file shall remain the exclusive property
 of Synaptics and may be the subject of Synaptics?patents, in
 whole or part. Synaptics?intellectual property rights in the
 information in this file are not expressly or implicitly licensed
 or otherwise transferred to you as a result of such information
 being made available to you.
 +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#include <linux/kernel.h>	/* printk */
#include <linux/delay.h>	/* msleep */
#include <linux/time.h>		/* struct timeval t_interval[TIME_PROFILE_MAX] */
#include <linux/math64.h>	/* for abs func */
#include <linux/string.h>	/* memset */
#include <linux/i2c.h>

/*
 *  Include to touch core Header File
 */
#include <touch_hwif.h>
#include <touch_core.h>

/*
 *  Include to Local Header File
 */
#include "touch_s3330.h"
#include "touch_s3330_prd.h"

#define TRX_MAX 32
#define TRX_BITMAP_LENGTH	4
#define TRX_mapping_max 54

//#define LOWER_ABS_RAW_CAP_LIMIT 20000	/* fF */
//#define UPPER_ABS_RAW_CAP_LIMIT 50000	/* fF */
//#define LOWER_ABS_LPWG_RAW_CAP_LIMIT 15000	/* fF */
//#define UPPER_ABS_LPWG_RAW_CAP_LIMIT 60000	/* fF */
#define REPORT_DATA_OFFEST 3
#define VERSION "1.0"

#define get_time_interval(a, b) (a >= b ? a - b : 1000000 + a - b)

enum {
	STARTTIME,
	ENDTIME,
	TIME_PROFILE_MAX
};

enum {
	eRT_DataCollect = 99,
	eRT_ExtendedHighResistance = 98,
	eRT_Configuration = 97,
	eRT_Attention = 96,
	eRT_Package = 95,
	eRT_ExternalReset = 94,
	eRT_Normalized16BitImageReport = 2,
	eRT_RawImageRT3 = 3,	/* Raw 16-Bit Image Report */
	eRT_FullRawCBC_on = 201,
	eRT_FullRawCBC_off = 202,
	eRT_FullRawCapacitance = 20,	/* Full Raw Capacitance with Receiver Offset Removed */
	eRT_ADCRange = 23,
	eRT_SensorSpeed = 22,
	eRT_TRexOpen = 24,	/* no sensor */
	eRT_TRexGround = 25,	/* no sensor */
	eRT_TRexShort = 26,	/* no sensor */
	eRT_ExtendedTRexShort = 261,
	eRT_HighResistance = 4,
	eRT_FullRawCapacitanceMaxMin = 13,
	eRT_AbsADCRange = 42,
	eRT_AbsDelta = 40,
	eRT_AbsRaw = 38,
	eRT_AbsLpwgRaw = 39,
	eRT_AbsOpenShort = 61,
	eRT_AbsOpen = 611,
	eRT_AbsShort = 612,
	eRT_GpioShortTest = 27,
	eRT_GpioOpenTest = 28,
	eRT_RxRxShort = 7,
	eRT_RxRxShort1 = 17,
	eRT_RxRxShort2 = 33,
	eRT_TxTxShort = 5,
	eRT_TxGndShort = 16,
	eRT_RxOpen = 14,
	eRT_RxOpen1 = 18,
	eRT_RxOpen2 = 35,
	eRT_TxOpen = 15,
	eRT_Bonding_Pad_Active_Guard_Opens = 48,
	eRT_GuardPinShort = 50,
	eRT_RawImageRT100 = 100,
	eRT_ExtendedTRexShortIncell = 262,
	eRT_HybirdRawCap = 63,
	eRT_TagsMoistureCBC_on = 761,
	eRT_TagsMoistureCBC_off = 762,
	eRT_TagsMoisture = 76,
};

int IncellTestMode(struct device * dev);

extern int UpperImage[TRX_MAX][TRX_MAX];
extern int LowerImage[TRX_MAX][TRX_MAX];
extern unsigned char RxChannelCount;
extern unsigned char TxChannelCount;

extern void SCAN_PDT(struct device *dev);
extern int F54Test(struct device *dev, u16 input, int mode, char *buf);
extern void write_file(struct device *dev, char *data, int write_time);
extern int Read8BitRegisters(struct device *dev, unsigned short regAddr,
		unsigned char *data, int length);
extern int Write8BitRegisters(struct device *dev, unsigned short regAddr,
		unsigned char *data, int length);
extern int synaptics_get_limit(struct device *dev, char *breakpoint,
					unsigned char Tx, unsigned char Rx,
					int limit_data[TRX_MAX][TRX_MAX]);

