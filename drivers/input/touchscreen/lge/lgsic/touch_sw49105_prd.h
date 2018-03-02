/* production_test.h
 *
 * Copyright (C) 2015 LGE.
 *
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

/*
 *  Include to touch core Header File
 */
#include <touch_core.h>

/*
 *  Include to Local Header File
 */
#include "touch_sw49105.h"

#ifndef PRODUCTION_TEST_H
#define PRODUCTION_TEST_H

/* production test */
#define tc_test_mode_ctl 		(0xC6E)
#define cmd_test_exit 			(0x0000)
#define cmd_test_enter 			(0x0001)

#define tc_tsp_test_ctl			(0xC04)
#define tc_tsp_test_sts			(0x265)
#define tc_tsp_test_pf_result   (0x266)
#define tc_tsp_test_off_info	(0x2FB)
#define tc_tsp_test_data_offset	(0x07B)
#define tc_tsp_data_access_addr (0x301)

#define RAWDATA_OFFSET			(0xE00)
#define rawdata_ctl_read		(0x2A4)
#define rawdata_ctl_write		(0xC49)

//Product test
#define Serial_Data_Offset		(0x0082)// [bringup] (0x007B) // tc_tsp_data_access_addr
#define DATA_BASE_ADDR 			(0xFD1)//  [bringup] (0x301) // tc_tsp_data_access_addr
#define m1_raw_data_offset		(0x02D7)// [bringup] (0x0287)
#define m2_raw_data_offset		(0x02D7)// [bringup] (0x0287)
#define prod_open1_open2_offset (0x02D7)// [bringup] (0x0287)
#define prod_open3_short_offset	(0x02D8)// [bringup] (0x0288)
#define M1_M2_RAWDATA_TEST_CNT	(2)
#define LINE_FILTER_OPTION		(0x40000)

//Firmware debugging
#define m2_raw_data_offset_2		(0xB85)// [bringup] ((0xA8C)
#define m2_delta_data_offset 		(0xDC5)// [bringup] (0xD95)
#define m2_label_data_offset		(0xF19)// [bringup] (0xE83)
#define m2_baseline_E_data_offset 	(0xCA5)// [bringup] (0xC0F)
#define m2_baseline_O_data_offset 	(0xCD2)
#define debug_data_offset			(0xB0F)// [bringup] (0xBCF)
#define ADDR_CMD_REG_SIC_IMAGECTRL_TYPE		(0x0C6C)
#define ADDR_CMD_REG_SIC_GETTER_READYSTATUS	(0x0C64)

struct sw49105_test_off {
	u16 offset0;
	u16 offset1;
} __packed;

struct sw49105_test_off_info {
	struct sw49105_test_off m1_m2_raw;
	struct sw49105_test_off frame0_1;
	struct sw49105_test_off frame2_short;
	struct sw49105_test_off os_result;
} __packed;

/* tune code */
#define tc_tune_code_size		260
#define tc_total_ch_size		32
#define TSP_TUNE_CODE_L_GOFT_OFFSET		0
#define TSP_TUNE_CODE_L_M1_OFT_OFFSET		2
#define TSP_TUNE_CODE_L_G1_OFT_OFFSET		(TSP_TUNE_CODE_L_M1_OFT_OFFSET + tc_total_ch_size)
#define TSP_TUNE_CODE_L_G2_OFT_OFFSET	(TSP_TUNE_CODE_L_G1_OFT_OFFSET + tc_total_ch_size)
#define TSP_TUNE_CODE_L_G3_OFT_OFFSET		(TSP_TUNE_CODE_L_G2_OFT_OFFSET + tc_total_ch_size)
#define TSP_TUNE_CODE_R_GOFT_OFFSET		(TSP_TUNE_CODE_L_G3_OFT_OFFSET + tc_total_ch_size)
#define TSP_TUNE_CODE_R_M1_OFT_OFFSET		(TSP_TUNE_CODE_R_GOFT_OFFSET + 2)
#define TSP_TUNE_CODE_R_G1_OFT_OFFSET		(TSP_TUNE_CODE_R_M1_OFT_OFFSET + tc_total_ch_size)
#define TSP_TUNE_CODE_R_G2_OFT_OFFSET		(TSP_TUNE_CODE_R_G1_OFT_OFFSET + tc_total_ch_size)
#define TSP_TUNE_CODE_R_G3_OFT_OFFSET		(TSP_TUNE_CODE_R_G2_OFT_OFFSET + tc_total_ch_size)
#define PATH_SIZE		64
#define BURST_SIZE		512
#define RAWDATA_SIZE		2
#define ROW_SIZE		30
#define COL_SIZE		18
#define M1_COL_SIZE		2
#define LOG_BUF_SIZE		256
#define BUF_SIZE (PAGE_SIZE * 2)
#define MAX_LOG_FILE_SIZE	(10 * 1024 * 1024) /* 10 M byte */
#define MAX_LOG_FILE_COUNT	(4)
#define DEBUG_ROW_SIZE		8
#define DEBUG_COL_SIZE		16

enum {
	TIME_INFO_SKIP,
	TIME_INFO_WRITE,
};

enum {
	CMD_RAWDATA = 1,
	CMD_BASE_E_DATA,
	CMD_DELTADATA ,
	CMD_LABELDATA,
	CMD_ALGORITHM,  // Not used
	CMD_TCMDATA,    // Not used
	CMD_DEBUGDATA,
	CMD_BASE_O_DATA,
};

enum {
	NO_TEST = 0,
	OPEN_SHORT_ALL_TEST,
	OPEN_NODE_TEST,
	SHORT_NODE_TEST,
	DOZE1_M2_RAWDATA_TEST = 5,
	DOZE1_M1_RAWDATA_TEST = 6,
	DOZE2_M2_RAWDATA_TEST,
	DOZE2_M1_RAWDATA_TEST,
	DOZE1_BLU_JITTER_TEST = 12, // 0x0c
};

enum {
	NORMAL_MODE = 0,
	PRODUCTION_MODE,
};

typedef enum
{
    IT_NONE = 0,
    IT_ALGORITHM_RAW_IMAGE,
    IT_BASELINE_E_IMAGE,
    IT_BASELINE_O_IMAGE,
    IT_DELTA_IMAGE,
    IT_LABEL_IMAGE,
    IT_FILTERED_DELTA_IMAGE,
    IT_WAIT = 0xFF
} eImageType_t;

typedef enum
{
    RS_READY    = 0xA0,
    RS_NONE     = 0x05,
    RS_LOG      = 0x77,
    RS_IMAGE	= 0xAA
} eProtocolReadyStatus_t;

extern void touch_msleep(unsigned int msecs);
int sw49105_prd_register_sysfs(struct device *dev);

#endif

