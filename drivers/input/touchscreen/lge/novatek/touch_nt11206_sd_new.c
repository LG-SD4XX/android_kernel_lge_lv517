/* touch_nt11206_sd.c
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/syscalls.h>
#include <linux/file.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>

/*
 *  Include to touch core Header File
 */
#include <touch_hwif.h>
#include <touch_core.h>

/*
 *  Include to Local Header File
 */
#include "touch_nt11206.h"



#define I2C_TANSFER_LENGTH  64

#define AIN_TX_NUM	20
#define AIN_RX_NUM	30
#define PSConfig_Tolerance_Postive			250
#define PSConfig_Tolerance_Negative			-250
#define PSConfig_DiffLimitG_Postive			250
#define PSConfig_DiffLimitG_Negative		-250
#define PSConfig_Tolerance_Postive_Short	250
#define PSConfig_Tolerance_Negative_Short	-250
#define PSConfig_DiffLimitG_Postive_Short	250
#define PSConfig_DiffLimitG_Negative_Short	-250
#define PSConfig_Tolerance_Postive_Mutual	250
#define PSConfig_Tolerance_Negative_Mutual	-250
#define PSConfig_DiffLimitG_Postive_Mutual	250
#define PSConfig_DiffLimitG_Negative_Mutual	-250
#define PSConfig_Tolerance_Postive_FW		250
#define PSConfig_Tolerance_Negative_FW		-250
#define PSConfig_DiffLimitG_Postive_FW		250
#define PSConfig_DiffLimitG_Negative_FW		-250

#define PSConfig_Rawdata_Limit_Postive_Short_RXRX	-1500
#define PSConfig_Rawdata_Limit_Negative_Short_RXRX	-8600
#define PSConfig_Rawdata_Limit_Postive_Short_TXRX	-1500
#define PSConfig_Rawdata_Limit_Negative_Short_TXRX	-6500
#define PSConfig_Rawdata_Limit_Postive_Short_TXTX	-1000
#define PSConfig_Rawdata_Limit_Negative_Short_TXTX	-10000

#define NORMAL_MODE 0x00
#define TEST_MODE_1 0x21    //before algo.
#define TEST_MODE_2 0x22    //after algo.
#define BASELINE_ADDR   0x11054
#define XDATA_SECTOR_SIZE   256
s16 BoundaryShort_RXRX[30] = {
	-2938, -2950, -2930, -2941, -2966, -2957, -2987, -2969,	-2964, -2975, -2988, -2986, -3014, -2982, -2980, -2982,	-2973, -3004, -3023, -2994, -2988, -3006, -3013, -3023,	-3026, -3032, -2997, -3029, -3008, -3036,
};

s16 BoundaryShort_TXRX[30 * 20] = {
	-2858, -2884, -2873, -2839, -2903, -2904, -2885, -2903, -2907, -2871, -2925, -2932, -2913, -2921, -2930, -2878, -2914, -2952, -2927, -2936, -2936, -2912, -2955, -2977, -2928, -2973, -2942, -2930, -2949, -2984,
	-2920, -2893, -2880, -2897, -2913, -2908, -2945, -2914, -2917, -2936, -2935, -2941, -2975, -2930, -2937, -2942, -2924, -2961, -2988, -2945, -2943, -2971, -2966, -2982, -2988, -2985, -2952, -2991, -2961, -2993,
	-2910, -2890, -2879, -2889, -2910, -2908, -2934, -2911, -2917, -2925, -2933, -2941, -2966, -2928, -2937, -2933, -2920, -2961, -2978, -2944, -2943, -2962, -2963, -2980, -2979, -2981, -2955, -2984, -2959, -2995,
	-2906, -2890, -2880, -2886, -2910, -2909, -2930, -2911, -2918, -2921, -2933, -2941, -2962, -2926, -2937, -2931, -2921, -2962, -2974, -2943, -2944, -2958, -2962, -2980, -2978, -2981, -2957, -2981, -2958, -2998,
	-2904, -2889, -2879, -2882, -2909, -2908, -2928, -2911, -2918, -2919, -2932, -2939, -2959, -2925, -2936, -2928, -2920, -2961, -2971, -2943, -2942, -2956, -2961, -2980, -2975, -2980, -2956, -2978, -2958, -2994,
	-2903, -2888, -2878, -2882, -2910, -2908, -2927, -2911, -2918, -2918, -2932, -2940, -2959, -2927, -2937, -2928, -2920, -2962, -2972, -2942, -2943, -2955, -2961, -2980, -2974, -2979, -2956, -2978, -2957, -2995,
	-2903, -2889, -2880, -2881, -2909, -2907, -2927, -2910, -2919, -2919, -2932, -2941, -2958, -2927, -2937, -2927, -2921, -2963, -2972, -2942, -2943, -2954, -2962, -2979, -2975, -2980, -2956, -2978, -2957, -2996,
	-2903, -2889, -2881, -2882, -2910, -2908, -2928, -2910, -2919, -2917, -2932, -2940, -2958, -2927, -2938, -2928, -2920, -2962, -2970, -2943, -2944, -2955, -2962, -2979, -2975, -2980, -2956, -2978, -2957, -2997,
	-2901, -2888, -2880, -2880, -2907, -2907, -2926, -2911, -2917, -2917, -2932, -2940, -2958, -2926, -2936, -2926, -2919, -2962, -2970, -2941, -2943, -2953, -2962, -2980, -2974, -2979, -2955, -2976, -2957, -2995,
	-2902, -2890, -2879, -2882, -2909, -2908, -2927, -2911, -2918, -2917, -2932, -2939, -2958, -2926, -2936, -2928, -2919, -2961, -2971, -2943, -2944, -2955, -2963, -2979, -2974, -2979, -2956, -2978, -2958, -2996,
	-2869, -2882, -2878, -2847, -2903, -2905, -2894, -2904, -2916, -2882, -2926, -2938, -2924, -2921, -2935, -2892, -2914, -2961, -2936, -2937, -2943, -2919, -2956, -2977, -2938, -2974, -2954, -2943, -2950, -2993,
	-2919, -2891, -2881, -2898, -2912, -2909, -2943, -2915, -2919, -2934, -2935, -2941, -2975, -2930, -2937, -2944, -2924, -2963, -2988, -2946, -2945, -2972, -2966, -2981, -2991, -2983, -2957, -2996, -2961, -2998,
	-2910, -2890, -2881, -2890, -2911, -2909, -2936, -2912, -2919, -2927, -2934, -2941, -2967, -2928, -2938, -2936, -2921, -2963, -2980, -2944, -2945, -2964, -2964, -2981, -2982, -2981, -2957, -2986, -2959, -2997,
	-2905, -2890, -2880, -2886, -2910, -2908, -2931, -2911, -2919, -2921, -2932, -2940, -2962, -2927, -2937, -2930, -2920, -2961, -2975, -2943, -2944, -2958, -2962, -2979, -2976, -2980, -2956, -2979, -2956, -2996,
	-2905, -2889, -2880, -2883, -2909, -2908, -2929, -2910, -2918, -2919, -2932, -2940, -2959, -2927, -2937, -2928, -2921, -2962, -2972, -2942, -2943, -2956, -2962, -2979, -2975, -2980, -2956, -2979, -2957, -2996,
	-2904, -2889, -2880, -2881, -2910, -2907, -2928, -2910, -2918, -2919, -2932, -2941, -2959, -2926, -2937, -2928, -2919, -2962, -2971, -2942, -2944, -2955, -2962, -2979, -2974, -2980, -2956, -2979, -2958, -2996,
	-2903, -2889, -2880, -2881, -2908, -2907, -2927, -2909, -2919, -2918, -2932, -2939, -2958, -2926, -2937, -2926, -2919, -2962, -2971, -2942, -2943, -2955, -2962, -2980, -2974, -2980, -2956, -2977, -2957, -2995,
	-2904, -2888, -2881, -2881, -2910, -2908, -2927, -2911, -2919, -2917, -2933, -2940, -2959, -2926, -2937, -2928, -2920, -2961, -2971, -2942, -2944, -2955, -2962, -2980, -2975, -2980, -2955, -2979, -2958, -2996,
	-2903, -2890, -2880, -2881, -2909, -2908, -2927, -2911, -2917, -2918, -2932, -2940, -2957, -2926, -2936, -2928, -2919, -2962, -2971, -2943, -2943, -2954, -2962, -2980, -2974, -2980, -2957, -2977, -2958, -2996,
	-2903, -2887, -2880, -2880, -2909, -2907, -2927, -2911, -2919, -2919, -2932, -2940, -2959, -2927, -2936, -2928, -2918, -2963, -2972, -2941, -2943, -2955, -2962, -2980, -2973, -2979, -2956, -2978, -2959, -2996,
};
s16 BoundaryShort_TXTX[20] = {
	-2903, -2888, -2879, -2881, -2909, -2907, -2927, -2910, -2919, -2917, -2932, -2939, -2958, -2926, -2937, -2928, -2920, -2962, -2972, -2941,
};

s16 BoundaryOpen[30*20] = {
	1073, 702,  1230, 1021, 696,  1235, 995,  690,  1237, 984,  690,  1239, 979,  690,  1246, 967,  677,  1223, 952,  675,  1241, 962,  680,  1240, 964,  678,  1239, 970,  681,  1235,
	489,  638,  1417, 491,  641,  1409, 482,  635,  1401, 478,  634,  1395, 476,  635,  1404, 470,  623,  1371, 459,  622,  1392, 463,  626,  1392, 463,  623,  1390, 461,  623,  1388,
	391,  587,  1312, 390,  589,  1309, 390,  587,  1303, 389,  587,  1300, 389,  588,  1309, 389,  578,  1276, 374,  574,  1295, 378,  578,  1295, 377,  575,  1293, 375,  574,  1287,
	405,  600,  1338, 402,  601,  1333, 406,  603,  1330, 405,  603,  1327, 406,  604,  1336, 407,  598,  1303, 389,  588,  1319, 392,  592,  1319, 391,  589,  1318, 389,  590,  1318,
	367,  551,  1213, 363,  550,  1207, 368,  554,  1208, 368,  555,  1204, 369,  556,  1214, 369,  551,  1188, 356,  541,  1197, 355,  543,  1197, 353,  541,  1195, 351,  538,  1190,
	371,  535,  1181, 369,  535,  1177, 371,  539,  1179, 373,  541,  1176, 374,  541,  1185, 374,  537,  1161, 369,  530,  1169, 361,  529,  1168, 359,  526,  1167, 357,  526,  1169,
	356,  511,  1080, 354,  512,  1070, 354,  511,  1071, 355,  513,  1067, 356,  514,  1077, 357,  510,  1052, 353,  509,  1064, 346,  504,  1062, 345,  502,  1060, 343,  501,  1055,
	360,  476,  995,  361,  479,  991,  359,  478,  989,  360,  479,  986,  360,  480,  995,  362,  476,  972,  358,  476,  988,  355,  471,  981,  351,  469,  980,  348,  468,  982,
	339,  444,  838,  340,  446,  834,  339,  445,  833,  340,  446,  829,  340,  447,  839,  341,  444,  817,  338,  444,  832,  340,  441,  824,  331,  437,  824,  329,  436,  821,
	376,  439,  818,  375,  441,  814,  376,  441,  815,  377,  441,  810,  377,  443,  824,  381,  442,  802,  376,  439,  813,  381,  442,  807,  371,  433,  806,  375,  437,  819,
	969,  682,  1238, 961,  680,  1239, 959,  678,  1232, 960,  679,  1234, 964,  683,  1234, 969,  679,  1223, 969,  685,  1228, 994,  689,  1229, 1010, 685,  1216, 1047, 688,  1213,
	467,  626,  1391, 466,  626,  1392, 466,  626,  1385, 468,  626,  1388, 469,  628,  1389, 472,  625,  1380, 470,  630,  1388, 481,  634,  1398, 486,  630,  1390, 477,  629,  1406,
	385,  582,  1300, 383,  582,  1300, 384,  581,  1293, 385,  582,  1295, 385,  583,  1296, 386,  580,  1287, 383,  584,  1295, 389,  588,  1303, 390,  587,  1295, 380,  579,  1300,
	397,  594,  1323, 397,  594,  1323, 397,  594,  1315, 398,  594,  1318, 398,  596,  1319, 400,  593,  1310, 396,  597,  1318, 401,  601,  1326, 403,  601,  1323, 397,  595,  1330,
	360,  546,  1200, 358,  545,  1200, 358,  545,  1193, 359,  545,  1195, 359,  546,  1196, 360,  543,  1188, 356,  547,  1194, 362,  550,  1202, 363,  551,  1199, 362,  544,  1197,
	364,  530,  1173, 364,  531,  1172, 365,  530,  1165, 365,  531,  1167, 365,  532,  1167, 367,  530,  1160, 363,  533,  1166, 368,  536,  1173, 368,  534,  1169, 364,  529,  1171,
	349,  506,  1067, 348,  505,  1065, 349,  505,  1058, 349,  505,  1059, 349,  506,  1060, 350,  503,  1053, 345,  504,  1056, 347,  505,  1061, 346,  503,  1057, 345,  502,  1064,
	356,  474,  989,  356,  474,  986,  356,  473,  979,  356,  472,  979,  354,  471,  977,  352,  467,  970,  347,  468,  973,  351,  470,  979,  352,  470,  978,  349,  467,  985,
	335,  443,  834,  334,  441,  828,  333,  437,  819,  330,  436,  819,  329,  436,  819,  329,  433,  814,  326,  435,  817,  330,  438,  822,  331,  437,  821,  328,  435,  822,
	387,  444,  827,  373,  442,  803,  377,  433,  801,  371,  440,  796,  377,  439,  810,  370,  441,  796,  373,  436,  803,  371,  445,  802,  377,  438,  805,  363,  439,  801,
};
s16 BoudaryFWMutual[20*30] = {
	1759, 1752, 1743, 1722, 1728, 1725, 1722, 1721, 1728, 1725, 1690, 1699, 1698, 1689, 1710, 1722, 1733, 1746, 1772, 1782,
	736,  742,  748,  742,  744,  749,  747,  750,  755,  755,  717,  723,  726,  726,  732,  748,  750,  758,  769,  767,
	1119, 1130, 1137, 1132, 1130, 1136, 1139, 1135, 1142, 1139, 1136, 1133, 1133, 1132, 1127, 1142, 1141, 1141, 1146, 1138,
	1182, 1176, 1179, 1171, 1169, 1170, 1173, 1169, 1173, 1173, 1174, 1171, 1173, 1172, 1163, 1184, 1179, 1184, 1189, 1191,
	482,  484,  492,  489,  487,  491,  490,  492,  494,  493,  478,  479,  481,  479,  483,  495,  494,  497,  502,  501,
	358,  363,  365,  363,  370,  369,  368,  370,  373,  371,  348,  351,  352,  351,  361,  366,  368,  371,  377,  378,
	360,  364,  364,  361,  364,  363,  364,  365,  364,  362,  354,  356,  357,  354,  366,  367,  367,  370,  368,  369,
	507,  510,  515,  511,  508,  512,  511,  512,  511,  510,  501,  504,  505,  504,  508,  519,  516,  519,  520,  519,
	1137, 1135, 1139, 1132, 1127, 1131, 1131, 1130, 1134, 1134, 1131, 1134, 1134, 1132, 1125, 1147, 1140, 1144, 1146, 1146,
	1126, 1125, 1123, 1117, 1114, 1115, 1116, 1117, 1119, 1119, 1118, 1117, 1119, 1117, 1112, 1132, 1127, 1130, 1132, 1131,
	496,  500,  501,  497,  499,  500,  499,  500,  501,  499,  491,  493,  494,  492,  503,  509,  508,  510,  510,  509,
	342,  345,  345,  342,  345,  344,  345,  345,  345,  343,  333,  336,  337,  336,  349,  350,  351,  352,  349,  351,
	333,  333,  333,  330,  332,  332,  332,  332,  332,  330,  323,  325,  326,  329,  338,  338,  339,  339,  335,  336,
	474,  477,  477,  475,  476,  478,  477,  478,  478,  477,  469,  472,  473,  472,  482,  488,  485,  487,  484,  484,
	1033, 1036, 1036, 1030, 1026, 1029, 1030, 1029, 1034, 1034, 1027, 1032, 1031, 1030, 1030, 1044, 1041, 1042, 1042, 1043,
	987,  984,  986,  981,  977,  980,  981,  981,  985,  986,  982,  980,  982,  981,  982,  995,  992,  993,  992,  996,
	449,  449,  451,  449,  448,  451,  450,  452,  452,  451,  442,  445,  446,  448,  455,  460,  458,  460,  456,  459,
	320,  320,  320,  318,  321,  320,  320,  321,  320,  318,  312,  314,  314,  321,  326,  325,  326,  325,  323,  324,
	284,  286,  287,  287,  290,  290,  291,  291,  291,  289,  282,  284,  284,  291,  294,  293,  294,  294,  295,  294,
	409,  414,  414,  414,  415,  418,  418,  419,  420,  419,  409,  411,  412,  417,  419,  423,  423,  424,  426,  423,
	869,  874,  874,  871,  869,  873,  874,  874,  879,  880,  869,  872,  873,  874,  871,  885,  882,  884,  887,  885,
	844,  841,  842,  838,  835,  839,  841,  842,  847,  848,  845,  843,  844,  847,  840,  852,  849,  851,  853,  854,
	403,  403,  403,  402,  403,  407,  408,  410,  411,  410,  402,  403,  405,  409,  410,  413,  414,  414,  415,  415,
	289,  289,  289,  288,  291,  292,  294,  295,  296,  294,  287,  289,  291,  295,  297,  297,  298,  298,  298,  298,
	276,  277,  277,  275,  276,  277,  278,  281,  282,  280,  277,  277,  284,  283,  285,  284,  285,  285,  286,  284,
	381,  383,  383,  380,  380,  383,  383,  385,  389,  389,  382,  383,  386,  388,  389,  392,  391,  392,  393,  392,
	735,  736,  736,  732,  730,  733,  734,  735,  741,  745,  736,  738,  739,  743,  736,  747,  744,  745,  747,  748,
	723,  728,  726,  726,  721,  731,  723,  727,  728,  744,  749,  740,  741,  744,  740,  751,  744,  747,  747,  748,
	388,  386,  391,  385,  390,  390,  391,  386,  392,  394,  387,  384,  391,  389,  392,  393,  392,  393,  393,  392,
	293,  301,  297,  300,  297,  303,  300,  304,  301,  309,  299,  296,  302,  300,  303,  301,  301,  301,  301,  301,
};
#define MaxStatisticsBuf 100
static int StatisticsNum[MaxStatisticsBuf];
static long int StatisticsSum[MaxStatisticsBuf];
static long int golden_Ratio[20*30] = {0, };
struct test_cmd {
	u32 addr;
	u8 len;
	u8 data[40];
};

struct test_cmd short_test_rxrx[] = {
	//# Stop WDT
	{.addr=0x1F028, .len=2,  .data={0x07, 0x55}},
	//# Trim OSC to 60MHz
	//{.addr=0x1F386, .len=1,  .data={0x2A}},
	//# Bypass ControlRAM
	{.addr=0x1F211, .len=1,  .data={0x01}},
	//# Demod @ DC DDFS_2_2_init = 90degree
	{.addr=0x1F150, .len=3,  .data={0x00, 0x00, 0x04}},
	//# Demod @ DC DDFS_2_2_step = 0
	{.addr=0x1F186, .len=2,  .data={0x00, 0x00}},
	//# TIA Rf gain = max
	{.addr=0x1F2D9, .len=1,  .data={0x0E}},
	//# STD_Gain
	{.addr=0x1F2DA, .len=1,  .data={0x0A}},
	//# Warm up
	{.addr=0x1F218, .len=1,  .data={0xFF}},
	//# Sensing start
	{.addr=0x1F220, .len=2,  .data={0xFF, 0xFF}},
	//# VRADC_SEL = 1.75v
	{.addr=0x1F30B, .len=1,  .data={0x02}},
	//# Enable all STD_EN
	{.addr=0x1F2D0, .len=7,  .data={0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x07}},
	//# Enable all ADC_EN
	{.addr=0x1F302, .len=7,  .data={0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x07}},
	//# Enable BIAS_En, STD_BIAS ; Disable TIA_BIAS CC_BIAS
	{.addr=0x1F2FD, .len=4,  .data={0x01, 0x00, 0x01, 0x00}},
	//# Unlock ADC power down
	{.addr=0x1F309, .len=1,  .data={0x00}},
	//# ADC input Range = 0.25~2.25V
	{.addr=0x1F30A, .len=1,  .data={0x00}},
	//# Enable Hanning filter
	{.addr=0x1F1C7, .len=1,  .data={0x20}},
	//# TIA bypass Switch Enable
	{.addr=0x1F310, .len=1,  .data={0x07}},
	//# OB_DM_EN
	{.addr=0x1F1C3, .len=2,  .data={0x00, 0x01}},
	//# Raw1_Base_Addr
	{.addr=0x1F196, .len=2,  .data={0x88, 0x0F}},
	//# POWER ON SEQUENCE
	{.addr=0x1F320, .len=1,  .data={0x06}},
	{.addr=0x1F321, .len=1,  .data={0x13}},
	{.addr=0x1F32A, .len=1,  .data={0x00}},
	{.addr=0x1F327, .len=1,  .data={0x03}},
	{.addr=0x1F328, .len=1,  .data={0x43}},
	{.addr=0x1F328, .len=1,  .data={0x13}},
	{.addr=0x1F328, .len=1,  .data={0x17}},
	//# DAC ON SEQUENCE
	{.addr=0x1F1D5, .len=1,  .data={0x40}},
	{.addr=0x1F1D6, .len=1,  .data={0xFF}},
	{.addr=0x1F1D5, .len=1,  .data={0x7F}},
	{.addr=0x1F1E0, .len=1,  .data={0x01}},
	{.addr=0x1F1DC, .len=1,  .data={0x01}},
	{.addr=0x1F1DE, .len=1,  .data={0x01}},
};

struct test_cmd short_test_txrx[] = {
	//# Stop WDT
	{.addr=0x1F028, .len=2,  .data={0x07, 0x55}},
	//# Trim OSC to 60MHz
	//{.addr=0x1F386, .len=1,  .data={0x2A}},
	//# Bypass ControlRAM
	{.addr=0x1F211, .len=1,  .data={0x01}},
	//# Demod @ DC DDFS_2_2_init = 90degree
	{.addr=0x1F150, .len=3,  .data={0x00, 0x00, 0x04}},
	//# Demod @ DC DDFS_2_2_step = 0
	{.addr=0x1F186, .len=2,  .data={0x00, 0x00}},
	//# TIA Rf gain
	{.addr=0x1F2D9, .len=1,  .data={0x0E}},
	//# STD_Gain
	{.addr=0x1F2DA, .len=1,  .data={0x0A}},
	//# Warm up
	{.addr=0x1F218, .len=1,  .data={0xFF}},
	//# Sensing start
	{.addr=0x1F220, .len=2,  .data={0xFF, 0xFF}},
	//# VRADC_SEL = 1.75v
	{.addr=0x1F30B, .len=1,  .data={0x02}},
	//# RX_CFG all sensing
	{.addr=0x1F238, .len=33, .data={0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02}},
	//# Enable all STD_EN
	{.addr=0x1F2D0, .len=7,  .data={0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x07}},
	//# Enable all ADC_EN
	{.addr=0x1F302, .len=7,  .data={0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x07}},
	//# Enable BIAS_En, STD_BIAS ; Disable TIA_BIAS CC_BIAS
	{.addr=0x1F2FD, .len=4,  .data={0x01, 0x00, 0x01, 0x00}},
	//# Unlock ADC power down
	{.addr=0x1F309, .len=1,  .data={0x00}},
	//# ADC input Range = 0.25~2.25V
	{.addr=0x1F30A, .len=1,  .data={0x00}},
	//# Enable Hanning filter
	{.addr=0x1F1C7, .len=1,  .data={0x20}},
	//# TIA bypass Switch Enable
	{.addr=0x1F310, .len=1,  .data={0x07}},
	//# OB_DM_EN
	{.addr=0x1F1C3, .len=2,  .data={0x00, 0x01}},
	//# Raw1_Base_Addr
	{.addr=0x1F196, .len=2,  .data={0x88, 0x0F}},
	//# POWER ON SEQUENCE
	{.addr=0x1F320, .len=1,  .data={0x06}},
	{.addr=0x1F321, .len=1,  .data={0x13}},
	{.addr=0x1F32A, .len=1,  .data={0x00}},
	{.addr=0x1F327, .len=1,  .data={0x03}},
	{.addr=0x1F328, .len=1,  .data={0x43}},
	{.addr=0x1F328, .len=1,  .data={0x13}},
	{.addr=0x1F328, .len=1,  .data={0x17}},
	//# DAC ON SEQUENCE
	{.addr=0x1F1D5, .len=1,  .data={0x40}},
	{.addr=0x1F1D6, .len=1,  .data={0xFF}},
	{.addr=0x1F1D5, .len=1,  .data={0x7F}},
	{.addr=0x1F1E0, .len=1,  .data={0x01}},
	{.addr=0x1F1DC, .len=1,  .data={0x01}},
	{.addr=0x1F1DE, .len=1,  .data={0x01}},
};

struct test_cmd short_test_txtx[] = {
	//# Stop WDT
	{.addr=0x1F028, .len=2,  .data={0x07, 0x55}},
	//# Trim OSC to 60MHz
	//{.addr=0x1F386, .len=1,  .data={0x2A}},
	//# Bypass ControlRAM
	{.addr=0x1F211, .len=1,  .data={0x01}},
	//# Demod @ DC DDFS_2_2_init = 90degree
	{.addr=0x1F150, .len=3,  .data={0x00, 0x00, 0x04}},
	//# Demod @ DC DDFS_2_2_step = 0
	{.addr=0x1F186, .len=2,  .data={0x00, 0x00}},
	//# Release Test Channel & Output ATEST[1:0] to RX_Channel
	{.addr=0x1F1E5, .len=1,  .data={0x09}},
	//# TIA Rf gain = max
	{.addr=0x1F2D9, .len=1,  .data={0x0E}},
	//# STD_Gain
	{.addr=0x1F2DA, .len=1,  .data={0x0A}},
	//# Warm up
	{.addr=0x1F218, .len=1,  .data={0xFF}},
	//# Sensing start
	{.addr=0x1F220, .len=2,  .data={0xFF, 0xFF}},
	//# VRADC_SEL = 1.75v
	{.addr=0x1F30B, .len=1,  .data={0x02}},
	//# RX_CFG all GND
	{.addr=0x1F238, .len=33,  .data={0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01}},
	//# TX_CFG all GND
	//# Enable all STD_EN
	{.addr=0x1F2D0, .len=7,  .data={0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x07}},
	//# Enable all ADC_EN
	{.addr=0x1F302, .len=7,  .data={0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x07}},
	//# Enable BIAS_En, STD_BIAS ; Disable TIA_BIAS CC_BIAS
	{.addr=0x1F2FD, .len=4,  .data={0x01, 0x00, 0x01, 0x00}},
	//# Unlock ADC power down
	{.addr=0x1F309, .len=1,  .data={0x00}},
	//# ADC input Range = 0.25~2.25V
	{.addr=0x1F30A, .len=1,  .data={0x00}},
	//# Enable Hanning filter
	{.addr=0x1F1C7, .len=1,  .data={0x20}},
	//# TIA bypass Switch Enable
	{.addr=0x1F310, .len=1,  .data={0x07}},
	//# OB_DM_EN
	{.addr=0x1F1C3, .len=2,  .data={0x00, 0x01}},
	//# Raw1_Base_Addr
	{.addr=0x1F196, .len=2,  .data={0x88, 0x0F}},
	//# POWER ON SEQUENCE
	{.addr=0x1F320, .len=1,  .data={0x06}},
	{.addr=0x1F321, .len=1,  .data={0x13}},
	{.addr=0x1F32A, .len=1,  .data={0x00}},
	{.addr=0x1F327, .len=1,  .data={0x03}},
	{.addr=0x1F328, .len=1,  .data={0x43}},
	{.addr=0x1F328, .len=1,  .data={0x13}},
	{.addr=0x1F328, .len=1,  .data={0x17}},
	//# DAC ON SEQUENCE
	{.addr=0x1F1D5, .len=1,  .data={0x40}},
	{.addr=0x1F1D6, .len=1,  .data={0xFF}},
	{.addr=0x1F1D5, .len=1,  .data={0x7F}},
	{.addr=0x1F1E0, .len=1,  .data={0x01}},
	{.addr=0x1F1DC, .len=1,  .data={0x01}},
	{.addr=0x1F1DE, .len=1,  .data={0x01}},
};

struct test_cmd short_test_rx_all_gnd[] = {
    {.addr=0x1F238, .len=AIN_RX_NUM, .data={0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01}},
};
struct test_cmd short_test_tx_all_vref[] = {
    {.addr=0x1F259, .len=AIN_TX_NUM, .data={0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07}},
};
struct test_cmd short_test_tx_all_gnd[] = {
    {.addr=0x1F259, .len=AIN_TX_NUM, .data={0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04}},
};

struct test_cmd open_test[] = {
	//# Stop WDT
	{.addr=0x1F028, .len=2,	 .data={0x07, 0x55}},
	//# Trim OSC to 60MHz
	//{.addr=0x1F386, .len=1,  .data={0x2A}},
	//# Bypass ControlRAM
	{.addr=0x1F211, .len=1,  .data={0x01}},
	//# Demod @ for Hanning DDFS_2_1_init = 90degree
	{.addr=0x1F14C,	.len=3,  .data={0x00, 0x00, 0x04}},
	//# Demod @ for Q DDFS_2_2_init = 90degree
	{.addr=0x1F150, .len=3,  .data={0x00, 0x00, 0x04}},
	//# Demod @ for I DDFS_2_3_init = 0
	{.addr=0x1F154, .len=3,  .data={0x00, 0x00, 0x00}},
	//# TIA Rf gain = max
	{.addr=0x1F2D9, .len=1,  .data={0x0E}},
	//# STD_Gain = max
	{.addr=0x1F2DA, .len=1,  .data={0x0A}},
	//# DDFS2_1_Step
	//#	{.addr=0x1F184, .len=2,  .data={0xA7, 0x0D}},
	//# DDFS2_2_Step
	{.addr=0x1F186, .len=2,  .data={0x93, 0x18}},
	//# DDFS2_3_Step
	{.addr=0x1F188, .len=2,  .data={0x93, 0x18}},
	//# VRADC_SEL = 1.75v
	{.addr=0x1F30B, .len=1,  .data={0x02}},
	//# DDFS1_1_Step
	{.addr=0x1F160, .len=3,  .data={0xBA, 0x49, 0x00}},
	//# RX_CFG all sensing
	{.addr=0x1F238, .len=33, .data={0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x01, 0x01, 0x01}},
	//# Enable all TIA
	{.addr=0x1F2C8, .len=7,  .data={0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x07}},
	//# Enable all STD_EN
	{.addr=0x1F2D0, .len=7,  .data={0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x07}},
	//# Enable all ADC_EN
	{.addr=0x1F302, .len=7,  .data={0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x07}},
	//# Enable BIAS_En, STD_BIAS ; Disable TIA_BIAS CC_BIAS
	{.addr=0x1F2FD, .len=4,  .data={0x01, 0x01, 0x01, 0x00}},
	//# Unlock ADC power down
	{.addr=0x1F309, .len=1,  .data={0x00}},
	//# Enable Hanning filter
	{.addr=0x1F1C7, .len=1,  .data={0x20}},
	//# RXIG Gain
	{.addr=0x1F2DB, .len=1,  .data={0x03}},
	//# Raw1_Base_Addr
	{.addr=0x1F196, .len=2,  .data={0x88, 0x0F}},
	//# Raw2_Base_Addr
	{.addr=0x1F198, .len=2,  .data={0xB0, 0x14}},
	//# Workaround : set UC num >1
	//{.addr=0x1F203, .len=1,  .data={0x02}},
	//# POWER ON SEQUENCE
	{.addr=0x1F320, .len=1,  .data={0x06}},
	{.addr=0x1F321, .len=1,  .data={0x13}},
	{.addr=0x1F32A, .len=1,  .data={0x00}},
	{.addr=0x1F327, .len=1,  .data={0x03}},
	{.addr=0x1F328, .len=1,  .data={0x43}},
	{.addr=0x1F328, .len=1,  .data={0x13}},
	{.addr=0x1F328, .len=1,  .data={0x17}},
	//# DAC ON SEQUENCE
	{.addr=0x1F1D5, .len=1,  .data={0x40}},
	{.addr=0x1F1D6, .len=1,  .data={0xFF}},
	{.addr=0x1F1D5, .len=1,  .data={0x7F}},
	{.addr=0x1F1E0, .len=1,  .data={0x01}},
	{.addr=0x1F1DC, .len=1,  .data={0x01}},
	{.addr=0x1F1DE, .len=1,  .data={0x01}},
};

struct test_cmd open_test_tx_all_gnd[] = {
    {.addr=0x1F259, .len=AIN_TX_NUM, .data={0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04}},
};

typedef enum {
	RESET_STATE_INIT = 0xA0,// IC reset
	RESET_STATE_REK,        // ReK baseline
	RESET_STATE_REK_FINISH, // baseline is ready
	RESET_STATE_NORMAL_RUN  // normal run
} RST_COMPLETE_STATE;

typedef enum {
	SHORT_RXRX = 0,
	SHORT_TXTX,
	SHORT_TXRX,
	OPEN,
	FW_MUTUAL
} CHANNEL_TEST_ITEM;

static int nvt_check_fw_reset_state(struct device *dev, RST_COMPLETE_STATE check_reset_state)
{
	int ret = 0;
	u8 buf[8] = {0, };
	int retry = 0;

	while(1) {
		msleep(20);

		//---read reset state---
		buf[0] = 0x60;
		buf[1] = 0x00;
		ret = nt11206_reg_read(dev, buf[0], &buf[1], 1);
		if(buf[1] >= check_reset_state) {
			ret = 0;
			break;
		}

		if(unlikely(retry++ > 50)) {
			ret = -1;
			TOUCH_E("%s: error, retry=%d, buf[1]=0x%02X\n", __func__, retry, buf[1]);
			break;
		}
	}

	return ret;
}

static void nvt_sw_reset_idle(struct device *dev)
{
	u8 buf[4] = {0, };
	int ret = 0;

	//---write i2c cmds to reset idle---
	buf[0] = 0xFF;
	buf[1] = 0x00;
	buf[2] = 0x00;
	ret = nt11206_bootloader_write(dev, buf, 3);

	//---write i2c cmds to reset idle---
	buf[0] = 0x00;
	buf[1] = 0xA5;
	ret = nt11206_bootloader_write(dev, buf, 2);

	large_mdelay(20);
	nvt_set_i2c_debounce(dev);
}

static int nvt_set_adc_oper(struct device *dev)
{
	int ret = 0;
	uint8_t buf[4] = {0,};
	int i;
	const int retry = 10;

	//---write i2c cmds to set ADC operation---
	buf[0] = 0x01;
	buf[1] = 0xF2;
	ret = nt11206_reg_write(dev, ADDR_CMD, buf, 2);

	//---write i2c cmds to set ADC operation---
	buf[0] = 0x10;
	buf[1] = 0x01;
	ret = nt11206_reg_write(dev, buf[0], &buf[1], 1);

	for(i=0; i<retry; i++) {
		//---read ADC status---
		buf[0]=0x10;
		ret = nt11206_reg_read(dev, buf[0], &buf[1], 1);

		if(buf[1] == 0x00) {
			break;
		}

		msleep(10);
	}

	if(i >= retry) {
		TOUCH_E("%s: Failed!\n", __func__);
		return -1;
	}
	else
		return 0;
}

static void nvt_write_test_cmd(struct device *dev, struct test_cmd *cmds, int cmd_num)
{
	int ret = 0;
	int i = 0;
	int j = 0;
	u8 buf[64] = {0, };

	for(i=0; i<cmd_num; i++) {
		//---set xdata index---
		buf[0] = ((cmds[i].addr >> 16) & 0xFF);
		buf[1] = ((cmds[i].addr >> 8) & 0xFF);
		ret = nt11206_reg_write(dev, ADDR_CMD, buf, 2);

		//---write test cmds---
		buf[0] = (cmds[i].addr & 0xFF);
		for(j=0; j<cmds[i].len; j++)
		{
			buf[1+j] = cmds[i].data[j];
		}
		ret = nt11206_reg_write(dev, buf[0], &buf[1], cmds[i].len);
	}
}

static int nvt_read_short_rxrx(struct device *dev, __s32* rawdata_short_rxrx)
{
	int ret = 0;
	int i = 0;
	u8 buf[64] = {0, };

	//---write i2c cmds to set RX-RX mode---
	nvt_write_test_cmd(dev, short_test_rxrx, sizeof(short_test_rxrx)/sizeof(short_test_rxrx[0]));
	if(nvt_set_adc_oper(dev) != 0) {
		return -EAGAIN;
	}

	//---write i2c cmds to set RX all GND---
	for(i=0; i<AIN_RX_NUM; i++) {
		short_test_rx_all_gnd->data[i] = 0x02;	//set test pin to sample
		nvt_write_test_cmd(dev, short_test_rx_all_gnd, sizeof(short_test_rx_all_gnd)/sizeof(short_test_rx_all_gnd[0]));

		if(nvt_set_adc_oper(dev) != 0) {
			return -EAGAIN;
		}

		//---change xdata index---
		buf[0]=0x01;
		buf[1]=0x0F;
		ret = nt11206_reg_write(dev, ADDR_CMD, buf, 2);

		//---read data---
		buf[0]=(0x88 + i*2);
		ret = nt11206_reg_read(dev, buf[0], &buf[1], 2);
		rawdata_short_rxrx[i] = (s16)(buf[1] + 256*buf[2]);

		short_test_rx_all_gnd->data[i] = 0x01; //restore to default
	}
#if 0 //debuf
	TOUCH_I("[SHORT_RXRX_RAWDATA]\n");
	for(i=0; i<AIN_RX_NUM; i++) {
		printk("%5d, ", rawdata_short_rxrx[i]);
	}
	printk("\n");
#endif
	return 0;
}

static int nvt_read_short_txrx(struct device *dev, __s32 *rawdata_short_txrx)
{
	int ret = 0;
	int i = 0;
	int j = 0;
	u8 buf[64] = {0, };

	//---write i2c cmds to set RX-RX mode---
	nvt_write_test_cmd(dev, short_test_txrx, sizeof(short_test_txrx)/sizeof(short_test_txrx[0]));
	if(nvt_set_adc_oper(dev) != 0) {
		return -EAGAIN;
	}

	//---write i2c cmds to set TX all Vref---
	for(i=0; i<AIN_TX_NUM; i++) {
		short_test_tx_all_vref->data[i] = 0x04;	//set test pin to GND
		nvt_write_test_cmd(dev, short_test_tx_all_vref, sizeof(short_test_tx_all_vref)/sizeof(short_test_tx_all_vref[0]));

		if(nvt_set_adc_oper(dev) != 0) {
			return -EAGAIN;
		}

		//---change xdata index---
		buf[0]=0x01;
		buf[1]=0x0F;
		ret = nt11206_reg_write(dev, ADDR_CMD, buf, 2);
		//---read data---
		buf[0]=0x88;
		ret = nt11206_reg_read(dev, buf[0], &buf[1], AIN_RX_NUM*2);

		for(j=0; j<AIN_RX_NUM; j++) {
			rawdata_short_txrx[i*AIN_RX_NUM+j] = (s16)(buf[j*2+1] + 256*buf[j*2+2]);
		}

		short_test_tx_all_vref->data[i] = 0x07; //restore to default
	}

#if 0 //debuf
	TOUCH_I("[SHORT_TXRX_RAWDATA]\n");
	for(i=0; i<AIN_TX_NUM; i++) {
		for(j=0; j<AIN_RX_NUM; j++) {
			printk("%5d", rawdata_short_txrx[i*AIN_RX_NUM+j]);
		}
		printk("\n");
	}
#endif
	return 0;
}

static int nvt_read_short_txtx(struct device *dev, __s32 *rawdata_short_txtx)
{
	int ret = 0;
	int i = 0;
	u8 buf[64] = {0, };

	//---write i2c cmds to set TX-TX mode---
	nvt_write_test_cmd(dev, short_test_txtx, sizeof(short_test_txtx)/sizeof(short_test_txtx[0]));
	if(nvt_set_adc_oper(dev) != 0) {
		return -EAGAIN;
	}

	//---write i2c cmds to set TX all GND---
	for(i=0; i<AIN_TX_NUM; i++) {
		short_test_tx_all_gnd->data[i] = 0x05;	//set test pin to sample
		nvt_write_test_cmd(dev, short_test_tx_all_gnd, sizeof(short_test_tx_all_gnd)/sizeof(short_test_tx_all_gnd[0]));

		if(nvt_set_adc_oper(dev) != 0) {
			return -EAGAIN;
		}

		//---change xdata index---
		buf[0]=0x01;
		buf[1]=0x0F;
		ret = nt11206_reg_write(dev, ADDR_CMD, buf, 2);
		//---read data---
		buf[0]=(0x88 + i*2);
		ret = nt11206_reg_read(dev, buf[0], &buf[1], 2);
		rawdata_short_txtx[i] = (s16)(buf[1] + 256 * buf[2]);

		short_test_tx_all_gnd->data[i] = 0x01; //restore to default
	}
#if 0 //debug
	TOUCH_I("[SHORT_TXTX_RAWDATA]\n");
	for(i=0; i<AIN_TX_NUM; i++) {
		printk("%5d, ", rawdata_short_txtx[i]);
	}
	printk("\n");
#endif
	return 0;
}
static u32 nvt_sqrt(u32 sqsum)
{
	u32 sq_rt = 0;

	int g0 = 0;
	int g1 = 0;
	int g2 = 0;
	int g3 = 0;
	int g4 = 0;
	int seed = 0;
	int next = 0;
	int step = 0;

	g4 =  sqsum / 100000000;
	g3 = (sqsum - g4*100000000) / 1000000;
	g2 = (sqsum - g4*100000000 - g3*1000000) / 10000;
	g1 = (sqsum - g4*100000000 - g3*1000000 - g2*10000) / 100;
	g0 = (sqsum - g4*100000000 - g3*1000000 - g2*10000 - g1*100);

	next = g4;
	step = 0;
	seed = 0;
	while(((seed + 1) * (step + 1)) <= next) {
		step++;
		seed++;
	}

	sq_rt = seed * 10000;
	next = (next - (seed * step))*100 + g3;

	step = 0;
	seed = 2 * seed * 10;
	while(((seed + 1)*(step + 1)) <= next)
	{
		step++;
		seed++;
	}

	sq_rt = sq_rt + step * 1000;
	next = (next - seed * step) * 100 + g2;
	seed = (seed + step) * 10;
	step = 0;
	while(((seed + 1) * (step + 1)) <= next) {
		step++;
		seed++;
	}

	sq_rt = sq_rt + step * 100;
	next = (next - seed * step) * 100 + g1;
	seed = (seed + step) * 10;
	step = 0;

	while(((seed + 1) * (step + 1)) <= next) {
		step++;
		seed++;
	}

	sq_rt = sq_rt + step * 10;
	next = (next - seed * step) * 100 + g0;
	seed = (seed + step) * 10;
	step = 0;

	while(((seed + 1) * (step + 1)) <= next) {
		step++;
		seed++;
	}

	sq_rt = sq_rt + step;

	return sq_rt;
}

static int nvt_read_open(struct device *dev, s16 rawdata_open_raw1[], s16 rawdata_open_raw2[], __s32 rawdata_open[], __s32 rawdata_open_aver[], int index)
{
	int ret = 0;
	int i = 0;
	int j = 0;
	u8 buf[64] = {0, };
	int raw_index = 0;

	//---write i2c cmds to set SingleScan mode---
	nvt_write_test_cmd(dev, open_test, sizeof(open_test)/sizeof(open_test[0]));
	if(nvt_set_adc_oper(dev) != 0) {
		return -EAGAIN;
	}

	//---write i2c cmds to set TX all GND---
	for(i=0; i<AIN_TX_NUM; i++) {
		open_test_tx_all_gnd->data[i] = 0x08; //set test pin to drive
		nvt_write_test_cmd(dev, open_test_tx_all_gnd, sizeof(open_test_tx_all_gnd)/sizeof(open_test_tx_all_gnd[0]));

		if(nvt_set_adc_oper(dev) != 0) {
			return -EAGAIN;
		}

		if(nvt_set_adc_oper(dev) != 0) {
			return -EAGAIN;
		}

		//---change xdata index---
		buf[0] = 0x01;
		buf[1] = 0x0F;
		ret = nt11206_reg_write(dev, ADDR_CMD, buf, 2);

		//---read data raw1---
		buf[0] = 0x88;
		ret = nt11206_reg_read(dev, buf[0], &buf[1], AIN_RX_NUM * 2);

		for(j=0; j<AIN_RX_NUM; j++)	{
			rawdata_open_raw1[i*AIN_RX_NUM+j] = (s16)(buf[j * 2 + 1] + 256 * buf[j * 2 + 2]);
		}

		//---change xdata index---
		buf[0] = 0x01;
		buf[1] = 0x14;
		ret = nt11206_reg_write(dev, ADDR_CMD, buf, 2);
		//---read data raw2---
		buf[0] = 0xB0;
		ret = nt11206_reg_read(dev, buf[0], &buf[1], AIN_RX_NUM * 2);

		for(j=0; j<AIN_RX_NUM; j++) {
			rawdata_open_raw2[i * AIN_RX_NUM+j] = (s16)(buf[j * 2 + 1] + 256 * buf[j * 2 + 2]);
		}

		open_test_tx_all_gnd->data[i] = 0x04; //restore to default

		//--IQ---
		for(j=0; j<AIN_RX_NUM; j++) {
			raw_index = i * AIN_RX_NUM + j;
			rawdata_open[raw_index] = nvt_sqrt(rawdata_open_raw1[raw_index] * rawdata_open_raw1[raw_index] + rawdata_open_raw2[raw_index] * rawdata_open_raw2[raw_index]);
			rawdata_open_aver[raw_index] += rawdata_open[raw_index];
		}
	}
	if(index == 2) {
		for(i = 0; i < AIN_TX_NUM; i++) {
			for(j = 0; j < AIN_RX_NUM; j++) {
				raw_index = i * AIN_RX_NUM + j;
				rawdata_open_aver[raw_index] = rawdata_open_aver[raw_index] / 3;
			}
		}
	}
#if 0 //debug
	TOUCH_I("[OPEN_RAWDATA]\n");
	for(i=0; i<AIN_TX_NUM; i++) {
		for(j=0; j<AIN_RX_NUM; j++)	{
			printk("%5d, ", rawdata_open[i*AIN_RX_NUM+j]);
		}
		printk("\n");
	}
#endif
	return 0;
}
static int nvt_read_baseline(struct device *dev, __s32 *xdata)
{
	u8 x_num = 0;
	u8 y_num = 0;

	nvt_change_mode(dev, TEST_MODE_1);

	if(nvt_clear_fw_status(dev) != 0) {
		return -EAGAIN;
	}

	if(nvt_check_fw_status(dev) != 0) {
		return -EAGAIN;
	}

	nvt_get_fw_info(dev);
	nvt_read_mdata(dev, BASELINE_ADDR);
	nvt_get_mdata(xdata, &x_num, &y_num);

	nvt_change_mode(dev, MODE_CHANGE_NORMAL_MODE);

	return 0;
}
static int Test_CaluateGRatioAndNormal(s16 boundary[], __s32 rawdata[], u8 x_len, u8 y_len)
{
	int i = 0;
	int j = 0;
	int k = 0;
	long int tmpValue = 0;
	long int MaxSum = 0;
	int SumCnt = 0;
	int MaxNum = 0;
	int MaxIndex = 0;
	int Max = -99999999;
	int Min =  99999999;
	int offset = 0;
	int Data = 0;	// double
	int StatisticsStep = 0;

	//--------------------------------------------------
	//1. (Testing_CM - Golden_CM ) / Testing_CM
	//--------------------------------------------------
	for(j=0; j<y_len; j++) {
		for(i=0; i<x_len; i++) {
			Data = rawdata[j * x_len + i];
			if(Data == 0)
				Data = 1;

			golden_Ratio[j * x_len + i] = Data - boundary[j * x_len + i];
			golden_Ratio[j * x_len + i] = ((golden_Ratio[j * x_len + i]*1000) / Data);	// *1000 before division
		}
	}

	//--------------------------------------------------------
	// 2. Mutual_GoldenRatio*1000
	//--------------------------------------------------------
	for(j=0; j<y_len; j++) {
		for(i=0; i<x_len; i++) {
			golden_Ratio[j * x_len + i] *= 1000;
		}
	}

	//--------------------------------------------------------
	// 3. Calculate StatisticsStep
	//--------------------------------------------------------
	for(j=0; j<y_len; j++) {
		for(i=0; i<x_len; i++) {
			if (Max < golden_Ratio[j * x_len + i])
				Max = (int)golden_Ratio[j * x_len + i];
			if (Min > golden_Ratio[j * x_len + i])
				Min = (int)golden_Ratio[j * x_len + i];
		}
	}

	offset = 0;
	if(Min < 0) // add offset to get erery element Positive
	{
		offset = 0 - Min;
		for(j=0; j<y_len; j++) {
			for(i=0; i<x_len; i++) {
				golden_Ratio[j * x_len + i] += offset;
			}
		}
		Max += offset;
	}
	StatisticsStep = Max / MaxStatisticsBuf;
	StatisticsStep += 1;
	if(StatisticsStep < 0) {
		TOUCH_E("FAIL! (StatisticsStep < 0)\n");
		return 1;
	}

	//--------------------------------------------------------
	// 4. Start Statistics and Average
	//--------------------------------------------------------
	memset(StatisticsSum, 0, sizeof(long int)*MaxStatisticsBuf);
	memset(StatisticsNum, 0, sizeof(int)* MaxStatisticsBuf);
	for(i=0; i<MaxStatisticsBuf; i++) {
		StatisticsSum[i] = 0;
		StatisticsNum[i] = 0;
	}
	for(j=0; j<y_len; j++) {
		for(i=0; i<x_len; i++) {
			tmpValue = golden_Ratio[j * x_len + i];
			tmpValue /= StatisticsStep;
			StatisticsNum[tmpValue] += 2;
			StatisticsSum[tmpValue] += (2 * golden_Ratio[j * x_len + i]);

			if((tmpValue + 1) < MaxStatisticsBuf) {
				StatisticsNum[tmpValue + 1] += 1;
				StatisticsSum[tmpValue + 1] += golden_Ratio[j * x_len + i];
			}

			if ((tmpValue - 1) >= 0) {
				StatisticsNum[tmpValue - 1] += 1;
				StatisticsSum[tmpValue - 1] += golden_Ratio[j * x_len + i];
			}
		}
	}
	//Find out Max Statistics
	MaxNum = 0;
	for(k=0; k<MaxStatisticsBuf; k++) {
		if(MaxNum < StatisticsNum[k]) {
			MaxSum = StatisticsSum[k];
			MaxNum = StatisticsNum[k];
			MaxIndex = k;
		}
	}
	//Caluate Statistics Average
	if(MaxSum > 0) {
		if (StatisticsNum[MaxIndex] != 0) {
			tmpValue = (long)(StatisticsSum[MaxIndex] / StatisticsNum[MaxIndex]) * 2;
			SumCnt += 2;
		}

		if ((MaxIndex + 1) < (MaxStatisticsBuf)) {
			if (StatisticsNum[MaxIndex + 1] != 0)
			{
				tmpValue += (long)(StatisticsSum[MaxIndex + 1] / StatisticsNum[MaxIndex + 1]);
				SumCnt++;
			}
		}

		if ((MaxIndex - 1) >= 0) {
			if (StatisticsNum[MaxIndex - 1] != 0) {
				tmpValue += (long)(StatisticsSum[MaxIndex - 1] / StatisticsNum[MaxIndex - 1]);
				SumCnt++;
			}
		}

		if (SumCnt > 0) {
			tmpValue /= SumCnt;
		}
	}
	else {// Too Separately
		StatisticsSum[0] = 0;
		StatisticsNum[0] = 0;
		for(j=0; j<y_len; j++) 	{
			for(i=0; i<x_len; i++) {
				StatisticsSum[0] += (long int)golden_Ratio[j * x_len + i];
				StatisticsNum[0]++;
			}
		}
		tmpValue = StatisticsSum[0] / StatisticsNum[0];
	}

	tmpValue -= offset;
	for(j= 0; j<y_len; j++) {
		for(i=0; i<x_len; i++) {
			golden_Ratio[j * x_len + i] -= offset;

			golden_Ratio[j * x_len + i] = golden_Ratio[j * x_len + i] - tmpValue;
			golden_Ratio[j * x_len + i] = golden_Ratio[j * x_len + i] / 1000;
		}
	}
	return 0;
}
static int RawDataTest_Sub(s16 boundary[], __s32 rawdata[], u8 RecordResult[], u8 x_ch, u8 y_ch,
		int Tol_P, int Tol_N, int Dif_P, int Dif_N, int Rawdata_Limit_Postive, int Rawdata_Limit_Negative)
{
	int i = 0;
	int j = 0;
	int iArrayIndex=0;
	__s32 iBoundary=0;
	s64 iTolLowBound = 0;
	s64 iTolHighBound = 0;
	bool isAbsCriteria = false;
	bool isPass = true;

	if(Rawdata_Limit_Postive != 0 || Rawdata_Limit_Negative != 0) {
		isAbsCriteria = true;
	}

	for(j=0; j<y_ch; j++) {
		for(i=0; i<x_ch; i++) {
			iArrayIndex = j * x_ch + i;
			iBoundary = boundary[iArrayIndex];

			RecordResult[iArrayIndex] = 0x00;	// default value for PASS

			if(isAbsCriteria) {
				iTolLowBound = Rawdata_Limit_Negative * 1000;
				iTolHighBound = Rawdata_Limit_Postive * 1000;
			}
			else {
				if(iBoundary > 0) {
					iTolLowBound = (iBoundary * (1000 + Tol_N));
					iTolHighBound = (iBoundary * (1000 + Tol_P));
				}
				else {
					iTolLowBound = (iBoundary * (1000 - Tol_N));
					iTolHighBound = (iBoundary * (1000 - Tol_P));
				}
			}

			if((rawdata[iArrayIndex] * 1000) > iTolHighBound) {
				RecordResult[iArrayIndex] |= 0x01;
			}

			if((rawdata[iArrayIndex] * 1000) < iTolLowBound) {
				RecordResult[iArrayIndex] |= 0x02;
			}
		}
	}

	if(!isAbsCriteria) {
		Test_CaluateGRatioAndNormal(boundary, rawdata, x_ch, y_ch);

		for(j=0; j<y_ch; j++) {
			for(i=0; i<x_ch; i++) {
				iArrayIndex = j * x_ch + i;

				if(golden_Ratio[iArrayIndex] > Dif_P) {
					RecordResult[iArrayIndex] |= 0x04;
				}

				if(golden_Ratio[iArrayIndex] < Dif_N) {
					RecordResult[iArrayIndex] |= 0x08;
				}
			}
		}
	}

	//---Check RecordResult---
	for(j=0; j<y_ch; j++) {
		for(i=0; i<x_ch; i++) {
			if(RecordResult[j * x_ch + i] != 0) {
				isPass = false;
				break;
			}
		}
	}

	if(isPass == false) {
		return -1;	// FAIL
	}
	else {
		return 0;	// PASS
	}
}
void print_selftest_result(struct device *dev, CHANNEL_TEST_ITEM item, int TestResult, u8 RecordResult[], __s32 rawdata[], u8 x_len, u8 y_len, int mode)
{
	struct nt11206_data *d = to_nt11206_data(dev);
	char *write_buf = NULL;
	int ret = 0;
    int i = 0;
	int j = 0;

	if(mode) {
		return;
	}

	write_buf = kmalloc(sizeof(char) * LOG_BUF_SIZE, GFP_KERNEL);
	if(write_buf) {
		memset(write_buf, 0, sizeof(char) * LOG_BUF_SIZE);
	}
	else {
		return;
	}

	switch(item) {
		case SHORT_RXRX:
			TOUCH_I("[SHORT_RXRX_RAWDATA]\n");
			ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "[SHORT_RXRX_RAWDATA]\n");
		break;

		case SHORT_TXTX:
			TOUCH_I("[SHORT_TXTX_RAWDATA]\n");
			ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "[SHORT_TXTX_RAWDATA]\n");
		break;

		case SHORT_TXRX:
			TOUCH_I("[SHORT_TXRX_RAWDATA]\n");
			ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "[SHORT_TXRX_RAWDATA]\n");
		break;

		case OPEN:
			TOUCH_I("[OPEN_RAWDATA]\n");
			ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "[OPEN_RAWDATA]\n");
		break;

		case FW_MUTUAL:
			TOUCH_I("[FW_MUTUAL_RAWDATA]\n");
			ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "[FW_MUTUAL_RAWDATA]\n");
		break;

		default:

		break;
	}

	for(i=0; i<y_len; i++) {
		for(j=0; j<x_len; j++) {
			printk("%7d", rawdata[i*x_len+j]);
			ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "%7d", rawdata[i*x_len+j]);
		}
		printk("\n");
		ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "\n");
	}

	switch(item) {
		case SHORT_RXRX:
			TOUCH_I("[SHORT_RXRX_RESULT]\n");
			ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "[SHORT_RXRX_RESULT]\n");
		break;

		case SHORT_TXTX:
			TOUCH_I("[SHORT_TXTX_RESULT]\n");
			ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "[SHORT_TXTX_RESULT]\n");
		break;

		case SHORT_TXRX:
			TOUCH_I("[SHORT_TXRX_RESULT]\n");
			ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "[SHORT_TXRX_RESULT]\n");
		break;

		case OPEN:
			TOUCH_I("[OPEN_RESULT]\n");
			ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "[OPEN_RESULT]\n");
		break;

		case FW_MUTUAL:
			TOUCH_I("[FW_MUTUAL_RESULT]\n");
			ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "[FW_MUTUAL_RESULT]\n");
		break;
		default:

		break;
	}

	switch(TestResult) {
		case 0:
			TOUCH_I("PASS!\n\n");
			ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "[PASS]\n\n");
		break;

		case 1:
			TOUCH_E("ERROR! Read Data FAIL!\n");
			ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "ERROR! Read Data FAIL!\n");
		break;

		case -1:
			TOUCH_E ("FAIL!\n");
			ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "[FAIL]\n");
		break;
	}

	if(d->boot_mode == NORMAL_BOOT) {
		write_file(NORMAL_SELF_TEST_FILE_PATH, write_buf, 1);
		log_file_size_check(dev, NORMAL_SELF_TEST_FILE_PATH);
	}
	else {
		write_file(SELF_TEST_FILE_PATH, write_buf, 1);
		log_file_size_check(dev, SELF_TEST_FILE_PATH);
	}

	if(TestResult == -1) {
		memset(write_buf, 0, sizeof(char) * LOG_BUF_SIZE);
		ret = 0;
		TOUCH_I("RecordResult:\n");
		ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "[RECORD_RESULT]\n");
		for(i=0; i<y_len; i++) {
			for(j=0; j<x_len; j++) {
				printk("0x%02X, ", RecordResult[i*x_len+j]);
				ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "0x%02X, ", RecordResult[i*x_len+j]);
			}
			printk("\n");
			ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "\n");
		}
		ret += snprintf(write_buf + ret, LOG_BUF_SIZE - ret, "\n");
		printk("\n");
		if(d->boot_mode == NORMAL_BOOT) {
			write_file(NORMAL_SELF_TEST_FILE_PATH, write_buf, 1);
			log_file_size_check(dev, NORMAL_SELF_TEST_FILE_PATH);
		}
		else {
			write_file(SELF_TEST_FILE_PATH, write_buf, 1);
			log_file_size_check(dev, SELF_TEST_FILE_PATH);
		}
	}

	if(write_buf) {
		kfree(write_buf);
	}
}
#if 0 //FOR DEBUGGING
void print_baseline_check_result(struct device *dev, CHANNEL_TEST_ITEM item, int TestResult, u8 RecordResult[], __s32 rawdata[], u8 x_len, u8 y_len)
{
    int i = 0;
	int j = 0;

	switch(TestResult) {
		case 0:
			TOUCH_I("PASS!\n\n");
		break;

		case 1:
			TOUCH_E("ERROR! Read Data FAIL!\n");
		break;

		case -1:
			TOUCH_E ("FAIL!\n");
		break;
	}

	if(TestResult == -1) {
		TOUCH_I("[FW_MUTUAL_RAWDATA]\n");

		for(i=0; i<y_len; i++) {
			for(j=0; j<x_len; j++) {
				printk("%7d", rawdata[i*x_len+j]);
			}
			printk("\n");
		}

		TOUCH_I("[FW_MUTUAL_RESULT]\n");

		TOUCH_I("RecordResult:\n");
		for(i=0; i<y_len; i++) {
			for(j=0; j<x_len; j++) {
				printk("0x%02X, ", RecordResult[i*x_len+j]);
			}
			printk("\n");
		}
	}
}
#endif
int nt11206_selftest(struct device *dev, char* buf, u8 mode)
{
	struct nt11206_data *d = to_nt11206_data(dev);
	u8 x_num=0;
	u8 y_num=0;
	int ret = 0;
	int i = 0;
	int TestResult_Short_RXRX = 0;
	int TestResult_Short_TXRX = 0;
	int TestResult_Short_TXTX = 0;
	int TestResult_Open = 0;
	int TestResult_FW_MUTUAL = 0;

	u8 *RecordResult = NULL;
	s16 *rawdata_16 = NULL;
	s16 *rawdata_16_1 = NULL;
	__s32 *rawdata_32 = NULL;
	__s32 *open_rawdata_32 = NULL;

	RecordResult = (u8*)kmalloc(sizeof(u8) * 20 * 30, GFP_KERNEL);
	rawdata_16 = (s16*)kmalloc(sizeof(s16) * 20 * 30, GFP_KERNEL);
	rawdata_16_1 = (s16*)kmalloc(sizeof(s16) * 20 * 30, GFP_KERNEL);
	rawdata_32 = (__s32*)kmalloc(sizeof(u32) * 20 * 30, GFP_KERNEL);
	open_rawdata_32 = (__s32*)kmalloc(sizeof(u32) * 20 * 30, GFP_KERNEL);

	x_num = d->fw.x_axis_num;
	y_num = d->fw.y_axis_num;

	if((RecordResult == NULL) || (rawdata_16 == NULL) || (rawdata_16_1 == NULL) || (rawdata_32 == NULL) || (open_rawdata_32 == NULL)) {
		TOUCH_E("Alloc Failed\n");
		if(RecordResult)
			kfree(RecordResult);

		if(rawdata_16)
			kfree(rawdata_16);

		if(rawdata_16_1)
			kfree(rawdata_16_1);

		if(rawdata_32)
			kfree(rawdata_32);

		if(open_rawdata_32)
			kfree(open_rawdata_32);
		return -1;
	}

	if(	d->resume_state == 0 && !mode) {
		TOUCH_E("LCD OFF, mode:%d\n", mode);
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "LCD OFF\n");
		if(RecordResult)
			kfree(RecordResult);

		if(rawdata_16)
			kfree(rawdata_16);

		if(rawdata_16_1)
			kfree(rawdata_16_1);

		if(rawdata_32)
			kfree(rawdata_32);

		if(open_rawdata_32)
			kfree(open_rawdata_32);
		return ret;
	}
	TOUCH_I("x_axis_num:%d, y_axis_num:%d\n", d->fw.x_axis_num, d->fw.y_axis_num);
	if((x_num != 20) || (y_num != 30)) {
		TOUCH_E("FW Info is broken\n");
		x_num = 20;
		y_num = 30;
	}

	//---Reset IC & into idle---
	nt11206_hw_reset(dev);
	nvt_change_mode(dev, AUTORC_OFF);
	nvt_check_fw_reset_state(dev, RESET_STATE_REK_FINISH);

	if(nvt_read_baseline(dev, rawdata_32) != 0) {
		TestResult_FW_MUTUAL = 1; //1:ERROR
	}
	else {
		//---Self Test Check ---	// 0:PASS, -1:FAIL
		TestResult_FW_MUTUAL = RawDataTest_Sub(BoudaryFWMutual, rawdata_32, RecordResult, AIN_TX_NUM, AIN_RX_NUM,
				PSConfig_Tolerance_Postive_FW, PSConfig_Tolerance_Negative_FW, PSConfig_DiffLimitG_Postive_FW, PSConfig_DiffLimitG_Negative_FW,
				0, 0);
	}
	print_selftest_result(dev, FW_MUTUAL, TestResult_FW_MUTUAL, RecordResult, rawdata_32, AIN_TX_NUM, AIN_RX_NUM, mode);
	if(mode) {
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "LPWG RAWDATA : ");
	}
	else {
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "Raw Data : ");
	}
	switch(TestResult_FW_MUTUAL) {
		case 0:
			ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "Pass\n");
			break;

		case 1:
			ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "Fail\n");
			ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "Self Test ERROR! Read Data FAIL!\n");
			break;

		case -1:
			ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "Fail\n");
			break;
	}
	if(mode) {
		if(RecordResult)
			kfree(RecordResult);

		if(rawdata_16)
			kfree(rawdata_16);

		if(rawdata_16_1)
			kfree(rawdata_16_1);

		if(rawdata_32)
			kfree(rawdata_32);

		if(open_rawdata_32)
			kfree(open_rawdata_32);

		nt11206_hw_reset(dev);
		return ret;
	}
	//---Reset IC & into idle---
	nvt_sw_reset_idle(dev);
	msleep(100);

	//---Short Test RX-RX---
	memset(RecordResult, 0, sizeof(u8) * 20 * 30);
	memset(rawdata_16, 0, sizeof(s16) * 20 * 30);
	memset(rawdata_16_1, 0, sizeof(s16) * 20 * 30);
	memset(rawdata_32, 0, sizeof(__s32) * 20 * 30);
	if(nvt_read_short_rxrx(dev, rawdata_32) != 0) {
		TestResult_Short_RXRX = 1;	// 1:ERROR
	}
	else {
		//---Self Test Check --- 		// 0:PASS, -1:FAIL
		TestResult_Short_RXRX = RawDataTest_Sub(BoundaryShort_RXRX, rawdata_32, RecordResult, AIN_RX_NUM, 1,
												PSConfig_Tolerance_Postive_Short, PSConfig_Tolerance_Negative_Short, PSConfig_DiffLimitG_Postive_Short, PSConfig_DiffLimitG_Negative_Short,
												PSConfig_Rawdata_Limit_Postive_Short_RXRX, PSConfig_Rawdata_Limit_Negative_Short_RXRX);
	}
	print_selftest_result(dev, SHORT_RXRX, TestResult_Short_RXRX, RecordResult, rawdata_32, AIN_RX_NUM, 1, mode);

	//---Short Test TX-RX---
	memset(RecordResult, 0, sizeof(u8) * 20 * 30);
	memset(rawdata_16, 0, sizeof(s16) * 20 * 30);
	memset(rawdata_16_1, 0, sizeof(s16) * 20 * 30);
	memset(rawdata_32, 0, sizeof(__s32) * 20 * 30);
	if(nvt_read_short_txrx(dev, rawdata_32) != 0) {
		TestResult_Short_TXRX = 1;	// 1:ERROR
	}
	else {
		//---Self Test Check ---		// 0:PASS, -1:FAIL
		TestResult_Short_TXRX = RawDataTest_Sub(BoundaryShort_TXRX, rawdata_32, RecordResult, AIN_TX_NUM, AIN_RX_NUM,
    											PSConfig_Tolerance_Postive_Short, PSConfig_Tolerance_Negative_Short, PSConfig_DiffLimitG_Postive_Short, PSConfig_DiffLimitG_Negative_Short,
    											PSConfig_Rawdata_Limit_Postive_Short_TXRX, PSConfig_Rawdata_Limit_Negative_Short_TXRX);
	}
	print_selftest_result(dev, SHORT_TXRX, TestResult_Short_TXRX, RecordResult, rawdata_32, AIN_TX_NUM, AIN_RX_NUM, mode);

	//---Short Test TX-TX---
	memset(RecordResult, 0, sizeof(u8) * 20 * 30);
	memset(rawdata_16, 0, sizeof(s16) * 20 * 30);
	memset(rawdata_16_1, 0, sizeof(s16) * 20 * 30);
	memset(rawdata_32, 0, sizeof(__s32) * 20 * 30);
	if(nvt_read_short_txtx(dev, rawdata_32) != 0) {
		TestResult_Short_TXTX = 1;	// 1:ERROR
	}
	else {
		//---Self Test Check ---		// 0:PASS, -1:FAIL
		TestResult_Short_TXTX = RawDataTest_Sub(BoundaryShort_TXTX, rawdata_32, RecordResult, AIN_TX_NUM, 1,
												PSConfig_Tolerance_Postive_Short, PSConfig_Tolerance_Negative_Short, PSConfig_DiffLimitG_Postive_Short, PSConfig_DiffLimitG_Negative_Short,
												PSConfig_Rawdata_Limit_Postive_Short_TXTX, PSConfig_Rawdata_Limit_Negative_Short_TXTX);
	}

	print_selftest_result(dev, SHORT_TXTX, TestResult_Short_TXTX, RecordResult, rawdata_32, AIN_TX_NUM, 1, mode);

	//---Reset IC & into idle---
	nvt_sw_reset_idle(dev);
	msleep(100);
	memset(open_rawdata_32, 0, sizeof(__s32) * 20 * 30);
	for(i = 0; i < 3; i++) {
		//---Open Test---
		memset(RecordResult, 0, sizeof(u8) * 20 * 30);
		memset(rawdata_16, 0, sizeof(s16) * 20 * 30);
		memset(rawdata_16_1, 0, sizeof(s16) * 20 * 30);
		memset(rawdata_32, 0, sizeof(__s32) * 20 * 30);
		TestResult_Open= nvt_read_open(dev, rawdata_16, rawdata_16_1, rawdata_32, open_rawdata_32, i);
		if(TestResult_Open != 0) {
			TestResult_Open = 1;	// 1:ERROR
			break;
		}
		msleep(10);
	}
	TestResult_Open = RawDataTest_Sub(BoundaryOpen, open_rawdata_32, RecordResult,AIN_TX_NUM, AIN_RX_NUM,
			PSConfig_Tolerance_Postive_Mutual, PSConfig_Tolerance_Negative_Mutual, PSConfig_DiffLimitG_Postive_Mutual, PSConfig_DiffLimitG_Negative_Mutual,
			0, 0);
	print_selftest_result(dev, OPEN, TestResult_Open, RecordResult, open_rawdata_32, AIN_TX_NUM, AIN_RX_NUM, mode);

	ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "Channel Status : ");
	if(!TestResult_Short_RXRX && !TestResult_Short_TXRX && !TestResult_Short_TXTX && !TestResult_Open) {
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "Pass\n");
	}
	else {
		ret += snprintf(buf + ret, LOG_BUF_SIZE - ret, "Fail\n");
	}

	nt11206_hw_reset(dev);

	if(RecordResult)
		kfree(RecordResult);

	if(rawdata_16)
		kfree(rawdata_16);

	if(rawdata_16_1)
		kfree(rawdata_16_1);

	if(rawdata_32)
		kfree(rawdata_32);

	if(open_rawdata_32)
		kfree(open_rawdata_32);

	return ret;
}
