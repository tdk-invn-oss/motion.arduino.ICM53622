/*
 *
 * Copyright (c) [2025] by InvenSense, Inc.
 * 
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

#ifndef _INV_IMU_DEFS_H_
#define _INV_IMU_DEFS_H_

#ifdef __cplusplus
extern "C" {
#endif

/** @file inv_imu_defs.h
 * File exposing the device register map
 */

#include <stdint.h>

/* Include device definition */
#include "imu/inv_imu.h"

/* Include regmap internal (le = little endian, be = big endian) */
#include "imu/inv_imu_regmap_le.h"
/* #include "imu/inv_imu_regmap_be.h" */

/* Error/Success codes */
#define INV_IMU_OK                   0 /**< Success */
#define INV_IMU_ERROR                -1 /**< Unspecified error */
#define INV_IMU_ERROR_TRANSPORT      -3 /**< Error occurred at transport level */
#define INV_IMU_ERROR_TIMEOUT        -4 /**< Action did not complete in the expected time window */
#define INV_IMU_ERROR_BAD_ARG        -11 /**< Invalid argument provided */
#define INV_IMU_ERROR_EDMP_ODR       -126 /**< EDMP ODR decimator reconfiguration is needed */
#define INV_IMU_ERROR_EDMP_BUF_EMPTY -127 /**< EDMP buffer is empty */

/* Enable and Disable state */
#define INV_IMU_DISABLE (0U)
#define INV_IMU_ENABLE  (1U)

/* Startup times */
#define ACC_STARTUP_TIME_US 10000
#define GYR_STARTUP_TIME_US 70000

/* FIFO: Data size */
#define ACCEL_DATA_SIZE               6
#define GYRO_DATA_SIZE                6
#define TEMP_DATA_SIZE                2
#define FIFO_HEADER_SIZE              1
#define FIFO_TEMP_DATA_SIZE           1
#define FIFO_TS_FSYNC_SIZE            2
#define FIFO_TEMP_HIGH_RES_SIZE       1
#define FIFO_ACCEL_GYRO_HIGH_RES_SIZE 3

/* FIFO: Special values */
#define INVALID_VALUE_FIFO                   ((int16_t)0x8000)
#define INVALID_VALUE_FIFO_1B                ((int8_t)0x80)
#define OUT_OF_BOUND_TEMPERATURE_NEG_FIFO_1B ((int8_t)0x81)
#define OUT_OF_BOUND_TEMPERATURE_POS_FIFO_1B ((int8_t)0x7F)

/** @brief Macro to convert 2 bytes in 1 half-word depending on IMU endianness */
#define FORMAT_16_BITS_DATA(is_big_endian, pIn8, pOut16)                                           \
	*(pOut16) = ((is_big_endian) == 1) ? ((pIn8)[0] << 8) | (pIn8)[1] : ((pIn8)[1] << 8) | (pIn8)[0]

/** Sensor data from registers */
typedef struct {
	int16_t temp_data;
	int16_t accel_data[3];
	int16_t gyro_data[3];
} inv_imu_sensor_data_t;

typedef struct {
	int16_t temp_data;
	int16_t accel_data[3];
	int16_t gyro_data[3];
	/** High-res portion of the accel raw data (if using high-res mode) */
	int8_t accel_high_res[3];
	/** High-res portion of the accel raw data (if using high-res mode) */
	int8_t gyro_high_res[3];
} inv_imu_ois_sensor_data_t;

/** Interrupt number */
typedef enum {
	INV_IMU_INT1,
	INV_IMU_INT2,
} inv_imu_int_num_t;

/** Describe the content of the FIFO header */
typedef union {
	unsigned char Byte;
	struct {
		unsigned char gyro_odr_different : 1;
		unsigned char accel_odr_different : 1;
		unsigned char fsync_bit : 1;
		unsigned char timestamp_bit : 1;
		unsigned char twentybits_bit : 1;
		unsigned char gyro_bit : 1;
		unsigned char accel_bit : 1;
		unsigned char msg_bit : 1;
	} bits;
} fifo_header_t;

/* ----------------------------------------------------------------------------
 * Device registers description
 *
 * Next section defines some of the registers bitfield and declare corresponding
 * accessors.
 * Note that descriptors and accessors are not provided for all the registers 
 * but only for the most useful ones.
 * For all details on registers and bitfields functionalities please refer to 
 * the device datasheet.
 * ---------------------------------------------------------------------------- */

/* ---------------------------------------------------------------------------
 * register bank 0 
 * ---------------------------------------------------------------------------- */

/*
 * DEVICE_CONFIG
 */

/* SPI_MODE */
typedef enum {
	DEVICE_CONFIG_SPI_MODE_1_2 = 0x1,
	DEVICE_CONFIG_SPI_MODE_0_3 = 0x0,
} device_config_spi_mode_t;

/*
 * DRIVE_CONFIG3
 */

/* SPI_SLEW_RATE */
typedef enum {
	DRIVE_CONFIG3_SPI_SLEW_RATE_MAX_2_NS  = 0x05,
	DRIVE_CONFIG3_SPI_SLEW_RATE_MAX_8_NS  = 0x04,
	DRIVE_CONFIG3_SPI_SLEW_RATE_MAX_14_NS = 0x03,
	DRIVE_CONFIG3_SPI_SLEW_RATE_MAX_19_NS = 0x02,
	DRIVE_CONFIG3_SPI_SLEW_RATE_MAX_36_NS = 0x01,
	DRIVE_CONFIG3_SPI_SLEW_RATE_MAX_60_NS = 0x00,
} drive_config3_spi_slew_rate_t;

/*
 * INT_CONFIG
 */

/* INTX_MODE */
typedef enum {
	INT_CONFIG_INTX_MODE_LATCHED = 0x01,
	INT_CONFIG_INTX_MODE_PULSED  = 0x00,
} int_config_intx_mode_t;

/* INTX_DRIVE_CIRCUIT */
typedef enum {
	INT_CONFIG_INTX_DRIVE_CIRCUIT_PP = 0x01,
	INT_CONFIG_INTX_DRIVE_CIRCUIT_OD = 0x00,
} int_config_intx_drive_circuit_t;

/* INTX_POLARITY */
typedef enum {
	INT_CONFIG_INTX_POLARITY_HIGH = 0x01,
	INT_CONFIG_INTX_POLARITY_LOW  = 0x00,
} int_config_intx_polarity_t;

/** @brief Interrupts pin configuration */
typedef struct {
	int_config_intx_polarity_t      int_polarity;
	int_config_intx_mode_t          int_mode;
	int_config_intx_drive_circuit_t int_drive;
} inv_imu_int_pin_config_t;

/*
 * PWR_MGMT0
 */

/* ACCEL_LP_CLK_SEL */
typedef enum {
	PWR_MGMT0_ACCEL_LP_CLK_WUOSC = 0x00,
	PWR_MGMT0_ACCEL_LP_CLK_RCOSC = 0x01,
} pwr_mgmt0_accel_lp_clk_t;

/* GYRO_MODE */
typedef enum {
	PWR_MGMT0_GYRO_MODE_LN      = 0x03,
	PWR_MGMT0_GYRO_MODE_STANDBY = 0x01,
	PWR_MGMT0_GYRO_MODE_OFF     = 0x00,
} pwr_mgmt0_gyro_mode_t;

/* ACCEL_MODE */
typedef enum {
	PWR_MGMT0_ACCEL_MODE_LN  = 0x03,
	PWR_MGMT0_ACCEL_MODE_LP  = 0x02,
	PWR_MGMT0_ACCEL_MODE_OFF = 0x00,
} pwr_mgmt0_accel_mode_t;

/* GYRO_OIS_MODE */
typedef enum {
	PWR_MGMT0_GYRO_OIS_MODE_LN  = 0x01,
	PWR_MGMT0_GYRO_OIS_MODE_OFF = 0x00,
} pwr_mgmt0_gyro_ois_mode_t;

/* ACCEL_OIS_MODE */
typedef enum {
	PWR_MGMT0_ACCEL_OIS_MODE_LN  = 0x01,
	PWR_MGMT0_ACCEL_OIS_MODE_OFF = 0x00,
} pwr_mgmt0_accel_ois_mode_t;

/*
 * GYRO_CONFIG0
 */

/* GYRO_FS_SEL*/
typedef enum {
#if INV_IMU_GYR_HFSR_SUPPORTED
	GYRO_CONFIG0_FS_SEL_31dps   = 7,
	GYRO_CONFIG0_FS_SEL_62dps   = 6,
	GYRO_CONFIG0_FS_SEL_125dps  = 5,
	GYRO_CONFIG0_FS_SEL_250dps  = 4,
	GYRO_CONFIG0_FS_SEL_500dps  = 3,
	GYRO_CONFIG0_FS_SEL_1000dps = 2,
	GYRO_CONFIG0_FS_SEL_2000dps = 1,
	GYRO_CONFIG0_FS_SEL_4000dps = 0,
#else
	GYRO_CONFIG0_FS_SEL_15dps   = 7,
	GYRO_CONFIG0_FS_SEL_31dps   = 6,
	GYRO_CONFIG0_FS_SEL_62dps   = 5,
	GYRO_CONFIG0_FS_SEL_125dps  = 4,
	GYRO_CONFIG0_FS_SEL_250dps  = 3,
	GYRO_CONFIG0_FS_SEL_500dps  = 2,
	GYRO_CONFIG0_FS_SEL_1000dps = 1,
	GYRO_CONFIG0_FS_SEL_2000dps = 0,
#endif
} gyro_config0_fs_sel_t;

/* GYRO_ODR */
typedef enum {
	GYRO_CONFIG0_ODR_1_5625_HZ = 0xF,
	GYRO_CONFIG0_ODR_3_125_HZ  = 0xE,
	GYRO_CONFIG0_ODR_6_25_HZ   = 0xD,
	GYRO_CONFIG0_ODR_12_5_HZ   = 0xC,
	GYRO_CONFIG0_ODR_25_HZ     = 0xB,
	GYRO_CONFIG0_ODR_50_HZ     = 0xA,
	GYRO_CONFIG0_ODR_100_HZ    = 0x9,
	GYRO_CONFIG0_ODR_200_HZ    = 0x8,
	GYRO_CONFIG0_ODR_400_HZ    = 0x7,
	GYRO_CONFIG0_ODR_800_HZ    = 0x6,
	GYRO_CONFIG0_ODR_1600_HZ   = 0x5,
} gyro_config0_odr_t;

/*
 * ACCEL_CONFIG0
 */

/* ACCEL_FS_SEL */
typedef enum {
#if INV_IMU_ACC_HFSR_SUPPORTED
	ACCEL_CONFIG0_FS_SEL_4g  = 0x3,
	ACCEL_CONFIG0_FS_SEL_8g  = 0x2,
	ACCEL_CONFIG0_FS_SEL_16g = 0x1,
	ACCEL_CONFIG0_FS_SEL_32g = 0x0,
#else
	ACCEL_CONFIG0_FS_SEL_2g  = 0x3,
	ACCEL_CONFIG0_FS_SEL_4g  = 0x2,
	ACCEL_CONFIG0_FS_SEL_8g  = 0x1,
	ACCEL_CONFIG0_FS_SEL_16g = 0x0,
#endif
} accel_config0_fs_sel_t;

/* ACCEL_ODR */
typedef enum {
	ACCEL_CONFIG0_ODR_1_5625_HZ = 0xF,
	ACCEL_CONFIG0_ODR_3_125_HZ  = 0xE,
	ACCEL_CONFIG0_ODR_6_25_HZ   = 0xD,
	ACCEL_CONFIG0_ODR_12_5_HZ   = 0xC,
	ACCEL_CONFIG0_ODR_25_HZ     = 0xB,
	ACCEL_CONFIG0_ODR_50_HZ     = 0xA,
	ACCEL_CONFIG0_ODR_100_HZ    = 0x9,
	ACCEL_CONFIG0_ODR_200_HZ    = 0x8,
	ACCEL_CONFIG0_ODR_400_HZ    = 0x7,
	ACCEL_CONFIG0_ODR_800_HZ    = 0x6,
	ACCEL_CONFIG0_ODR_1600_HZ   = 0x5,
} accel_config0_odr_t;

/*
 * GYRO_CONFIG1
 */

/* GYRO_UI_FILT_BW_IND */
typedef enum {
	GYRO_CONFIG1_GYRO_FILT_BW_8         = 0x8,
	GYRO_CONFIG1_GYRO_FILT_BW_16        = 0x7,
	GYRO_CONFIG1_GYRO_FILT_BW_25        = 0x6,
	GYRO_CONFIG1_GYRO_FILT_BW_34        = 0x5,
	GYRO_CONFIG1_GYRO_FILT_BW_53        = 0x4,
	GYRO_CONFIG1_GYRO_FILT_BW_73        = 0x3,
	GYRO_CONFIG1_GYRO_FILT_BW_121       = 0x2,
	GYRO_CONFIG1_GYRO_FILT_BW_180       = 0x1,
	GYRO_CONFIG1_GYRO_FILT_BW_NO_FILTER = 0x0,
} gyro_config1_gyro_filt_bw_t;

/*
 * ACCEL_CONFIG1
 */

/* ACCEL_UI_AVG_IND */
typedef enum {
	ACCEL_CONFIG1_ACCEL_FILT_AVG_64 = 0x5,
	ACCEL_CONFIG1_ACCEL_FILT_AVG_32 = 0x4,
	ACCEL_CONFIG1_ACCEL_FILT_AVG_16 = 0x3,
	ACCEL_CONFIG1_ACCEL_FILT_AVG_8  = 0x2,
	ACCEL_CONFIG1_ACCEL_FILT_AVG_4  = 0x1,
	ACCEL_CONFIG1_ACCEL_FILT_AVG_2  = 0x0,
} accel_config1_accel_filt_avg_t;

/* ACCEL_UI_FILT_BW_IND */
typedef enum {
	ACCEL_CONFIG1_ACCEL_FILT_BW_8         = 0x8,
	ACCEL_CONFIG1_ACCEL_FILT_BW_16        = 0x7,
	ACCEL_CONFIG1_ACCEL_FILT_BW_25        = 0x6,
	ACCEL_CONFIG1_ACCEL_FILT_BW_34        = 0x5,
	ACCEL_CONFIG1_ACCEL_FILT_BW_53        = 0x4,
	ACCEL_CONFIG1_ACCEL_FILT_BW_73        = 0x3,
	ACCEL_CONFIG1_ACCEL_FILT_BW_121       = 0x2,
	ACCEL_CONFIG1_ACCEL_FILT_BW_180       = 0x1,
	ACCEL_CONFIG1_ACCEL_FILT_BW_NO_FILTER = 0x0,
} accel_config1_accel_filt_bw_t;

/*
 * APEX_CONFIG1
 */

/* DMP_ODR */
typedef enum {
	APEX_CONFIG1_DMP_ODR_25Hz   = 0x0,
	APEX_CONFIG1_DMP_ODR_50Hz   = 0x2,
	APEX_CONFIG1_DMP_ODR_100Hz  = 0x3,
	APEX_CONFIG1_DMP_ODR_800Hz  = 0x4,
	APEX_CONFIG1_DMP_ODR_1600Hz = 0x5,
	APEX_CONFIG1_DMP_ODR_400Hz  = 0x1,
} apex_config1_dmp_odr_t;

/*
 * WOM_CONFIG
 */

/* WOM_INT_DUR */
typedef enum {
	WOM_CONFIG_WOM_INT_DUR_1_SMPL = 0x00,
	WOM_CONFIG_WOM_INT_DUR_2_SMPL = 0x01,
	WOM_CONFIG_WOM_INT_DUR_3_SMPL = 0x02,
	WOM_CONFIG_WOM_INT_DUR_4_SMPL = 0x03,
} wom_config_wom_int_dur_t;

/* WOM_INT_MODE */
typedef enum {
	WOM_CONFIG_WOM_INT_MODE_ANDED = 0x01,
	WOM_CONFIG_WOM_INT_MODE_ORED  = 0x00,
} wom_config_wom_int_mode_t;

/* WOM_MODE */
typedef enum {
	WOM_CONFIG_WOM_MODE_CMP_PREV = 0x01,
	WOM_CONFIG_WOM_MODE_CMP_INIT = 0x00,
} wom_config_wom_mode_t;

/*
 * FIFO_CONFIG1
 */

/* FIFO_MODE */
typedef enum {
	FIFO_CONFIG1_FIFO_MODE_SNAPSHOT = 0x01,
	FIFO_CONFIG1_FIFO_MODE_STREAM   = 0x00
} fifo_config1_fifo_mode_t;

typedef enum {
	FIFO_CONFIG1_FIFO_BYPASS_EN  = 0x01,
	FIFO_CONFIG1_FIFO_BYPASS_DIS = 0x00
} fifo_config1_fifo_bypass_t;

/*
 * APEX_DATA0
 */

/* TAP_DIRECTION */
typedef enum {
	APEX_DATA0_TAP_DIRECTION_NEGATIVE = 0x01,
	APEX_DATA0_TAP_DIRECTION_POSITIVE = 0x00,
} apex_data0_tap_direction_t;

/* TAP_AXIS */
typedef enum {
	APEX_DATA0_TAP_AXIS_Z = 0x02,
	APEX_DATA0_TAP_AXIS_Y = 0x01,
	APEX_DATA0_TAP_AXIS_X = 0x00,
} apex_data0_tap_axis_t;

/* TAP_EVENT */
typedef enum {
	APEX_DATA0_TAP_EVENT_TRIPLE = 0x03,
	APEX_DATA0_TAP_EVENT_DOUBLE = 0x02,
	APEX_DATA0_TAP_EVENT_SINGLE = 0x01,
	APEX_DATA0_TAP_EVENT_NONE   = 0x00,
} apex_data_0_tap_event_t;

/*
 * INTF_CONFIG0
 */

/* FIFO_COUNT_REC */
typedef enum {
	INTF_CONFIG0_FIFO_COUNT_REC_RECORD = 0x01,
	INTF_CONFIG0_FIFO_COUNT_REC_BYTE   = 0x00,
} intf_config0_fifo_count_rec_t;

/* FIFO_COUNT_ENDIAN */
typedef enum {
	INTF_CONFIG0_FIFO_COUNT_BIG_ENDIAN    = 0x01,
	INTF_CONFIG0_FIFO_COUNT_LITTLE_ENDIAN = 0x00,
} intf_config0_fifo_count_endian_t;

/* DATA_ENDIAN */
typedef enum {
	INTF_CONFIG0_DATA_BIG_ENDIAN    = 0x01,
	INTF_CONFIG0_DATA_LITTLE_ENDIAN = 0x00,
} intf_config0_data_endian_t;

/*
 * OIS_CONFIG0
 */

/* OIS_ACCEL_LP_CLK_SEL */
typedef enum {
	OIS_CONFIG0_ACCEL_LP_CLK_WUOSC = 0x00,
	OIS_CONFIG0_ACCEL_LP_CLK_RCOSC = 0x01,
} ois_config0_accel_lp_clk_t;

/*
 * OIS_CONFIG1
 */
/* GYRO_OIS_FS_SEL */
typedef enum {
#if INV_IMU_GYR_HFSR_SUPPORTED
	OIS_CONFIG1_GYRO_FS_SEL_31dps   = 7,
	OIS_CONFIG1_GYRO_FS_SEL_62dps   = 6,
	OIS_CONFIG1_GYRO_FS_SEL_125dps  = 5,
	OIS_CONFIG1_GYRO_FS_SEL_250dps  = 4,
	OIS_CONFIG1_GYRO_FS_SEL_500dps  = 3,
	OIS_CONFIG1_GYRO_FS_SEL_1000dps = 2,
	OIS_CONFIG1_GYRO_FS_SEL_2000dps = 1,
	OIS_CONFIG1_GYRO_FS_SEL_4000dps = 0,
#else
	OIS_CONFIG1_GYRO_FS_SEL_15dps   = 7,
	OIS_CONFIG1_GYRO_FS_SEL_31dps   = 6,
	OIS_CONFIG1_GYRO_FS_SEL_62dps   = 5,
	OIS_CONFIG1_GYRO_FS_SEL_125dps  = 4,
	OIS_CONFIG1_GYRO_FS_SEL_250dps  = 3,
	OIS_CONFIG1_GYRO_FS_SEL_500dps  = 2,
	OIS_CONFIG1_GYRO_FS_SEL_1000dps = 1,
	OIS_CONFIG1_GYRO_FS_SEL_2000dps = 0,
#endif
} ois_config1_gyro_fs_sel_t;

/* ACCEL_OIS_FS_SEL */
typedef enum {
#if INV_IMU_ACC_HFSR_SUPPORTED
	OIS_CONFIG1_ACCEL_FS_SEL_4g  = 0x3,
	OIS_CONFIG1_ACCEL_FS_SEL_8g  = 0x2,
	OIS_CONFIG1_ACCEL_FS_SEL_16g = 0x1,
	OIS_CONFIG1_ACCEL_FS_SEL_32g = 0x0,
#else
	OIS_CONFIG1_ACCEL_FS_SEL_2g  = 0x3,
	OIS_CONFIG1_ACCEL_FS_SEL_4g  = 0x2,
	OIS_CONFIG1_ACCEL_FS_SEL_8g  = 0x1,
	OIS_CONFIG1_ACCEL_FS_SEL_16g = 0x0,
#endif
} ois_config1_accel_fs_sel_t;

/*
 * OIS_CONFIG2
 */

/* FSYNC_OIS_SEL */
typedef enum {
	OIS_CONFIG2_FSYNC_SEL_NO      = 0x0,
	OIS_CONFIG2_FSYNC_SEL_TEMP    = 0x1,
	OIS_CONFIG2_FSYNC_SEL_GYRO_X  = 0x2,
	OIS_CONFIG2_FSYNC_SEL_GYRO_Y  = 0x3,
	OIS_CONFIG2_FSYNC_SEL_GYRO_Z  = 0x4,
	OIS_CONFIG2_FSYNC_SEL_ACCEL_X = 0x5,
	OIS_CONFIG2_FSYNC_SEL_ACCEL_Y = 0x6,
	OIS_CONFIG2_FSYNC_SEL_ACCEL_Z = 0x7,
} ois_config2_fsync_sel_t;

/*
 * OIS_CONFIG3
 */

/* GYRO_OIS_FILT_BW_IND */
typedef enum {
	OIS_CONFIG3_GYRO_FILT_BW_IND_8         = 12,
	OIS_CONFIG3_GYRO_FILT_BW_IND_16        = 11,
	OIS_CONFIG3_GYRO_FILT_BW_IND_24        = 10,
	OIS_CONFIG3_GYRO_FILT_BW_IND_32        = 9,
	OIS_CONFIG3_GYRO_FILT_BW_IND_48        = 8,
	OIS_CONFIG3_GYRO_FILT_BW_IND_66        = 7,
	OIS_CONFIG3_GYRO_FILT_BW_IND_100       = 6,
	OIS_CONFIG3_GYRO_FILT_BW_IND_136       = 5,
	OIS_CONFIG3_GYRO_FILT_BW_IND_210       = 4,
	OIS_CONFIG3_GYRO_FILT_BW_IND_289       = 3,
	OIS_CONFIG3_GYRO_FILT_BW_IND_462       = 2,
	OIS_CONFIG3_GYRO_FILT_BW_IND_655       = 1,
	OIS_CONFIG3_GYRO_FILT_BW_IND_NO_FILTER = 0,
} ois_config3_gyro_filt_bw_ind_t;

/* ACCEL_OIS_FILT_BW_IND */
typedef enum {
	OIS_CONFIG3_ACCEL_FILT_BW_IND_8         = 12,
	OIS_CONFIG3_ACCEL_FILT_BW_IND_16        = 11,
	OIS_CONFIG3_ACCEL_FILT_BW_IND_24        = 10,
	OIS_CONFIG3_ACCEL_FILT_BW_IND_32        = 9,
	OIS_CONFIG3_ACCEL_FILT_BW_IND_48        = 8,
	OIS_CONFIG3_ACCEL_FILT_BW_IND_66        = 7,
	OIS_CONFIG3_ACCEL_FILT_BW_IND_100       = 6,
	OIS_CONFIG3_ACCEL_FILT_BW_IND_136       = 5,
	OIS_CONFIG3_ACCEL_FILT_BW_IND_210       = 4,
	OIS_CONFIG3_ACCEL_FILT_BW_IND_289       = 3,
	OIS_CONFIG3_ACCEL_FILT_BW_IND_462       = 2,
	OIS_CONFIG3_ACCEL_FILT_BW_IND_655       = 1,
	OIS_CONFIG3_ACCEL_FILT_BW_IND_NO_FILTER = 0,
} ois_config3_accel_filt_bw_ind_t;

/* ---------------------------------------------------------------------------
 * register bank MREG1 
 * ---------------------------------------------------------------------------- */

/*
 * TMST_CONFIG1
 */

/* TMST_RES */
typedef enum {
	TMST_CONFIG1_RESOL_16us = 0x01,
	TMST_CONFIG1_RESOL_1us  = 0x00,
} tmst_config1_resol_t;

/*
 * INT_CONFIG1
 * Register Name: INT_CONFIG1
 */

/* INT_PULSE_DURATION */
typedef enum {
	INT_CONFIG1_INT_PULSE_DURATION_100_US = 0x0,
	INT_CONFIG1_INT_PULSE_DURATION_8_US   = 0x1,
} int_config1_int_pulse_duration_t;

/*
 * FIFO_CONFIG5
 */

/* FIFO_WM_GT_TH */
typedef enum {
	FIFO_CONFIG5_FIFO_WM_EQ_OR_GT_TH = 0x1,
	FIFO_CONFIG5_FIFO_WM_EQ_TH       = 0x0,
} fifo_config5_fifo_wm_gt_th_t;

/*
 * FSYNC_CONFIG
 */

/* FSYNC_UI_SEL */
typedef enum {
	FSYNC_CONFIG_UI_SEL_NO      = 0x0,
	FSYNC_CONFIG_UI_SEL_TEMP    = 0x1,
	FSYNC_CONFIG_UI_SEL_GYRO_X  = 0x2,
	FSYNC_CONFIG_UI_SEL_GYRO_Y  = 0x3,
	FSYNC_CONFIG_UI_SEL_GYRO_Z  = 0x4,
	FSYNC_CONFIG_UI_SEL_ACCEL_X = 0x5,
	FSYNC_CONFIG_UI_SEL_ACCEL_Y = 0x6,
	FSYNC_CONFIG_UI_SEL_ACCEL_Z = 0x7,
} fsync_config_ui_sel_t;

/*
 * ST_CONFIG
 */
typedef enum {
	ST_CONFIG_16_SAMPLES  = 0,
	ST_CONFIG_200_SAMPLES = 1,
} st_config_num_samples_t;

typedef enum {
	ST_CONFIG_ACCEL_ST_LIM_50 = 7,
} st_config_accel_st_lim_t;

typedef enum {
	ST_CONFIG_GYRO_ST_LIM_50 = 7,
} st_config_gyro_st_lim_t;

/*
 * OTP_CONFIG
 */

/* OTP_CONFIG */
typedef enum {
	OTP_CONFIG_OTP_COPY_TRIM    = 1,
	OTP_CONFIG_OTP_COPY_ST_DATA = 3,
} otp_config_copy_mode_t;

/*
 * APEX_CONFIG2
*/

/* TAP_MAX */
typedef enum {
	APEX_CONFIG2_TAP_MAX_TRIPLE = 3,
	APEX_CONFIG2_TAP_MAX_DOUBLE = 2,
	APEX_CONFIG2_TAP_MAX_SINGLE = 1,
} apex_config2_tap_max_t;

/* TAP_MIN */
typedef enum {
	APEX_CONFIG2_TAP_MIN_TRIPLE = 3,
	APEX_CONFIG2_TAP_MIN_DOUBLE = 2,
	APEX_CONFIG2_TAP_MIN_SINGLE = 1,
} apex_config2_tap_min_t;

/* DMP_POWER_SAVE_TIME_SEL */
typedef enum {
	APEX_CONFIG2_DMP_POWER_SAVE_TIME_SEL_0_S  = 0x0,
	APEX_CONFIG2_DMP_POWER_SAVE_TIME_SEL_4_S  = 0x1,
	APEX_CONFIG2_DMP_POWER_SAVE_TIME_SEL_8_S  = 0x2,
	APEX_CONFIG2_DMP_POWER_SAVE_TIME_SEL_12_S = 0x3,
	APEX_CONFIG2_DMP_POWER_SAVE_TIME_SEL_16_S = 0x4,
	APEX_CONFIG2_DMP_POWER_SAVE_TIME_SEL_20_S = 0x5,
	APEX_CONFIG2_DMP_POWER_SAVE_TIME_SEL_24_S = 0x6,
	APEX_CONFIG2_DMP_POWER_SAVE_TIME_SEL_28_S = 0x7,
	APEX_CONFIG2_DMP_POWER_SAVE_TIME_SEL_32_S = 0x8,
	APEX_CONFIG2_DMP_POWER_SAVE_TIME_SEL_36_S = 0x9,
	APEX_CONFIG2_DMP_POWER_SAVE_TIME_SEL_40_S = 0xA,
	APEX_CONFIG2_DMP_POWER_SAVE_TIME_SEL_44_S = 0xB,
	APEX_CONFIG2_DMP_POWER_SAVE_TIME_SEL_48_S = 0xC,
	APEX_CONFIG2_DMP_POWER_SAVE_TIME_SEL_52_S = 0xD,
	APEX_CONFIG2_DMP_POWER_SAVE_TIME_SEL_56_S = 0xE,
	APEX_CONFIG2_DMP_POWER_SAVE_TIME_SEL_60_S = 0xF,
} apex_config2_dmp_power_save_time_t;

/*
 * APEX_CONFIG3
*/

/* TAP_TAVG */
typedef enum {
	APEX_CONFIG3_TAP_TAVG_8 = 3,
	APEX_CONFIG3_TAP_TAVG_4 = 2,
	APEX_CONFIG3_TAP_TAVG_2 = 1,
	APEX_CONFIG3_TAP_TAVG_1 = 0,
} apex_config3_tap_tavg_t;

/* TAP_TMAX */
typedef enum {
	APEX_CONFIG3_TAP_TMAX_1125_MS = 7,
	APEX_CONFIG3_TAP_TMAX_1000_MS = 6,
	APEX_CONFIG3_TAP_TMAX_875_MS  = 5,
	APEX_CONFIG3_TAP_TMAX_750_MS  = 4,
	APEX_CONFIG3_TAP_TMAX_625_MS  = 3,
	APEX_CONFIG3_TAP_TMAX_500_MS  = 2,
	APEX_CONFIG3_TAP_TMAX_375_MS  = 1,
	APEX_CONFIG3_TAP_TMAX_250_MS  = 0,
} apex_config3_tap_tmax_t;

/* TAP_TMIN */
typedef enum {
	APEX_CONFIG3_TAP_TMIN_230_MS = 7,
	APEX_CONFIG3_TAP_TMIN_215_MS = 6,
	APEX_CONFIG3_TAP_TMIN_200_MS = 5,
	APEX_CONFIG3_TAP_TMIN_185_MS = 4,
	APEX_CONFIG3_TAP_TMIN_165_MS = 3,
	APEX_CONFIG3_TAP_TMIN_150_MS = 2,
	APEX_CONFIG3_TAP_TMIN_135_MS = 1,
	APEX_CONFIG3_TAP_TMIN_120_MS = 0,
} apex_config3_tap_tmin_t;

/*
 * APEX_CONFIG4
*/

/* TAP_MAX_PEAK_TOL */
typedef enum {
	APEX_CONFIG4_TAP_MAX_PEAK_TOL_12_5 = 3,
	APEX_CONFIG4_TAP_MAX_PEAK_TOL_25   = 2,
	APEX_CONFIG4_TAP_MAX_PEAK_TOL_37_5 = 1,
	APEX_CONFIG4_TAP_MAX_PEAK_TOL_50   = 0,
} apex_config4_tap_max_peak_tol_t;

/* TAP_MIN_JERK_THR */
typedef enum {
	APEX_CONFIG4_TAP_MIN_JERK_THR_4000mg = 63,
	APEX_CONFIG4_TAP_MIN_JERK_THR_3937mg = 62,
	APEX_CONFIG4_TAP_MIN_JERK_THR_3875mg = 61,
	APEX_CONFIG4_TAP_MIN_JERK_THR_3812mg = 60,
	APEX_CONFIG4_TAP_MIN_JERK_THR_3750mg = 59,
	APEX_CONFIG4_TAP_MIN_JERK_THR_3687mg = 58,
	APEX_CONFIG4_TAP_MIN_JERK_THR_3625mg = 57,
	APEX_CONFIG4_TAP_MIN_JERK_THR_3562mg = 56,
	APEX_CONFIG4_TAP_MIN_JERK_THR_3500mg = 55,
	APEX_CONFIG4_TAP_MIN_JERK_THR_3437mg = 54,
	APEX_CONFIG4_TAP_MIN_JERK_THR_3375mg = 53,
	APEX_CONFIG4_TAP_MIN_JERK_THR_3312mg = 52,
	APEX_CONFIG4_TAP_MIN_JERK_THR_3250mg = 51,
	APEX_CONFIG4_TAP_MIN_JERK_THR_3187mg = 50,
	APEX_CONFIG4_TAP_MIN_JERK_THR_3125mg = 49,
	APEX_CONFIG4_TAP_MIN_JERK_THR_3062mg = 48,
	APEX_CONFIG4_TAP_MIN_JERK_THR_3000mg = 47,
	APEX_CONFIG4_TAP_MIN_JERK_THR_2937mg = 46,
	APEX_CONFIG4_TAP_MIN_JERK_THR_2875mg = 45,
	APEX_CONFIG4_TAP_MIN_JERK_THR_2812mg = 44,
	APEX_CONFIG4_TAP_MIN_JERK_THR_2750mg = 43,
	APEX_CONFIG4_TAP_MIN_JERK_THR_2687mg = 42,
	APEX_CONFIG4_TAP_MIN_JERK_THR_2625mg = 41,
	APEX_CONFIG4_TAP_MIN_JERK_THR_2562mg = 40,
	APEX_CONFIG4_TAP_MIN_JERK_THR_2500mg = 39,
	APEX_CONFIG4_TAP_MIN_JERK_THR_2437mg = 38,
	APEX_CONFIG4_TAP_MIN_JERK_THR_2375mg = 37,
	APEX_CONFIG4_TAP_MIN_JERK_THR_2312mg = 36,
	APEX_CONFIG4_TAP_MIN_JERK_THR_2250mg = 35,
	APEX_CONFIG4_TAP_MIN_JERK_THR_2187mg = 34,
	APEX_CONFIG4_TAP_MIN_JERK_THR_2125mg = 33,
	APEX_CONFIG4_TAP_MIN_JERK_THR_2062mg = 32,
	APEX_CONFIG4_TAP_MIN_JERK_THR_2000mg = 31,
	APEX_CONFIG4_TAP_MIN_JERK_THR_1937mg = 30,
	APEX_CONFIG4_TAP_MIN_JERK_THR_1875mg = 29,
	APEX_CONFIG4_TAP_MIN_JERK_THR_1812mg = 28,
	APEX_CONFIG4_TAP_MIN_JERK_THR_1750mg = 27,
	APEX_CONFIG4_TAP_MIN_JERK_THR_1687mg = 26,
	APEX_CONFIG4_TAP_MIN_JERK_THR_1625mg = 25,
	APEX_CONFIG4_TAP_MIN_JERK_THR_1562mg = 24,
	APEX_CONFIG4_TAP_MIN_JERK_THR_1500mg = 23,
	APEX_CONFIG4_TAP_MIN_JERK_THR_1437mg = 22,
	APEX_CONFIG4_TAP_MIN_JERK_THR_1375mg = 21,
	APEX_CONFIG4_TAP_MIN_JERK_THR_1312mg = 20,
	APEX_CONFIG4_TAP_MIN_JERK_THR_1250mg = 19,
	APEX_CONFIG4_TAP_MIN_JERK_THR_1187mg = 18,
	APEX_CONFIG4_TAP_MIN_JERK_THR_1125mg = 17,
	APEX_CONFIG4_TAP_MIN_JERK_THR_1062mg = 16,
	APEX_CONFIG4_TAP_MIN_JERK_THR_1000mg = 15,
	APEX_CONFIG4_TAP_MIN_JERK_THR_937mg  = 14,
	APEX_CONFIG4_TAP_MIN_JERK_THR_875mg  = 13,
	APEX_CONFIG4_TAP_MIN_JERK_THR_812mg  = 12,
	APEX_CONFIG4_TAP_MIN_JERK_THR_750mg  = 11,
	APEX_CONFIG4_TAP_MIN_JERK_THR_687mg  = 10,
	APEX_CONFIG4_TAP_MIN_JERK_THR_625mg  = 9,
	APEX_CONFIG4_TAP_MIN_JERK_THR_562mg  = 8,
	APEX_CONFIG4_TAP_MIN_JERK_THR_500mg  = 7,
	APEX_CONFIG4_TAP_MIN_JERK_THR_437mg  = 6,
	APEX_CONFIG4_TAP_MIN_JERK_THR_375mg  = 5,
	APEX_CONFIG4_TAP_MIN_JERK_THR_312mg  = 4,
	APEX_CONFIG4_TAP_MIN_JERK_THR_250mg  = 3,
	APEX_CONFIG4_TAP_MIN_JERK_THR_187mg  = 2,
	APEX_CONFIG4_TAP_MIN_JERK_THR_125mg  = 1,
	APEX_CONFIG4_TAP_MIN_JERK_THR_62mg   = 0,
} apex_config4_tap_min_jerk_thr_t;

/*
 * APEX_CONFIG5
*/

/* TILT_WAIT_TIME_SEL */
typedef enum {
	APEX_CONFIG5_TILT_WAIT_TIME_0_S = 0,
	APEX_CONFIG5_TILT_WAIT_TIME_2_S = 1,
	APEX_CONFIG5_TILT_WAIT_TIME_4_S = 2,
	APEX_CONFIG5_TILT_WAIT_TIME_6_S = 3,
} apex_config5_tilt_wait_time_t;

/* LOWG_PEAK_TH_HYST_SEL */
typedef enum {
	APEX_CONFIG5_LOWG_PEAK_TH_HYST_31_MG  = 0,
	APEX_CONFIG5_LOWG_PEAK_TH_HYST_63_MG  = 1,
	APEX_CONFIG5_LOWG_PEAK_TH_HYST_94_MG  = 2,
	APEX_CONFIG5_LOWG_PEAK_TH_HYST_125_MG = 3,
	APEX_CONFIG5_LOWG_PEAK_TH_HYST_156_MG = 4,
	APEX_CONFIG5_LOWG_PEAK_TH_HYST_188_MG = 5,
	APEX_CONFIG5_LOWG_PEAK_TH_HYST_219_MG = 6,
	APEX_CONFIG5_LOWG_PEAK_TH_HYST_250_MG = 7,
} apex_config5_lowg_peak_th_hyst_t;

/* HIGHG_PEAK_TH_HYST_SEL */
typedef enum {
	APEX_CONFIG5_HIGHG_PEAK_TH_HYST_31_MG  = 0,
	APEX_CONFIG5_HIGHG_PEAK_TH_HYST_63_MG  = 1,
	APEX_CONFIG5_HIGHG_PEAK_TH_HYST_94_MG  = 2,
	APEX_CONFIG5_HIGHG_PEAK_TH_HYST_125_MG = 3,
	APEX_CONFIG5_HIGHG_PEAK_TH_HYST_156_MG = 4,
	APEX_CONFIG5_HIGHG_PEAK_TH_HYST_188_MG = 5,
	APEX_CONFIG5_HIGHG_PEAK_TH_HYST_219_MG = 6,
	APEX_CONFIG5_HIGHG_PEAK_TH_HYST_250_MG = 7,
} apex_config5_highg_peak_th_hyst_t;

/*
 * APEX_CONFIG9
*/

/* FF_DEBOUNCE_DURATION_SEL */
typedef enum {
	APEX_CONFIG9_FF_DEBOUNCE_DURATION_1125_MS = 0,
	APEX_CONFIG9_FF_DEBOUNCE_DURATION_1250_MS = 1,
	APEX_CONFIG9_FF_DEBOUNCE_DURATION_1375_MS = 2,
	APEX_CONFIG9_FF_DEBOUNCE_DURATION_1500_MS = 3,
	APEX_CONFIG9_FF_DEBOUNCE_DURATION_1625_MS = 4,
	APEX_CONFIG9_FF_DEBOUNCE_DURATION_1750_MS = 5,
	APEX_CONFIG9_FF_DEBOUNCE_DURATION_1875_MS = 6,
	APEX_CONFIG9_FF_DEBOUNCE_DURATION_2000_MS = 7,
	APEX_CONFIG9_FF_DEBOUNCE_DURATION_2125_MS = 8,
	APEX_CONFIG9_FF_DEBOUNCE_DURATION_2250_MS = 9,
	APEX_CONFIG9_FF_DEBOUNCE_DURATION_2375_MS = 10,
	APEX_CONFIG9_FF_DEBOUNCE_DURATION_2500_MS = 11,
	APEX_CONFIG9_FF_DEBOUNCE_DURATION_2625_MS = 12,
	APEX_CONFIG9_FF_DEBOUNCE_DURATION_2750_MS = 13,
	APEX_CONFIG9_FF_DEBOUNCE_DURATION_2875_MS = 14,
	APEX_CONFIG9_FF_DEBOUNCE_DURATION_3000_MS = 15,
} apex_config9_ff_debounce_duration_t;

/* TAP_SMUDGE_REJECT_THR */
typedef enum {
	APEX_CONFIG9_TAP_SMUDGE_REJECT_THR_115_MS = 7,
	APEX_CONFIG9_TAP_SMUDGE_REJECT_THR_110_MS = 6,
	APEX_CONFIG9_TAP_SMUDGE_REJECT_THR_100_MS = 5,
	APEX_CONFIG9_TAP_SMUDGE_REJECT_THR_95_MS  = 4,
	APEX_CONFIG9_TAP_SMUDGE_REJECT_THR_85_MS  = 3,
	APEX_CONFIG9_TAP_SMUDGE_REJECT_THR_80_MS  = 2,
	APEX_CONFIG9_TAP_SMUDGE_REJECT_THR_70_MS  = 1,
	APEX_CONFIG9_TAP_SMUDGE_REJECT_THR_65_MS  = 0,
} apex_config9_tap_smudge_reject_thr_t;

/*
 * APEX_CONFIG10
*/

/* LOWG_PEAK_TH_SEL */
typedef enum {
	APEX_CONFIG10_LOWG_PEAK_TH_31_MG   = 0x00,
	APEX_CONFIG10_LOWG_PEAK_TH_63_MG   = 0x01,
	APEX_CONFIG10_LOWG_PEAK_TH_94_MG   = 0x02,
	APEX_CONFIG10_LOWG_PEAK_TH_125_MG  = 0x03,
	APEX_CONFIG10_LOWG_PEAK_TH_156_MG  = 0x04,
	APEX_CONFIG10_LOWG_PEAK_TH_188_MG  = 0x05,
	APEX_CONFIG10_LOWG_PEAK_TH_219_MG  = 0x06,
	APEX_CONFIG10_LOWG_PEAK_TH_250_MG  = 0x07,
	APEX_CONFIG10_LOWG_PEAK_TH_281_MG  = 0x08,
	APEX_CONFIG10_LOWG_PEAK_TH_313_MG  = 0x09,
	APEX_CONFIG10_LOWG_PEAK_TH_344_MG  = 0x0A,
	APEX_CONFIG10_LOWG_PEAK_TH_375_MG  = 0x0B,
	APEX_CONFIG10_LOWG_PEAK_TH_406_MG  = 0x0C,
	APEX_CONFIG10_LOWG_PEAK_TH_438_MG  = 0x0D,
	APEX_CONFIG10_LOWG_PEAK_TH_469_MG  = 0x0E,
	APEX_CONFIG10_LOWG_PEAK_TH_500_MG  = 0x0F,
	APEX_CONFIG10_LOWG_PEAK_TH_531_MG  = 0x10,
	APEX_CONFIG10_LOWG_PEAK_TH_563_MG  = 0x11,
	APEX_CONFIG10_LOWG_PEAK_TH_594_MG  = 0x12,
	APEX_CONFIG10_LOWG_PEAK_TH_625_MG  = 0x13,
	APEX_CONFIG10_LOWG_PEAK_TH_656_MG  = 0x14,
	APEX_CONFIG10_LOWG_PEAK_TH_688_MG  = 0x15,
	APEX_CONFIG10_LOWG_PEAK_TH_719_MG  = 0x16,
	APEX_CONFIG10_LOWG_PEAK_TH_750_MG  = 0x17,
	APEX_CONFIG10_LOWG_PEAK_TH_781_MG  = 0x18,
	APEX_CONFIG10_LOWG_PEAK_TH_813_MG  = 0x19,
	APEX_CONFIG10_LOWG_PEAK_TH_844_MG  = 0x1A,
	APEX_CONFIG10_LOWG_PEAK_TH_875_MG  = 0x1B,
	APEX_CONFIG10_LOWG_PEAK_TH_906_MG  = 0x1C,
	APEX_CONFIG10_LOWG_PEAK_TH_938_MG  = 0x1D,
	APEX_CONFIG10_LOWG_PEAK_TH_969_MG  = 0x1E,
	APEX_CONFIG10_LOWG_PEAK_TH_1000_MG = 0x1F,
} apex_config10_lowg_peak_th_t;

/* LOWG_TIME_TH_SEL */
typedef enum {
	APEX_CONFIG10_LOWG_TIME_TH_1_SAMPLE  = 0x00,
	APEX_CONFIG10_LOWG_TIME_TH_2_SAMPLES = 0x01,
	APEX_CONFIG10_LOWG_TIME_TH_3_SAMPLES = 0x02,
	APEX_CONFIG10_LOWG_TIME_TH_4_SAMPLES = 0x03,
	APEX_CONFIG10_LOWG_TIME_TH_5_SAMPLES = 0x04,
	APEX_CONFIG10_LOWG_TIME_TH_6_SAMPLES = 0x05,
	APEX_CONFIG10_LOWG_TIME_TH_7_SAMPLES = 0x06,
	APEX_CONFIG10_LOWG_TIME_TH_8_SAMPLES = 0x07,
} apex_config10_lowg_time_th_samples_t;

/*
 * APEX_CONFIG11
*/

/* HIGHG_PEAK_TH_SEL */
typedef enum {
	APEX_CONFIG11_HIGHG_PEAK_TH_250_MG  = 0x00,
	APEX_CONFIG11_HIGHG_PEAK_TH_500_MG  = 0x01,
	APEX_CONFIG11_HIGHG_PEAK_TH_750_MG  = 0x02,
	APEX_CONFIG11_HIGHG_PEAK_TH_1000MG  = 0x03,
	APEX_CONFIG11_HIGHG_PEAK_TH_1250_MG = 0x04,
	APEX_CONFIG11_HIGHG_PEAK_TH_1500_MG = 0x05,
	APEX_CONFIG11_HIGHG_PEAK_TH_1750_MG = 0x06,
	APEX_CONFIG11_HIGHG_PEAK_TH_2000_MG = 0x07,
	APEX_CONFIG11_HIGHG_PEAK_TH_2250_MG = 0x08,
	APEX_CONFIG11_HIGHG_PEAK_TH_2500_MG = 0x09,
	APEX_CONFIG11_HIGHG_PEAK_TH_2750_MG = 0x0A,
	APEX_CONFIG11_HIGHG_PEAK_TH_3000_MG = 0x0B,
	APEX_CONFIG11_HIGHG_PEAK_TH_3250_MG = 0x0C,
	APEX_CONFIG11_HIGHG_PEAK_TH_3500_MG = 0x0D,
	APEX_CONFIG11_HIGHG_PEAK_TH_3750_MG = 0x0E,
	APEX_CONFIG11_HIGHG_PEAK_TH_4000_MG = 0x0F,
	APEX_CONFIG11_HIGHG_PEAK_TH_4250_MG = 0x10,
	APEX_CONFIG11_HIGHG_PEAK_TH_4500_MG = 0x11,
	APEX_CONFIG11_HIGHG_PEAK_TH_4750_MG = 0x12,
	APEX_CONFIG11_HIGHG_PEAK_TH_5000_MG = 0x13,
	APEX_CONFIG11_HIGHG_PEAK_TH_5250_MG = 0x14,
	APEX_CONFIG11_HIGHG_PEAK_TH_5500_MG = 0x15,
	APEX_CONFIG11_HIGHG_PEAK_TH_5750_MG = 0x16,
	APEX_CONFIG11_HIGHG_PEAK_TH_6000_MG = 0x17,
	APEX_CONFIG11_HIGHG_PEAK_TH_6250_MG = 0x18,
	APEX_CONFIG11_HIGHG_PEAK_TH_6500_MG = 0x19,
	APEX_CONFIG11_HIGHG_PEAK_TH_6750_MG = 0x1A,
	APEX_CONFIG11_HIGHG_PEAK_TH_7000_MG = 0x1B,
	APEX_CONFIG11_HIGHG_PEAK_TH_7250_MG = 0x1C,
	APEX_CONFIG11_HIGHG_PEAK_TH_7500_MG = 0x1D,
	APEX_CONFIG11_HIGHG_PEAK_TH_7750_MG = 0x1E,
} apex_config11_highg_peak_th_t;

/* HIGHG_TIME_TH_SEL */
typedef enum {
	APEX_CONFIG11_HIGHG_TIME_TH_1_SAMPLE  = 0x00,
	APEX_CONFIG11_HIGHG_TIME_TH_2_SAMPLES = 0x01,
	APEX_CONFIG11_HIGHG_TIME_TH_3_SAMPLES = 0x02,
	APEX_CONFIG11_HIGHG_TIME_TH_4_SAMPLES = 0x03,
	APEX_CONFIG11_HIGHG_TIME_TH_5_SAMPLES = 0x04,
	APEX_CONFIG11_HIGHG_TIME_TH_6_SAMPLES = 0x05,
	APEX_CONFIG11_HIGHG_TIME_TH_7_SAMPLES = 0x06,
	APEX_CONFIG11_HIGHG_TIME_TH_8_SAMPLES = 0x07,
} apex_config11_highg_time_th_samples_t;

/*
 * FDR_CONFIG
 */

/* FDR_SEL */
typedef enum {
	FDR_CONFIG_FDR_SEL_DIS        = 0x0,
	FDR_CONFIG_FDR_SEL_FACTOR_2   = 0x8,
	FDR_CONFIG_FDR_SEL_FACTOR_4   = 0x9,
	FDR_CONFIG_FDR_SEL_FACTOR_8   = 0xA,
	FDR_CONFIG_FDR_SEL_FACTOR_16  = 0xB,
	FDR_CONFIG_FDR_SEL_FACTOR_32  = 0xC,
	FDR_CONFIG_FDR_SEL_FACTOR_64  = 0xD,
	FDR_CONFIG_FDR_SEL_FACTOR_128 = 0xE,
	FDR_CONFIG_FDR_SEL_FACTOR_256 = 0xF,
} fdr_config_fdr_sel_t;

/*
 * APEX_CONFIG12
*/
/* FF_MAX_DURATION_SEL */
typedef enum {
	APEX_CONFIG12_FF_MAX_DURATION_102_CM = 0,
	APEX_CONFIG12_FF_MAX_DURATION_120_CM = 1,
	APEX_CONFIG12_FF_MAX_DURATION_139_CM = 2,
	APEX_CONFIG12_FF_MAX_DURATION_159_CM = 3,
	APEX_CONFIG12_FF_MAX_DURATION_181_CM = 4,
	APEX_CONFIG12_FF_MAX_DURATION_204_CM = 5,
	APEX_CONFIG12_FF_MAX_DURATION_228_CM = 6,
	APEX_CONFIG12_FF_MAX_DURATION_254_CM = 7,
	APEX_CONFIG12_FF_MAX_DURATION_281_CM = 8,
	APEX_CONFIG12_FF_MAX_DURATION_310_CM = 9,
	APEX_CONFIG12_FF_MAX_DURATION_339_CM = 10,
	APEX_CONFIG12_FF_MAX_DURATION_371_CM = 11,
	APEX_CONFIG12_FF_MAX_DURATION_403_CM = 12,
	APEX_CONFIG12_FF_MAX_DURATION_438_CM = 13,
	APEX_CONFIG12_FF_MAX_DURATION_473_CM = 14,
	APEX_CONFIG12_FF_MAX_DURATION_510_CM = 15,
} apex_config12_ff_max_duration_t;

/* FF_MIN_DURATION_SEL */
typedef enum {
	APEX_CONFIG12_FF_MIN_DURATION_10_CM = 0,
	APEX_CONFIG12_FF_MIN_DURATION_12_CM = 1,
	APEX_CONFIG12_FF_MIN_DURATION_13_CM = 2,
	APEX_CONFIG12_FF_MIN_DURATION_16_CM = 3,
	APEX_CONFIG12_FF_MIN_DURATION_18_CM = 4,
	APEX_CONFIG12_FF_MIN_DURATION_20_CM = 5,
	APEX_CONFIG12_FF_MIN_DURATION_23_CM = 6,
	APEX_CONFIG12_FF_MIN_DURATION_25_CM = 7,
	APEX_CONFIG12_FF_MIN_DURATION_28_CM = 8,
	APEX_CONFIG12_FF_MIN_DURATION_31_CM = 9,
	APEX_CONFIG12_FF_MIN_DURATION_34_CM = 10,
	APEX_CONFIG12_FF_MIN_DURATION_38_CM = 11,
	APEX_CONFIG12_FF_MIN_DURATION_41_CM = 12,
	APEX_CONFIG12_FF_MIN_DURATION_45_CM = 13,
	APEX_CONFIG12_FF_MIN_DURATION_48_CM = 14,
	APEX_CONFIG12_FF_MIN_DURATION_52_CM = 15,
} apex_config12_ff_min_duration_t;

/*
 * APEX_CONFIG14
*/

/* EXT_HIGHG_TIME_TH_SEL */
typedef enum {
	APEX_CONFIG14_EXT_HIGHG_TIME_TH_1_SAMPLE  = 0x00,
	APEX_CONFIG14_EXT_HIGHG_TIME_TH_2_SAMPLES = 0x01,
	APEX_CONFIG14_EXT_HIGHG_TIME_TH_3_SAMPLES = 0x02,
	APEX_CONFIG14_EXT_HIGHG_TIME_TH_4_SAMPLES = 0x03,
	APEX_CONFIG14_EXT_HIGHG_TIME_TH_5_SAMPLES = 0x04,
	APEX_CONFIG14_EXT_HIGHG_TIME_TH_6_SAMPLES = 0x05,
	APEX_CONFIG14_EXT_HIGHG_TIME_TH_7_SAMPLES = 0x06,
	APEX_CONFIG14_EXT_HIGHG_TIME_TH_8_SAMPLES = 0x07,
} apex_config14_ext_highg_time_th_samples_t;

/* EXT_HIGHG_PEAK_TH_HYST_SEL */
typedef enum {
	APEX_CONFIG14_EXT_HIGHG_PEAK_TH_HYST_250_MG  = 0,
	APEX_CONFIG14_EXT_HIGHG_PEAK_TH_HYST_500_MG  = 1,
	APEX_CONFIG14_EXT_HIGHG_PEAK_TH_HYST_750_MG  = 2,
	APEX_CONFIG14_EXT_HIGHG_PEAK_TH_HYST_1000_MG = 3,
	APEX_CONFIG14_EXT_HIGHG_PEAK_TH_HYST_1250_MG = 4,
	APEX_CONFIG14_EXT_HIGHG_PEAK_TH_HYST_1500_MG = 5,
	APEX_CONFIG14_EXT_HIGHG_PEAK_TH_HYST_1750_MG = 6,
	APEX_CONFIG14_EXT_HIGHG_PEAK_TH_HYST_2000_MG = 7,
} apex_config14_ext_highg_peak_th_hyst_t;

/* EXT_HIGHG_PEAK_TH_SEL */
typedef enum {
	APEX_CONFIG14_EXT_HIGHG_PEAK_TH_20_G = 0x00,
	APEX_CONFIG14_EXT_HIGHG_PEAK_TH_24_G = 0x01,
	APEX_CONFIG14_EXT_HIGHG_PEAK_TH_28_G = 0x02,
	APEX_CONFIG14_EXT_HIGHG_PEAK_TH_32_G = 0x03,
} apex_config14_ext_highg_peak_th_t;

/** @brief Converts frequency in Hertz to OIS accel BW enum.
 *         Returns an error if the requested value is not supported.
 *  @param[in] bw  Value of the expected BW.
 *  @return        BW converted into enum value.
 */
static inline int inv_imu_convert_hz_to_accel_ois_bw(const uint32_t bw)
{
	switch (bw) {
	case 0:
		return OIS_CONFIG3_ACCEL_FILT_BW_IND_NO_FILTER;
	case 8:
		return OIS_CONFIG3_ACCEL_FILT_BW_IND_8;
	case 16:
		return OIS_CONFIG3_ACCEL_FILT_BW_IND_16;
	case 24:
	case 25:
		return OIS_CONFIG3_ACCEL_FILT_BW_IND_24;
	case 32:
	case 33:
		return OIS_CONFIG3_ACCEL_FILT_BW_IND_32;
	case 48:
	case 49:
		return OIS_CONFIG3_ACCEL_FILT_BW_IND_48;
	case 66:
	case 67:
		return OIS_CONFIG3_ACCEL_FILT_BW_IND_66;
	case 100:
		return OIS_CONFIG3_ACCEL_FILT_BW_IND_100;
	case 136:
		return OIS_CONFIG3_ACCEL_FILT_BW_IND_136;
	case 210:
		return OIS_CONFIG3_ACCEL_FILT_BW_IND_210;
	case 289:
		return OIS_CONFIG3_ACCEL_FILT_BW_IND_289;
	case 462:
		return OIS_CONFIG3_ACCEL_FILT_BW_IND_462;
	case 655:
		return OIS_CONFIG3_ACCEL_FILT_BW_IND_655;
	default:
		return INV_IMU_ERROR_BAD_ARG;
	}
}

/** @brief Converts frequency in Hertz to OIS gyro BW enum.
 *         Returns an error if the requested value is not supported.
 *  @param[in] bw    Value of the expected BW.
 *  @return          BW converted into enum value..
 */
static inline int inv_imu_convert_hz_to_gyro_ois_bw(const uint32_t bw)
{
	switch (bw) {
	case 0:
		return OIS_CONFIG3_GYRO_FILT_BW_IND_NO_FILTER;
	case 8:
		return OIS_CONFIG3_GYRO_FILT_BW_IND_8;
	case 16:
		return OIS_CONFIG3_GYRO_FILT_BW_IND_16;
	case 24:
	case 25:
		return OIS_CONFIG3_GYRO_FILT_BW_IND_24;
	case 32:
	case 33:
		return OIS_CONFIG3_GYRO_FILT_BW_IND_32;
	case 48:
	case 49:
		return OIS_CONFIG3_GYRO_FILT_BW_IND_48;
	case 66:
	case 67:
		return OIS_CONFIG3_GYRO_FILT_BW_IND_66;
	case 100:
		return OIS_CONFIG3_GYRO_FILT_BW_IND_100;
	case 136:
		return OIS_CONFIG3_GYRO_FILT_BW_IND_136;
	case 210:
		return OIS_CONFIG3_GYRO_FILT_BW_IND_210;
	case 289:
		return OIS_CONFIG3_GYRO_FILT_BW_IND_289;
	case 462:
		return OIS_CONFIG3_GYRO_FILT_BW_IND_462;
	case 655:
		return OIS_CONFIG3_GYRO_FILT_BW_IND_655;
	default:
		return INV_IMU_ERROR_BAD_ARG;
	}
}

/** @brief Converts g to OIS accel FSR enum.
 *         Returns an error if the requested value is not supported.
 *  @param[in] fsr  Value of the FSR in g.
 *  @return         FSR converted into enum value.
 */
static inline int inv_imu_convert_g_to_accel_fsr(const uint32_t fsr)
{
	switch (fsr) {
#if !INV_IMU_ACC_HFSR_SUPPORTED
	case 2:
		return ACCEL_CONFIG0_FS_SEL_2g;
#endif
	case 4:
		return ACCEL_CONFIG0_FS_SEL_4g;
	case 8:
		return ACCEL_CONFIG0_FS_SEL_8g;
	case 16:
		return ACCEL_CONFIG0_FS_SEL_16g;
#if INV_IMU_ACC_HFSR_SUPPORTED
	case 32:
		return ACCEL_CONFIG0_FS_SEL_32g;
#endif
	default:
		return INV_IMU_ERROR_BAD_ARG;
	}
}

/** @brief Converts accel FSR enum to g.
 *  @param[in] bitfield Value of the FSR as enum.
 *  @return             FSR converted in g.
 */
static inline int inv_imu_convert_accel_fsr_to_g(const accel_config0_fs_sel_t bitfield)
{
	switch (bitfield) {
#if !INV_IMU_ACC_HFSR_SUPPORTED
	case ACCEL_CONFIG0_FS_SEL_2g:
		return 2;
#endif
	case ACCEL_CONFIG0_FS_SEL_4g:
		return 4;
	case ACCEL_CONFIG0_FS_SEL_8g:
		return 8;
	case ACCEL_CONFIG0_FS_SEL_16g:
		return 16;
#if INV_IMU_ACC_HFSR_SUPPORTED
	case ACCEL_CONFIG0_FS_SEL_32g:
		return 32;
#endif
	default:
		return INV_IMU_ERROR_BAD_ARG;
	}
}

/** @brief Converts dps to OIS gyro FSR enum.
 *         Returns an error if the requested value is not supported.
 *  @param[in] fsr   Value of the FSR in dps.
 *  @return          FSR converted into enum value.
 */
static inline int inv_imu_convert_dps_to_gyro_fsr(const uint32_t fsr)
{
	switch (fsr) {
#if !INV_IMU_GYR_HFSR_SUPPORTED
	case 15:
	case 16:
		return GYRO_CONFIG0_FS_SEL_15dps;
#endif
	case 31:
	case 32:
		return GYRO_CONFIG0_FS_SEL_31dps;
	case 62:
	case 63:
		return GYRO_CONFIG0_FS_SEL_62dps;
	case 125:
		return GYRO_CONFIG0_FS_SEL_125dps;
	case 250:
		return GYRO_CONFIG0_FS_SEL_250dps;
	case 500:
		return GYRO_CONFIG0_FS_SEL_500dps;
	case 1000:
		return GYRO_CONFIG0_FS_SEL_1000dps;
	case 2000:
		return GYRO_CONFIG0_FS_SEL_2000dps;
#if INV_IMU_GYR_HFSR_SUPPORTED
	case 4000:
		return GYRO_CONFIG0_FS_SEL_4000dps;
#endif
	default:
		return INV_IMU_ERROR_BAD_ARG;
	}
}

/** @brief Converts gyro FSR enum to dps.
 *  @param[in] bitfield   Value of the FSR as enum.
 *  @return               FSR converted in dps.
 */
static inline int inv_imu_convert_gyro_fsr_to_dps(const gyro_config0_fs_sel_t bitfield)
{
	switch (bitfield) {
#if !INV_IMU_GYR_HFSR_SUPPORTED
	case GYRO_CONFIG0_FS_SEL_15dps:
		return 15;
#endif
	case GYRO_CONFIG0_FS_SEL_31dps:
		return 31;
	case GYRO_CONFIG0_FS_SEL_62dps:
		return 62;
	case GYRO_CONFIG0_FS_SEL_125dps:
		return 125;
	case GYRO_CONFIG0_FS_SEL_250dps:
		return 250;
	case GYRO_CONFIG0_FS_SEL_500dps:
		return 500;
	case GYRO_CONFIG0_FS_SEL_1000dps:
		return 1000;
	case GYRO_CONFIG0_FS_SEL_2000dps:
		return 2000;
#if INV_IMU_GYR_HFSR_SUPPORTED
	case GYRO_CONFIG0_FS_SEL_4000dps:
		return 4000;
#endif
	default:
		return INV_IMU_ERROR_BAD_ARG;
	}
}

#ifdef __cplusplus
}
#endif

#endif /* #ifndef _INV_IMU_DEFS_H_ */
