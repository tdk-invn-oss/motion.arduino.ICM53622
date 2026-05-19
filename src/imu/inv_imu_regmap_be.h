/*
 *________________________________________________________________________________________________________
 * Copyright (c) 2017 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively "Software") is subject
 * to InvenSense and its licensors intellectual property rights under U.S. and international copyright 
 * and other intellectual property rights laws.
 *
 * InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from InvenSense is strictly prohibited.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE SOFTWARE IS
 * PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * INVENSENSE BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, OR ANY
 * DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
   OF THE SOFTWARE.
 * ________________________________________________________________________________________________________
 */
#ifndef _INV_IMU_REGMAP_BE_H_
#define _INV_IMU_REGMAP_BE_H_

#ifdef __cplusplus
extern "C" {
#endif

/** @file inv_imu_regmap_be.h
 * File exposing the device register map
 */

#include <stdint.h>

/* BANK0 */
#define MCLK_RDY                                                                0x10000
typedef struct {
	uint8_t resv_2                                                                 : 4;
	uint8_t mclk_rdy                                                               : 1;
	uint8_t resv_1                                                                 : 3;
} mclk_rdy_t;

#define DEVICE_CONFIG                                                           0x10001
typedef struct {
	uint8_t resv_2                                                                 : 5;
	uint8_t spi_ap_4wire                                                           : 1;
	uint8_t resv_1                                                                 : 1;
	uint8_t spi_mode                                                               : 1;
} device_config_t;

#define SIGNAL_PATH_RESET                                                       0x10002
typedef struct {
	uint8_t resv_3                                                                 : 3;
	uint8_t soft_reset_device_config                                               : 1;
	uint8_t resv_2                                                                 : 1;
	uint8_t fifo_flush                                                             : 1;
	uint8_t resv_1                                                                 : 2;
} signal_path_reset_t;

#define DRIVE_CONFIG1                                                           0x10003
#define DRIVE_CONFIG2                                                           0x10004
#define DRIVE_CONFIG3                                                           0x10005
typedef struct {
	uint8_t resv_1                                                                 : 5;
	uint8_t spi_slew_rate                                                          : 3;
} drive_config3_t;

#define INT_CONFIG                                                              0x10006
typedef struct {
	uint8_t resv_1                                                                 : 2;
	uint8_t int2_mode                                                              : 1;
	uint8_t int2_drive_circuit                                                     : 1;
	uint8_t int2_polarity                                                          : 1;
	uint8_t int1_mode                                                              : 1;
	uint8_t int1_drive_circuit                                                     : 1;
	uint8_t int1_polarity                                                          : 1;
} int_config_t;

#define TEMP_DATA1                                                              0x10009
#define TEMP_DATA0                                                              0x1000a
#define ACCEL_DATA_X1                                                           0x1000b
#define ACCEL_DATA_X0                                                           0x1000c
#define ACCEL_DATA_Y1                                                           0x1000d
#define ACCEL_DATA_Y0                                                           0x1000e
#define ACCEL_DATA_Z1                                                           0x1000f
#define ACCEL_DATA_Z0                                                           0x10010
#define GYRO_DATA_X1                                                            0x10011
#define GYRO_DATA_X0                                                            0x10012
#define GYRO_DATA_Y1                                                            0x10013
#define GYRO_DATA_Y0                                                            0x10014
#define GYRO_DATA_Z1                                                            0x10015
#define GYRO_DATA_Z0                                                            0x10016
#define TMST_FSYNCH                                                             0x10017
#define TMST_FSYNCL                                                             0x10018
#define DRIVE_CONFIG4                                                           0x1001a
typedef struct {
	uint8_t resv_1                                                                 : 2;
	uint8_t i3c_aux_ddr_slew_rate                                                  : 3;
	uint8_t i3c_aux_sdr_slew_rate                                                  : 3;
} drive_config4_t;

#define DRIVE_CONFIG5                                                           0x1001b
typedef struct {
	uint8_t resv_2                                                                 : 2;
	uint8_t i3c_aux_od_slew_rate                                                   : 3;
	uint8_t resv_1                                                                 : 3;
} drive_config5_t;

#define DRIVE_CONFIG6                                                           0x1001c
typedef struct {
	uint8_t resv_1                                                                 : 5;
	uint8_t spi_aux_slew_rate                                                      : 3;
} drive_config6_t;

#define APEX_DATA4                                                              0x1001d
#define APEX_DATA5                                                              0x1001e
#define PWR_MGMT0                                                               0x1001f
typedef struct {
	uint8_t accel_lp_clk_sel                                                       : 1;
	uint8_t gyro_ois_en_ap                                                         : 1;
	uint8_t accel_ois_en_ap                                                        : 1;
	uint8_t idle                                                                   : 1;
	uint8_t gyro_mode                                                              : 2;
	uint8_t accel_mode                                                             : 2;
} pwr_mgmt0_t;

#define GYRO_CONFIG0                                                            0x10020
typedef struct {
	uint8_t gyro_ui_fs_sel                                                         : 3;
	uint8_t resv_1                                                                 : 1;
	uint8_t gyro_odr                                                               : 4;
} gyro_config0_t;

#define ACCEL_CONFIG0                                                           0x10021
typedef struct {
	uint8_t resv_2                                                                 : 1;
	uint8_t accel_ui_fs_sel                                                        : 2;
	uint8_t resv_1                                                                 : 1;
	uint8_t accel_odr                                                              : 4;
} accel_config0_t;

#define TEMP_CONFIG0                                                            0x10022
typedef struct {
	uint8_t resv_2                                                                 : 1;
	uint8_t temp_filt_bw                                                           : 3;
	uint8_t resv_1                                                                 : 4;
} temp_config0_t;

#define GYRO_CONFIG1                                                            0x10023
typedef struct {
	uint8_t resv_1                                                                 : 4;
	uint8_t gyro_ui_filt_bw                                                        : 4;
} gyro_config1_t;

#define ACCEL_CONFIG1                                                           0x10024
typedef struct {
	uint8_t resv_1                                                                 : 1;
	uint8_t accel_ui_avg                                                           : 3;
	uint8_t accel_ui_filt_bw                                                       : 4;
} accel_config1_t;

#define APEX_CONFIG0                                                            0x10025
typedef struct {
	uint8_t resv_1                                                                 : 4;
	uint8_t dmp_power_save_en                                                      : 1;
	uint8_t dmp_init_en                                                            : 1;
	uint8_t dmp_mem_reset_en                                                       : 2;
} apex_config0_t;

#define APEX_CONFIG1                                                            0x10026
typedef struct {
	uint8_t resv_1                                                                 : 1;
	uint8_t ext_highg_en                                                           : 1;
	uint8_t ff_enable                                                              : 1;
	uint8_t tilt_enable                                                            : 1;
	uint8_t tap_enable                                                             : 1;
	uint8_t dmp_odr                                                                : 3;
} apex_config1_t;

#define WOM_CONFIG                                                              0x10027
typedef struct {
	uint8_t resv_1                                                                 : 3;
	uint8_t wom_int_dur                                                            : 2;
	uint8_t wom_int_mode                                                           : 1;
	uint8_t wom_mode                                                               : 1;
	uint8_t wom_en                                                                 : 1;
} wom_config_t;

#define FIFO_CONFIG1                                                            0x10028
typedef struct {
	uint8_t resv_1                                                                 : 6;
	uint8_t fifo_mode                                                              : 1;
	uint8_t fifo_bypass                                                            : 1;
} fifo_config1_t;

#define FIFO_CONFIG2                                                            0x10029
#define FIFO_CONFIG3                                                            0x1002a
#define INT_SOURCE0                                                             0x1002b
typedef struct {
	uint8_t st_int1_en                                                             : 1;
	uint8_t fsync_int1_en                                                          : 1;
	uint8_t pll_rdy_int1_en                                                        : 1;
	uint8_t reset_done_int1_en                                                     : 1;
	uint8_t drdy_int1_en                                                           : 1;
	uint8_t fifo_ths_int1_en                                                       : 1;
	uint8_t fifo_full_int1_en                                                      : 1;
	uint8_t agc_rdy_int1_en                                                        : 1;
} int_source0_t;

#define INT_SOURCE1                                                             0x1002c
typedef struct {
	uint8_t resv_1                                                                 : 1;
	uint8_t i3c_protocol_error_int1_en                                             : 1;
	uint8_t fsync_ois_int1_en                                                      : 1;
	uint8_t drdy_ois_int1_en                                                       : 1;
	uint8_t ext_highg_int1_en                                                      : 1;
	uint8_t wom_z_int1_en                                                          : 1;
	uint8_t wom_y_int1_en                                                          : 1;
	uint8_t wom_x_int1_en                                                          : 1;
} int_source1_t;

#define INT_SOURCE3                                                             0x1002d
typedef struct {
	uint8_t st_int2_en                                                             : 1;
	uint8_t fsync_int2_en                                                          : 1;
	uint8_t pll_rdy_int2_en                                                        : 1;
	uint8_t reset_done_int2_en                                                     : 1;
	uint8_t drdy_int2_en                                                           : 1;
	uint8_t fifo_ths_int2_en                                                       : 1;
	uint8_t fifo_full_int2_en                                                      : 1;
	uint8_t agc_rdy_int2_en                                                        : 1;
} int_source3_t;

#define INT_SOURCE4                                                             0x1002e
typedef struct {
	uint8_t resv_1                                                                 : 1;
	uint8_t i3c_protocol_error_int2_en                                             : 1;
	uint8_t fsync_ois_int2_en                                                      : 1;
	uint8_t drdy_ois_int2_en                                                       : 1;
	uint8_t ext_highg_int2_en                                                      : 1;
	uint8_t wom_z_int2_en                                                          : 1;
	uint8_t wom_y_int2_en                                                          : 1;
	uint8_t wom_x_int2_en                                                          : 1;
} int_source4_t;

#define FIFO_LOST_PKT0                                                          0x1002f
#define FIFO_LOST_PKT1                                                          0x10030
#define APEX_DATA0                                                              0x10031
typedef struct {
	uint8_t resv_1                                                                 : 3;
	uint8_t tap_direction                                                          : 1;
	uint8_t tap_axis                                                               : 2;
	uint8_t tap_event                                                              : 2;
} apex_data0_t;

#define APEX_DATA1                                                              0x10032
typedef struct {
	uint8_t double_tap_timing                                                      : 8;
} apex_data1_t;

#define APEX_DATA2                                                              0x10033
typedef struct {
	uint8_t triple_tap_timing                                                      : 8;
} apex_data2_t;

#define APEX_DATA3                                                              0x10034
typedef struct {
	uint8_t resv_2                                                                 : 5;
	uint8_t dmp_idle                                                               : 1;
	uint8_t resv_1                                                                 : 2;
} apex_data3_t;

#define INTF_CONFIG0                                                            0x10035
typedef struct {
	uint8_t resv_2                                                                 : 1;
	uint8_t fifo_count_format                                                      : 1;
	uint8_t fifo_count_endian                                                      : 1;
	uint8_t sensor_data_endian                                                     : 1;
	uint8_t st_setting                                                             : 2;
	uint8_t resv_1                                                                 : 2;
} intf_config0_t;

#define INTF_CONFIG1                                                            0x10036
typedef struct {
	uint8_t resv_1                                                                 : 4;
	uint8_t i3c_sdr_en                                                             : 1;
	uint8_t i3c_ddr_en                                                             : 1;
	uint8_t clksel                                                                 : 2;
} intf_config1_t;

#define INT_STATUS_DRDY                                                         0x10039
typedef struct {
	uint8_t resv_1                                                                 : 6;
	uint8_t data_rdy_ois_int                                                       : 1;
	uint8_t data_rdy_int                                                           : 1;
} int_status_drdy_t;

#define INT_STATUS                                                              0x1003a
typedef struct {
	uint8_t st_int                                                                 : 1;
	uint8_t fsync_int                                                              : 1;
	uint8_t pll_rdy_int                                                            : 1;
	uint8_t reset_done_int                                                         : 1;
	uint8_t fsync_ois_int                                                          : 1;
	uint8_t fifo_ths_int                                                           : 1;
	uint8_t fifo_full_int                                                          : 1;
	uint8_t agc_rdy_int                                                            : 1;
} int_status_t;

#define INT_STATUS2                                                             0x1003b
typedef struct {
	uint8_t resv_1                                                                 : 5;
	uint8_t wom_x_int                                                              : 1;
	uint8_t wom_y_int                                                              : 1;
	uint8_t wom_z_int                                                              : 1;
} int_status2_t;

#define INT_STATUS3                                                             0x1003c
typedef struct {
	uint8_t resv_1                                                                 : 2;
	uint8_t highg_det_int                                                          : 1;
	uint8_t tap_det_int                                                            : 1;
	uint8_t tilt_det_int                                                           : 1;
	uint8_t ff_det_int                                                             : 1;
	uint8_t lowg_det_int                                                           : 1;
	uint8_t ext_highg_det_int                                                      : 1;
} int_status3_t;

#define FIFO_COUNTH                                                             0x1003d
#define FIFO_COUNTL                                                             0x1003e
#define FIFO_DATA                                                               0x1003f
typedef struct {
	uint8_t fifo_data                                                              : 8;
} fifo_data_t;

#define DEVICE_CONFIG1                                                          0x10040
typedef struct {
	uint8_t resv_2                                                                 : 5;
	uint8_t spi_aux_4wires                                                         : 1;
	uint8_t resv_1                                                                 : 1;
	uint8_t spi_aux_mode                                                           : 1;
} device_config1_t;

#define GYRO_NOTCH_CONFIG                                                       0x10041
typedef struct {
	uint8_t resv_1                                                                 : 6;
	uint8_t gyro_notch_pref_en                                                     : 1;
	uint8_t gyro_notch_en                                                          : 1;
} gyro_notch_config_t;

#define INTF_CONFIG12                                                           0x10042
typedef struct {
	uint8_t pads_fsync_int2_cfg                                                    : 1;
	uint8_t resv_1                                                                 : 2;
	uint8_t ois_on_ap_if                                                           : 1;
	uint8_t i3c_aux_sdr_en                                                         : 1;
	uint8_t i3c_aux_ddr_en                                                         : 1;
	uint8_t sifs_aux_cfg                                                           : 2;
} intf_config12_t;

#define INT_SOURCE2                                                             0x10043
typedef struct {
	uint8_t resv_1                                                                 : 4;
	uint8_t aux_i3c_prot_err_int1_en                                               : 1;
	uint8_t ois_fsync_int1_en                                                      : 1;
	uint8_t ois_drdy_int1_en                                                       : 1;
	uint8_t ois_agc_rdy_int1_en                                                    : 1;
} int_source2_t;

#define INT_SOURCE5                                                             0x10044
typedef struct {
	uint8_t resv_2                                                                 : 4;
	uint8_t aux_i3c_prot_err_int2_en                                               : 1;
	uint8_t resv_1                                                                 : 1;
	uint8_t ois_drdy_int2_en                                                       : 1;
	uint8_t ois_agc_rdy_int2_en                                                    : 1;
} int_source5_t;

#define OIS_CONFIG0                                                             0x1004d
typedef struct {
	uint8_t resv_1                                                                 : 5;
	uint8_t ois_accel_lp_clk_sel                                                   : 1;
	uint8_t gyro_ois_en                                                            : 1;
	uint8_t accel_ois_en                                                           : 1;
} ois_config0_t;

#define OIS_CONFIG1                                                             0x1004e
typedef struct {
	uint8_t resv_2                                                                 : 2;
	uint8_t gyro_ois_fs_sel                                                        : 3;
	uint8_t resv_1                                                                 : 1;
	uint8_t accel_ois_fs_sel                                                       : 2;
} ois_config1_t;

#define OIS_CONFIG2                                                             0x1004f
typedef struct {
	uint8_t resv_1                                                                 : 4;
	uint8_t fsync_ois_flag_clear_sel                                               : 1;
	uint8_t fsync_ois_sel                                                          : 3;
} ois_config2_t;

#define OIS_CONFIG3                                                             0x10050
typedef struct {
	uint8_t gyro_ois_filt_bw_ind                                                   : 4;
	uint8_t accel_ois_filt_bw_ind                                                  : 4;
} ois_config3_t;

#define INT_STATUS_OIS                                                          0x10051
typedef struct {
	uint8_t resv_1                                                                 : 4;
	uint8_t aux_i3c_prot_err_int                                                   : 1;
	uint8_t ois_fsync_int                                                          : 1;
	uint8_t ois_drdy_int                                                           : 1;
	uint8_t ois_agc_rdy_int                                                        : 1;
} int_status_ois_t;

#define DMP_STATUS_OIS                                                          0x10052
typedef struct {
	uint8_t dmp_status_ois                                                         : 8;
} dmp_status_ois_t;

#define TEMP_DATA1_OIS                                                          0x10053
#define TEMP_DATA0_OIS                                                          0x10054
#define ACCEL_DATA_X1_OIS                                                       0x10055
#define ACCEL_DATA_X0_OIS                                                       0x10056
#define ACCEL_DATA_Y1_OIS                                                       0x10057
#define ACCEL_DATA_Y0_OIS                                                       0x10058
#define ACCEL_DATA_Z1_OIS                                                       0x10059
#define ACCEL_DATA_Z0_OIS                                                       0x1005a
#define GYRO_DATA_X1_OIS                                                        0x1005b
#define GYRO_DATA_X0_OIS                                                        0x1005c
#define GYRO_DATA_Y1_OIS                                                        0x1005d
#define GYRO_DATA_Y0_OIS                                                        0x1005e
#define GYRO_DATA_Z1_OIS                                                        0x1005f
#define GYRO_DATA_Z0_OIS                                                        0x10060
#define TMST_FSYNCH_OIS                                                         0x10061
#define TMST_FSYNCL_OIS                                                         0x10062
#define EXT_DATA_X_OIS                                                          0x10063
typedef struct {
	uint8_t ext_accel_data_x_ois                                                   : 4;
	uint8_t ext_gyro_data_x_ois                                                    : 4;
} ext_data_x_ois_t;

#define EXT_DATA_Y_OIS                                                          0x10064
typedef struct {
	uint8_t ext_accel_data_y_ois                                                   : 4;
	uint8_t ext_gyro_data_y_ois                                                    : 4;
} ext_data_y_ois_t;

#define EXT_DATA_Z_OIS                                                          0x10065
typedef struct {
	uint8_t ext_accel_data_z_ois                                                   : 4;
	uint8_t ext_gyro_data_z_ois                                                    : 4;
} ext_data_z_ois_t;

#define WHO_AM_I                                                                0x10075
typedef struct {
	uint8_t whoami                                                                 : 8;
} who_am_i_t;

#define BLK_SEL_W                                                               0x10079
typedef struct {
	uint8_t blk_sel_w                                                              : 8;
} blk_sel_w_t;

#define MADDR_W                                                                 0x1007a
typedef struct {
	uint8_t maddr_w                                                                : 8;
} maddr_w_t;

#define M_W                                                                     0x1007b
typedef struct {
	uint8_t m_w                                                                    : 8;
} m_w_t;

#define BLK_SEL_R                                                               0x1007c
typedef struct {
	uint8_t blk_sel_r                                                              : 8;
} blk_sel_r_t;

#define MADDR_R                                                                 0x1007d
typedef struct {
	uint8_t maddr_r                                                                : 8;
} maddr_r_t;

#define M_R                                                                     0x1007e
typedef struct {
	uint8_t m_r                                                                    : 8;
} m_r_t;


/* MREG_BAR */

/* MMEM_SIGP */

/* MREG1 */
#define TMST_CONFIG1                                                            0x00
typedef struct {
	uint8_t resv_1                                                                 : 3;
	uint8_t tmst_on_sreg_en                                                        : 1;
	uint8_t tmst_res                                                               : 1;
	uint8_t tmst_delta_en                                                          : 1;
	uint8_t tmst_fsync_en                                                          : 1;
	uint8_t tmst_en                                                                : 1;
} tmst_config1_t;

#define FIFO_CONFIG5                                                            0x01
typedef struct {
	uint8_t resv_1                                                                 : 2;
	uint8_t fifo_wm_gt_th                                                          : 1;
	uint8_t fifo_resume_partial_rd                                                 : 1;
	uint8_t fifo_hires_en                                                          : 1;
	uint8_t fifo_tmst_fsync_en                                                     : 1;
	uint8_t fifo_gyro_en                                                           : 1;
	uint8_t fifo_accel_en                                                          : 1;
} fifo_config5_t;

#define FIFO_CONFIG6                                                            0x02
typedef struct {
	uint8_t resv_2                                                                 : 3;
	uint8_t fifo_empty_indicator_dis                                               : 1;
	uint8_t resv_1                                                                 : 3;
	uint8_t rcosc_req_on_fifo_ths_dis                                              : 1;
} fifo_config6_t;

#define FSYNC_CONFIG                                                            0x03
typedef struct {
	uint8_t resv_2                                                                 : 1;
	uint8_t fsync_ui_sel                                                           : 3;
	uint8_t resv_1                                                                 : 2;
	uint8_t fsync_ui_flag_clear_sel                                                : 1;
	uint8_t fsync_polarity                                                         : 1;
} fsync_config_t;

#define INT_CONFIG0                                                             0x04
typedef struct {
	uint8_t ois_drdy_int_clear                                                     : 2;
	uint8_t ui_drdy_int_clear                                                      : 2;
	uint8_t fifo_ths_int_clear                                                     : 2;
	uint8_t fifo_full_int_clear                                                    : 2;
} int_config0_t;

#define INT_CONFIG1                                                             0x05
typedef struct {
	uint8_t resv_3                                                                 : 1;
	uint8_t int_tpulse_duration                                                    : 1;
	uint8_t resv_2                                                                 : 1;
	uint8_t int_async_reset                                                        : 1;
	uint8_t resv_1                                                                 : 4;
} int_config1_t;

#define SENSOR_CONFIG3                                                          0x06
typedef struct {
	uint8_t resv_3                                                                 : 1;
	uint8_t apex_disable                                                           : 1;
	uint8_t resv_2                                                                 : 1;
	uint8_t ois_config_disable                                                     : 1;
	uint8_t resv_1                                                                 : 4;
} sensor_config3_t;

#define ST_CONFIG                                                               0x13
typedef struct {
	uint8_t accel_st_reg                                                           : 1;
	uint8_t st_number_sample                                                       : 1;
	uint8_t accel_st_lim                                                           : 3;
	uint8_t gyro_st_lim                                                            : 3;
} st_config_t;

#define SELFTEST                                                                0x14
typedef struct {
	uint8_t gyro_st_en                                                             : 1;
	uint8_t accel_st_en                                                            : 1;
	uint8_t en_gz_st                                                               : 1;
	uint8_t en_gy_st                                                               : 1;
	uint8_t en_gx_st                                                               : 1;
	uint8_t en_az_st                                                               : 1;
	uint8_t en_ay_st                                                               : 1;
	uint8_t en_ax_st                                                               : 1;
} selftest_t;

#define PADS_CONFIG5                                                            0x19
typedef struct {
	uint8_t resv_1                                                                 : 7;
	uint8_t pads_fsync_int2_tp1_from_pad_disable_d2a                               : 1;
} pads_config5_t;

#define PADS_CONFIG8                                                            0x1c
typedef struct {
	uint8_t pads_ap_sdo_pe_trim_d2a                                                : 1;
	uint8_t pads_ap_sdo_pud_trim_d2a                                               : 1;
	uint8_t pads_ap_cs_pe_trim_d2a                                                 : 1;
	uint8_t pads_ap_cs_pud_trim_d2a                                                : 1;
	uint8_t pads_ap_sdi_pe_trim_d2a                                                : 1;
	uint8_t pads_ap_sdi_pud_trim_d2a                                               : 1;
	uint8_t pads_sclk_pe_trim_d2a                                                  : 1;
	uint8_t pads_sclk_pud_trim_d2a                                                 : 1;
} pads_config8_t;

#define PADS_CONFIG9                                                            0x1d
typedef struct {
	uint8_t pads_tp0_pe_trim_d2a                                                   : 1;
	uint8_t pads_tp0_pud_trim_d2a                                                  : 1;
	uint8_t pads_tp1_pe_trim_d2a                                                   : 1;
	uint8_t pads_tp1_pud_trim_d2a                                                  : 1;
	uint8_t pads_tp2_pe_trim_d2a                                                   : 1;
	uint8_t pads_tp2_pud_trim_d2a                                                  : 1;
	uint8_t pads_tp3_pe_trim_d2a                                                   : 1;
	uint8_t pads_tp3_pud_trim_d2a                                                  : 1;
} pads_config9_t;

#define INTF_CONFIG6                                                            0x23
typedef struct {
	uint8_t resv_2                                                                 : 3;
	uint8_t i3c_timeout_en                                                         : 1;
	uint8_t i3c_ibi_byte_en                                                        : 1;
	uint8_t i3c_ibi_en                                                             : 1;
	uint8_t resv_1                                                                 : 2;
} intf_config6_t;

#define INTF_CONFIG10                                                           0x25
typedef struct {
	uint8_t asynctime0_dis                                                         : 1;
	uint8_t resv_1                                                                 : 7;
} intf_config10_t;

#define INTF_CONFIG7                                                            0x28
typedef struct {
	uint8_t i3c_aux_ddr_wr_mode                                                    : 1;
	uint8_t resv_2                                                                 : 3;
	uint8_t i3c_ddr_wr_mode                                                        : 1;
	uint8_t resv_1                                                                 : 3;
} intf_config7_t;

#define OTP_CONFIG                                                              0x2b
typedef struct {
	uint8_t resv_2                                                                 : 4;
	uint8_t otp_copy_mode                                                          : 2;
	uint8_t resv_1                                                                 : 2;
} otp_config_t;

#define INT_SOURCE6                                                             0x2f
typedef struct {
	uint8_t ff_int1_en                                                             : 1;
	uint8_t lowg_int1_en                                                           : 1;
	uint8_t highg_int1_en                                                          : 1;
	uint8_t tap_int1_en                                                            : 1;
	uint8_t tilt_det_int1_en                                                       : 1;
	uint8_t resv_1                                                                 : 3;
} int_source6_t;

#define INT_SOURCE7                                                             0x30
typedef struct {
	uint8_t ff_int2_en                                                             : 1;
	uint8_t lowg_int2_en                                                           : 1;
	uint8_t highg_int2_en                                                          : 1;
	uint8_t tap_int2_en                                                            : 1;
	uint8_t tilt_det_int2_en                                                       : 1;
	uint8_t resv_1                                                                 : 3;
} int_source7_t;

#define INT_SOURCE8                                                             0x31
typedef struct {
	uint8_t resv_1                                                                 : 1;
	uint8_t ois_drdy_ibi_en                                                        : 1;
	uint8_t fsync_ibi_en                                                           : 1;
	uint8_t pll_rdy_ibi_en                                                         : 1;
	uint8_t ui_drdy_ibi_en                                                         : 1;
	uint8_t fifo_ths_ibi_en                                                        : 1;
	uint8_t fifo_full_ibi_en                                                       : 1;
	uint8_t agc_rdy_ibi_en                                                         : 1;
} int_source8_t;

#define INT_SOURCE9                                                             0x32
typedef struct {
	uint8_t i3c_protocol_error_ibi_en                                              : 1;
	uint8_t ff_ibi_en                                                              : 1;
	uint8_t lowg_ibi_en                                                            : 1;
	uint8_t ext_highg_ibi_en                                                       : 1;
	uint8_t wom_z_ibi_en                                                           : 1;
	uint8_t wom_y_ibi_en                                                           : 1;
	uint8_t wom_x_ibi_en                                                           : 1;
	uint8_t st_done_ibi_en                                                         : 1;
} int_source9_t;

#define INT_SOURCE10                                                            0x33
typedef struct {
	uint8_t resv_2                                                                 : 2;
	uint8_t highg_ibi_en                                                           : 1;
	uint8_t tap_ibi_en                                                             : 1;
	uint8_t tilt_det_ibi_en                                                        : 1;
	uint8_t resv_1                                                                 : 3;
} int_source10_t;

#define APEX_CONFIG2                                                            0x46
typedef struct {
	uint8_t tap_max                                                                : 2;
	uint8_t tap_min                                                                : 2;
	uint8_t dmp_power_save_time_sel                                                : 4;
} apex_config2_t;

#define APEX_CONFIG3                                                            0x47
typedef struct {
	uint8_t tap_tavg                                                               : 2;
	uint8_t tap_tmax                                                               : 3;
	uint8_t tap_tmin                                                               : 3;
} apex_config3_t;

#define APEX_CONFIG4                                                            0x48
typedef struct {
	uint8_t tap_max_peak_tol                                                       : 2;
	uint8_t tap_min_jerk_thr                                                       : 6;
} apex_config4_t;

#define APEX_CONFIG5                                                            0x49
typedef struct {
	uint8_t tilt_wait_time_sel                                                     : 2;
	uint8_t lowg_peak_th_hyst_sel                                                  : 3;
	uint8_t highg_peak_th_hyst_sel                                                 : 3;
} apex_config5_t;

#define APEX_CONFIG9                                                            0x4a
typedef struct {
	uint8_t ff_debounce_duration_sel                                               : 4;
	uint8_t resv_1                                                                 : 1;
	uint8_t tap_smudge_reject_thr                                                  : 3;
} apex_config9_t;

#define APEX_CONFIG10                                                           0x4b
typedef struct {
	uint8_t lowg_peak_th_sel                                                       : 5;
	uint8_t lowg_time_th_sel                                                       : 3;
} apex_config10_t;

#define APEX_CONFIG11                                                           0x4c
typedef struct {
	uint8_t highg_peak_th_sel                                                      : 5;
	uint8_t highg_time_th_sel                                                      : 3;
} apex_config11_t;

#define ACCEL_WOM_X_THR                                                         0x4d
typedef struct {
	uint8_t wom_x_th                                                               : 8;
} accel_wom_x_thr_t;

#define ACCEL_WOM_Y_THR                                                         0x4e
typedef struct {
	uint8_t wom_y_th                                                               : 8;
} accel_wom_y_thr_t;

#define ACCEL_WOM_Z_THR                                                         0x4f
typedef struct {
	uint8_t wom_z_th                                                               : 8;
} accel_wom_z_thr_t;

#define OFFSET_USER0                                                            0x50
#define OFFSET_USER1                                                            0x51
#define OFFSET_USER2                                                            0x52
#define OFFSET_USER3                                                            0x53
#define OFFSET_USER4                                                            0x54
#define OFFSET_USER5                                                            0x55
#define OFFSET_USER6                                                            0x56
#define OFFSET_USER7                                                            0x57
#define OFFSET_USER8                                                            0x58
#define ST_STATUS1                                                              0x66
typedef struct {
	uint8_t resv_2                                                                 : 2;
	uint8_t accel_st_pass                                                          : 1;
	uint8_t accel_st_done                                                          : 1;
	uint8_t az_st_pass                                                             : 1;
	uint8_t ay_st_pass                                                             : 1;
	uint8_t ax_st_pass                                                             : 1;
	uint8_t resv_1                                                                 : 1;
} st_status1_t;

#define ST_STATUS2                                                              0x67
typedef struct {
	uint8_t resv_2                                                                 : 1;
	uint8_t st_incomplete                                                          : 1;
	uint8_t gyro_st_pass                                                           : 1;
	uint8_t gyro_st_done                                                           : 1;
	uint8_t gz_st_pass                                                             : 1;
	uint8_t gy_st_pass                                                             : 1;
	uint8_t gx_st_pass                                                             : 1;
	uint8_t resv_1                                                                 : 1;
} st_status2_t;

#define FDR_CONFIG                                                              0x69
typedef struct {
	uint8_t fdr_sel                                                                : 8;
} fdr_config_t;

#define APEX_CONFIG12                                                           0x6a
typedef struct {
	uint8_t ff_max_duration_sel                                                    : 4;
	uint8_t ff_min_duration_sel                                                    : 4;
} apex_config12_t;

#define INTF_CONFIG13                                                           0x6c
typedef struct {
	uint8_t resv_2                                                                 : 3;
	uint8_t i3c_aux_timeout_en                                                     : 1;
	uint8_t resv_1                                                                 : 4;
} intf_config13_t;

#define PADS_CONFIG11                                                           0x77
typedef struct {
	uint8_t resv_2                                                                 : 2;
	uint8_t pads_int1_pe_trim_d2a                                                  : 1;
	uint8_t pads_int1_pud_trim_d2a                                                 : 1;
	uint8_t resv_1                                                                 : 2;
	uint8_t pads_aux_sdi_pe_trim_d2a                                               : 1;
	uint8_t pads_aux_sdi_pud_trim_d2a                                              : 1;
} pads_config11_t;

#define APEX_CONFIG14                                                           0x79
typedef struct {
	uint8_t ext_highg_time_th_sel                                                  : 3;
	uint8_t ext_highg_peak_th_hyst_sel                                             : 3;
	uint8_t ext_highg_peak_th_sel                                                  : 2;
} apex_config14_t;


/* MREG3 */
#define XA_ST_DATA                                                              0x5000
typedef struct {
	uint8_t xa_st_data                                                             : 8;
} xa_st_data_t;

#define YA_ST_DATA                                                              0x5001
typedef struct {
	uint8_t ya_st_data                                                             : 8;
} ya_st_data_t;

#define ZA_ST_DATA                                                              0x5002
typedef struct {
	uint8_t za_st_data                                                             : 8;
} za_st_data_t;

#define XG_ST_DATA                                                              0x5003
typedef struct {
	uint8_t xg_st_data                                                             : 8;
} xg_st_data_t;

#define YG_ST_DATA                                                              0x5004
typedef struct {
	uint8_t yg_st_data                                                             : 8;
} yg_st_data_t;

#define ZG_ST_DATA                                                              0x5005
typedef struct {
	uint8_t zg_st_data                                                             : 8;
} zg_st_data_t;

/* MREG2 */
#define OTP_CTRL7                                                               0x2806
typedef struct {
	uint8_t resv_3                                                                 : 4;
	uint8_t otp_reload                                                             : 1;
	uint8_t resv_2                                                                 : 1;
	uint8_t otp_pwr_down                                                           : 1;
	uint8_t resv_1                                                                 : 1;
} otp_ctrl7_t;


/* MREG_FPGA */

/* ROM */


#ifdef __cplusplus
}
#endif

#endif  /*#ifndef _INV_IMU_REGMAP_BE_H_*/
