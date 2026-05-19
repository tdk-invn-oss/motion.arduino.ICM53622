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

#include "imu/inv_imu_driver.h"
#include "imu/inv_imu_version.h"

#include <stddef.h> /* NULL */
#include <string.h> /* memset */

enum power_mode {
	POWER_MODE_SLEEP,
	POWER_MODE_LN,
	POWER_MODE_ALP_RC,
	POWER_MODE_ALP_WU
};

/* Static functions declaration */
static enum power_mode get_power_mode(inv_imu_device_t *s, pwr_mgmt0_t pwr_mgmt0);
static int             move_from_alp_rc_to_alp_wu(inv_imu_device_t *s);
static int             move_from_alp_wu_to_alp_rc(inv_imu_device_t *s);
static int             configure_serial_interface(inv_imu_device_t *s);
static int             inv_imu_set_config_gpio_int(inv_imu_device_t *s, const inv_imu_int_num_t num,
                                                   const inv_imu_int_state_t *it);
static int             inv_imu_get_config_gpio_int(inv_imu_device_t *s, const inv_imu_int_num_t num,
                                                   inv_imu_int_state_t *it);
static int             inv_imu_set_config_ibi(inv_imu_device_t *s, const inv_imu_int_state_t *it);
static int             inv_imu_get_config_ibi(inv_imu_device_t *s, inv_imu_int_state_t *it);


void inv_imu_sleep_us(inv_imu_device_t *s, uint32_t us)
{
	if (s->transport.sleep_us != NULL)
		s->transport.sleep_us(us);
}

int inv_imu_soft_reset(inv_imu_device_t *s)
{
	int                 status = INV_IMU_OK;
	int_status_t        int_status;
	signal_path_reset_t signal_path_reset = { 0 };
	fifo_config6_t      fifo_config6;
#if INV_IMU_INTERFACE == 1
	sensor_config3_t sensor_config3;
#endif

	if (s->transport.configure_host_interface != NULL)
		s->transport.configure_host_interface(s->transport.serif_type);

	/* Trigger soft reset */
	signal_path_reset.soft_reset_device_config = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, SIGNAL_PATH_RESET, 1, (uint8_t *)&signal_path_reset);

	/* Wait 1ms for soft reset to be effective */
	inv_imu_sleep_us(s, 1000);

	if (s->transport.configure_host_interface != NULL)
		s->transport.configure_host_interface(s->transport.serif_type);

	/* Re-configure serial interface since it was reset */
	status |= configure_serial_interface(s);

	/* Clear the reset done interrupt */
	status |= inv_imu_read_reg(s, INT_STATUS, 1, (uint8_t *)&int_status);
	if (!int_status.reset_done_int || int_status.agc_rdy_int || int_status.fifo_full_int ||
	    int_status.fifo_ths_int || int_status.fsync_int || int_status.fsync_ois_int ||
	    int_status.pll_rdy_int || int_status.st_int)
		status |= INV_IMU_ERROR;

	/* Set `endianness_data` variable */
	status |= inv_imu_get_endianness(s);

	/* Force register cache reload. */
	status |= inv_imu_init_transport(s);

	/* Mark DMP off to make sure driver status reflects actual device state. */
	s->dmp_is_on = 0;

	/* FIFO disabled by default */
	s->fifo_is_used    = 0;
	s->fifo_frame_size = 0;

	/*
	 * Disable the automatic RCOSC power on to avoid extra power consumption in sleep mode (all
	 * sensors and clocks off).
	 * Refer to FIFO_CONFIG6 description in datasheet for more details.
	 */
	status |= inv_imu_read_reg(s, FIFO_CONFIG6, 1, (uint8_t *)&fifo_config6);
	fifo_config6.rcosc_req_on_fifo_ths_dis = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, FIFO_CONFIG6, 1, (uint8_t *)&fifo_config6);

#if INV_IMU_INTERFACE == 1
	status |= inv_imu_read_reg(s, SENSOR_CONFIG3, 1, (uint8_t *)&sensor_config3);
	sensor_config3.ois_config_disable = INV_IMU_DISABLE;
	status |= inv_imu_write_reg(s, SENSOR_CONFIG3, 1, (uint8_t *)&sensor_config3);
#endif

	return status;
}

int inv_imu_get_who_am_i(inv_imu_device_t *s, uint8_t *who_am_i)
{
	return inv_imu_read_reg(s, WHO_AM_I, 1, who_am_i);
}

int inv_imu_set_accel_mode(inv_imu_device_t *s, pwr_mgmt0_accel_mode_t accel_mode)
{
	int         status = 0;
	pwr_mgmt0_t pwr_mgmt0;

	status |= inv_imu_get_endianness(s);

	/* Exit if current accel mode is the requested one. */
	status |= inv_imu_read_reg(s, PWR_MGMT0, 1, (uint8_t *)&pwr_mgmt0);
	if (status || (pwr_mgmt0.accel_mode == accel_mode))
		return status;

	/* If current power mode is ALP_WU, we should switch to ALP_RC first. */
	if (POWER_MODE_ALP_WU == get_power_mode(s, pwr_mgmt0))
		status |= move_from_alp_wu_to_alp_rc(s);

	/* Read PWR_MGMT0 again as move_from_alp_wu_to_alp_rc() writes it. */
	status |= inv_imu_read_reg(s, PWR_MGMT0, 1, (uint8_t *)&pwr_mgmt0);
	pwr_mgmt0.accel_mode = accel_mode;
	status |= inv_imu_write_reg(s, PWR_MGMT0, 1, (uint8_t *)&pwr_mgmt0);
	inv_imu_sleep_us(s, 200);

	/* If power mode is now ALP_RC, we can switch to ALP_WU to lower power consumption. */
	if (POWER_MODE_ALP_RC == get_power_mode(s, pwr_mgmt0))
		status |= move_from_alp_rc_to_alp_wu(s);

	return status;
}

int inv_imu_set_gyro_mode(inv_imu_device_t *s, pwr_mgmt0_gyro_mode_t gyro_mode)
{
	int         status = 0;
	pwr_mgmt0_t pwr_mgmt0;

	status |= inv_imu_get_endianness(s);

	/* Exit if current gyro mode is the requested one. */
	status |= inv_imu_read_reg(s, PWR_MGMT0, 1, (uint8_t *)&pwr_mgmt0);
	if (status || (pwr_mgmt0.gyro_mode == gyro_mode))
		return status;

	/* If current power mode is ALP_WU, we should switch to ALP_RC first. */
	if (POWER_MODE_ALP_WU == get_power_mode(s, pwr_mgmt0))
		status |= move_from_alp_wu_to_alp_rc(s);

	/* Read PWR_MGMT0 again as move_from_alp_wu_to_alp_rc() writes it. */
	status |= inv_imu_read_reg(s, PWR_MGMT0, 1, (uint8_t *)&pwr_mgmt0);
	pwr_mgmt0.gyro_mode = gyro_mode;
	status |= inv_imu_write_reg(s, PWR_MGMT0, 1, (uint8_t *)&pwr_mgmt0);
	inv_imu_sleep_us(s, 200);

	/* If power mode is now ALP_RC, we can switch to ALP_WU to lower power consumption. */
	if (POWER_MODE_ALP_RC == get_power_mode(s, pwr_mgmt0))
		move_from_alp_rc_to_alp_wu(s);

	return status;
}

int inv_imu_set_accel_frequency(inv_imu_device_t *s, const accel_config0_odr_t frequency)
{
	int             status = 0;
	accel_config0_t accel_config0;

	status |= inv_imu_read_reg(s, ACCEL_CONFIG0, 1, (uint8_t *)&accel_config0);
	accel_config0.accel_odr = frequency;
	status |= inv_imu_write_reg(s, ACCEL_CONFIG0, 1, (uint8_t *)&accel_config0);

	return status;
}

int inv_imu_set_gyro_frequency(inv_imu_device_t *s, const gyro_config0_odr_t frequency)
{
	int            status = 0;
	gyro_config0_t gyro_config0;

	status |= inv_imu_read_reg(s, GYRO_CONFIG0, 1, (uint8_t *)&gyro_config0);
	gyro_config0.gyro_odr = frequency;
	status |= inv_imu_write_reg(s, GYRO_CONFIG0, 1, (uint8_t *)&gyro_config0);

	return status;
}

int inv_imu_set_accel_fsr(inv_imu_device_t *s, accel_config0_fs_sel_t accel_fsr_g)
{
	int             status = 0;
	accel_config0_t accel_config0;

	status |= inv_imu_read_reg(s, ACCEL_CONFIG0, 1, (uint8_t *)&accel_config0);
	accel_config0.accel_ui_fs_sel = accel_fsr_g;
	status |= inv_imu_write_reg(s, ACCEL_CONFIG0, 1, (uint8_t *)&accel_config0);

	return status;
}

int inv_imu_set_gyro_fsr(inv_imu_device_t *s, gyro_config0_fs_sel_t gyro_fsr_dps)
{
	int            status = 0;
	gyro_config0_t gyro_config0;

	status |= inv_imu_read_reg(s, GYRO_CONFIG0, 1, (uint8_t *)&gyro_config0);
	gyro_config0.gyro_ui_fs_sel = gyro_fsr_dps;
	status |= inv_imu_write_reg(s, GYRO_CONFIG0, 1, (uint8_t *)&gyro_config0);

	return status;
}

int inv_imu_set_accel_lp_avg(inv_imu_device_t *s, accel_config1_accel_filt_avg_t acc_avg)
{
	int             status = 0;
	accel_config1_t accel_config1;

	status |= inv_imu_read_reg(s, ACCEL_CONFIG1, 1, (uint8_t *)&accel_config1);
	accel_config1.accel_ui_avg = acc_avg;
	status |= inv_imu_write_reg(s, ACCEL_CONFIG1, 1, (uint8_t *)&accel_config1);

	return status;
}

int inv_imu_set_accel_ln_bw(inv_imu_device_t *s, accel_config1_accel_filt_bw_t bw)
{
	int             status = 0;
	accel_config1_t accel_config1;

	status |= inv_imu_read_reg(s, ACCEL_CONFIG1, 1, (uint8_t *)&accel_config1);
	accel_config1.accel_ui_filt_bw = bw;
	status |= inv_imu_write_reg(s, ACCEL_CONFIG1, 1, (uint8_t *)&accel_config1);

	return status;
}

int inv_imu_set_gyro_ln_bw(inv_imu_device_t *s, gyro_config1_gyro_filt_bw_t bw)
{
	int            status = 0;
	gyro_config1_t gyro_config1;

	status |= inv_imu_read_reg(s, GYRO_CONFIG1, 1, (uint8_t *)&gyro_config1);
	gyro_config1.gyro_ui_filt_bw = bw;
	status |= inv_imu_write_reg(s, GYRO_CONFIG1, 1, (uint8_t *)&gyro_config1);

	return status;
}

#if INV_IMU_HIGH_ODR_SUPPORTED
int inv_imu_set_accel_ois_bw(inv_imu_device_t *s, ois_config3_accel_filt_bw_ind_t bw)
{
	int             status = 0;
	ois_config3_t   ois_config3;
	intf_config12_t intf_config12;

	status |= inv_imu_read_reg(s, INTF_CONFIG12, 1, (uint8_t *)&intf_config12);
	intf_config12.ois_on_ap_if = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, INTF_CONFIG12, 1, (uint8_t *)&intf_config12);

	status |= inv_imu_read_reg(s, OIS_CONFIG3, 1, (uint8_t *)&ois_config3);
	ois_config3.accel_ois_filt_bw_ind = bw;
	status |= inv_imu_write_reg(s, OIS_CONFIG3, 1, (uint8_t *)&ois_config3);

	status |= inv_imu_read_reg(s, INTF_CONFIG12, 1, (uint8_t *)&intf_config12);
	intf_config12.ois_on_ap_if = INV_IMU_DISABLE;
	status |= inv_imu_write_reg(s, INTF_CONFIG12, 1, (uint8_t *)&intf_config12);

	return status;
}

int inv_imu_get_accel_ois_bw(inv_imu_device_t *s, ois_config3_accel_filt_bw_ind_t *bw)
{
	int             status = 0;
	ois_config3_t   ois_config3;
	intf_config12_t intf_config12;

	status |= inv_imu_read_reg(s, INTF_CONFIG12, 1, (uint8_t *)&intf_config12);
	intf_config12.ois_on_ap_if = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, INTF_CONFIG12, 1, (uint8_t *)&intf_config12);

	status |= inv_imu_read_reg(s, OIS_CONFIG3, 1, (uint8_t *)&ois_config3);
	*bw = (ois_config3_accel_filt_bw_ind_t)ois_config3.accel_ois_filt_bw_ind;
	status |= inv_imu_write_reg(s, OIS_CONFIG3, 1, (uint8_t *)&ois_config3);

	status |= inv_imu_read_reg(s, INTF_CONFIG12, 1, (uint8_t *)&intf_config12);
	intf_config12.ois_on_ap_if = INV_IMU_DISABLE;
	status |= inv_imu_write_reg(s, INTF_CONFIG12, 1, (uint8_t *)&intf_config12);

	return status;
}

int inv_imu_set_gyro_ois_bw(inv_imu_device_t *s, ois_config3_gyro_filt_bw_ind_t bw)
{
	int             status = 0;
	ois_config3_t   ois_config3;
	intf_config12_t intf_config12;

	status |= inv_imu_read_reg(s, INTF_CONFIG12, 1, (uint8_t *)&intf_config12);
	intf_config12.ois_on_ap_if = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, INTF_CONFIG12, 1, (uint8_t *)&intf_config12);

	status |= inv_imu_read_reg(s, OIS_CONFIG3, 1, (uint8_t *)&ois_config3);
	ois_config3.gyro_ois_filt_bw_ind = bw;
	status |= inv_imu_write_reg(s, OIS_CONFIG3, 1, (uint8_t *)&ois_config3);

	status |= inv_imu_read_reg(s, INTF_CONFIG12, 1, (uint8_t *)&intf_config12);
	intf_config12.ois_on_ap_if = INV_IMU_DISABLE;
	status |= inv_imu_write_reg(s, INTF_CONFIG12, 1, (uint8_t *)&intf_config12);

	return status;
}

int inv_imu_get_gyro_ois_bw(inv_imu_device_t *s, ois_config3_gyro_filt_bw_ind_t *bw)
{
	int             status = 0;
	ois_config3_t   ois_config3;
	intf_config12_t intf_config12;

	status |= inv_imu_read_reg(s, INTF_CONFIG12, 1, (uint8_t *)&intf_config12);
	intf_config12.ois_on_ap_if = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, INTF_CONFIG12, 1, (uint8_t *)&intf_config12);

	status |= inv_imu_read_reg(s, OIS_CONFIG3, 1, (uint8_t *)&ois_config3);
	*bw = (ois_config3_gyro_filt_bw_ind_t)ois_config3.gyro_ois_filt_bw_ind;
	status |= inv_imu_write_reg(s, OIS_CONFIG3, 1, (uint8_t *)&ois_config3);

	status |= inv_imu_read_reg(s, INTF_CONFIG12, 1, (uint8_t *)&intf_config12);
	intf_config12.ois_on_ap_if = INV_IMU_DISABLE;
	status |= inv_imu_write_reg(s, INTF_CONFIG12, 1, (uint8_t *)&intf_config12);

	return status;
}
#endif // #if INV_IMU_HIGH_ODR_SUPPORTED

int inv_imu_get_register_data(inv_imu_device_t *s, inv_imu_sensor_data_t *data)
{
	int status = 0;

	status |= inv_imu_read_reg(s, TEMP_DATA1, sizeof(inv_imu_sensor_data_t), (uint8_t *)data);

	/* Format temperature data from sensor registers. */
	FORMAT_16_BITS_DATA(s->endianness_data, (uint8_t *)&data->temp_data,
	                    (uint16_t *)&data->temp_data);

	/* Format accel data from sensor registers. */
	FORMAT_16_BITS_DATA(s->endianness_data, (uint8_t *)&data->accel_data[0],
	                    (uint16_t *)&data->accel_data[0]);
	FORMAT_16_BITS_DATA(s->endianness_data, (uint8_t *)&data->accel_data[1],
	                    (uint16_t *)&data->accel_data[1]);
	FORMAT_16_BITS_DATA(s->endianness_data, (uint8_t *)&data->accel_data[2],
	                    (uint16_t *)&data->accel_data[2]);

	/* Format gyro data from sensor registers. */
	FORMAT_16_BITS_DATA(s->endianness_data, (uint8_t *)&data->gyro_data[0],
	                    (uint16_t *)&data->gyro_data[0]);
	FORMAT_16_BITS_DATA(s->endianness_data, (uint8_t *)&data->gyro_data[1],
	                    (uint16_t *)&data->gyro_data[1]);
	FORMAT_16_BITS_DATA(s->endianness_data, (uint8_t *)&data->gyro_data[2],
	                    (uint16_t *)&data->gyro_data[2]);

	return status;
}

int inv_imu_set_fifo_config(inv_imu_device_t *s, const inv_imu_fifo_config_t *fifo_config)
{
	int            status = 0;
	fifo_config1_t fifo_config1;
	fifo_config5_t fifo_config5;
	intf_config0_t intf_config0;
	tmst_config1_t tmst_config1;
	uint8_t        wm[2];
	uint8_t        fifo_frame_size = 0;

	inv_imu_switch_on_mclk(s);

	/* Disable FIFO to safely apply configuration */
	status |= inv_imu_read_reg(s, FIFO_CONFIG1, 1, (uint8_t *)&fifo_config1);
	fifo_config1.fifo_bypass = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, FIFO_CONFIG1, 1, (uint8_t *)&fifo_config1);

	/* Set FIFO record mode i.e FIFO count unit is packet, stored in little-endian. */
	status |= inv_imu_read_reg(s, INTF_CONFIG0, 1, (uint8_t *)&intf_config0);
	intf_config0.fifo_count_format = INTF_CONFIG0_FIFO_COUNT_REC_RECORD;
	intf_config0.fifo_count_endian = INTF_CONFIG0_FIFO_COUNT_LITTLE_ENDIAN;
	status |= inv_imu_write_reg(s, INTF_CONFIG0, 1, (uint8_t *)&intf_config0);

	/* Enable timestamping with default 1us resolution. */
	status |= inv_imu_read_reg(s, TMST_CONFIG1, 1, (uint8_t *)&tmst_config1);
	tmst_config1.tmst_en  = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, TMST_CONFIG1, 1, (uint8_t *)&tmst_config1);

	/* Set WM */
	wm[0] = (uint8_t)fifo_config->fifo_wm_th;
	wm[1] = (uint8_t)(fifo_config->fifo_wm_th >> 8);
	status |= inv_imu_write_reg(s, FIFO_CONFIG2, sizeof(wm), &wm[0]);

	status |= inv_imu_read_reg(s, FIFO_CONFIG5, 1, (uint8_t *)&fifo_config5);
	/* Set fifo WM triggering condition: interrupts when equal or greater than threshold */
	fifo_config5.fifo_wm_gt_th = INV_IMU_ENABLE;

	/* Set which sensors go to FIFO */
	fifo_config5.fifo_accel_en = fifo_config->accel_en;
	fifo_config5.fifo_gyro_en  = fifo_config->gyro_en;
	fifo_config5.fifo_hires_en = fifo_config->hires_en;
	status |= inv_imu_write_reg(s, FIFO_CONFIG5, 1, (uint8_t *)&fifo_config5);

	/* Set FIFO mode and bypass. */
	status |= inv_imu_read_reg(s, FIFO_CONFIG1, 1, (uint8_t *)&fifo_config1);
	fifo_config1.fifo_mode   = fifo_config->fifo_mode;
	fifo_config1.fifo_bypass = !fifo_config->fifo_en; /* FIFO enabled means bypass is off). */
	status |= inv_imu_write_reg(s, FIFO_CONFIG1, 1, (uint8_t *)&fifo_config1);

	status |= inv_imu_switch_off_mclk(s);

	/* Calculate FIFO frame size */
	if (fifo_config->hires_en) {
		fifo_frame_size = 20;
	} else {
		if (fifo_config->accel_en)
			fifo_frame_size += 8;
		if (fifo_config->gyro_en)
			fifo_frame_size += 8;
	}
	s->fifo_frame_size = fifo_frame_size;

	s->fifo_is_used = fifo_config->fifo_en;

	return status;
}

int inv_imu_get_fifo_config(inv_imu_device_t *s, inv_imu_fifo_config_t *fifo_config)
{
	int            status = 0;
	fifo_config1_t fifo_config1;
	fifo_config5_t fifo_config5;
	uint8_t        wm[2];

	status |= inv_imu_read_reg(s, FIFO_CONFIG2, sizeof(wm), &wm[0]);
	fifo_config->fifo_wm_th = (uint16_t)((uint16_t)wm[1] << 8) | wm[0];

	status |= inv_imu_read_reg(s, FIFO_CONFIG5, 1, (uint8_t *)&fifo_config5);
	fifo_config->accel_en = fifo_config5.fifo_accel_en;
	fifo_config->gyro_en  = fifo_config5.fifo_gyro_en;
	fifo_config->hires_en = fifo_config5.fifo_hires_en;

	status |= inv_imu_read_reg(s, FIFO_CONFIG1, 1, (uint8_t *)&fifo_config1);
	fifo_config->fifo_mode = (fifo_config1_fifo_mode_t)fifo_config1.fifo_mode;
	fifo_config->fifo_en   = !fifo_config1.fifo_bypass;

	return status;
}

int inv_imu_flush_fifo(inv_imu_device_t *s)
{
	int                 status            = 0;
	signal_path_reset_t signal_path_reset = { 0 };

	signal_path_reset.fifo_flush = INV_IMU_ENABLE;

	status |= inv_imu_switch_on_mclk(s);

	status |= inv_imu_write_reg(s, SIGNAL_PATH_RESET, 1, (uint8_t *)&signal_path_reset);
	inv_imu_sleep_us(s, 10);

	/* Wait for FIFO flush (idle bit will go high at appropriate time and unlock flush) */
	while ((status == 0) && signal_path_reset.fifo_flush) {
		status |= inv_imu_read_reg(s, SIGNAL_PATH_RESET, 1, (uint8_t *)&signal_path_reset);
	}

	status |= inv_imu_switch_off_mclk(s);

	return status;
}

int inv_imu_get_frame_count(inv_imu_device_t *s, uint16_t *frame_count)
{
	int status = 0;

	status |= inv_imu_read_reg(s, FIFO_COUNTH, 2, (uint8_t *)frame_count);
	FORMAT_16_BITS_DATA(INTF_CONFIG0_FIFO_COUNT_LITTLE_ENDIAN, (uint8_t *)frame_count, frame_count);

	return status;
}

int inv_imu_get_fifo_frame(inv_imu_device_t *s, inv_imu_fifo_data_t *data)
{
	int     status = INV_IMU_OK;
	uint8_t frame[20];
	int16_t reg16[3];

	status |= inv_imu_read_reg(s, FIFO_DATA, s->fifo_frame_size, frame);

	data->header.Byte = frame[0];

	switch (s->fifo_frame_size) {
	case 8:
		FORMAT_16_BITS_DATA(s->endianness_data, &frame[1], &data->byte_8.sensor_data[0]);
		FORMAT_16_BITS_DATA(s->endianness_data, &frame[3], &data->byte_8.sensor_data[1]);
		FORMAT_16_BITS_DATA(s->endianness_data, &frame[5], &data->byte_8.sensor_data[2]);
		data->byte_8.temp_data = frame[7]; // Temp data is only one byte
		break;
	case 16:
		FORMAT_16_BITS_DATA(s->endianness_data, &frame[1], &data->byte_16.accel_data[0]);
		FORMAT_16_BITS_DATA(s->endianness_data, &frame[3], &data->byte_16.accel_data[1]);
		FORMAT_16_BITS_DATA(s->endianness_data, &frame[5], &data->byte_16.accel_data[2]);
		FORMAT_16_BITS_DATA(s->endianness_data, &frame[7], &data->byte_16.gyro_data[0]);
		FORMAT_16_BITS_DATA(s->endianness_data, &frame[9], &data->byte_16.gyro_data[1]);
		FORMAT_16_BITS_DATA(s->endianness_data, &frame[11], &data->byte_16.gyro_data[2]);
		data->byte_16.temp_data = frame[13]; // Temp data is only one byte
		FORMAT_16_BITS_DATA(s->endianness_data, &frame[14], &data->byte_16.timestamp);
		break;
	case 20:
		FORMAT_16_BITS_DATA(s->endianness_data, &frame[1], &reg16[0]);
		FORMAT_16_BITS_DATA(s->endianness_data, &frame[3], &reg16[1]);
		FORMAT_16_BITS_DATA(s->endianness_data, &frame[5], &reg16[2]);
		data->byte_20.accel_data[0] = (reg16[0] << 4) | (frame[17] >> 4);
		data->byte_20.accel_data[1] = (reg16[1] << 4) | (frame[18] >> 4);
		data->byte_20.accel_data[2] = (reg16[2] << 4) | (frame[19] >> 4);
		FORMAT_16_BITS_DATA(s->endianness_data, &frame[7], &reg16[0]);
		FORMAT_16_BITS_DATA(s->endianness_data, &frame[9], &reg16[1]);
		FORMAT_16_BITS_DATA(s->endianness_data, &frame[11], &reg16[2]);
		data->byte_20.gyro_data[0] = (reg16[0] << 4) | (frame[17] & 0x0F);
		data->byte_20.gyro_data[1] = (reg16[1] << 4) | (frame[18] & 0x0F);
		data->byte_20.gyro_data[2] = (reg16[2] << 4) | (frame[19] & 0x0F);
		FORMAT_16_BITS_DATA(s->endianness_data, &frame[13], &data->byte_20.temp_data);
		FORMAT_16_BITS_DATA(s->endianness_data, &frame[15], &data->byte_20.timestamp);
		break;
	default:
		data->header.Byte = 0;
		return INV_IMU_ERROR;
	}

	return status;
}

#if INV_IMU_HIGH_ODR_SUPPORTED
int inv_imu_set_accel_ois_mode(inv_imu_device_t *s, pwr_mgmt0_accel_ois_mode_t mode)
{
	int         status = 0;
	pwr_mgmt0_t pwr_mgmt0;

	status |= inv_imu_read_reg(s, PWR_MGMT0, 1, (uint8_t *)&pwr_mgmt0);
	if (status || (pwr_mgmt0.accel_ois_en_ap == mode))
		return status;

	/* If current power mode is ALP_WU, we should switch to ALP_RC first. */
	if (POWER_MODE_ALP_WU == get_power_mode(s, pwr_mgmt0))
		move_from_alp_wu_to_alp_rc(s);

	/* Read PWR_MGMT0 again as move_from_alp_wu_to_alp_rc() writes it. */
	status |= inv_imu_read_reg(s, PWR_MGMT0, 1, (uint8_t *)&pwr_mgmt0);
	pwr_mgmt0.accel_ois_en_ap = mode;
	status |= inv_imu_write_reg(s, PWR_MGMT0, 1, (uint8_t *)&pwr_mgmt0);
	inv_imu_sleep_us(s, 200);

	/* If power mode is now ALP_RC, we can switch to ALP_WU to lower power consumption. */
	if (POWER_MODE_ALP_RC == get_power_mode(s, pwr_mgmt0))
		move_from_alp_rc_to_alp_wu(s);

	return status;
}

int inv_imu_set_gyro_ois_mode(inv_imu_device_t *s, pwr_mgmt0_gyro_ois_mode_t mode)
{
	int         status = 0;
	pwr_mgmt0_t pwr_mgmt0;

	status |= inv_imu_read_reg(s, PWR_MGMT0, 1, (uint8_t *)&pwr_mgmt0);
	if (status || (pwr_mgmt0.gyro_ois_en_ap == INV_IMU_ENABLE))
		return status;

	if (POWER_MODE_ALP_WU == get_power_mode(s, pwr_mgmt0))
		move_from_alp_wu_to_alp_rc(s);

	/* Read PWR_MGMT0 again as move_from_alp_wu_to_alp_rc() writes it. */
	status |= inv_imu_read_reg(s, PWR_MGMT0, 1, (uint8_t *)&pwr_mgmt0);
	pwr_mgmt0.gyro_ois_en_ap = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, PWR_MGMT0, 1, (uint8_t *)&pwr_mgmt0);
	inv_imu_sleep_us(s, 200);

	/* If power mode is now ALP_RC, we can switch to ALP_WU to lower power consumption. */
	if (POWER_MODE_ALP_RC == get_power_mode(s, pwr_mgmt0))
		move_from_alp_rc_to_alp_wu(s);

	return status;
}

int inv_imu_get_ois_register_data(inv_imu_device_t *s, inv_imu_ois_sensor_data_t *data)
{
	int status = 0;
	struct {
		uint8_t temp[2];
		uint8_t accel[6];
		uint8_t gyro[6];
		uint8_t tmst[2];
		uint8_t ext_dat[3];
	} regs;

	status |= inv_imu_read_reg(s, TEMP_DATA1_OIS, sizeof(regs), (uint8_t *)&regs);

	/* Format temperature data from sensor registers. */
	FORMAT_16_BITS_DATA(s->endianness_data, &regs.temp[0], (uint16_t *)&data->temp_data);

	/* Format accel data from sensor registers. */
	FORMAT_16_BITS_DATA(s->endianness_data, &regs.accel[0], (uint16_t *)&data->accel_data[0]);
	FORMAT_16_BITS_DATA(s->endianness_data, &regs.accel[2], (uint16_t *)&data->accel_data[1]);
	FORMAT_16_BITS_DATA(s->endianness_data, &regs.accel[4], (uint16_t *)&data->accel_data[2]);

	/* Format gyro data from sensor registers. */
	FORMAT_16_BITS_DATA(s->endianness_data, &regs.gyro[0], (uint16_t *)&data->gyro_data[0]);
	FORMAT_16_BITS_DATA(s->endianness_data, &regs.gyro[2], (uint16_t *)&data->gyro_data[1]);
	FORMAT_16_BITS_DATA(s->endianness_data, &regs.gyro[4], (uint16_t *)&data->gyro_data[2]);

	/* Format high-resolution portion of the sensor data. */
	data->accel_high_res[0] = (regs.ext_dat[0] >> 4) & 0xF;
	data->accel_high_res[1] = (regs.ext_dat[1] >> 4) & 0xF;
	data->accel_high_res[2] = (regs.ext_dat[2] >> 4) & 0xF;
	data->gyro_high_res[0]  = (regs.ext_dat[0]) & 0xF;
	data->gyro_high_res[1]  = (regs.ext_dat[1]) & 0xF;
	data->gyro_high_res[2]  = (regs.ext_dat[2]) & 0xF;
	return status;
}
#endif

int inv_imu_set_config_int(inv_imu_device_t *s, const inv_imu_int_num_t num,
                           const inv_imu_int_state_t *it)
{
	if (s->transport.serif_type == UI_I3C_IBI)
		return inv_imu_set_config_ibi(s, it);
	else
		return inv_imu_set_config_gpio_int(s, num, it);
}

static int inv_imu_set_config_gpio_int(inv_imu_device_t *s, const inv_imu_int_num_t num,
                                       const inv_imu_int_state_t *it)
{
	int      status = 0;
	uint32_t reg_0, reg_1;

	/** Required registers to configure interrupts.
	 *  INT1 and INT2 share the same bit location in different register sets:
	 *   - INT1: INT_SOURCE0, INT_SOURCE1, INT_SOURCE6
	 *   - INT2: INT_SOURCE3, INT_SOURCE4, INT_SOURCE7
 	 */
	struct {
		int_source0_t src0;
		int_source1_t src1;
	} int_src;
	int_source6_t int_source6;

	switch (num) {
	case INV_IMU_INT1:
		reg_0 = INT_SOURCE0;
		reg_1 = INT_SOURCE6;
		break;
	case INV_IMU_INT2:
		reg_0 = INT_SOURCE3;
		reg_1 = INT_SOURCE7;
		break;
	default:
		return INV_IMU_ERROR_BAD_ARG;
	}

	status |= inv_imu_read_reg(s, reg_0, sizeof(int_src), (uint8_t *)&int_src);

	int_src.src0.fsync_int1_en     = (it->INV_UI_FSYNC != 0);
	int_src.src0.drdy_int1_en      = (it->INV_UI_DRDY != 0);
	int_src.src0.fifo_ths_int1_en  = (it->INV_FIFO_THS != 0);
	int_src.src0.fifo_full_int1_en = (it->INV_FIFO_FULL != 0);

	int_src.src1.wom_x_int1_en     = (it->INV_WOM_X != 0);
	int_src.src1.wom_y_int1_en     = (it->INV_WOM_Y != 0);
	int_src.src1.wom_z_int1_en     = (it->INV_WOM_Z != 0);
	int_src.src1.ext_highg_int1_en = (it->INV_EXT_HIGHG != 0);
	int_src.src1.drdy_ois_int1_en  = (it->INV_AP_OIS_DRDY != 0);

	status |= inv_imu_write_reg(s, reg_0, sizeof(int_src), (uint8_t *)&int_src);

	status |= inv_imu_read_reg(s, reg_1, 1, (uint8_t *)&int_source6);

	int_source6.ff_int1_en       = (it->INV_FF != 0);
	int_source6.lowg_int1_en     = (it->INV_LOWG != 0);
	int_source6.highg_int1_en    = (it->INV_HIGHG != 0);
	int_source6.tap_int1_en      = (it->INV_TAP != 0);
	int_source6.tilt_det_int1_en = (it->INV_TILT_DET != 0);

	status |= inv_imu_write_reg(s, reg_1, 1, (uint8_t *)&int_source6);

	return status;
}

static int inv_imu_set_config_ibi(inv_imu_device_t *s, const inv_imu_int_state_t *it)
{
	int      status = 0;
	struct {
		int_source8_t int_src8;
		int_source9_t int_src9;
		int_source10_t int_src10;
	} regs;

	status |= inv_imu_read_reg(s, INT_SOURCE8, sizeof(regs), (uint8_t *)&regs);

	regs.int_src8.ois_drdy_ibi_en  = (it->INV_AP_OIS_DRDY != 0);
	regs.int_src8.fsync_ibi_en     = (it->INV_UI_FSYNC != 0);
	regs.int_src8.ui_drdy_ibi_en   = (it->INV_UI_DRDY != 0);
	regs.int_src8.fifo_ths_ibi_en  = (it->INV_FIFO_THS != 0);
	regs.int_src8.fifo_full_ibi_en = (it->INV_FIFO_FULL != 0);

	regs.int_src9.ff_ibi_en        = (it->INV_FF != 0);
	regs.int_src9.lowg_ibi_en      = (it->INV_LOWG != 0);
	regs.int_src9.ext_highg_ibi_en = (it->INV_EXT_HIGHG != 0);
	regs.int_src9.wom_z_ibi_en     = (it->INV_WOM_X != 0);
	regs.int_src9.wom_y_ibi_en     = (it->INV_WOM_Y != 0);
	regs.int_src9.wom_x_ibi_en     = (it->INV_WOM_Z != 0);

	regs.int_src10.highg_ibi_en    = (it->INV_HIGHG != 0);
	regs.int_src10.tap_ibi_en      = (it->INV_TAP != 0);
	regs.int_src10.tilt_det_ibi_en = (it->INV_TILT_DET != 0);

	status |= inv_imu_write_reg(s, INT_SOURCE8, sizeof(regs), (uint8_t *)&regs);

	return status;
}

int inv_imu_get_config_int(inv_imu_device_t *s, const inv_imu_int_num_t num,
                           inv_imu_int_state_t *it)
{
	if (s->transport.serif_type == UI_I3C_IBI)
		return inv_imu_get_config_ibi(s, it);
	else
		return inv_imu_get_config_gpio_int(s, num, it);
}

static int inv_imu_get_config_gpio_int(inv_imu_device_t *s, const inv_imu_int_num_t num,
                                       inv_imu_int_state_t *it)
{
	int      status = 0;
	uint32_t reg_0, reg_1;

	/** Required registers to configure interrupts.
	 *  INT1 and INT2 share the same bit location in different register sets:
	 *   - INT1: INT_SOURCE0, INT_SOURCE1, INT_SOURCE6
	 *   - INT2: INT_SOURCE3, INT_SOURCE4, INT_SOURCE7
 	 */
	struct {
		int_source0_t src0;
		int_source1_t src1;
	} int_src;
	int_source6_t int_source6;

	switch (num) {
	case INV_IMU_INT1:
		reg_0 = INT_SOURCE0;
		reg_1 = INT_SOURCE6;
		break;
	case INV_IMU_INT2:
		reg_0 = INT_SOURCE3;
		reg_1 = INT_SOURCE7;
		break;
	default:
		return INV_IMU_ERROR_BAD_ARG;
	}

	status |= inv_imu_read_reg(s, reg_0, sizeof(int_src), (uint8_t *)&int_src);
	it->INV_UI_FSYNC  = int_src.src0.fsync_int1_en;
	it->INV_UI_DRDY   = int_src.src0.drdy_int1_en;
	it->INV_FIFO_THS  = int_src.src0.fifo_ths_int1_en;
	it->INV_FIFO_FULL = int_src.src0.fifo_full_int1_en;

	it->INV_WOM_X       = int_src.src1.wom_x_int1_en;
	it->INV_WOM_Y       = int_src.src1.wom_y_int1_en;
	it->INV_WOM_Z       = int_src.src1.wom_z_int1_en;
	it->INV_EXT_HIGHG   = int_src.src1.ext_highg_int1_en;
	it->INV_AP_OIS_DRDY = int_src.src1.drdy_ois_int1_en;

	status |= inv_imu_read_reg(s, reg_1, 1, (uint8_t *)&int_source6);
	it->INV_FF       = int_source6.ff_int1_en;
	it->INV_LOWG     = int_source6.lowg_int1_en;
	it->INV_HIGHG    = int_source6.highg_int1_en;
	it->INV_TAP      = int_source6.tap_int1_en;
	it->INV_TILT_DET = int_source6.tilt_det_int1_en;

	return status;
}

static int inv_imu_get_config_ibi(inv_imu_device_t *s, inv_imu_int_state_t *it)
{
	int      status = 0;
	struct {
		int_source8_t int_src8;
		int_source9_t int_src9;
		int_source10_t int_src10;
	} regs;

	status |= inv_imu_read_reg(s, INT_SOURCE8, sizeof(regs), (uint8_t *)&regs);

	it->INV_AP_OIS_DRDY = regs.int_src8.ois_drdy_ibi_en;
	it->INV_UI_FSYNC    = regs.int_src8.fsync_ibi_en;
	it->INV_UI_DRDY     = regs.int_src8.ui_drdy_ibi_en;
	it->INV_FIFO_THS    = regs.int_src8.fifo_ths_ibi_en;
	it->INV_FIFO_FULL   = regs.int_src8.fifo_full_ibi_en;

	it->INV_FF        = regs.int_src9.ff_ibi_en;
	it->INV_LOWG      = regs.int_src9.lowg_ibi_en;
	it->INV_EXT_HIGHG = regs.int_src9.ext_highg_ibi_en;
	it->INV_WOM_X     = regs.int_src9.wom_z_ibi_en;
	it->INV_WOM_Y     = regs.int_src9.wom_y_ibi_en;
	it->INV_WOM_Z     = regs.int_src9.wom_x_ibi_en;

	it->INV_HIGHG    = regs.int_src10.highg_ibi_en;
	it->INV_TAP      = regs.int_src10.tap_ibi_en;
	it->INV_TILT_DET = regs.int_src10.tilt_det_ibi_en;

	return status;
}

int inv_imu_set_pin_config_int(inv_imu_device_t *s, const inv_imu_int_num_t num,
                               const inv_imu_int_pin_config_t *conf)
{
	int          status = 0;
	int_config_t int_config;

	/* If I3C IBI is used, there is no interrupt pin to configure. Just return. */
	if (s->transport.serif_type == UI_I3C_IBI)
		return status;

	status |= inv_imu_read_reg(s, INT_CONFIG, 1, (uint8_t *)&int_config);

	switch (num) {
	case INV_IMU_INT1:
		int_config.int1_polarity      = conf->int_polarity;
		int_config.int1_mode          = conf->int_mode;
		int_config.int1_drive_circuit = conf->int_drive;
		break;
	case INV_IMU_INT2:
		int_config.int2_polarity      = conf->int_polarity;
		int_config.int2_mode          = conf->int_mode;
		int_config.int2_drive_circuit = conf->int_drive;
		break;
	default:
		return INV_IMU_ERROR_BAD_ARG;
	}

	status |= inv_imu_write_reg(s, INT_CONFIG, 1, (uint8_t *)&int_config);

	return status;
}

int inv_imu_get_int_status(inv_imu_device_t *s, const inv_imu_int_num_t num,
                           inv_imu_int_state_t *it)
{
	int status = 0;

	(void)num;

	/**
	 * All interrupts (INT1, INT2 and IBI) share the same register sets for interrupt status:
	 *   - INT_STATUS_DRDY, INT_STATUS, INT_STATUS2, INT_STATUS3
 	 */
	struct {
		int_status_drdy_t int_status_drdy;
		int_status_t      int_status;
		int_status2_t     int_status2;
		int_status3_t     int_status3;
	} regs;

	status |= inv_imu_read_reg(s, INT_STATUS_DRDY, sizeof(regs), (uint8_t *)&regs);
	it->INV_UI_DRDY     = regs.int_status_drdy.data_rdy_int;
	it->INV_AP_OIS_DRDY = regs.int_status_drdy.data_rdy_ois_int;

	it->INV_UI_FSYNC  = regs.int_status.fsync_int;
	it->INV_FIFO_THS  = regs.int_status.fifo_ths_int;
	it->INV_FIFO_FULL = regs.int_status.fifo_full_int;

	it->INV_WOM_X = regs.int_status2.wom_x_int;
	it->INV_WOM_Y = regs.int_status2.wom_y_int;
	it->INV_WOM_Z = regs.int_status2.wom_z_int;

	it->INV_EXT_HIGHG = regs.int_status3.ext_highg_det_int;
	it->INV_FF        = regs.int_status3.ff_det_int;
	it->INV_LOWG      = regs.int_status3.lowg_det_int;
	it->INV_HIGHG     = regs.int_status3.highg_det_int;
	it->INV_TAP       = regs.int_status3.tap_det_int;
	it->INV_TILT_DET  = regs.int_status3.tilt_det_int;

	return status;
}

int inv_imu_get_endianness(inv_imu_device_t *s)
{
	int            status = 0;
	intf_config0_t intf_config0;

	status |= inv_imu_read_reg(s, INTF_CONFIG0, 1, (uint8_t *)&intf_config0);
	if (!status)
		s->endianness_data = intf_config0.sensor_data_endian;

	return status;
}

uint32_t inv_imu_convert_odr_bitfield_to_us(uint32_t odr_bitfield)
{
	switch (odr_bitfield) {
	case ACCEL_CONFIG0_ODR_1600_HZ:
		return 625;
	case ACCEL_CONFIG0_ODR_800_HZ:
		return 1250;
	case ACCEL_CONFIG0_ODR_400_HZ:
		return 2500;
	case ACCEL_CONFIG0_ODR_200_HZ:
		return 5000;
	case ACCEL_CONFIG0_ODR_100_HZ:
		return 10000;
	case ACCEL_CONFIG0_ODR_50_HZ:
		return 20000;
	case ACCEL_CONFIG0_ODR_25_HZ:
		return 40000;
	case ACCEL_CONFIG0_ODR_12_5_HZ:
		return 80000;
	case ACCEL_CONFIG0_ODR_6_25_HZ:
		return 160000;
	case ACCEL_CONFIG0_ODR_3_125_HZ:
		return 320000;
	case ACCEL_CONFIG0_ODR_1_5625_HZ:
	default:
		return 640000;
	}
}

const char *inv_imu_get_version(void)
{
	return INV_IMU_VERSION_STRING;
}

/*
 * Static functions definition
 */

static enum power_mode get_power_mode(inv_imu_device_t *s, pwr_mgmt0_t pwr_mgmt0)
{
	if ((pwr_mgmt0.gyro_mode == PWR_MGMT0_GYRO_MODE_OFF ||
	     pwr_mgmt0.gyro_mode == PWR_MGMT0_GYRO_MODE_STANDBY) &&
	    pwr_mgmt0.accel_ois_en_ap == INV_IMU_DISABLE &&
	    pwr_mgmt0.gyro_ois_en_ap == INV_IMU_DISABLE &&
	    pwr_mgmt0.accel_mode == PWR_MGMT0_ACCEL_MODE_OFF)
		return POWER_MODE_SLEEP;

	else if ((pwr_mgmt0.gyro_mode == PWR_MGMT0_GYRO_MODE_OFF ||
	          pwr_mgmt0.gyro_mode == PWR_MGMT0_GYRO_MODE_STANDBY) &&
	         pwr_mgmt0.accel_ois_en_ap == INV_IMU_DISABLE &&
	         pwr_mgmt0.gyro_ois_en_ap == INV_IMU_DISABLE &&
	         pwr_mgmt0.accel_mode == PWR_MGMT0_ACCEL_MODE_LP &&
	         pwr_mgmt0.accel_lp_clk_sel == PWR_MGMT0_ACCEL_LP_CLK_RCOSC)
		return POWER_MODE_ALP_RC;

	else if ((pwr_mgmt0.gyro_mode == PWR_MGMT0_GYRO_MODE_OFF ||
	          pwr_mgmt0.gyro_mode == PWR_MGMT0_GYRO_MODE_STANDBY) &&
	         pwr_mgmt0.accel_ois_en_ap == INV_IMU_DISABLE &&
	         pwr_mgmt0.gyro_ois_en_ap == INV_IMU_DISABLE &&
	         pwr_mgmt0.accel_mode == PWR_MGMT0_ACCEL_MODE_LP &&
	         pwr_mgmt0.accel_lp_clk_sel == PWR_MGMT0_ACCEL_LP_CLK_WUOSC)
		return POWER_MODE_ALP_WU;

	else
		return POWER_MODE_LN;
}

static int move_from_alp_rc_to_alp_wu(inv_imu_device_t *s)
{
	int             status = 0;
	accel_config0_t accel_config0;
	pwr_mgmt0_t     pwr_mgmt0;

	/* Wait 1 accel ODR then select Wake-up oscillator for Accel clock source. */
	status |= inv_imu_read_reg(s, ACCEL_CONFIG0, 1, (uint8_t *)&accel_config0);
	inv_imu_sleep_us(s, inv_imu_convert_odr_bitfield_to_us(accel_config0.accel_odr));

	status |= inv_imu_read_reg(s, PWR_MGMT0, 1, (uint8_t *)&pwr_mgmt0);
	pwr_mgmt0.accel_lp_clk_sel = PWR_MGMT0_ACCEL_LP_CLK_WUOSC;
	status |= inv_imu_write_reg(s, PWR_MGMT0, 1, (uint8_t *)&pwr_mgmt0);

	if (s->fifo_is_used) {
		fifo_config6_t fifo_config6;

		/* Enable the automatic RCOSC power on when FIFO threshold is reached. */
		status |= inv_imu_read_reg(s, FIFO_CONFIG6, 1, (uint8_t *)&fifo_config6);
		fifo_config6.rcosc_req_on_fifo_ths_dis = INV_IMU_DISABLE;
		status |= inv_imu_write_reg(s, FIFO_CONFIG6, 1, (uint8_t *)&fifo_config6);
	}

	return status;
}

static int move_from_alp_wu_to_alp_rc(inv_imu_device_t *s)
{
	int             status = 0;
	accel_config0_t accel_config0;
	fifo_config6_t  fifo_config6;
	pwr_mgmt0_t     pwr_mgmt0;

	status |= inv_imu_read_reg(s, ACCEL_CONFIG0, 1, (uint8_t *)&accel_config0);

	/*
	 * It is recommended to disable the automatic RCOSC power-on on FIFO threshold event before
	 * exiting ALP+WUOSC with a wait time of 1 ODR or higher.
	 */
	status |= inv_imu_read_reg(s, FIFO_CONFIG6, 1, (uint8_t *)&fifo_config6);
	if (fifo_config6.rcosc_req_on_fifo_ths_dis == INV_IMU_DISABLE) {
		fifo_config6.rcosc_req_on_fifo_ths_dis = INV_IMU_ENABLE;
		status |= inv_imu_write_reg(s, FIFO_CONFIG6, 1, (uint8_t *)&fifo_config6);
		inv_imu_sleep_us(s, inv_imu_convert_odr_bitfield_to_us(accel_config0.accel_odr));
	}

	/* Select RC oscillator for Accel clock source and wait 1 accel ODR. */
	status |= inv_imu_read_reg(s, PWR_MGMT0, 1, (uint8_t *)&pwr_mgmt0);
	pwr_mgmt0.accel_lp_clk_sel = PWR_MGMT0_ACCEL_LP_CLK_RCOSC;
	status |= inv_imu_write_reg(s, PWR_MGMT0, 1, (uint8_t *)&pwr_mgmt0);

	inv_imu_sleep_us(s, inv_imu_convert_odr_bitfield_to_us(accel_config0.accel_odr));

	return status;
}

static int configure_serial_interface(inv_imu_device_t *s)
{
	/*
	 * WARNING: this function is called in the particular context of IMU soft reset.
	 *          To ensure system reliability, it should access direct mode registers only (address
	 *          0x100hh in register map), but no mclk register (address 0xhh in register map).
	 */
	int             status = 0;
	intf_config1_t  intf_config1;
	device_config_t device_config;

	switch (s->transport.serif_type) {
	case UI_I2C:
		status |= inv_imu_read_reg(s, INTF_CONFIG1, 1, (uint8_t *)&intf_config1);
		intf_config1.i3c_sdr_en = INV_IMU_DISABLE;
		intf_config1.i3c_ddr_en = INV_IMU_DISABLE;
		status |= inv_imu_write_reg(s, INTF_CONFIG1, 1, (uint8_t *)&intf_config1);
		break;

	case UI_SPI4:
	case UI_I3C_IRQ:
	case UI_I3C_IBI:
		break; /* Nothing to do */

	case UI_SPI3:
		status |= inv_imu_read_reg(s, DEVICE_CONFIG, 1, (uint8_t *)&device_config);
		device_config.spi_ap_4wire = INV_IMU_DISABLE;
		status |= inv_imu_write_reg(s, DEVICE_CONFIG, 1, (uint8_t *)&device_config);
		break;

	default:
		return INV_IMU_ERROR_BAD_ARG;
	}

	return status;
}
