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

#include "imu/inv_imu_driver_advanced.h"

/* Fail build if `condition` is true */
#define BUILD_BUG_ON(condition) ((void)sizeof(char[1 - 2 * !!(condition)]))

#ifndef INV_IMU_POST_RESET_HOOK
#define INV_IMU_POST_RESET_HOOK(s) INV_IMU_OK
#endif

int inv_imu_adv_init(inv_imu_device_t *s)
{
	int     status = 0;
	uint8_t whoami;

	/* Ensure `inv_imu_adv_var_t` fits within `adv_var` */
	BUILD_BUG_ON(sizeof(s->adv_var) < sizeof(inv_imu_adv_var_t));

	/* Verify required callback are assigned */
	if (s->transport.read_reg == NULL || s->transport.write_reg == NULL ||
	    s->transport.sleep_us == NULL)
		return INV_IMU_ERROR;

	/* Wait 3 ms to ensure device is properly supplied  */
	inv_imu_sleep_us(s, 3000);

	status |= inv_imu_adv_device_reset(s);

	/* Read and check whoami */
	status |= inv_imu_get_who_am_i(s, &whoami);
	if (whoami != INV_IMU_WHOAMI)
		return INV_IMU_ERROR;

	status |= INV_IMU_POST_RESET_HOOK(s);

	return status;
}

int inv_imu_adv_device_reset(inv_imu_device_t *s)
{
	return inv_imu_soft_reset(s);
}

int inv_imu_adv_enable_accel_lp(inv_imu_device_t *s)
{
	return inv_imu_set_accel_mode(s, PWR_MGMT0_ACCEL_MODE_LP);
}

int inv_imu_adv_enable_accel_ln(inv_imu_device_t *s)
{
	return inv_imu_set_accel_mode(s, PWR_MGMT0_ACCEL_MODE_LN);
}

int inv_imu_adv_disable_accel(inv_imu_device_t *s)
{
	return inv_imu_set_accel_mode(s, PWR_MGMT0_ACCEL_MODE_OFF);
}

int inv_imu_adv_enable_gyro_ln(inv_imu_device_t *s)
{
	return inv_imu_set_gyro_mode(s, PWR_MGMT0_GYRO_MODE_LN);
}

int inv_imu_adv_disable_gyro(inv_imu_device_t *s)
{
	return inv_imu_set_gyro_mode(s, PWR_MGMT0_GYRO_MODE_OFF);
}

int inv_imu_adv_configure_fsync_ap_tag(inv_imu_device_t *s, fsync_config_ui_sel_t sensor_tag)
{
	int                status = INV_IMU_OK;
	inv_imu_adv_var_t *e      = (inv_imu_adv_var_t *)s->adv_var;
	fsync_config_t     fsync_config;

	e->fsync_tag = sensor_tag;

	status |= inv_imu_read_reg(s, FSYNC_CONFIG, 1, (uint8_t *)&fsync_config);
	fsync_config.fsync_ui_sel = (uint8_t)sensor_tag;
	status |= inv_imu_write_reg(s, FSYNC_CONFIG, 1, (uint8_t *)&fsync_config);

	return status;
}

int inv_imu_adv_enable_fsync(inv_imu_device_t *s)
{
	int             status = 0;
	intf_config12_t intf_config12;
	tmst_config1_t  tmst_config1;
#if INV_IMU_FSYNC_PAD_CTRL_EN
	pads_config5_t pads_config5;
#endif

	status |= inv_imu_switch_on_mclk(s);

#if INV_IMU_FSYNC_PAD_CTRL_EN
	/* set FSYNC pad orientation to input */
	status |= inv_imu_read_reg(s, PADS_CONFIG5, 1, (uint8_t *)&pads_config5);
	pads_config5.pads_fsync_int2_tp1_from_pad_disable_d2a = 0;
	status |= inv_imu_write_reg(s, PADS_CONFIG5, 1, (uint8_t *)&pads_config5);
#endif

	status |= inv_imu_read_reg(s, INTF_CONFIG12, 1, (uint8_t *)&intf_config12);
	intf_config12.pads_fsync_int2_cfg = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, INTF_CONFIG12, 1, (uint8_t *)&intf_config12);

	status |= inv_imu_read_reg(s, TMST_CONFIG1, 1, (uint8_t *)&tmst_config1);
	tmst_config1.tmst_delta_en = INV_IMU_ENABLE;
	tmst_config1.tmst_fsync_en = INV_IMU_ENABLE;
	tmst_config1.tmst_en       = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, TMST_CONFIG1, 1, (uint8_t *)&tmst_config1);

	status |= inv_imu_switch_off_mclk(s);

	return status;
}

int inv_imu_adv_disable_fsync(inv_imu_device_t *s)
{
	int             status = 0;
	tmst_config1_t  tmst_config1;
	intf_config12_t intf_config12;
#if INV_IMU_FSYNC_PAD_CTRL_EN
	pads_config5_t pads_config5;
#endif

	status |= inv_imu_switch_on_mclk(s);

	status |= inv_imu_read_reg(s, TMST_CONFIG1, 1, (uint8_t *)&tmst_config1);
	tmst_config1.tmst_delta_en = INV_IMU_DISABLE;
	tmst_config1.tmst_fsync_en = INV_IMU_DISABLE;
	tmst_config1.tmst_en       = INV_IMU_DISABLE;
	status |= inv_imu_write_reg(s, TMST_CONFIG1, 1, (uint8_t *)&tmst_config1);

	status |= inv_imu_read_reg(s, INTF_CONFIG12, 1, (uint8_t *)&intf_config12);
	intf_config12.pads_fsync_int2_cfg = INV_IMU_DISABLE;
	status |= inv_imu_write_reg(s, INTF_CONFIG12, 1, (uint8_t *)&intf_config12);

#if INV_IMU_FSYNC_PAD_CTRL_EN
	/* set FSYNC pad orientation to output. */
	status |= inv_imu_read_reg(s, PADS_CONFIG5, 1, (uint8_t *)&pads_config5);
	pads_config5.pads_fsync_int2_tp1_from_pad_disable_d2a = 1;
	status |= inv_imu_write_reg(s, PADS_CONFIG5, 1, (uint8_t *)&pads_config5);
#endif

	status |= inv_imu_switch_off_mclk(s);

	return status;
}

int inv_imu_adv_get_data_from_registers(inv_imu_device_t *s)
{
	int                      status = 0;
	inv_imu_sensor_event_t   event  = { 0 };
	const inv_imu_adv_var_t *e      = (const inv_imu_adv_var_t *)s->adv_var;
	uint8_t data[TEMP_DATA_SIZE + ACCEL_DATA_SIZE + GYRO_DATA_SIZE + FIFO_TS_FSYNC_SIZE];
	uint8_t fsync_tag_in_accel = 0;
	uint8_t fsync_tag_in_gyro  = 0;
	uint8_t fsync_tag_in_temp  = 0;

	/* Read sensor data from registers. */
	status |= inv_imu_read_reg(s, TEMP_DATA1, sizeof(data), data);

	/* Extract the various sensor data and store them in sensor event structure. */
	/* Temperature */
	FORMAT_16_BITS_DATA(s->endianness_data, &data[0], (uint16_t *)&event.temperature);
	if (event.temperature != INVALID_VALUE_FIFO) {
		event.sensor_mask |= (1 << INV_SENSOR_TEMPERATURE);
	}

	/* Accel */
	FORMAT_16_BITS_DATA(s->endianness_data, &data[2], (uint16_t *)&event.accel[0]);
	FORMAT_16_BITS_DATA(s->endianness_data, &data[4], (uint16_t *)&event.accel[1]);
	FORMAT_16_BITS_DATA(s->endianness_data, &data[6], (uint16_t *)&event.accel[2]);

	if ((event.accel[0] != INVALID_VALUE_FIFO) && (event.accel[1] != INVALID_VALUE_FIFO) &&
	    (event.accel[2] != INVALID_VALUE_FIFO)) {
		event.sensor_mask |= (1 << INV_SENSOR_ACCEL);
	}

	/* Gyro */
	FORMAT_16_BITS_DATA(s->endianness_data, &data[8], (uint16_t *)&event.gyro[0]);
	FORMAT_16_BITS_DATA(s->endianness_data, &data[10], (uint16_t *)&event.gyro[1]);
	FORMAT_16_BITS_DATA(s->endianness_data, &data[12], (uint16_t *)&event.gyro[2]);
	if ((event.gyro[0] != INVALID_VALUE_FIFO) && (event.gyro[1] != INVALID_VALUE_FIFO) &&
	    (event.gyro[2] != INVALID_VALUE_FIFO)) {
		event.sensor_mask |= (1 << INV_SENSOR_GYRO);
	}

	/* 
	 * Check if fsync flag is set and then get FSYNC counter.
	 */
	switch (e->fsync_tag) {
	case FSYNC_CONFIG_UI_SEL_ACCEL_X:
		fsync_tag_in_accel =
		    (event.sensor_mask & (1 << INV_SENSOR_ACCEL)) && (event.accel[0] & 0x1);
		break;
	case FSYNC_CONFIG_UI_SEL_ACCEL_Y:
		fsync_tag_in_accel =
		    (event.sensor_mask & (1 << INV_SENSOR_ACCEL)) && (event.accel[1] & 0x1);
		break;
	case FSYNC_CONFIG_UI_SEL_ACCEL_Z:
		fsync_tag_in_accel =
		    (event.sensor_mask & (1 << INV_SENSOR_ACCEL)) && (event.accel[2] & 0x1);
		break;
	case FSYNC_CONFIG_UI_SEL_GYRO_X:
		fsync_tag_in_gyro = (event.sensor_mask & (1 << INV_SENSOR_GYRO)) && (event.gyro[0] & 0x1);
		break;
	case FSYNC_CONFIG_UI_SEL_GYRO_Y:
		fsync_tag_in_gyro = (event.sensor_mask & (1 << INV_SENSOR_GYRO)) && (event.gyro[1] & 0x1);
		break;
	case FSYNC_CONFIG_UI_SEL_GYRO_Z:
		fsync_tag_in_gyro = (event.sensor_mask & (1 << INV_SENSOR_GYRO)) && (event.gyro[2] & 0x1);
		break;
	case FSYNC_CONFIG_UI_SEL_TEMP:
		fsync_tag_in_temp =
		    (event.sensor_mask & (1 << INV_SENSOR_TEMPERATURE)) && (event.temperature & 0x1);
		break;
	default:
		break;
	}

	/* Sensor data register is configured to expose fsync flag. If fsync flag is set, process fsync counter. */
	if (fsync_tag_in_accel || fsync_tag_in_gyro || fsync_tag_in_temp) {
		FORMAT_16_BITS_DATA(s->endianness_data, &data[14], (uint16_t *)&event.timestamp_fsync);
		event.sensor_mask |= (1 << INV_SENSOR_FSYNC_EVENT);
	}

	/* call sensor event callback */
	if (e->sensor_event_cb)
		e->sensor_event_cb(&event);

	return status;
}

int inv_imu_adv_reset_fifo(inv_imu_device_t *s)
{
	int status = INV_IMU_OK;

	status |= inv_imu_flush_fifo(s);

	return status;
}

int inv_imu_adv_get_fifo_config(inv_imu_device_t *s, inv_imu_adv_fifo_config_t *conf)
{
	int            status = INV_IMU_OK;
	fifo_config5_t fifo_config5;
	fdr_config_t   fdr_config;

	status = inv_imu_get_fifo_config(s, &conf->base_conf);

	status |= inv_imu_read_reg(s, FIFO_CONFIG5, 1, (uint8_t *)&fifo_config5);
	conf->fifo_wm_gt_th = (fifo_config5_fifo_wm_gt_th_t)fifo_config5.fifo_wm_gt_th;
	conf->tmst_fsync_en = fifo_config5.fifo_tmst_fsync_en;

	status |= inv_imu_read_reg(s, FDR_CONFIG, 1, (uint8_t *)&fdr_config);
	conf->decim_rate = (fdr_config_fdr_sel_t)fdr_config.fdr_sel;

	return status;
}

int inv_imu_adv_set_fifo_config(inv_imu_device_t *s, const inv_imu_adv_fifo_config_t *conf)
{
	int            status = INV_IMU_OK;
	fifo_config5_t fifo_config5;
	fdr_config_t   fdr_config;

	status = inv_imu_set_fifo_config(s, &conf->base_conf);

	status |= inv_imu_read_reg(s, FIFO_CONFIG5, 1, (uint8_t *)&fifo_config5);
	fifo_config5.fifo_wm_gt_th      = conf->fifo_wm_gt_th;
	fifo_config5.fifo_tmst_fsync_en = conf->tmst_fsync_en;
	status |= inv_imu_write_reg(s, FIFO_CONFIG5, 1, (uint8_t *)&fifo_config5);

	status |= inv_imu_read_reg(s, FDR_CONFIG, 1, (uint8_t *)&fdr_config);
	fdr_config.fdr_sel = conf->decim_rate;
	status |= inv_imu_write_reg(s, FDR_CONFIG, 1, (uint8_t *)&fdr_config);

	return status;
}

int inv_imu_adv_get_data_from_fifo(inv_imu_device_t *s, uint8_t fifo_data[FIFO_MIRRORING_SIZE],
                                   uint16_t *fifo_count)
{
	int status = INV_IMU_OK;

	/*
	 * Make sure RCOSC is enabled to guarantee FIFO read.
	 * For power optimization, this call can be omitted under specific conditions:
	 *  - If using WM interrupt and you can guarantee entire FIFO will be read at once.
	 *  - If gyro is enabled or accel is in LN or LP+RCOSC mode.
	 *  - In accel LP+WUOSC mode, if you wait 100 us after reading FIFO_COUNT and 
	 *    you can guarantee that the FIFO will be read within 1 ms.
	 * Please refer to the AN-000324 for more information.
	 */
	status |= inv_imu_switch_on_mclk(s);

	/* Read FIFO count */
	status |= inv_imu_get_frame_count(s, fifo_count);

	if (status != INV_IMU_OK) {
		status |= inv_imu_switch_off_mclk(s);
		return status;
	}

	/* Read FIFO data */
	status |= inv_imu_read_reg(s, FIFO_DATA, *fifo_count * s->fifo_frame_size, fifo_data);

	if (status != INV_IMU_OK)
		status |= inv_imu_flush_fifo(s);

	status |= inv_imu_switch_off_mclk(s);

	return status;
}

/** @brief Parse a FIFO frame and generate a sensor event.
 *  @param[in] s      Pointer to device.
 *  @param[in] frame  Data to parse.
 */
static int parse_fifo_frame(inv_imu_device_t *s, uint8_t *frame)
{
	int                      status    = INV_IMU_OK;
	const inv_imu_adv_var_t *e         = (const inv_imu_adv_var_t *)s->adv_var;
	uint16_t                 frame_idx = 0;
	const fifo_header_t *    header;
	inv_imu_sensor_event_t   event;

	event.sensor_mask = 0;
	header            = (const fifo_header_t *)&(frame[frame_idx]);
	frame_idx += FIFO_HEADER_SIZE;

	/* Read accel data */
	if (header->bits.accel_bit) {
		FORMAT_16_BITS_DATA(s->endianness_data, &(frame[0 + frame_idx]),
		                    (uint16_t *)&event.accel[0]);
		FORMAT_16_BITS_DATA(s->endianness_data, &(frame[2 + frame_idx]),
		                    (uint16_t *)&event.accel[1]);
		FORMAT_16_BITS_DATA(s->endianness_data, &(frame[4 + frame_idx]),
		                    (uint16_t *)&event.accel[2]);
		frame_idx += ACCEL_DATA_SIZE;
	}

	/* Read gyro data */
	if (header->bits.gyro_bit) {
		FORMAT_16_BITS_DATA(s->endianness_data, &(frame[0 + frame_idx]),
		                    (uint16_t *)&event.gyro[0]);
		FORMAT_16_BITS_DATA(s->endianness_data, &(frame[2 + frame_idx]),
		                    (uint16_t *)&event.gyro[1]);
		FORMAT_16_BITS_DATA(s->endianness_data, &(frame[4 + frame_idx]),
		                    (uint16_t *)&event.gyro[2]);
		frame_idx += GYRO_DATA_SIZE;
	}

	/* Read temperature data */
	if ((header->bits.accel_bit) || (header->bits.gyro_bit)) {
		if (header->bits.twentybits_bit) {
			FORMAT_16_BITS_DATA(s->endianness_data, &(frame[0 + frame_idx]),
			                    (uint16_t *)&event.temperature);
			frame_idx += FIFO_TEMP_DATA_SIZE + FIFO_TEMP_HIGH_RES_SIZE;

			if (event.temperature != INVALID_VALUE_FIFO)
				event.sensor_mask |= (1 << INV_SENSOR_TEMPERATURE);
		} else {
			event.temperature = (int8_t)frame[0 + frame_idx];
			frame_idx += FIFO_TEMP_DATA_SIZE;

			if (event.temperature != INVALID_VALUE_FIFO_1B)
				event.sensor_mask |= (1 << INV_SENSOR_TEMPERATURE);
		}
	}

	if ((header->bits.timestamp_bit) || (header->bits.fsync_bit)) {
		FORMAT_16_BITS_DATA(s->endianness_data, &(frame[0 + frame_idx]), &event.timestamp_fsync);
		frame_idx += FIFO_TS_FSYNC_SIZE;

		if (header->bits.fsync_bit)
			event.sensor_mask |= (1 << INV_SENSOR_FSYNC_EVENT);
	}

	if (header->bits.accel_bit) {
		if ((event.accel[0] != INVALID_VALUE_FIFO) && (event.accel[1] != INVALID_VALUE_FIFO) &&
		    (event.accel[2] != INVALID_VALUE_FIFO)) {
			event.sensor_mask |= (1 << INV_SENSOR_ACCEL);

			if (header->bits.twentybits_bit) {
				event.accel_high_res[0] = (frame[0 + frame_idx] >> 4) & 0xF;
				event.accel_high_res[1] = (frame[1 + frame_idx] >> 4) & 0xF;
				event.accel_high_res[2] = (frame[2 + frame_idx] >> 4) & 0xF;
			}
		}
	}

	if (header->bits.gyro_bit) {
		if ((event.gyro[0] != INVALID_VALUE_FIFO) && (event.gyro[1] != INVALID_VALUE_FIFO) &&
		    (event.gyro[2] != INVALID_VALUE_FIFO)) {
			event.sensor_mask |= (1 << INV_SENSOR_GYRO);

			if (header->bits.twentybits_bit) {
				event.gyro_high_res[0] = (frame[0 + frame_idx]) & 0xF;
				event.gyro_high_res[1] = (frame[1 + frame_idx]) & 0xF;
				event.gyro_high_res[2] = (frame[2 + frame_idx]) & 0xF;
			}
		}
	}

	/* call sensor event callback */
	if (e->sensor_event_cb)
		e->sensor_event_cb(&event);

	return status;
}

int inv_imu_adv_parse_fifo_data(inv_imu_device_t *s, const uint8_t fifo_data[FIFO_MIRRORING_SIZE],
                                const uint16_t fifo_count)
{
	int      status   = INV_IMU_OK;
	uint16_t fifo_idx = 0;

	/* Foreach packet in the FIFO */
	for (uint16_t i = 0; i < fifo_count; i++) {
		uint8_t frame[32] = { 0 };

		/* Create frame */
		for (int j = 0; j < s->fifo_frame_size; j++)
			frame[j] = fifo_data[fifo_idx + j];
		fifo_idx += s->fifo_frame_size;

		status |= parse_fifo_frame(s, frame);
	}

	return status;
}

int inv_imu_adv_get_accel_fsr(inv_imu_device_t *s, accel_config0_fs_sel_t *accel_fsr)
{
	int             status = 0;
	accel_config0_t accel_config0;
	fifo_config5_t  fifo_config5;

	status |= inv_imu_read_reg(s, FIFO_CONFIG5, 1, (uint8_t *)&fifo_config5);

	if (s->fifo_is_used && fifo_config5.fifo_hires_en)
		*accel_fsr = ACCEL_CONFIG0_FS_SEL_MAX;
	else {
		status |= inv_imu_read_reg(s, ACCEL_CONFIG0, 1, (uint8_t *)&accel_config0);
		*accel_fsr = (accel_config0_fs_sel_t)accel_config0.accel_ui_fs_sel;
	}

	return status;
}

int inv_imu_adv_get_gyro_fsr(inv_imu_device_t *s, gyro_config0_fs_sel_t *gyro_fsr)
{
	int            status = 0;
	gyro_config0_t gyro_config0;
	fifo_config5_t fifo_config5;

	status |= inv_imu_read_reg(s, FIFO_CONFIG5, 1, (uint8_t *)&fifo_config5);

	if (s->fifo_is_used && fifo_config5.fifo_hires_en)
		*gyro_fsr = GYRO_CONFIG0_FS_SEL_MAX;
	else {
		status |= inv_imu_read_reg(s, GYRO_CONFIG0, 1, (uint8_t *)&gyro_config0);
		*gyro_fsr = (gyro_config0_fs_sel_t)gyro_config0.gyro_ui_fs_sel;
	}

	return status;
}

int inv_imu_adv_set_timestamp_resolution(inv_imu_device_t *         s,
                                         const tmst_config1_resol_t timestamp_resol)
{
	int            status = 0;
	tmst_config1_t tmst_config1;

	status |= inv_imu_read_reg(s, TMST_CONFIG1, 1, (uint8_t *)&tmst_config1);
	tmst_config1.tmst_res = timestamp_resol;
	status |= inv_imu_write_reg(s, TMST_CONFIG1, 1, (uint8_t *)&tmst_config1);

	return status;
}

uint32_t inv_imu_adv_get_timestamp_resolution_us(inv_imu_device_t *s)
{
	int            status = 0;
	tmst_config1_t tmst_config1;

	status |= inv_imu_read_reg(s, TMST_CONFIG1, 1, (uint8_t *)&tmst_config1);
	if (status < 0)
		return INV_IMU_ERROR;

	if (tmst_config1.tmst_res == TMST_CONFIG1_RESOL_16us)
		return 16;
	else if (tmst_config1.tmst_res == TMST_CONFIG1_RESOL_1us)
		return 1;

	/* Should not happen, return 0 */
	return 0;
}

int inv_imu_adv_configure_wom(inv_imu_device_t *s, const uint8_t wom_x_th, const uint8_t wom_y_th,
                              const uint8_t wom_z_th, wom_config_wom_int_mode_t wom_int,
                              wom_config_wom_int_dur_t wom_dur)
{
	int          status = 0;
	uint8_t      data[3];
	wom_config_t wom_config;

	data[0] = wom_x_th; /* Set X threshold */
	data[1] = wom_y_th; /* Set Y threshold */
	data[2] = wom_z_th; /* Set Z threshold */
	status |= inv_imu_write_reg(s, ACCEL_WOM_X_THR, sizeof(data), &data[0]);

	/*
	 * Configure WOM:
	 *  - Compare current sample with the previous sample.
	 *  - WOM from the 3 axis are ORed or ANDed to produce WOM signal.
	 *  - Set number of overthreshold events to wait before producing the WOM signal.
	 */
	status |= inv_imu_read_reg(s, WOM_CONFIG, 1, (uint8_t *)&wom_config);
	wom_config.wom_mode     = WOM_CONFIG_WOM_MODE_CMP_PREV;
	wom_config.wom_int_mode = wom_int;
	wom_config.wom_int_dur  = wom_dur;
	status |= inv_imu_write_reg(s, WOM_CONFIG, 1, (uint8_t *)&wom_config);

	return status;
}

int inv_imu_adv_enable_wom(inv_imu_device_t *s)
{
	int          status = 0;
	wom_config_t wom_config;

	/* Enable WOM */
	status |= inv_imu_read_reg(s, WOM_CONFIG, 1, (uint8_t *)&wom_config);
	wom_config.wom_en = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, WOM_CONFIG, 1, (uint8_t *)&wom_config);

	return status;
}

int inv_imu_adv_disable_wom(inv_imu_device_t *s)
{
	int          status = 0;
	wom_config_t wom_config;

	/* Disable WOM */
	status |= inv_imu_read_reg(s, WOM_CONFIG, 1, (uint8_t *)&wom_config);
	wom_config.wom_en = INV_IMU_DISABLE;
	status |= inv_imu_write_reg(s, WOM_CONFIG, 1, (uint8_t *)&wom_config);

	return status;
}

int inv_imu_adv_set_endianness(inv_imu_device_t *s, intf_config0_data_endian_t endianness)
{
	int            status = 0;
	intf_config0_t intf_config0;

	status |= inv_imu_read_reg(s, INTF_CONFIG0, 1, (uint8_t *)&intf_config0);
	intf_config0.sensor_data_endian = endianness;
	status |= inv_imu_write_reg(s, INTF_CONFIG0, 1, (uint8_t *)&intf_config0);

	if (!status)
		s->endianness_data = (uint8_t)endianness;

	return status;
}

int inv_imu_adv_set_spi_slew_rate(inv_imu_device_t *                  s,
                                  const drive_config3_spi_slew_rate_t slew_rate)
{
	int             status = 0;
	drive_config3_t drive_config3;

	status |= inv_imu_read_reg(s, DRIVE_CONFIG3, 1, (uint8_t *)&drive_config3);
	drive_config3.spi_slew_rate = slew_rate;
	status |= inv_imu_write_reg(s, DRIVE_CONFIG3, 1, (uint8_t *)&drive_config3);

	return status;
}

int inv_imu_adv_set_int_pulse(inv_imu_device_t *s, const int_config1_int_pulse_duration_t pulse_dur)
{
	int           status = 0;
	int_config1_t int_config1;

	status |= inv_imu_read_reg(s, INT_CONFIG1, 1, (uint8_t *)&int_config1);
	int_config1.int_tpulse_duration = pulse_dur;
	status |= inv_imu_write_reg(s, INT_CONFIG1, 1, (uint8_t *)&int_config1);

	return status;
}

#if INV_IMU_HIGH_ODR_SUPPORTED
int inv_imu_adv_enable_accel_ois(inv_imu_device_t *s)
{
	return inv_imu_set_accel_ois_mode(s, PWR_MGMT0_ACCEL_OIS_MODE_LN);
}

int inv_imu_adv_disable_accel_ois(inv_imu_device_t *s)
{
	return inv_imu_set_accel_ois_mode(s, PWR_MGMT0_ACCEL_OIS_MODE_OFF);
}

int inv_imu_adv_enable_gyro_ois(inv_imu_device_t *s)
{
	return inv_imu_set_gyro_ois_mode(s, PWR_MGMT0_GYRO_OIS_MODE_LN);
}

int inv_imu_adv_disable_gyro_ois(inv_imu_device_t *s)
{
	return inv_imu_set_gyro_ois_mode(s, PWR_MGMT0_GYRO_OIS_MODE_OFF);
}

int inv_imu_adv_get_ois_data_from_registers(inv_imu_device_t *s)
{
	int                       status = 0;
	const inv_imu_adv_var_t * e      = (const inv_imu_adv_var_t *)s->adv_var;
	inv_imu_sensor_event_t    event  = { 0 };
	inv_imu_ois_sensor_data_t data   = { 0 };

	status |= inv_imu_get_ois_register_data(s, &data);

	/* Copy data read from registers into sensor event structure. */
	event.temperature = data.temp_data;
	memcpy(event.accel, data.accel_data, ACCEL_DATA_SIZE + GYRO_DATA_SIZE);
	memcpy(event.accel_high_res, data.accel_high_res, 6);

	if (event.temperature != INVALID_VALUE_FIFO) {
		event.sensor_mask |= (1 << INV_SENSOR_TEMPERATURE);
	}

	if ((event.accel[0] != INVALID_VALUE_FIFO) && (event.accel[1] != INVALID_VALUE_FIFO) &&
	    (event.accel[2] != INVALID_VALUE_FIFO)) {
		event.sensor_mask |= (1 << INV_SENSOR_ACCEL);
	}

	if ((event.gyro[0] != INVALID_VALUE_FIFO) && (event.gyro[1] != INVALID_VALUE_FIFO) &&
	    (event.gyro[2] != INVALID_VALUE_FIFO)) {
		event.sensor_mask |= (1 << INV_SENSOR_GYRO);
	}

	/* call sensor event callback */
	if (e->sensor_event_cb)
		e->sensor_event_cb(&event);

	return status;
}

int inv_imu_adv_enable_ois_virtualization(inv_imu_device_t *s)
{
	int             status = 0;
	intf_config12_t intf_config12;

	status |= inv_imu_read_reg(s, INTF_CONFIG12, 1, (uint8_t *)&intf_config12);
	intf_config12.ois_on_ap_if = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, INTF_CONFIG12, 1, (uint8_t *)&intf_config12);

	return status;
}

int inv_imu_adv_disable_ois_virtualization(inv_imu_device_t *s)
{
	int             status = 0;
	intf_config12_t intf_config12;

	status |= inv_imu_read_reg(s, INTF_CONFIG12, 1, (uint8_t *)&intf_config12);
	intf_config12.ois_on_ap_if = INV_IMU_DISABLE;
	status |= inv_imu_write_reg(s, INTF_CONFIG12, 1, (uint8_t *)&intf_config12);

	return status;
}
#endif
