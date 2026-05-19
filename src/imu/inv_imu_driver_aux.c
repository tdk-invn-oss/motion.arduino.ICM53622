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

#include "imu/inv_imu_driver_aux.h"

int inv_imu_aux_enable_accel(inv_imu_transport_t *t)
{
	int           status = 0;
	ois_config0_t ois_config0;

	/* AUX path using LN power mode, requires RCOSC ON before enabling sensor. */
	status |= inv_imu_read_reg(t, OIS_CONFIG0, 1, (uint8_t *)&ois_config0);
	if (ois_config0.ois_accel_lp_clk_sel != OIS_CONFIG0_ACCEL_LP_CLK_RCOSC) {
		ois_config0.ois_accel_lp_clk_sel = OIS_CONFIG0_ACCEL_LP_CLK_RCOSC;
		status |= inv_imu_write_reg(t, OIS_CONFIG0, 1, (uint8_t *)&ois_config0);

		/* Wait for 1 * 12.5Hz ODR for timestamp continuity. */
		t->sleep_us(80000U);
	}

	status |= inv_imu_read_reg(t, OIS_CONFIG0, 1, (uint8_t *)&ois_config0);
	ois_config0.accel_ois_en = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(t, OIS_CONFIG0, 1, (uint8_t *)&ois_config0);

	return status;
}

int inv_imu_aux_disable_accel(inv_imu_transport_t *t)
{
	int           status = 0;
	ois_config0_t ois_config0;

	status |= inv_imu_read_reg(t, OIS_CONFIG0, 1, (uint8_t *)&ois_config0);
	ois_config0.accel_ois_en = INV_IMU_DISABLE;
	status |= inv_imu_write_reg(t, OIS_CONFIG0, 1, (uint8_t *)&ois_config0);

	if (!ois_config0.gyro_ois_en) {
		/* Wait for 1 * 12.5Hz ODR for timestamp continuity. */
		t->sleep_us(80000U);

		status |= inv_imu_read_reg(t, OIS_CONFIG0, 1, (uint8_t *)&ois_config0);
		ois_config0.ois_accel_lp_clk_sel = OIS_CONFIG0_ACCEL_LP_CLK_WUOSC;
		status |= inv_imu_write_reg(t, OIS_CONFIG0, 1, (uint8_t *)&ois_config0);
	}

	return status;
}

int inv_imu_aux_set_accel_fsr(inv_imu_transport_t *t, ois_config1_accel_fs_sel_t fsr)
{
	int           status = 0;
	ois_config1_t ois_config1;

	status |= inv_imu_read_reg(t, OIS_CONFIG1, 1, (uint8_t *)&ois_config1);
	ois_config1.accel_ois_fs_sel = fsr;
	status |= inv_imu_write_reg(t, OIS_CONFIG1, 1, (uint8_t *)&ois_config1);

	return status;
}

int inv_imu_aux_get_accel_fsr(inv_imu_transport_t *t, ois_config1_accel_fs_sel_t *fsr)
{
	int           status = 0;
	ois_config1_t ois_config1;

	status |= inv_imu_read_reg(t, OIS_CONFIG1, 1, (uint8_t *)&ois_config1);
	*fsr = (ois_config1_accel_fs_sel_t)ois_config1.accel_ois_fs_sel;

	return status;
}

int inv_imu_aux_set_accel_bw(inv_imu_transport_t *t, ois_config3_accel_filt_bw_ind_t bw)
{
	int           status = 0;
	ois_config3_t ois_config3;

	status |= inv_imu_read_reg(t, OIS_CONFIG3, 1, (uint8_t *)&ois_config3);
	ois_config3.accel_ois_filt_bw_ind = bw;
	status |= inv_imu_write_reg(t, OIS_CONFIG3, 1, (uint8_t *)&ois_config3);

	return status;
}

int inv_imu_aux_get_accel_bw(inv_imu_transport_t *t, ois_config3_accel_filt_bw_ind_t *bw)
{
	int           status = 0;
	ois_config3_t ois_config3;

	status |= inv_imu_read_reg(t, OIS_CONFIG3, 1, (uint8_t *)&ois_config3);
	*bw = (ois_config3_accel_filt_bw_ind_t)ois_config3.accel_ois_filt_bw_ind;

	return status;
}

int inv_imu_aux_enable_gyro(inv_imu_transport_t *t)
{
	int           status = 0;
	ois_config0_t ois_config0;

	/* AUX path using LN power mode, requires RCOSC ON before enabling sensor. */
	status |= inv_imu_read_reg(t, OIS_CONFIG0, 1, (uint8_t *)&ois_config0);
	if (ois_config0.ois_accel_lp_clk_sel != OIS_CONFIG0_ACCEL_LP_CLK_RCOSC) {
		ois_config0.ois_accel_lp_clk_sel = OIS_CONFIG0_ACCEL_LP_CLK_RCOSC;
		status |= inv_imu_write_reg(t, OIS_CONFIG0, 1, (uint8_t *)&ois_config0);

		/* Wait for 1 * 12.5Hz ODR for timestamp continuity. */
		t->sleep_us(80000U);
	}

	status |= inv_imu_read_reg(t, OIS_CONFIG0, 1, (uint8_t *)&ois_config0);
	ois_config0.gyro_ois_en = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(t, OIS_CONFIG0, 1, (uint8_t *)&ois_config0);

	return status;
}

int inv_imu_aux_disable_gyro(inv_imu_transport_t *t)
{
	int           status = 0;
	ois_config0_t ois_config0;

	status |= inv_imu_read_reg(t, OIS_CONFIG0, 1, (uint8_t *)&ois_config0);
	ois_config0.gyro_ois_en = INV_IMU_DISABLE;
	status |= inv_imu_write_reg(t, OIS_CONFIG0, 1, (uint8_t *)&ois_config0);

	if (!ois_config0.accel_ois_en) {
		/* Wait for 1 * 12.5Hz ODR for timestamp continuity. */
		t->sleep_us(80000U);

		status |= inv_imu_read_reg(t, OIS_CONFIG0, 1, (uint8_t *)&ois_config0);
		ois_config0.ois_accel_lp_clk_sel = OIS_CONFIG0_ACCEL_LP_CLK_WUOSC;
		status |= inv_imu_write_reg(t, OIS_CONFIG0, 1, (uint8_t *)&ois_config0);
	}

	return status;
}

int inv_imu_aux_set_gyro_fsr(inv_imu_transport_t *t, ois_config1_gyro_fs_sel_t fsr)
{
	int           status = 0;
	ois_config1_t ois_config1;

	status |= inv_imu_read_reg(t, OIS_CONFIG1, 1, (uint8_t *)&ois_config1);
	ois_config1.gyro_ois_fs_sel = fsr;
	status |= inv_imu_write_reg(t, OIS_CONFIG1, 1, (uint8_t *)&ois_config1);

	return status;
}

int inv_imu_aux_get_gyro_fsr(inv_imu_transport_t *t, ois_config1_gyro_fs_sel_t *fsr)
{
	int           status = 0;
	ois_config1_t ois_config1;

	status |= inv_imu_read_reg(t, OIS_CONFIG1, 1, (uint8_t *)&ois_config1);
	*fsr = (ois_config1_gyro_fs_sel_t)ois_config1.gyro_ois_fs_sel;

	return status;
}

int inv_imu_aux_set_gyro_bw(inv_imu_transport_t *t, ois_config3_gyro_filt_bw_ind_t bw)
{
	int           status = 0;
	ois_config3_t ois_config3;

	status |= inv_imu_read_reg(t, OIS_CONFIG3, 1, (uint8_t *)&ois_config3);
	ois_config3.gyro_ois_filt_bw_ind = bw;
	status |= inv_imu_write_reg(t, OIS_CONFIG3, 1, (uint8_t *)&ois_config3);

	return status;
}

int inv_imu_aux_get_gyro_bw(inv_imu_transport_t *t, ois_config3_gyro_filt_bw_ind_t *bw)
{
	int           status = 0;
	ois_config3_t ois_config3;

	status |= inv_imu_read_reg(t, OIS_CONFIG3, 1, (uint8_t *)&ois_config3);
	*bw = (ois_config3_gyro_filt_bw_ind_t)ois_config3.gyro_ois_filt_bw_ind;

	return status;
}

int inv_imu_aux_set_config_int(inv_imu_transport_t *t, const inv_imu_int_num_t num,
                               const inv_imu_aux_int_state_t *it)
{
	int           status = 0;
	uint32_t      reg;
	int_source2_t int_source;

	/** Register to configure AUX port interrupts (INT1 and INT2 share the same bit location in
	 *  different register):
	 *   - INT1: INT_SOURCE2
	 *   - INT2: INT_SOURCE5
 	 */
	switch (num) {
	case INV_IMU_INT1:
		reg = INT_SOURCE2;
		break;
	case INV_IMU_INT2:
		reg = INT_SOURCE5;
		break;
	default:
		return INV_IMU_ERROR_BAD_ARG;
	}

	status |= inv_imu_read_reg(t, reg, 1, (uint8_t *)&int_source);

	int_source.aux_i3c_prot_err_int1_en = (it->INT_AUX_I3C_PROT != 0);
	int_source.ois_drdy_int1_en         = (it->INT_OIS_DRDY != 0);
	int_source.ois_agc_rdy_int1_en      = (it->INT_OIS_AGC_RDY != 0);
	int_source.ois_fsync_int1_en        = (num == INV_IMU_INT1) ? (it->INT_OIS_FSYNC != 0) : 0;

	status |= inv_imu_write_reg(t, reg, 1, (uint8_t *)&int_source);

	return status;
}

int inv_imu_aux_get_config_int(inv_imu_transport_t *t, const inv_imu_int_num_t num,
                               inv_imu_aux_int_state_t *it)
{
	int           status = 0;
	uint32_t      reg;
	int_source2_t int_source;

	/** Register to configure AUX port interrupts (INT1 and INT2 share the same bit location in
	 *  different register):
	 *   - INT1: INT_SOURCE2
	 *   - INT2: INT_SOURCE5
 	 */
	switch (num) {
	case INV_IMU_INT1:
		reg = INT_SOURCE2;
		break;
	case INV_IMU_INT2:
		reg = INT_SOURCE5;
		break;
	default:
		return INV_IMU_ERROR_BAD_ARG;
	}

	status |= inv_imu_read_reg(t, reg, 1, (uint8_t *)&int_source);

	it->INT_AUX_I3C_PROT = int_source.aux_i3c_prot_err_int1_en;
	it->INT_OIS_DRDY     = int_source.ois_drdy_int1_en;
	it->INT_OIS_AGC_RDY  = int_source.ois_agc_rdy_int1_en;
	it->INT_OIS_FSYNC    = (num == INV_IMU_INT1) ? int_source.ois_fsync_int1_en : 0;

	return status;
}

int inv_imu_aux_enable_fsync(inv_imu_transport_t *t)
{
	int           status = 0;
	ois_config2_t ois_config2;

	status |= inv_imu_read_reg(t, OIS_CONFIG2, 1, (uint8_t *)&ois_config2);
	ois_config2.fsync_ois_sel = OIS_CONFIG2_FSYNC_SEL_TEMP;
	status |= inv_imu_write_reg(t, OIS_CONFIG2, 1, (uint8_t *)&ois_config2);

	return status;
}

int inv_imu_aux_disable_fsync(inv_imu_transport_t *t)
{
	int           status = 0;
	ois_config2_t ois_config2;

	status |= inv_imu_read_reg(t, OIS_CONFIG2, 1, (uint8_t *)&ois_config2);
	ois_config2.fsync_ois_sel = OIS_CONFIG2_FSYNC_SEL_NO;
	status |= inv_imu_write_reg(t, OIS_CONFIG2, 1, (uint8_t *)&ois_config2);

	return status;
}

int inv_imu_aux_get_register_data(inv_imu_transport_t *t, inv_imu_sensor_data_t *data)
{
	int rc = 0;

	rc |= inv_imu_read_reg(t, TEMP_DATA1_OIS, sizeof(inv_imu_sensor_data_t), (uint8_t *)data);

	return rc;
}
