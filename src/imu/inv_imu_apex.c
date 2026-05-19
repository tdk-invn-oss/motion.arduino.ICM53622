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

#include "imu/inv_imu_apex.h"

int inv_imu_apex_enable_ff(inv_imu_device_t *s)
{
	int            status = 0;
	apex_config1_t apex_config1;

	status |= inv_imu_apex_start_dmp(s);

	status |= inv_imu_read_reg(s, APEX_CONFIG1, 1, (uint8_t *)&apex_config1);
	apex_config1.ff_enable = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, APEX_CONFIG1, 1, (uint8_t *)&apex_config1);

	return status;
}

int inv_imu_apex_disable_ff(inv_imu_device_t *s)
{
	int            status = 0;
	apex_config1_t apex_config1;

	status |= inv_imu_read_reg(s, APEX_CONFIG1, 1, (uint8_t *)&apex_config1);
	apex_config1.ff_enable = INV_IMU_DISABLE;
	status |= inv_imu_write_reg(s, APEX_CONFIG1, 1, (uint8_t *)&apex_config1);

	return status;
}

#if INV_IMU_ACC_HFSR_SUPPORTED
int inv_imu_apex_enable_ext_highg(inv_imu_device_t *s)
{
	int            status = 0;
	apex_config1_t apex_config1;

	status |= inv_imu_apex_start_dmp(s);

	inv_imu_read_reg(s, APEX_CONFIG1, 1, (uint8_t *)&apex_config1);
	apex_config1.ext_highg_en = INV_IMU_ENABLE;
	inv_imu_write_reg(s, APEX_CONFIG1, 1, (uint8_t *)&apex_config1);

	return status;
}

int inv_imu_apex_disable_ext_highg(inv_imu_device_t *s)
{
	int            status = 0;
	apex_config1_t apex_config1;

	status |= inv_imu_read_reg(s, APEX_CONFIG1, 1, (uint8_t *)&apex_config1);
	apex_config1.ext_highg_en = INV_IMU_DISABLE;
	status |= inv_imu_write_reg(s, APEX_CONFIG1, 1, (uint8_t *)&apex_config1);

	return status;
}
#endif

int inv_imu_apex_init_parameters_struct(inv_imu_device_t *s, inv_imu_apex_parameters_t *apex_inputs)
{
	int status = 0;
	(void)s;

	/* Default parameters at POR */
	apex_inputs->tilt_wait_time       = APEX_CONFIG5_TILT_WAIT_TIME_4_S;
	apex_inputs->power_save_time      = APEX_CONFIG2_DMP_POWER_SAVE_TIME_SEL_8_S;
	apex_inputs->power_save           = INV_IMU_ENABLE;
	apex_inputs->ff_debounce_duration = APEX_CONFIG9_FF_DEBOUNCE_DURATION_2000_MS;
	apex_inputs->ff_max_duration_cm   = APEX_CONFIG12_FF_MAX_DURATION_204_CM;
	apex_inputs->ff_min_duration_cm   = APEX_CONFIG12_FF_MIN_DURATION_10_CM;
	apex_inputs->lowg_peak_th         = APEX_CONFIG10_LOWG_PEAK_TH_563_MG;
	apex_inputs->lowg_peak_hyst       = APEX_CONFIG5_LOWG_PEAK_TH_HYST_156_MG;
	apex_inputs->lowg_samples_th      = APEX_CONFIG10_LOWG_TIME_TH_1_SAMPLE;
	apex_inputs->highg_peak_th        = APEX_CONFIG11_HIGHG_PEAK_TH_2500_MG;
	apex_inputs->highg_peak_hyst      = APEX_CONFIG5_HIGHG_PEAK_TH_HYST_156_MG;
	apex_inputs->highg_samples_th     = APEX_CONFIG11_HIGHG_TIME_TH_1_SAMPLE;
	apex_inputs->tap_max              = APEX_CONFIG2_TAP_MAX_DOUBLE;
	apex_inputs->tap_min              = APEX_CONFIG2_TAP_MIN_SINGLE;
	apex_inputs->tap_tavg             = APEX_CONFIG3_TAP_TAVG_8;
	apex_inputs->tap_tmax             = APEX_CONFIG3_TAP_TMAX_500_MS;
	apex_inputs->tap_tmin             = APEX_CONFIG3_TAP_TMIN_165_MS;
	apex_inputs->tap_max_peak_tol     = APEX_CONFIG4_TAP_MAX_PEAK_TOL_37_5;
	apex_inputs->tap_min_jerk_thr     = APEX_CONFIG4_TAP_MIN_JERK_THR_187mg;
	apex_inputs->tap_smudge_reject_th = APEX_CONFIG9_TAP_SMUDGE_REJECT_THR_85_MS;
#if INV_IMU_ACC_HFSR_SUPPORTED
	apex_inputs->ext_highg_samples_th = APEX_CONFIG14_EXT_HIGHG_TIME_TH_1_SAMPLE;
	apex_inputs->ext_highg_peak_hyst  = APEX_CONFIG14_EXT_HIGHG_PEAK_TH_HYST_250_MG;
	apex_inputs->ext_highg_peak_th    = APEX_CONFIG14_EXT_HIGHG_PEAK_TH_20_G;
#endif

	return status;
}

int inv_imu_apex_configure_parameters(inv_imu_device_t *               s,
                                      const inv_imu_apex_parameters_t *apex_inputs)
{
	int             status = 0;
	apex_config0_t  apex_config0;
	apex_config1_t  apex_config1;
	apex_config12_t apex_config12;
#if INV_IMU_ACC_HFSR_SUPPORTED
	apex_config14_t apex_config14;
#endif
	struct {
		apex_config2_t  cfg2;
		apex_config3_t  cfg3;
		apex_config4_t  cfg4;
		apex_config5_t  cfg5;
		apex_config9_t  cfg9;
		apex_config10_t cfg10;
		apex_config11_t cfg11;
	} regs;

	/* DMP cannot be configured if it is running, hence make sure all APEX algorithms are off */
	status |= inv_imu_read_reg(s, APEX_CONFIG1, 1, (uint8_t *)&apex_config1);
	if (apex_config1.tap_enable)
		return INV_IMU_ERROR;
	if (apex_config1.tilt_enable)
		return INV_IMU_ERROR;
	if (apex_config1.ff_enable)
		return INV_IMU_ERROR;
#if INV_IMU_ACC_HFSR_SUPPORTED
	if (apex_config1.ext_highg_en)
		return INV_IMU_ERROR;
#endif

	status |= inv_imu_switch_on_mclk(s);

	/* APEX_CONFIG0 */
	status |= inv_imu_read_reg(s, APEX_CONFIG0, 1, (uint8_t *)&apex_config0);
	apex_config0.dmp_power_save_en = apex_inputs->power_save;
	status |= inv_imu_write_reg(s, APEX_CONFIG0, 1, (uint8_t *)&apex_config0);

	/* APEX_CONFIG2 */
	regs.cfg2.dmp_power_save_time_sel = apex_inputs->power_save_time;
	regs.cfg2.tap_max                 = apex_inputs->tap_max;
	regs.cfg2.tap_min                 = apex_inputs->tap_min;

	/* APEX_CONFIG3 */
	regs.cfg3.tap_tavg = apex_inputs->tap_tavg;
	regs.cfg3.tap_tmax = apex_inputs->tap_tmax;
	regs.cfg3.tap_tmin = apex_inputs->tap_tmin;

	/* APEX_CONFIG4 */
	regs.cfg4.tap_max_peak_tol = apex_inputs->tap_max_peak_tol;
	regs.cfg4.tap_min_jerk_thr = apex_inputs->tap_min_jerk_thr;

	/* APEX_CONFIG5 */
	regs.cfg5.tilt_wait_time_sel     = apex_inputs->tilt_wait_time;
	regs.cfg5.lowg_peak_th_hyst_sel  = apex_inputs->lowg_peak_hyst;
	regs.cfg5.highg_peak_th_hyst_sel = apex_inputs->highg_peak_hyst;

	/* APEX_CONFIG9 */
	regs.cfg9.ff_debounce_duration_sel = apex_inputs->ff_debounce_duration;
	regs.cfg9.tap_smudge_reject_thr    = apex_inputs->tap_smudge_reject_th;

	/* APEX_CONFIG10 */
	regs.cfg10.lowg_peak_th_sel = apex_inputs->lowg_peak_th;
	regs.cfg10.lowg_time_th_sel = apex_inputs->lowg_samples_th;

	/* APEX_CONFIG11 */
	regs.cfg11.highg_peak_th_sel = apex_inputs->highg_peak_th;
	regs.cfg11.highg_time_th_sel = apex_inputs->highg_samples_th;

	status |= inv_imu_write_reg(s, APEX_CONFIG2, sizeof(regs), (uint8_t *)&regs);

	/* APEX_CONFIG12 */
	apex_config12.ff_max_duration_sel = apex_inputs->ff_max_duration_cm;
	apex_config12.ff_min_duration_sel = apex_inputs->ff_min_duration_cm;

	status |= inv_imu_write_reg(s, APEX_CONFIG12, 1, (uint8_t *)&apex_config12);

#if INV_IMU_ACC_HFSR_SUPPORTED
	/* APEX_CONFIG14 */
	apex_config14.ext_highg_time_th_sel      = apex_inputs->ext_highg_samples_th;
	apex_config14.ext_highg_peak_th_hyst_sel = apex_inputs->ext_highg_peak_hyst;
	apex_config14.ext_highg_peak_th_sel      = apex_inputs->ext_highg_peak_th;

	status |= inv_imu_write_reg(s, APEX_CONFIG14, 1, (uint8_t *)&apex_config14);
#endif

	status |= inv_imu_switch_off_mclk(s);

	return status;
}

int inv_imu_apex_get_parameters(inv_imu_device_t *s, inv_imu_apex_parameters_t *param)
{
	int             status = 0;
	apex_config0_t  apex_config0;
	apex_config12_t apex_config12;
#if INV_IMU_ACC_HFSR_SUPPORTED
	apex_config14_t apex_config14;
#endif
	struct {
		apex_config2_t  cfg2;
		apex_config3_t  cfg3;
		apex_config4_t  cfg4;
		apex_config5_t  cfg5;
		apex_config9_t  cfg9;
		apex_config10_t cfg10;
		apex_config11_t cfg11;
	} regs;

	status |= inv_imu_read_reg(s, APEX_CONFIG0, 1, (uint8_t *)&apex_config0);
	param->power_save = apex_config0.dmp_power_save_en;

	/* Access continuous config registers (CONFIG2-CONFIG11) */
	status |= inv_imu_read_reg(s, APEX_CONFIG2, sizeof(regs), (uint8_t *)&regs);

	/* APEX_CONFIG2 */
	param->power_save_time = (apex_config2_dmp_power_save_time_t)regs.cfg2.dmp_power_save_time_sel;
	param->tap_max         = (apex_config2_tap_max_t)regs.cfg2.tap_max;
	param->tap_min         = (apex_config2_tap_min_t)regs.cfg2.tap_min;

	/* APEX_CONFIG3 */
	param->tap_tavg = (apex_config3_tap_tavg_t)regs.cfg3.tap_tavg;
	param->tap_tmax = (apex_config3_tap_tmax_t)regs.cfg3.tap_tmax;
	param->tap_tmin = (apex_config3_tap_tmin_t)regs.cfg3.tap_tmin;

	/* APEX_CONFIG4 */
	param->tap_max_peak_tol = (apex_config4_tap_max_peak_tol_t)regs.cfg4.tap_max_peak_tol;
	param->tap_min_jerk_thr = (apex_config4_tap_min_jerk_thr_t)regs.cfg4.tap_min_jerk_thr;

	/* APEX_CONFIG5 */
	param->tilt_wait_time  = (apex_config5_tilt_wait_time_t)regs.cfg5.tilt_wait_time_sel;
	param->lowg_peak_hyst  = (apex_config5_lowg_peak_th_hyst_t)regs.cfg5.lowg_peak_th_hyst_sel;
	param->highg_peak_hyst = (apex_config5_highg_peak_th_hyst_t)regs.cfg5.highg_peak_th_hyst_sel;

	/* APEX_CONFIG9 */
	param->ff_debounce_duration =
	    (apex_config9_ff_debounce_duration_t)regs.cfg9.ff_debounce_duration_sel;
	param->tap_smudge_reject_th =
	    (apex_config9_tap_smudge_reject_thr_t)regs.cfg9.tap_smudge_reject_thr;

	/* APEX_CONFIG10 */
	param->lowg_peak_th    = (apex_config10_lowg_peak_th_t)regs.cfg10.lowg_peak_th_sel;
	param->lowg_samples_th = (apex_config10_lowg_time_th_samples_t)regs.cfg10.lowg_time_th_sel;

	/* APEX_CONFIG11 */
	param->highg_peak_th    = (apex_config11_highg_peak_th_t)regs.cfg11.highg_peak_th_sel;
	param->highg_samples_th = (apex_config11_highg_time_th_samples_t)regs.cfg11.highg_time_th_sel;

	/* APEX_CONFIG12 */
	status |= inv_imu_read_reg(s, APEX_CONFIG12, 1, (uint8_t *)&apex_config12);
	param->ff_max_duration_cm = (apex_config12_ff_max_duration_t)apex_config12.ff_max_duration_sel;
	param->ff_min_duration_cm = (apex_config12_ff_min_duration_t)apex_config12.ff_min_duration_sel;

#if INV_IMU_ACC_HFSR_SUPPORTED
	/* APEX_CONFIG14 */
	status |= inv_imu_read_reg(s, APEX_CONFIG14, 1, (uint8_t *)&apex_config14);
	param->ext_highg_samples_th =
	    (apex_config14_ext_highg_time_th_samples_t)apex_config14.ext_highg_time_th_sel;
	param->ext_highg_peak_hyst =
	    (apex_config14_ext_highg_peak_th_hyst_t)apex_config14.ext_highg_peak_th_hyst_sel;
	param->ext_highg_peak_th =
	    (apex_config14_ext_highg_peak_th_t)apex_config14.ext_highg_peak_th_sel;
#endif

	return status;
}

int inv_imu_apex_set_frequency(inv_imu_device_t *s, const apex_config1_dmp_odr_t frequency)
{
	int            status = 0;
	apex_config1_t apex_config1;

	status |= inv_imu_read_reg(s, APEX_CONFIG1, 1, (uint8_t *)&apex_config1);
	apex_config1.dmp_odr = frequency;
	status |= inv_imu_write_reg(s, APEX_CONFIG1, 1, (uint8_t *)&apex_config1);

	return status;
}

int inv_imu_apex_enable_tap(inv_imu_device_t *s)
{
	int            status = 0;
	apex_config1_t apex_config1;

	status |= inv_imu_apex_start_dmp(s);

	/* Enable TAP */
	status |= inv_imu_read_reg(s, APEX_CONFIG1, 1, (uint8_t *)&apex_config1);
	apex_config1.tap_enable = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, APEX_CONFIG1, 1, (uint8_t *)&apex_config1);

	return status;
}

int inv_imu_apex_disable_tap(inv_imu_device_t *s)
{
	int            status = 0;
	apex_config1_t apex_config1;

	/* Disable TAP */
	status |= inv_imu_read_reg(s, APEX_CONFIG1, 1, (uint8_t *)&apex_config1);
	apex_config1.tap_enable = INV_IMU_DISABLE;
	status |= inv_imu_write_reg(s, APEX_CONFIG1, 1, (uint8_t *)&apex_config1);

	return status;
}

int inv_imu_apex_enable_tilt(inv_imu_device_t *s)
{
	int            status = 0;
	apex_config1_t apex_config1;

	status |= inv_imu_apex_start_dmp(s);

	/* Enable Tilt */
	status |= inv_imu_read_reg(s, APEX_CONFIG1, 1, (uint8_t *)&apex_config1);
	apex_config1.tilt_enable = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, APEX_CONFIG1, 1, (uint8_t *)&apex_config1);

	return status;
}

int inv_imu_apex_disable_tilt(inv_imu_device_t *s)
{
	int            status = 0;
	apex_config1_t apex_config1;

	/* Disable Tilt */
	status |= inv_imu_read_reg(s, APEX_CONFIG1, 1, (uint8_t *)&apex_config1);
	apex_config1.tilt_enable = INV_IMU_DISABLE;
	status |= inv_imu_write_reg(s, APEX_CONFIG1, 1, (uint8_t *)&apex_config1);

	return status;
}

int inv_imu_apex_get_data_tap(inv_imu_device_t *s, inv_imu_apex_tap_event_t *tap_out)
{
	int status = 0;
	struct {
		apex_data0_t dat0;
		apex_data1_t dat1;
		apex_data2_t dat2;
	} apex;

	status = inv_imu_read_reg(s, APEX_DATA0, sizeof(apex), (uint8_t *)&apex);

	tap_out->tap           = apex.dat0.tap_event;
	tap_out->tap_axis      = apex.dat0.tap_axis;
	tap_out->tap_direction = apex.dat0.tap_direction;
	/*
	 * APEX_DATA registers report TAP timings as a number of samples r-shifted by 4. So let's
	 * scale them to get the actual number of samples.
	 */
	tap_out->double_tap_timing = (uint16_t)apex.dat1.double_tap_timing << 4;
	tap_out->triple_tap_timing = (uint16_t)apex.dat2.triple_tap_timing << 4;

	return status;
}

int inv_imu_apex_get_data_free_fall(inv_imu_device_t *s, uint16_t *freefall_duration)
{
	uint8_t data[2];
	int     status = inv_imu_read_reg(s, APEX_DATA4, 2, &data[0]);

	*freefall_duration = (uint16_t)((data[1] << 8) | data[0]);

	return status;
}

int inv_imu_apex_start_dmp(inv_imu_device_t *s)
{
	int status = 0;

	// On first enabling of DMP, reset internal state
	if (!s->dmp_is_on) {
		// Reset SRAM to 0's
		status |= inv_imu_apex_reset_dmp(s);
		if (status)
			return status;
		s->dmp_is_on = 1;
	}

	// Initialize DMP
	status |= inv_imu_apex_resume_dmp(s);

	return status;
}

int inv_imu_apex_resume_dmp(inv_imu_device_t *s)
{
	int            status = 0;
	apex_config0_t apex_config0;
	int            timeout_us = 50000;

	status |= inv_imu_read_reg(s, APEX_CONFIG0, 1, (uint8_t *)&apex_config0);
	apex_config0.dmp_init_en = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, APEX_CONFIG0, 1, (uint8_t *)&apex_config0);

	/* wait to make sure dmp_init_en = 0 */
	while (status == INV_IMU_OK) {
		inv_imu_read_reg(s, APEX_CONFIG0, 1, (uint8_t *)&apex_config0);

		if (apex_config0.dmp_init_en == 0)
			break;

		inv_imu_sleep_us(s, 100);
		timeout_us -= 100;

		if (timeout_us <= 0)
			return INV_IMU_ERROR_TIMEOUT;
	}

	return status;
}

int inv_imu_apex_reset_dmp(inv_imu_device_t *s)
{
	const int      ref_timeout = 5000; /*50 ms*/
	int            status      = 0;
	int            timeout     = ref_timeout;
	apex_config0_t apex_config0;

	status |= inv_imu_switch_on_mclk(s);

	// Reset DMP internal memories
	status |= inv_imu_read_reg(s, APEX_CONFIG0, 1, (uint8_t *)&apex_config0);
	apex_config0.dmp_mem_reset_en = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, APEX_CONFIG0, 1, (uint8_t *)&apex_config0);

	inv_imu_sleep_us(s, 1000);

	// Make sure reset procedure has finished by reading back mem_reset_en bit
	do {
		inv_imu_sleep_us(s, 10);
		status |= inv_imu_read_reg(s, APEX_CONFIG0, 1, (uint8_t *)&apex_config0);
	} while (apex_config0.dmp_mem_reset_en && timeout-- && !status);

	status |= inv_imu_switch_off_mclk(s);

	if (timeout <= 0)
		return INV_IMU_ERROR_TIMEOUT;

	/* Reset `dmp_is_on` since APEX features will have to restart from scratch */
	s->dmp_is_on = 0;

	return status;
}
