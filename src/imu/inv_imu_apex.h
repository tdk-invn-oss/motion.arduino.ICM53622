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

/** @defgroup Apex APEX
 *  @brief High-level functions to drive APEX features
 *  @{
 */

/** @file inv_imu_apex.h */

#ifndef _INV_IMU_APEX_H_
#define _INV_IMU_APEX_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "imu/inv_imu_driver.h"

/** IMU APEX inputs parameters definition */
typedef struct {
	apex_config5_tilt_wait_time_t         tilt_wait_time;
	apex_config2_dmp_power_save_time_t    power_save_time;
	uint32_t                              power_save;
	apex_config9_ff_debounce_duration_t   ff_debounce_duration;
	apex_config12_ff_max_duration_t       ff_max_duration_cm;
	apex_config12_ff_min_duration_t       ff_min_duration_cm;
	apex_config10_lowg_peak_th_t          lowg_peak_th;
	apex_config5_lowg_peak_th_hyst_t      lowg_peak_hyst;
	apex_config10_lowg_time_th_samples_t  lowg_samples_th;
	apex_config11_highg_peak_th_t         highg_peak_th;
	apex_config5_highg_peak_th_hyst_t     highg_peak_hyst;
	apex_config11_highg_time_th_samples_t highg_samples_th;
	apex_config2_tap_max_t                tap_max;
	apex_config2_tap_min_t                tap_min;
	apex_config3_tap_tavg_t               tap_tavg;
	apex_config3_tap_tmax_t               tap_tmax;
	apex_config3_tap_tmin_t               tap_tmin;
	apex_config4_tap_max_peak_tol_t       tap_max_peak_tol;
	apex_config4_tap_min_jerk_thr_t       tap_min_jerk_thr;
	apex_config9_tap_smudge_reject_thr_t  tap_smudge_reject_th;
#if INV_IMU_ACC_HFSR_SUPPORTED
	apex_config14_ext_highg_time_th_samples_t ext_highg_samples_th;
	apex_config14_ext_highg_peak_th_hyst_t    ext_highg_peak_hyst;
	apex_config14_ext_highg_peak_th_t         ext_highg_peak_th;
#endif
} inv_imu_apex_parameters_t;

/** APEX TAP outputs */
typedef struct {
	uint8_t  tap; /**< 0: no tap, 1: single tap, 2: double tap, 3: triple tap */
	uint8_t  tap_axis; /**< 0: ax, 1: ay, 2: az */
	uint8_t  tap_direction; /**< 0: positive, 1: negative */
	uint16_t double_tap_timing; /**< the sample count of second pulse in detection window */
	uint16_t triple_tap_timing; /**< the sample count of third pulse in detection window */
} inv_imu_apex_tap_event_t;

/** @brief Start DMP for APEX algorithms and self-test.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_apex_start_dmp(inv_imu_device_t *s);

/** @brief Resume DMP operations.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_apex_resume_dmp(inv_imu_device_t *s);

/** @brief Reset DMP for APEX algorithms and self-test.
 *  @param[in] s           Pointer to device.
 *  @return                0 on success, negative value on error.
 */
int inv_imu_apex_reset_dmp(inv_imu_device_t *s);

/** @brief Enable Free Fall.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_apex_enable_ff(inv_imu_device_t *s);

/** @brief Disable Free Fall.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_apex_disable_ff(inv_imu_device_t *s);

#if INV_IMU_ACC_HFSR_SUPPORTED
/** @brief Enable extended highg.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_apex_enable_ext_highg(inv_imu_device_t *s);

/** @brief Disable extended highg.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_apex_disable_ext_highg(inv_imu_device_t *s);
#endif

/** @brief Fill the APEX parameters structure with all the default parameters for APEX algorithms.
 *  @param[in] s             Pointer to device.
 *  @param[out] apex_inputs  Default input parameters.
 *  @return                  0 on success, negative value on error.
 */
int inv_imu_apex_init_parameters_struct(inv_imu_device_t *         s,
                                        inv_imu_apex_parameters_t *apex_inputs);

/** @brief Configures DMP parameters for APEX algorithms. 
 *  @param[in] s            Pointer to device.
 *  @param[in] apex_inputs  The requested input parameters.
 *  @return                 0 on success, negative value on error.
 *  @warning APEX inputs can't change on the fly, this should be called before enabling 
 *           any APEX features.
 *  @warning This API can't be called twice within 10 ms.
 */
int inv_imu_apex_configure_parameters(inv_imu_device_t *               s,
                                      const inv_imu_apex_parameters_t *apex_inputs);

/** @brief Returns current DMP parameters for APEX algorithms.
 *  @param[in] s             Pointer to device.
 *  @param[out] apex_params  The current parameter, fetched from registers.
 *  @return                  0 on success, negative value on error.
 */
int inv_imu_apex_get_parameters(inv_imu_device_t *s, inv_imu_apex_parameters_t *apex_params);

/** @brief Configure DMP Output Data Rate for APEX algorithms.
 *  @param[in] s          Pointer to device.
 *  @param[in] frequency  The requested frequency.
 *  @return               0 on success, negative value on error.
 *  @warning Accel frequency must be higher or equal to DMP frequency.
 */
int inv_imu_apex_set_frequency(inv_imu_device_t *s, const apex_config1_dmp_odr_t frequency);

/** @brief Enable APEX algorithm TAP.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_apex_enable_tap(inv_imu_device_t *s);

/** @brief Disable APEX algorithm TAP.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_apex_disable_tap(inv_imu_device_t *s);

/** @brief Enable APEX algorithm Tilt.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_apex_enable_tilt(inv_imu_device_t *s);

/** @brief Disable APEX algorithm Tilt.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_apex_disable_tilt(inv_imu_device_t *s);

/** @brief  Retrieve APEX TAP outputs and format them
 *  @param[in] s         Pointer to device.
 *  @param[out] tap_out  TAP event, axis and direction.
 *  @return              0 on success, negative value on error.
 */
int inv_imu_apex_get_data_tap(inv_imu_device_t *s, inv_imu_apex_tap_event_t *tap_out);

/** @brief  Retrieve APEX free fall outputs and format them
 *  @param[in] s                   Pointer to device.
 *  @param[out] freefall_duration  Free fall duration in number of sample.
 *  @return                        0 on success, negative value on error.
 */
int inv_imu_apex_get_data_free_fall(inv_imu_device_t *s, uint16_t *freefall_duration);

#ifdef __cplusplus
}
#endif

#endif /* _INV_IMU_APEX_H_ */

/** @} */
