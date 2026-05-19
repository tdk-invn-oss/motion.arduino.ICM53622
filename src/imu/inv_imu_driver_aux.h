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

/** @defgroup DriverAUX AUX driver
 *  @brief Basic functions to drive AUX interface of the device
 *  @{
 */

/** @file inv_imu_driver_aux.h */

#ifndef _INV_IMU_DRIVER_OIS_H_
#define _INV_IMU_DRIVER_OIS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "imu/inv_imu_defs.h"
#include "imu/inv_imu_transport.h"

/** Interrupt definition */
typedef struct {
	uint8_t INT_AUX_I3C_PROT;
	uint8_t INT_OIS_FSYNC;
	uint8_t INT_OIS_DRDY;
	uint8_t INT_OIS_AGC_RDY;
} inv_imu_aux_int_state_t;

/** @brief Enable accel on AUX interface.
 *  @param[in] t   Pointer to transport.
 *  @return        0 on success, negative value on error.
 */
int inv_imu_aux_enable_accel(inv_imu_transport_t *t);

/** @brief Disable accel for OIS interface.
 *  @param[in] t   Pointer to transport.
 *  @return        0 on success, negative value on error.
 */
int inv_imu_aux_disable_accel(inv_imu_transport_t *t);

/** @brief Set accel full scale range for OIS interface.
 *  @param[in] t    Pointer to transport.
 *  @param[in] fsr  Requested full scale range.
 *  @return         0 on success, negative value on error.
 */
int inv_imu_aux_set_accel_fsr(inv_imu_transport_t *t, ois_config1_accel_fs_sel_t fsr);

/** @brief Get accel full scale range for OIS interface.
 *  @param[in] t     Pointer to transport.
 *  @param[out] fsr  Current full scale range for the OIS interface.
 *  @return          0 on success, negative value on error.
 */
int inv_imu_aux_get_accel_fsr(inv_imu_transport_t *t, ois_config1_accel_fs_sel_t *fsr);

/** @brief Set accel full scale range for OIS interface.
 *  @param[in] t   Pointer to transport.
 *  @param[in] bw  Requested filtering bandwidth.
 *  @return        0 on success, negative value on error.
 */
int inv_imu_aux_set_accel_bw(inv_imu_transport_t *t, ois_config3_accel_filt_bw_ind_t bw);

/** @brief Get accel full scale range for OIS interface.
 *  @param[in] t    Pointer to transport.
 *  @param[out] bw  Current filtering bandwidth for the AUX interface.
 *  @return         0 on success, negative value on error.
 */
int inv_imu_aux_get_accel_bw(inv_imu_transport_t *t, ois_config3_accel_filt_bw_ind_t *bw);

/** @brief Enable gyro on AUX interface.
 *  @param[in] t   Pointer to transport.
 *  @return        0 on success, negative value on error.
 */
int inv_imu_aux_enable_gyro(inv_imu_transport_t *t);

/** @brief Disable gyro on AUX interface.
 *  @param[in] t   Pointer to transport.
 *  @return        0 on success, negative value on error.
 */
int inv_imu_aux_disable_gyro(inv_imu_transport_t *t);

/** @brief Set gyro full scale range for OIS interface.
 *  @param[in] t    Pointer to transport.
 *  @param[in] fsr  Requested full scale range.
 *  @return         0 on success, negative value on error.
 */
int inv_imu_aux_set_gyro_fsr(inv_imu_transport_t *t, ois_config1_gyro_fs_sel_t fsr);

/** @brief Get gyro full scale range for OIS interface.
 *  @param[in] t     Pointer to transport.
 *  @param[out] fsr  Current full scale range for the OIS interface.
 *  @return          0 on success, negative value on error.
 */
int inv_imu_aux_get_gyro_fsr(inv_imu_transport_t *t, ois_config1_gyro_fs_sel_t *fsr);

/** @brief Set gyro full scale range for OIS interface.
 *  @param[in] t   Pointer to transport.
 *  @param[in] bw  Requested filtering bandwidth.
 *  @return        0 on success, negative value on error.
 */
int inv_imu_aux_set_gyro_bw(inv_imu_transport_t *t, ois_config3_gyro_filt_bw_ind_t bw);

/** @brief Get gyro full scale range for OIS interface.
 *  @param[in] t    Pointer to transport.
 *  @param[out] bw  Current filtering bandwidth for the AUX interface.
 *  @return         0 on success, negative value on error.
 */
int inv_imu_aux_get_gyro_bw(inv_imu_transport_t *t, ois_config3_gyro_filt_bw_ind_t *bw);

/** @brief Configure interrupts source.
 *  @param[in] t    Pointer to device.
 *  @param[in] num  Interrupt number
 *  @param[in] it   State of each interrupt
 *  @return         0 on success, negative value on error.
 */
int inv_imu_aux_set_config_int(inv_imu_transport_t *t, const inv_imu_int_num_t num,
                               const inv_imu_aux_int_state_t *it);

/** @brief Retrieve interrupts configuration.
 *  @param[in] t    Pointer to device.
 *  @param[in] num  Interrupt number
 *  @param[out] it  State of each interrupt
 *  @return         0 on success, negative value on error.
 */
int inv_imu_aux_get_config_int(inv_imu_transport_t *t, const inv_imu_int_num_t num,
                               inv_imu_aux_int_state_t *it);

/** @brief Enable fsync tagging functionality on OIS port.
 *  @param[in] t  Pointer to transport.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_aux_enable_fsync(inv_imu_transport_t *t);

/** @brief Disable fsync tagging functionality on OIS port.
 *  @param[in] t  Pointer to transport.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_aux_disable_fsync(inv_imu_transport_t *t);

/** @brief Get OIS interface register data.
 *  @param[in] t      Pointer to transport.
 *  @param[out] data  Current accel, gyro and temperature data from the OIS data registers.
 *  @return           0 on success, negative value on error.
 */
int inv_imu_aux_get_register_data(inv_imu_transport_t *t, inv_imu_sensor_data_t *data);

#ifdef __cplusplus
}
#endif

#endif /* _INV_IMU_DRIVER_OIS_H_ */

/** @} */
