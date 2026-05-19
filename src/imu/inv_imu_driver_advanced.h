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

/** @defgroup DriverAdv Driver Advanced
 *  @brief    High-level API for advanced functionalities.
 *  @{
 */

/** @file inv_imu_driver_advanced.h */

#ifndef _INV_IMU_DRIVER_ADVANCED_H_
#define _INV_IMU_DRIVER_ADVANCED_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "imu/inv_imu_driver.h"

#include <stdint.h>
#include <string.h>

/** @brief Maximum buffer size mirrored from FIFO */
#define FIFO_MIRRORING_SIZE 9 * 256 /* max FIFO size = 2.25kB */

/*
 * Driver's structures definitions 
 */

/** @brief Sensor event structure definition */
typedef struct {
	/** Specifies which sensors are available in the event 
	 * (defined by inv_imu_sensor_id_t as a mask)
	 */
	int sensor_mask;

	/** Value of the FIFO timestamp (if FIFO is used) */
	uint16_t timestamp_fsync;

	/** Accel raw data */
	int16_t accel[3];

	/** Gyro raw data */
	int16_t gyro[3];

	/** Temperature raw data */
	int16_t temperature;

	/** High-res portion of the accel raw data (if using high-res mode) */
	int8_t accel_high_res[3];

	/** High-res portion of the accel raw data (if using high-res mode) */
	int8_t gyro_high_res[3];
} inv_imu_sensor_event_t;

/** @brief Definition of extended variables */
typedef struct {
	/** @brief Callback executed when a new sensor event is available.
	 *  @param[in] event  Pointer to the sensor event.
	 */
	void (*sensor_event_cb)(inv_imu_sensor_event_t *event);

	/* Variables related to FSYNC tag */
	fsync_config_ui_sel_t fsync_tag; /**< Keeps track of FSYNC tag in sensor data regs */
} inv_imu_adv_var_t;

/** @brief Sensor identifier enumeration */
typedef enum {
	INV_SENSOR_ACCEL,
	INV_SENSOR_GYRO,
	INV_SENSOR_FSYNC_EVENT,
	INV_SENSOR_TEMPERATURE,
	INV_SENSOR_DMP_TILT,
	INV_SENSOR_DMP_FF,
	INV_SENSOR_DMP_LOWG,
	INV_SENSOR_DMP_HIGHG,
	INV_SENSOR_DMP_TAP,
	INV_SENSOR_DMP_EXT_HIGHG,
	INV_SENSOR_MAX
} inv_imu_sensor_id_t;

/** @brief FIFO configuration structure */
typedef struct {
	/** @brief Basic FIFO configuration */
	inv_imu_fifo_config_t base_conf;

	/** @brief Condition to trig watermark interrupt */
	fifo_config5_fifo_wm_gt_th_t fifo_wm_gt_th;

	/** @brief Enable Timestamp or FSYNC delay in FIFO */
	uint8_t tmst_fsync_en;

	/** @brief FIFO decimation rate */
	fdr_config_fdr_sel_t decim_rate;

} inv_imu_adv_fifo_config_t;

/*
 * API definitions 
 */

/** @brief Initializes device.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_adv_init(inv_imu_device_t *s);

/** @brief Performs a soft reset of the device.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_adv_device_reset(inv_imu_device_t *s);

/** @brief Enable accel in low power mode
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_adv_enable_accel_lp(inv_imu_device_t *s);

/** @brief Enable accel in low noise mode
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_adv_enable_accel_ln(inv_imu_device_t *s);

/** @brief Disable accel.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_adv_disable_accel(inv_imu_device_t *s);

/** @brief Enable gyro in low noise mode.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_adv_enable_gyro_ln(inv_imu_device_t *s);

/** @brief Disable gyro
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_adv_disable_gyro(inv_imu_device_t *s);

/** @brief Configures FSYNC tag in sensor registers
 *  @param[in] s           Pointer to device.
 *  @param[in] sensor_tag  Indicates which sensor data register LSB should be set when a FSYNC event is detected.
 *  @return                0 on success, negative value on error.
 */
int inv_imu_adv_configure_fsync_ap_tag(inv_imu_device_t *s, fsync_config_ui_sel_t sensor_tag);

/** @brief Enable fsync tagging functionnality.
 *  In details it:
 *     - enables fsync
 *     - enables timestamp to registers. Once fysnc is enabled fsync counter is pushed to 
 *       fifo instead of timestamp. So timestamp is made available in registers. Note that 
 *       this increase power consumption.
 *     - enables fsync related interrupt
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_adv_enable_fsync(inv_imu_device_t *s);

/** @brief Disable fsync tagging functionnality.
 *  In details it:
 *     - disables fsync
 *     - disables timestamp to registers. Once fysnc is disabled  timestamp is pushed to fifo 
 *        instead of fsync counter. So in order to decrease power consumption, timestamp is no 
 *        more available in registers.
 *     - disables fsync related interrupt
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_adv_disable_fsync(inv_imu_device_t *s);

/** @brief Read all registers containing data (temperature, accelerometer and gyroscope). 
 *         It will then call `sensor_event_cb` function provided 
 *         in the `inv_imu_device_t` for each packet.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_adv_get_data_from_registers(inv_imu_device_t *s);

/** @brief reset IMU fifo
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_adv_reset_fifo(inv_imu_device_t *s);

/** @brief Retrieve FIFO configuration.
 *  @param[in] s     Pointer to device.
 *  @param[in] conf  Structure that will be filled with current configuration.
 *  @return          0 on success, negative value on error.
 */
int inv_imu_adv_get_fifo_config(inv_imu_device_t *s, inv_imu_adv_fifo_config_t *conf);

/** @brief Set FIFO configuration.
 *  @param[in] s     Pointer to device.
 *  @param[in] conf  Structure containing the requested configuration.
 *  @return          0 on success, negative value on error.
 */
int inv_imu_adv_set_fifo_config(inv_imu_device_t *s, const inv_imu_adv_fifo_config_t *conf);

/** @brief Read all available packets from the FIFO. 
 *  @param[in] s            Pointer to device.
 *  @param[out] fifo_data   Pointer to FIFO data buffer.
 *  @param[out] fifo_count  Number of packet read in FIFO.
 *  @return                 0 on success, negative value on error.
 */
int inv_imu_adv_get_data_from_fifo(inv_imu_device_t *s, uint8_t fifo_data[FIFO_MIRRORING_SIZE],
                                   uint16_t *fifo_count);

/** @brief Parse packets from FIFO buffer. For each packet function builds a
 *         sensor event containing packet data and validity information. Then it calls 
 *         sensor_event_cb funtion passed in parameter of inv_imu_init function for each 
 *         packet.
 *  @param[in] s           Pointer to device.
 *  @param[in] fifo_data   Pointer to FIFO data buffer.
 *  @param[in] fifo_count  Number of packet read in FIFO.
 *  @return                0 on success, negative value on error.
 */
int inv_imu_adv_parse_fifo_data(inv_imu_device_t *s, const uint8_t fifo_data[FIFO_MIRRORING_SIZE],
                                const uint16_t fifo_count);

/** @brief Access accel full scale range
 *  @param[in] s           Pointer to device.
 *  @param[out] accel_fsr  Current full scale range.
 *  @return                0 on success, negative value on error.
 */
int inv_imu_adv_get_accel_fsr(inv_imu_device_t *s, accel_config0_fs_sel_t *accel_fsr);

/** @brief Access gyro full scale range
 *  @param[in] s          Pointer to device.
 *  @param[out] gyro_fsr  Current full scale range.
 *  @return               0 on success, negative value on error.
 */
int inv_imu_adv_get_gyro_fsr(inv_imu_device_t *s, gyro_config0_fs_sel_t *gyro_fsr);

/** @brief Set timestamp resolution
 *  @param[in] s                Pointer to device.
 *  @param[in] timestamp_resol  Requested timestamp resolution
 *  @return                     0 on success, negative value on error.
 */
int inv_imu_adv_set_timestamp_resolution(inv_imu_device_t *         s,
                                         const tmst_config1_resol_t timestamp_resol);

/** @brief Get timestamp resolution.
 *  @param[in] s  Pointer to device.
 *  @return       Timestamp resolution in us, negative value on error
 */
uint32_t inv_imu_adv_get_timestamp_resolution_us(inv_imu_device_t *s);

/** @brief  Enable Wake On Motion.
 *  @param[in] s         Pointer to device. 
 *  @param[in] wom_x_th  Threshold for X axis with 1g/256 resolution (wom_x_th = mg * 256 / 1000).
 *  @param[in] wom_y_th  Threshold for Y axis with 1g/256 resolution (wom_y_th = mg * 256 / 1000).
 *  @param[in] wom_z_th  Threshold for Z axis with 1g/256 resolution (wom_z_th = mg * 256 / 1000).
 *  @param[in] wom_int   Mode used to generate interrupt (AND/OR).
 *  @param[in] wom_dur   Number of overthreshold events to wait before generating interrupt.
 *  @return              0 on success, negative value on error.
 */
int inv_imu_adv_configure_wom(inv_imu_device_t *s, const uint8_t wom_x_th, const uint8_t wom_y_th,
                              const uint8_t wom_z_th, wom_config_wom_int_mode_t wom_int,
                              wom_config_wom_int_dur_t wom_dur);

/** @brief  Enable Wake On Motion.
 *  note : WoM requests to have the accelerometer enabled to work. 
 *  As a consequence Fifo water-mark interrupt is disabled to only trigger WoM interrupts.
 *  To have good performance, it's recommended to set accelerometer ODR (Output Data Rate) to 20ms
 *  and the accelerometer in Low Power Mode.
 *  @param[in] s  Pointer to device. 
   @return        0 on success, negative value on error.
 */
int inv_imu_adv_enable_wom(inv_imu_device_t *s);

/** @brief  Disable Wake On Motion.
 *  note : Fifo water-mark interrupt is re-enabled when WoM is disabled.
 *  @param[in] s  Pointer to device. 
 *  @return       0 on success, negative value on error.
 */
int inv_imu_adv_disable_wom(inv_imu_device_t *s);

/** @brief Set the UI endianness and set the inv_device endianness field
 *  @param[in] s           Pointer to device. 
 *  @param[in] endianness  Requested endianness value. 
 *  @return                0 on success, negative value on error.
 */
int inv_imu_adv_set_endianness(inv_imu_device_t *s, intf_config0_data_endian_t endianness);

/** @brief Configure SPI slew-rate.
 *  @param[in] s          Pointer to device.
 *  @param[in] slew_rate  Requested slew-rate.
 *  @return               0 on success, negative value on error.
 */
int inv_imu_adv_set_spi_slew_rate(inv_imu_device_t *                  s,
                                  const drive_config3_spi_slew_rate_t slew_rate);

/** @brief Configure INT pulse duration.
 *  @param[in] s            Pointer to device.
 *  @param[in] pulse_dur    Pulse Duration (100 us or 8 us).
 *  @return          0 on success, negative value on error.
 */
int inv_imu_adv_set_int_pulse(inv_imu_device_t *                     s,
                              const int_config1_int_pulse_duration_t pulse_dur);

#if INV_IMU_HIGH_ODR_SUPPORTED
/** @brief Enable accel OIS datapath (6.4kHz).
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_adv_enable_accel_ois(inv_imu_device_t *s);

/** @brief Disable accel OIS datapath (6.4kHz).
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_adv_disable_accel_ois(inv_imu_device_t *s);

/** @brief Enable gyro OIS datapath (6.4kHz).
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_adv_enable_gyro_ois(inv_imu_device_t *s);

/** @brief Disable gyro OIS datapath (6.4kHz).
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_adv_disable_gyro_ois(inv_imu_device_t *s);

/** @brief Read all registers containing data (temperature, accelerometer and gyroscope). 
 *         It will then call `sensor_event_cb` function provided in the `inv_imu_device_t`.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_adv_get_ois_data_from_registers(inv_imu_device_t *s);

/** @brief Enable OIS on AP (virtual mode)
 *  @return       0 on success, negative value on error.
 */
int inv_imu_adv_enable_ois_virtualization(inv_imu_device_t *s);

/** @brief Diasble OIS on AP (virtual mode)
 *  @return       0 on success, negative value on error.
 */
int inv_imu_adv_disable_ois_virtualization(inv_imu_device_t *s);
#endif

#ifdef __cplusplus
}
#endif

#endif /* _INV_IMU_DRIVER_ADVANCED_H_ */

/** @} */
