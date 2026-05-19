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

/** @defgroup Transport Transport
 *  @brief    Abstraction layer to access device's registers
 *  @{
 */

/** @file  inv_imu_transport.h */

#ifndef _INV_IMU_TRANSPORT_H_
#define _INV_IMU_TRANSPORT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** @brief Function pointer to read register(s).
 *  @param[in] context  Pointer to context.
 *  @param[in] reg      Register address to be read.
 *  @param[out] buf     Output data from the register.
 *  @param[in] len      Number of byte to be read.
 *  @return             0 on success, negative value on error.
 */
typedef int (*inv_imu_read_reg_t)(void *context, uint8_t reg, uint8_t *buf, uint32_t len);

/** @brief Function pointer to write register(s).
 *  @param[in] context  Pointer to context.
 *  @param[in] reg      Register address to be written.
 *  @param[in] buf      Input data to write.
 *  @param[in] len      Number of byte to be written.
 *  @return             0 on success, negative value on error.
 */
typedef int (*inv_imu_write_reg_t)(void *context, uint8_t reg, const uint8_t *buf, uint32_t len);

/* Available serial interface type. */
#define UI_I2C     0      /**< I2C interface. */
#define UI_SPI4    1      /**< 4-wire SPI interface. */
#define UI_SPI3    2      /**< 3-wire SPI interface. */
#define UI_I3C     3      /**< I3C interface with pin interrupts (default). */
#define UI_I3C_IRQ UI_I3C
#define UI_I3C_IBI 4      /**< I3C interface with IBI interrupts. */

/** @brief Serif type definition.
 *  @deprecated Kept for retrocompatibility. Replaced with `uint32_t` type
 *              in `inv_imu_transport_t` struct.
 */
typedef uint32_t inv_imu_serif_type_t;

/** @brief Structure dedicated to transport layer transport interface. */
typedef struct {
	/* Serial interface variables (should be initialized by application) */
	void *              context;
	inv_imu_read_reg_t  read_reg; /**< Function pointer to read register(s). */
	inv_imu_write_reg_t write_reg; /**< Function pointer to write register(s). */
	uint32_t            serif_type; /**< Serial interface type. */

	/** @brief Callback to sleep function.
	 *  @param[in] us  Time to sleep in microseconds.
	 */
	void (*sleep_us)(uint32_t us);

	/** @brief Driver calls this function before doing the first serial transaction to IMU during IMU
	 *         init and after SW reset. That's the correct place to finalize serial interface
	 *         configuration if needed.
	 *  @param[in] us  Time to sleep in microseconds.
	 *  @return 0 on success, negative value on error.
	 */
	int (*configure_host_interface)(uint32_t serif_type);

	/** Contains mirrored values of some IP registers. */
	struct register_cache {
		uint8_t pwr_mgmt0_reg;
		uint8_t gyro_config0_reg;
		uint8_t accel_config0_reg;
		uint8_t tmst_config1_reg;
	} register_cache;

	/** Internal counter for MCLK requests. */
	uint8_t need_mclk_cnt;
} inv_imu_transport_t;

/** @brief Init cache variable.
 *  @param[in] t  Pointer to transport (as void * so it can be called from any module).
 *  @return       0 on success, negative value on error.
 */
int inv_imu_init_transport(void *t);

/** @brief Reads data from a register on IMU.
 *  @param[in] t     Pointer to transport (as void * so it can be called from any module).
 *  @param[in] reg   Register address to be read.
 *  @param[in] len   Number of byte to be read.
 *  @param[out] buf  Output data from the register.
 *  @return          0 on success, negative value on error.
 */
int inv_imu_read_reg(void *t, uint32_t reg, uint32_t len, uint8_t *buf);

/** @brief Writes data to a register on IMU.
 *  @param[in] t    Pointer to transport (as void * so it can be called from any module).
 *  @param[in] reg  Register address to be written.
 *  @param[in] len  Number of byte to be written.
 *  @param[in] buf  Input data to write.
 *  @return         0 on success, negative value on error.
 */
int inv_imu_write_reg(void *t, uint32_t reg, uint32_t len, const uint8_t *buf);

/** @brief Enable MCLK.
 *  @param[in] t  Pointer to transport (as void * so it can be called from any module).
 *  @return       0 on success, negative value on error.
 */
int inv_imu_switch_on_mclk(void *t);

/** @brief Disable MCLK.
 *  @param[in] t  Pointer to transport (as void * so it can be called from any module).
 *  @return       0 on success, negative value on error.
 */
int inv_imu_switch_off_mclk(void *t);

#ifdef __cplusplus
}
#endif

#endif /* _INV_IMU_TRANSPORT_H_ */

/** @} */
