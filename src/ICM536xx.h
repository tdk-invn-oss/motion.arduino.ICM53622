/*
 *
 * Copyright (c) [2026] by InvenSense, Inc.
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
 
#ifndef ICM536xx_H
#define ICM536xx_H

#include "Arduino.h"
#include "SPI.h"
#include "Wire.h"
#include "Invn/platform_define.h"

extern "C" {
#include "Invn/InvError.h"
#include "imu/inv_imu_driver_advanced.h"
#include "imu/inv_imu_apex.h"
}

enum {
  ICM536XX_APEX_TILT=0,
  ICM536XX_APEX_TAP,
  ICM536XX_APEX_FF,
  ICM536XX_APEX_LOWG,
  ICM536XX_APEX_HIGHG,
  ICM536XX_APEX_MAX
};

enum {
  APEX_ERROR_EVENT        = -1,
  APEX_NO_EVENT           = 0,
  APEX_EVENT_DETECTED     = 1,
  APEX_B2S_DETECTED       = 1,
  APEX_B2S_REV_DETECTED   = 2,
  APEX_R2W_DETECTED       = 1,
  APEX_R2W_SLEEP_DETECTED = 2,
  APEX_MOTION_DETECTED    = 1,
  APEX_NOMOTION_DETECTED  = 2,
  APEX_FLAT_DETECTED      = 1,
  APEX_NOFLAT_DETECTED    = 2
};

enum {
	STEP_UNKNOWN = 0,
	STEP_WALK    = 1,
	STEP_RUN     = 2,
};


// This defines the handler called when retrieving a sample from the FIFO
// typedef void (*ICM536xx_sensor_event_cb)(inv_imu_sensor_data_t *event);
// This defines the handler called when receiving an irq
typedef void (*ICM536xx_irq_handler)(void);

class ICM536xx {
  public:
    ICM536xx(TwoWire &i2c,bool address_lsb, uint32_t freq);
    ICM536xx(TwoWire &i2c,bool address_lsb);
    ICM536xx(SPIClass &spi,uint8_t chip_select_id, uint32_t freq);
    ICM536xx(SPIClass &spi,uint8_t chip_select_id);
    int begin();
    int startAccel(uint16_t odr, uint16_t fsr);
    int startGyro(uint16_t odr, uint16_t fsr);
    int getDataFromRegisters(inv_imu_sensor_data_t& data);
    int enableFifoInterrupt(uint8_t intpin, bool accel, bool gyro, ICM536xx_irq_handler handler, uint8_t fifo_watermark);
    int getDataFromFifo(inv_imu_fifo_data_t& data);
    int stopAccel(void);
    int stopGyro(void);

    int startAllApex(uint8_t intpin, ICM536xx_irq_handler handler);
    int startApex(int apex_type, uint8_t intpin, ICM536xx_irq_handler handler);
    int getApex_data(int apex_type, uint32_t& p1, uint32_t& p2, uint32_t& p3);
    int getApex_data(int apex_type)
    {
      uint32_t dummy1, dummy2, dummy3;
      if(apex_type != ICM536XX_APEX_TAP && apex_type != ICM536XX_APEX_FF)
        return getApex_data(apex_type, dummy1, dummy2, dummy3);
      else
        return -1;
    }
    int getApex_data(int apex_type, uint32_t& p1)
    {
      uint32_t dummy1, dummy2;
      if(apex_type != ICM536XX_APEX_TAP)
        return getApex_data(apex_type, p1, dummy1, dummy2);
      else
        return -1;
    }

    int startWakeOnMotion(uint8_t intpin, ICM536xx_irq_handler handler);
    int getWom(bool& x, bool& y, bool& z);

    int updateApex(void);
    int setApexInterrupt(uint8_t intpin, ICM536xx_irq_handler handler);
    int readWhoami(void);
    int configure_and_enable_edmp_algo(void);

    inv_imu_int_state_t 	 int_status;
  	TwoWire *i2c;
	uint8_t i2c_address;
	SPIClass *spi;
	uint8_t chip_select_id;

  protected:
    inv_imu_device_t icm_driver;
    accel_config0_odr_t accel_freq_to_param(uint16_t accel_freq_hz);
    gyro_config0_odr_t gyro_freq_to_param(uint16_t gyro_freq_hz);
    accel_config0_fs_sel_t accel_fsr_g_to_param(uint16_t accel_fsr_g);
    gyro_config0_fs_sel_t gyro_fsr_dps_to_param(uint16_t gyro_fsr_dps);

    int setup_irq(uint8_t intpin, ICM536xx_irq_handler handler);
    bool apex_enable[ICM536XX_APEX_MAX];
    uint64_t current_odr_us;
    uint32_t step_cnt_ovflw;
};

#endif // ICM536xx_H
