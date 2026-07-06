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
 
#include "Arduino.h"
#include "ICM536xx.h"

static int i2c_write(void *context, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen);
static int i2c_read(void *context, uint8_t reg, uint8_t * rbuffer, uint32_t rlen);
static int spi_write(void *context, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen);
static int spi_read(void *context, uint8_t reg, uint8_t * rbuffer, uint32_t rlen);

static void sleep_us(uint32_t us);

// i2c and SPI interfaces are used from C driver callbacks, without any knowledge of the object
// As they are declared as static, they will be overriden each time a new ICM536xx object is created
// i2c
#define I2C_DEFAULT_CLOCK 400000
#define I2C_MAX_CLOCK 1000000
#define ICM536xx_I2C_ADDRESS 0x68
#define ARDUINO_I2C_BUFFER_LENGTH 32
// spi
#define SPI_READ 0x80
#define SPI_DEFAULT_CLOCK 12000000
#define SPI_MAX_CLOCK 24000000

// i2c/spi clock frequency
static uint32_t clk_freq = 0;

/* Initial WOM threshold to be applied to IMU in mg */
#define WOM_THRESHOLD_INITIAL_MG 200
/* WOM threshold to be applied to IMU in mg if low threshold is requested*/
#define WOM_THRESHOLD_LOW_MG 24

// ICM536xx constructor for I2c interface
ICM536xx::ICM536xx(TwoWire &i2c_ref,bool lsb, uint32_t freq) {
  i2c = &i2c_ref; 
  i2c_address = ICM536xx_I2C_ADDRESS | (lsb ? 0x1 : 0);
  if ((freq <= I2C_MAX_CLOCK) && (freq >= 100000))
  {
    clk_freq = freq;
  } else {
    clk_freq = I2C_DEFAULT_CLOCK;
  }
}

// ICM536xx constructor for I2c interface, default frequency
ICM536xx::ICM536xx(TwoWire &i2c_ref,bool lsb) {
  i2c = &i2c_ref; 
  i2c_address = ICM536xx_I2C_ADDRESS | (lsb ? 0x1 : 0);
  clk_freq = I2C_DEFAULT_CLOCK;
}

// ICM536xx constructor for spi interface
ICM536xx::ICM536xx(SPIClass &spi_ref,uint8_t cs_id, uint32_t freq) {
  spi = &spi_ref;
  chip_select_id = cs_id; 
  if ((freq <= SPI_MAX_CLOCK) && (freq >= 100000))
  {
    clk_freq = freq;
  } else {
    clk_freq = SPI_DEFAULT_CLOCK;
  }
}

// ICM536xx constructor for spi interface, default frequency
ICM536xx::ICM536xx(SPIClass &spi_ref,uint8_t cs_id) {
  spi = &spi_ref;
  chip_select_id = cs_id; 
  clk_freq = SPI_DEFAULT_CLOCK;
}

/* starts communication with the ICM536xx */
int ICM536xx::begin() {
  int rc = 0;
  uint8_t who_am_i;

  if (i2c != NULL) {
    i2c->begin();
    i2c->setClock(clk_freq);
    icm_driver.transport.serif_type = UI_I2C;
    icm_driver.transport.read_reg  = i2c_read;
    icm_driver.transport.write_reg = i2c_write;
  } else {
    spi->begin();
    pinMode(chip_select_id,OUTPUT);
    digitalWrite(chip_select_id,HIGH);
    icm_driver.transport.serif_type = UI_SPI4;
    icm_driver.transport.read_reg  = spi_read;
    icm_driver.transport.write_reg = spi_write;
  }

  icm_driver.transport.context	= (void *)this;
  icm_driver.transport.sleep_us = sleep_us;

  /* Disable APEX features */
  memset(apex_enable, 0x00, ICM536XX_APEX_MAX);

  sleep_us(3000);

  return inv_imu_adv_init(&icm_driver);
}

int ICM536xx::startAccel(uint16_t odr, uint16_t fsr) {
  int rc = 0;

  if(fsr)
    rc |= inv_imu_set_accel_fsr(&icm_driver, accel_fsr_g_to_param(fsr));
  rc |= inv_imu_set_accel_frequency(&icm_driver, accel_freq_to_param(odr));
  rc |= inv_imu_set_accel_mode(&icm_driver, PWR_MGMT0_ACCEL_MODE_LN);
  return rc;
}

int ICM536xx::startGyro(uint16_t odr, uint16_t fsr) {
  int rc = 0;

  if(fsr)
    rc |= inv_imu_set_gyro_fsr(&icm_driver, gyro_fsr_dps_to_param(fsr));
  rc |= inv_imu_set_gyro_frequency(&icm_driver, gyro_freq_to_param(odr));
  rc |= inv_imu_set_gyro_mode(&icm_driver, PWR_MGMT0_GYRO_MODE_LN);
  return rc;
}

int ICM536xx::stopAccel(void) {
  return inv_imu_set_accel_mode(&icm_driver, PWR_MGMT0_ACCEL_MODE_OFF);
}

int ICM536xx::stopGyro(void) {
  return inv_imu_set_gyro_mode(&icm_driver, PWR_MGMT0_GYRO_MODE_OFF);
}

int ICM536xx::getDataFromRegisters(inv_imu_sensor_data_t& data) {
    return inv_imu_get_register_data(&icm_driver, &data);
}

int ICM536xx::readWhoami(void) {
	uint8_t data;
    inv_imu_get_who_am_i(&icm_driver, &data);

	return data;
}

int ICM536xx::setup_irq(uint8_t intpin, ICM536xx_irq_handler handler)
{
  int rc = 0;
  inv_imu_int_state_t it_conf;
  const inv_imu_int_pin_config_t it_pins = {
    .int_polarity=INT_CONFIG_INTX_POLARITY_HIGH,
    .int_mode=INT_CONFIG_INTX_MODE_PULSED,
    .int_drive=INT_CONFIG_INTX_DRIVE_CIRCUIT_PP
  };
  memset(&it_conf, INV_IMU_DISABLE, sizeof(it_conf));
  it_conf.INV_FIFO_THS = INV_IMU_ENABLE;
  pinMode(intpin,INPUT);
  rc |= inv_imu_set_pin_config_int(&icm_driver, INV_IMU_INT1, &it_pins);
  rc |= inv_imu_set_config_int(&icm_driver, INV_IMU_INT1, &it_conf);
  attachInterrupt(intpin,handler,HIGH);
  return rc;
}

int ICM536xx::enableFifoInterrupt(uint8_t intpin, bool accel, bool gyro, ICM536xx_irq_handler handler, uint8_t fifo_watermark) {
  int rc = 0;
  inv_imu_int_state_t it_conf;
  const inv_imu_fifo_config_t fifo_config = {
    .fifo_en=INV_IMU_ENABLE,
    .gyro_en=gyro,
    .accel_en=accel,
    .hires_en=false,
    .fifo_wm_th=fifo_watermark,
    .fifo_mode=FIFO_CONFIG1_FIFO_MODE_SNAPSHOT
  };

  if(handler == NULL) {
    return -1;
  }

  rc |= inv_imu_set_fifo_config(&icm_driver, &fifo_config);

  rc |= setup_irq(intpin, handler);
  return rc;
}


int ICM536xx::getDataFromFifo(inv_imu_fifo_data_t& data) {
  return inv_imu_get_fifo_frame(&icm_driver,&data);
}

int ICM536xx::setApexInterrupt(uint8_t intpin, ICM536xx_irq_handler handler)
{
  int rc = 0;
  inv_imu_int_state_t config_int;
  inv_imu_int_pin_config_t int_pin_config;

  if (handler == NULL)
    return 0;
  
  pinMode(intpin,INPUT);
  attachInterrupt(intpin,handler,RISING);
  
  /*
   * Configure interrupts pins
   * - Polarity High
   * - Pulse mode
   * - Push-Pull drive
   */
  int_pin_config.int_polarity = INT_CONFIG_INTX_POLARITY_HIGH;
  int_pin_config.int_mode     = INT_CONFIG_INTX_MODE_PULSED;
  int_pin_config.int_drive    = INT_CONFIG_INTX_DRIVE_CIRCUIT_PP;
  rc |= inv_imu_set_pin_config_int(&icm_driver, INV_IMU_INT1, &int_pin_config);
  
  return rc;
}

int ICM536xx::startAllApex(uint8_t intpin, ICM536xx_irq_handler handler)
{
  int rc = 0;

  apex_enable[ICM536XX_APEX_TILT]  = true;
  apex_enable[ICM536XX_APEX_TAP]   = true;
  apex_enable[ICM536XX_APEX_FF]    = true;
  apex_enable[ICM536XX_APEX_LOWG]  = true;
  apex_enable[ICM536XX_APEX_HIGHG] = true;

  rc |= setApexInterrupt(intpin, handler);
  rc |= configure_and_enable_edmp_algo();
  
  return rc;
}

int ICM536xx::startApex(int apex_type, uint8_t intpin, ICM536xx_irq_handler handler)
{
  int rc = 0;

  apex_enable[apex_type] = true;

  if(handler != NULL)
  {
    rc |= setApexInterrupt(intpin, handler);
    rc |= configure_and_enable_edmp_algo();
  }
  
  return rc;
}

int ICM536xx::startWakeOnMotion(uint8_t intpin, ICM536xx_irq_handler handler)
{
  int rc = 0;
  inv_imu_int_pin_config_t int_pin_config;
  inv_imu_int_state_t int_config;
  int wom_high_thr_en = 1; /* Indicates WOM state */
  
  /* WOM threshold to be applied to IMU, ranges from 1 to 255, in 4mg unit */
  uint8_t wom_threshold_high = WOM_THRESHOLD_INITIAL_MG / 4;
  uint8_t wom_threshold_low	= WOM_THRESHOLD_LOW_MG / 4;

  pinMode(intpin,INPUT);
  attachInterrupt(intpin,handler,RISING);

  int_pin_config.int_polarity = INT_CONFIG_INTX_POLARITY_HIGH;
  int_pin_config.int_mode     = INT_CONFIG_INTX_MODE_PULSED;
  int_pin_config.int_drive    = INT_CONFIG_INTX_DRIVE_CIRCUIT_PP;
  rc |= inv_imu_set_pin_config_int(&icm_driver, INV_IMU_INT1, &int_pin_config);  

  /* Interrupts configuration: Enable only WOM interrupts */
  memset(&int_config, INV_IMU_DISABLE, sizeof(int_config));
  int_config.INV_WOM_X = INV_IMU_ENABLE;
  int_config.INV_WOM_Y = INV_IMU_ENABLE;
  int_config.INV_WOM_Z = INV_IMU_ENABLE;
  rc |= inv_imu_set_config_int(&icm_driver, INV_IMU_INT1, &int_config);

  /* Configure WOM to produce signal when at least one axis exceed wom_threshold_high/low */
  rc |= inv_imu_adv_configure_wom(&icm_driver,
                                  wom_high_thr_en ? wom_threshold_high : wom_threshold_low,
                                  wom_high_thr_en ? wom_threshold_high : wom_threshold_low,
	                              wom_high_thr_en ? wom_threshold_high : wom_threshold_low,
	                              WOM_CONFIG_WOM_INT_MODE_ORED,
	                              WOM_CONFIG_WOM_INT_DUR_1_SMPL);
  rc |= inv_imu_adv_enable_wom(&icm_driver);
  rc |= inv_imu_set_accel_frequency(&icm_driver, ACCEL_CONFIG0_ODR_12_5_HZ);
  /* Set 1x averaging, in order to minimize power consumption */
  rc |= inv_imu_set_accel_lp_avg(&icm_driver, ACCEL_CONFIG1_ACCEL_FILT_AVG_2);
  rc |= inv_imu_adv_enable_accel_lp(&icm_driver);
  
  return rc;
}

int ICM536xx::getWom(bool& x, bool& y, bool& z)
{
  if (int_status.INV_WOM_X || int_status.INV_WOM_Y || int_status.INV_WOM_Z) {
    x = int_status.INV_WOM_X;
	y = int_status.INV_WOM_Y;
	z = int_status.INV_WOM_Z;
	int_status.INV_WOM_X = int_status.INV_WOM_Y = int_status.INV_WOM_Z = 0;
    return APEX_EVENT_DETECTED;
  } else {
    return APEX_NO_EVENT;
  }
}

int ICM536xx::getApex_data(int apex_type, uint32_t& p1, uint32_t& p2, uint32_t& p3)
{
  int ret = APEX_NO_EVENT;
  
  if(apex_enable[apex_type] == false)
    return INV_ERROR;

  switch(apex_type)
  {
    case ICM536XX_APEX_TILT:
	  if (int_status.INV_TILT_DET){
        int_status.INV_TILT_DET = 0;
        ret = APEX_EVENT_DETECTED;
      }
	  break;
	case ICM536XX_APEX_TAP:
	  if (int_status.INV_TAP) {
        inv_imu_apex_tap_event_t tap_data;
		
        int_status.INV_TAP = 0;
        inv_imu_apex_get_data_tap(&icm_driver, &tap_data);
        p1 = tap_data.tap;
        p2 = tap_data.tap_axis;
        p3 = tap_data.tap_direction;
        ret = APEX_EVENT_DETECTED;
      }
	  break;
	case ICM536XX_APEX_FF:
	  if (int_status.INV_FF) {
	  	uint8_t  data[2];
    	uint16_t duration;
		
        int_status.INV_FF = 0;
		inv_imu_read_reg(&icm_driver, APEX_DATA4, 2, &data[0]);
        duration = (uint16_t)((data[1] << 8) | data[0]);
    	p1 = (duration * 2500) / 1000;
        ret = APEX_EVENT_DETECTED;
      }
	  break;
	case ICM536XX_APEX_HIGHG:
	  if (int_status.INV_HIGHG) {
        int_status.INV_HIGHG = 0;
        ret = APEX_EVENT_DETECTED;
      }
      break;
    case ICM536XX_APEX_LOWG:
      if (int_status.INV_LOWG) {
        int_status.INV_LOWG = 0;   
        ret = APEX_EVENT_DETECTED;
      }
      break;
    default:
      ret = APEX_ERROR_EVENT;
  }

  return ret;
}

int ICM536xx::configure_and_enable_edmp_algo(void)
{
  int rc = 0;
  accel_config0_odr_t    accel_odr;
  inv_imu_int_state_t    int_config;
  apex_config1_dmp_odr_t dmp_odr;
  inv_imu_fifo_config_t 	fifo_config;
  inv_imu_apex_parameters_t apex_inputs;
  
  /* Optimize APEX parameters for SmartMotion+EVB setup */
  rc |= inv_imu_apex_init_parameters_struct(&icm_driver, &apex_inputs);
  apex_inputs.tilt_wait_time		 = APEX_CONFIG5_TILT_WAIT_TIME_2_S;
  apex_inputs.power_save_time 	 = APEX_CONFIG2_DMP_POWER_SAVE_TIME_SEL_8_S;
  apex_inputs.power_save			 = INV_IMU_DISABLE;
  apex_inputs.ff_debounce_duration = APEX_CONFIG9_FF_DEBOUNCE_DURATION_2000_MS;
  apex_inputs.ff_max_duration_cm	 = APEX_CONFIG12_FF_MAX_DURATION_228_CM;
  apex_inputs.ff_min_duration_cm	 = APEX_CONFIG12_FF_MIN_DURATION_10_CM;
  apex_inputs.lowg_peak_th		 = APEX_CONFIG10_LOWG_PEAK_TH_500_MG;
  apex_inputs.lowg_peak_hyst		 = APEX_CONFIG5_LOWG_PEAK_TH_HYST_31_MG;
  apex_inputs.lowg_samples_th 	 = APEX_CONFIG10_LOWG_TIME_TH_8_SAMPLES;
  apex_inputs.highg_peak_th		 = APEX_CONFIG11_HIGHG_PEAK_TH_7250_MG;
  apex_inputs.highg_peak_hyst 	 = APEX_CONFIG5_HIGHG_PEAK_TH_HYST_156_MG;
  apex_inputs.highg_samples_th	 = APEX_CONFIG11_HIGHG_TIME_TH_1_SAMPLE;
  apex_inputs.tap_max 			 = APEX_CONFIG2_TAP_MAX_TRIPLE;
  apex_inputs.tap_min 			 = APEX_CONFIG2_TAP_MIN_SINGLE;
  apex_inputs.tap_tavg			 = APEX_CONFIG3_TAP_TAVG_8;
  apex_inputs.tap_tmax			 = APEX_CONFIG3_TAP_TMAX_500_MS;
  apex_inputs.tap_tmin			 = APEX_CONFIG3_TAP_TMIN_165_MS;
  apex_inputs.tap_max_peak_tol	 = APEX_CONFIG4_TAP_MAX_PEAK_TOL_37_5;
  apex_inputs.tap_min_jerk_thr	 = APEX_CONFIG4_TAP_MIN_JERK_THR_1125mg;
  apex_inputs.tap_smudge_reject_th = APEX_CONFIG9_TAP_SMUDGE_REJECT_THR_85_MS;
#if INV_IMU_ACC_HFSR_SUPPORTED
  apex_inputs.ext_highg_samples_th = APEX_CONFIG14_EXT_HIGHG_TIME_TH_1_SAMPLE;
  apex_inputs.ext_highg_peak_hyst  = APEX_CONFIG14_EXT_HIGHG_PEAK_TH_HYST_250_MG;
  apex_inputs.ext_highg_peak_th	 = APEX_CONFIG14_EXT_HIGHG_PEAK_TH_20_G;
#endif
  rc |= inv_imu_apex_configure_parameters(&icm_driver, &apex_inputs);

  /* Configure ODR depending on which feature is enabled */
  if (apex_enable[ICM536XX_APEX_FF] || apex_enable[ICM536XX_APEX_LOWG] || apex_enable[ICM536XX_APEX_HIGHG] || apex_enable[ICM536XX_APEX_TAP]) {
  /* 400 Hz */
    current_odr_us    = 2500;
    dmp_odr   = APEX_CONFIG1_DMP_ODR_400Hz;
    accel_odr = ACCEL_CONFIG0_ODR_400_HZ;
  } else {
  /* 50 Hz */
    current_odr_us    = 20000;
    dmp_odr	  = APEX_CONFIG1_DMP_ODR_50Hz;
    accel_odr = ACCEL_CONFIG0_ODR_50_HZ;
  }

  /* Disable FIFO to optimize power consumption */
  rc |= inv_imu_get_fifo_config(&icm_driver, &fifo_config);
  fifo_config.fifo_en = INV_IMU_DISABLE;
  rc |= inv_imu_set_fifo_config(&icm_driver, &fifo_config);
  
  /* Set 2X averaging to minimize power consumption */
  rc |= inv_imu_set_accel_lp_avg(&icm_driver, ACCEL_CONFIG1_ACCEL_FILT_AVG_2);
  
  /* Enable accel in LP mode */
  rc |= inv_imu_set_accel_mode(&icm_driver, PWR_MGMT0_ACCEL_MODE_LP);

  /* Set EDMP ODR */
  rc |= inv_imu_apex_set_frequency(&icm_driver, dmp_odr);

  /* Set ODR */
  rc |= inv_imu_set_accel_frequency(&icm_driver, accel_odr);

  /* Ensure all DMP features are disabled before running init procedure */
  rc |= inv_imu_apex_disable_tap(&icm_driver);
  rc |= inv_imu_apex_disable_tilt(&icm_driver);
  rc |= inv_imu_apex_disable_ff(&icm_driver);
#if INV_IMU_ACC_HFSR_SUPPORTED
	  rc |= inv_imu_apex_disable_ext_highg(&icm_driver);
#endif

  if (apex_enable[ICM536XX_APEX_TILT]) {
  	rc |= inv_imu_apex_enable_tilt(&icm_driver);
  	int_config.INV_TILT_DET = INV_IMU_ENABLE;
  }

  if (apex_enable[ICM536XX_APEX_TAP]) {
  	rc |= inv_imu_apex_enable_tap(&icm_driver);
  	int_config.INV_TAP = INV_IMU_ENABLE;
  }
  
  if (apex_enable[ICM536XX_APEX_FF]) {
  	rc |= inv_imu_apex_enable_ff(&icm_driver);
  	int_config.INV_FF = INV_IMU_ENABLE;
  }
  
  if (apex_enable[ICM536XX_APEX_HIGHG]) {
  	rc |= inv_imu_apex_enable_ff(&icm_driver);
  	int_config.INV_HIGHG = INV_IMU_ENABLE;
  }
  
  if (apex_enable[ICM536XX_APEX_LOWG]) {
    rc |= inv_imu_apex_enable_ff(&icm_driver);
    int_config.INV_LOWG = INV_IMU_ENABLE;
  }

  /* Apply interrupt configuration */
  rc |= inv_imu_set_config_int(&icm_driver, INV_IMU_INT1, &int_config);

  return rc;
}

int ICM536xx::updateApex(void)
{
  int rc = 0;
  inv_imu_int_state_t      int_state;

  /* Read interrupt status */
  rc |= inv_imu_get_int_status(&icm_driver, INV_IMU_INT1, &int_state);

  int_status.INV_TILT_DET   |= int_state.INV_TILT_DET;
  int_status.INV_FF         |= int_state.INV_FF;
  int_status.INV_HIGHG      |= int_state.INV_HIGHG;
  int_status.INV_TAP        |= int_state.INV_TAP;
  int_status.INV_LOWG       |= int_state.INV_LOWG;
  int_status.INV_WOM_X      |= int_state.INV_WOM_X;
  int_status.INV_WOM_Y      |= int_state.INV_WOM_Y;
  int_status.INV_WOM_Z      |= int_state.INV_WOM_Z;
  
  return rc;
}

static int i2c_write(void *context, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen)
{
  ICM536xx *instance = (ICM536xx *)context;
  instance->i2c->beginTransmission(instance->i2c_address);
  instance->i2c->write(reg);

  for(uint8_t i = 0; i < wlen; i++) {
    instance->i2c->write(wbuffer[i]);
  }
  instance->i2c->endTransmission();
  return 0;
}

static int i2c_read(void *context, uint8_t reg, uint8_t * rbuffer, uint32_t rlen)
{
  uint16_t offset = 0;
  ICM536xx *instance = (ICM536xx *)context;

  int recnt = 256;

  instance->i2c->beginTransmission(instance->i2c_address);
  instance->i2c->write(reg);
  instance->i2c->endTransmission(false);
  while(offset < rlen && recnt-- > 0)
  {
    uint16_t rx_bytes = 0;
    if(offset != 0)
      instance->i2c->beginTransmission(instance->i2c_address);
    uint8_t length = ((rlen - offset) > ARDUINO_I2C_BUFFER_LENGTH) ? ARDUINO_I2C_BUFFER_LENGTH : (rlen - offset) ;
    rx_bytes = instance->i2c->requestFrom(instance->i2c_address, length);
    if (rx_bytes == length) {
      for(uint8_t i = 0; i < length; i++) {
        rbuffer[offset+i] = instance->i2c->read();
      }
      offset += length;
      instance->i2c->endTransmission((offset == rlen));
    } else {
      instance->i2c->endTransmission((offset == rlen));
    }
  }
  if(offset == rlen)
  {
    return 0;
  } else {
    return -1;
  }
}

static int spi_write(void *context, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen)
{
  ICM536xx *instance = (ICM536xx *)context;

  instance->spi->beginTransaction(SPISettings(clk_freq, MSBFIRST, SPI_MODE3));
  digitalWrite(instance->chip_select_id,LOW);
  instance->spi->transfer(reg);
  for(uint8_t i = 0; i < wlen; i++) {
    instance->spi->transfer(wbuffer[i]);
  }
  digitalWrite(instance->chip_select_id,HIGH);
  instance->spi->endTransaction();
  return 0;
}

static int spi_read(void *context, uint8_t reg, uint8_t * rbuffer, uint32_t rlen)
{
  ICM536xx *instance = (ICM536xx *)context;

  instance->spi->beginTransaction(SPISettings(clk_freq, MSBFIRST, SPI_MODE3));
  digitalWrite(instance->chip_select_id,LOW);
  instance->spi->transfer(reg | SPI_READ);
  instance->spi->transfer(rbuffer,rlen);
  digitalWrite(instance->chip_select_id,HIGH);
  instance->spi->endTransaction();
  return 0;
}

accel_config0_fs_sel_t ICM536xx::accel_fsr_g_to_param(uint16_t accel_fsr_g) {
  accel_config0_fs_sel_t ret = ACCEL_CONFIG0_FS_SEL_16g;

  switch(accel_fsr_g) {
  case 4:  ret = ACCEL_CONFIG0_FS_SEL_4g;  break;
  case 8:  ret = ACCEL_CONFIG0_FS_SEL_8g;  break;
  case 16: ret = ACCEL_CONFIG0_FS_SEL_16g; break;
#if INV_IMU_ACC_HFSR_SUPPORTED
  case 32: ret = ACCEL_CONFIG0_FS_SEL_32g; break;
#else
  case 2:  ret = ACCEL_CONFIG0_FS_SEL_2g;  break;
#endif
  default:
    /* Unknown accel FSR. Set to default 16G */
    break;
  }
  return ret;
}

gyro_config0_fs_sel_t ICM536xx::gyro_fsr_dps_to_param(uint16_t gyro_fsr_dps) {
  gyro_config0_fs_sel_t ret = GYRO_CONFIG0_FS_SEL_2000dps;

  switch(gyro_fsr_dps) {
  case 31:   ret = GYRO_CONFIG0_FS_SEL_31dps;   break;
  case 62:   ret = GYRO_CONFIG0_FS_SEL_62dps;   break;
  case 125:  ret = GYRO_CONFIG0_FS_SEL_125dps;  break;
  case 250:  ret = GYRO_CONFIG0_FS_SEL_250dps;  break;
  case 500:  ret = GYRO_CONFIG0_FS_SEL_500dps;  break;
  case 1000: ret = GYRO_CONFIG0_FS_SEL_1000dps; break;
  case 2000: ret = GYRO_CONFIG0_FS_SEL_2000dps; break;
#if INV_IMU_GYR_HFSR_SUPPORTED
  case 4000: ret = GYRO_CONFIG0_FS_SEL_4000dps; break;
#else
  case 15:   ret = GYRO_CONFIG0_FS_SEL_15dps;   break;
#endif
  default:
    /* Unknown gyro FSR. Set to default 2000dps" */
    break;
  }
  return ret;
}

accel_config0_odr_t ICM536xx::accel_freq_to_param(uint16_t accel_freq_hz) {
  accel_config0_odr_t ret = ACCEL_CONFIG0_ODR_100_HZ;

  switch(accel_freq_hz) {
  case 1:    ret = ACCEL_CONFIG0_ODR_1_5625_HZ;  break;
  case 3:    ret = ACCEL_CONFIG0_ODR_3_125_HZ;  break;
  case 6:    ret = ACCEL_CONFIG0_ODR_6_25_HZ;  break;
  case 12:   ret = ACCEL_CONFIG0_ODR_12_5_HZ;  break;
  case 25:   ret = ACCEL_CONFIG0_ODR_25_HZ;  break;
  case 50:   ret = ACCEL_CONFIG0_ODR_50_HZ;  break;
  case 100:  ret = ACCEL_CONFIG0_ODR_100_HZ; break;
  case 200:  ret = ACCEL_CONFIG0_ODR_200_HZ; break;
  case 400:  ret = ACCEL_CONFIG0_ODR_400_HZ; break;
  case 800:  ret = ACCEL_CONFIG0_ODR_800_HZ; break;
  default:
    /* Unknown accel frequency. Set to default 100Hz */
    break;
  }
  return ret;
}

gyro_config0_odr_t ICM536xx::gyro_freq_to_param(uint16_t gyro_freq_hz) {
  gyro_config0_odr_t ret = GYRO_CONFIG0_ODR_100_HZ;

  switch(gyro_freq_hz) {
  case 1:   ret = GYRO_CONFIG0_ODR_1_5625_HZ;  break;
  case 3:   ret = GYRO_CONFIG0_ODR_3_125_HZ;  break;
  case 6:   ret = GYRO_CONFIG0_ODR_6_25_HZ;  break;
  case 12:   ret = GYRO_CONFIG0_ODR_12_5_HZ;  break;
  case 25:   ret = GYRO_CONFIG0_ODR_25_HZ;  break;
  case 50:   ret = GYRO_CONFIG0_ODR_50_HZ;  break;
  case 100:  ret = GYRO_CONFIG0_ODR_100_HZ; break;
  case 200:  ret = GYRO_CONFIG0_ODR_200_HZ; break;
  case 400:  ret = GYRO_CONFIG0_ODR_400_HZ; break;
  case 800:  ret = GYRO_CONFIG0_ODR_800_HZ; break;
  default:
    /* Unknown gyro ODR. Set to default 100Hz */
    break;
  }
  return ret;
}

static void sleep_us(uint32_t us)
{
    delayMicroseconds(us);
}

