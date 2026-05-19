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
 
#include "ICM536xx.h"

// Instantiate an ICM536xx with LSB address set to 0
ICM536xx IMU(Wire,0);

const char* axis_str[3] = {"X", "Y", "Z"}; 
const char* direction_str[2] = {"+", "-"}; 
volatile uint8_t irq_received = 0;

void irq_handler(void) {
  irq_received = 1;
}

void setup() {
  int ret;
  Serial.begin(115200);
  while(!Serial) {}

  // Initializing the ICM536xx
  ret = IMU.begin();
  if (ret != 0) {
    Serial.print("ICM536xx initialization failed: ");
    Serial.println(ret);
    while(1);
  }

  IMU.startAllApex(2,irq_handler);
}

void loop() {
  // Wait for interrupt to read data Pedometer status
  if(irq_received) {
    irq_received = 0;
    int ret = 0;
    uint32_t tap_count=0;
    uint32_t axis=0;
    uint32_t direction=0;
    uint32_t activity=0;
    uint32_t step_count=0;
    uint32_t step_cadence=0;

    IMU.updateApex();
    
    if(IMU.getApex_data(ICM536XX_APEX_TILT) == APEX_EVENT_DETECTED)
      Serial.println("Tilt Event");

    uint32_t duration_ms;  
    if(IMU.getApex_data(ICM536XX_APEX_FF, duration_ms) == APEX_EVENT_DETECTED)
    {
      Serial.print("FreeFall Event duration(ms):");
      Serial.println(duration_ms);
    }

    if(IMU.getApex_data(ICM536XX_APEX_HIGHG) == APEX_EVENT_DETECTED)
      Serial.println("HighG Event");

    if(IMU.getApex_data(ICM536XX_APEX_LOWG) == APEX_EVENT_DETECTED)
      Serial.println("LowG Event");

    if(IMU.getApex_data(ICM536XX_APEX_TAP,tap_count,axis,direction) == APEX_EVENT_DETECTED)
    {
      Serial.print("Tap count:");
      Serial.print(tap_count);
      Serial.print(",");
      Serial.print("Axis:");
      Serial.print(axis_str[axis]);
      Serial.print(",");
      Serial.print("Direction:");
      Serial.println(direction_str[direction]);
    }
  }
}

