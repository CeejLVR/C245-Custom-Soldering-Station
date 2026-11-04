#ifndef SENSORS_H
#define SENSORS_H

#include <stdbool.h>
#include <stdint.h>
#include "FreeRTOS.h"

// ADS1220 - thermocouple ADC on SPI0
bool ads_init(void);
bool ads_tip_source(bool on);    // 10uA from IDAC1 into AIN1
bool ads_convert(int32_t *code, uint32_t timeout_ms);   // blocks until DRDY or timeout
float ads_to_uv(int32_t code);

// MCP9808 - cold junction sensor on the shared I2C bus
bool cj_init(void);
bool cj_read(float *temp_c, TickType_t bus_wait);

// thermocouple voltage -> tip temperature
typedef struct {
    float sens_uv_per_c;
    float offset_uv;
    int polarity;           // +1 or -1
} tc_cal_t;

float tip_temp(float tc_uv, float cj_c, const tc_cal_t *cal);

#endif
