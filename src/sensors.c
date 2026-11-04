#include "sensors.h"

#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/time.h"
#include "task.h"

#include "PinDefinitions.h"
#include "config.h"
#include "board.h"


// ======== ADS1220 ========
// Single-shot conversions on AIN0-AIN1. The control task starts one, sleeps
// until DRDY falls (interrupt -> task notification), then reads 3 bytes.

#define ADS_SPI         spi0

// command bytes, datasheet table 8-7
#define ADS_RESET       0x06
#define ADS_START       0x08
#define ADS_RREG(r, n)  (uint8_t)(0x20 | ((r) << 2) | ((n) - 1))
#define ADS_WREG(r, n)  (uint8_t)(0x40 | ((r) << 2) | ((n) - 1))

// reg0 = AIN0/AIN1, gain 4, PGA bypassed. It has to be bypassed because the
//        thermocouple sits right at 0V and the PGA can't swing down there.
// reg1 = 330 SPS, normal mode, single shot
// reg2 = internal 2.048V reference
// reg3 = IDACs off, DRDY on its own pin
#define ADS_REG0        0x05
#define ADS_REG1        0x80
#define ADS_REG2        0x00
#define ADS_REG3        0x00
#define ADS_REG2_IDAC   0x01    // both IDACs set to 10uA
#define ADS_REG3_IDAC   0x40    // IDAC1 routed to AIN1

#define ADS_VREF        2.048f
#define ADS_GAIN        4.0f

static TaskHandle_t volatile ads_waiting_task;

static inline void cs_low(void) {
    gpio_put(PIN_ADC_CS, 0);
    busy_wait_us_32(1);
}

static inline void cs_high(void) {
    busy_wait_us_32(1);
    gpio_put(PIN_ADC_CS, 1);
}

static void ads_cmd(uint8_t c) {
    cs_low();
    spi_write_blocking(ADS_SPI, &c, 1);
    cs_high();
}

static void ads_write_regs(uint8_t reg, const uint8_t *vals, uint8_t n) {
    uint8_t c = ADS_WREG(reg, n);
    cs_low();
    spi_write_blocking(ADS_SPI, &c, 1);
    spi_write_blocking(ADS_SPI, vals, n);
    cs_high();
}

static void ads_read_regs(uint8_t reg, uint8_t *vals, uint8_t n) {
    uint8_t c = ADS_RREG(reg, n);
    cs_low();
    spi_write_blocking(ADS_SPI, &c, 1);
    spi_read_blocking(ADS_SPI, 0x00, vals, n);
    cs_high();
}

static void ads_read_data(uint8_t b[3]) {
    cs_low();
    spi_read_blocking(ADS_SPI, 0x00, b, 3);
    cs_high();
}

// DRDY is the only GPIO interrupt on the board now, so it can own the
// SDK's single GPIO callback. If stand detection ever adds another one this
// will need to become a little dispatcher.
static void drdy_callback(uint gpio, uint32_t events) {
    (void)events;
    if (gpio != PIN_ADC_DRDY) return;

    TaskHandle_t t = ads_waiting_task;
    if (t == NULL) return;

    BaseType_t woke = pdFALSE;
    vTaskNotifyGiveFromISR(t, &woke);
    portYIELD_FROM_ISR(woke);
}

bool ads_init(void) {
    spi_init(ADS_SPI, ADC_SPI_BAUD_HZ);
    spi_set_format(ADS_SPI, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);    // mode 1
    gpio_set_function(PIN_ADC_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_ADC_SCLK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_ADC_MOSI, GPIO_FUNC_SPI);

    gpio_init(PIN_ADC_CS);
    gpio_put(PIN_ADC_CS, 1);
    gpio_set_dir(PIN_ADC_CS, GPIO_OUT);

    gpio_init(PIN_ADC_DRDY);
    gpio_set_dir(PIN_ADC_DRDY, GPIO_IN);
    gpio_pull_up(PIN_ADC_DRDY);

    ads_cmd(ADS_RESET);
    sleep_us(100);      // datasheet wants 50us + 32 clocks after a reset

    const uint8_t regs[4] = { ADS_REG0, ADS_REG1, ADS_REG2, ADS_REG3 };
    ads_write_regs(0, regs, 4);

    // read them back - easiest way to tell if the chip is even there
    uint8_t check[4] = {0};
    ads_read_regs(0, check, 4);
    bool ok = check[0] == regs[0] && check[1] == regs[1] && check[2] == regs[2]
           && (check[3] & 0xFE) == regs[3];

    gpio_set_irq_enabled_with_callback(PIN_ADC_DRDY, GPIO_IRQ_EDGE_FALL, true, drdy_callback);
    return ok;
}

// Datasheet order: set the current first, then the routing. Turning it off
// goes the other way. Writing a register also kicks off a conversion.
bool ads_tip_source(bool on) {
    uint8_t r2 = on ? ADS_REG2_IDAC : ADS_REG2;
    uint8_t r3 = on ? ADS_REG3_IDAC : ADS_REG3;
    if (on) {
        ads_write_regs(2, &r2, 1);
        ads_write_regs(3, &r3, 1);
    } else {
        ads_write_regs(3, &r3, 1);
        ads_write_regs(2, &r2, 1);
    }
    uint8_t check2 = 0, check3 = 0;
    ads_read_regs(2, &check2, 1);
    ads_read_regs(3, &check3, 1);
    return check2 == r2 && (check3 & 0xFE) == r3;
}

bool ads_convert(int32_t *code, uint32_t timeout_ms) {
    uint8_t b[3];

    // If an old result was never read, DRDY is already low and we'd never see
    // a falling edge. Read it out to reset DRDY first.
    if (!gpio_get(PIN_ADC_DRDY))
        ads_read_data(b);

    ads_waiting_task = xTaskGetCurrentTaskHandle();
    ulTaskNotifyTake(pdTRUE, 0);    // throw away any stale notification

    ads_cmd(ADS_START);
    bool ready = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(timeout_ms)) > 0;
    ads_waiting_task = NULL;

    // in case the interrupt got missed somehow, check the pin directly
    if (!ready && !gpio_get(PIN_ADC_DRDY))
        ready = true;
    if (!ready)
        return false;

    ads_read_data(b);
    int32_t v = ((int32_t)b[0] << 16) | ((int32_t)b[1] << 8) | b[2];
    if (v & 0x800000)
        v -= 0x1000000;     // sign extend 24 -> 32 bits
    *code = v;
    return true;
}

float ads_to_uv(int32_t code) {
    // full scale is +-2.048V/4 over 2^23 counts, so 1 LSB is about 61nV
    return (float)code * (ADS_VREF / ADS_GAIN) / 8388608.0f * 1e6f;
}


// ======== MCP9808 ========

#define CJ_ADDR         0x18    // A0-A2 grounded
#define CJ_REG_TEMP     0x05
#define CJ_REG_MFG      0x06    // reads 0x0054
#define CJ_REG_DEV      0x07    // upper byte reads 0x04

static bool cj_read_reg(uint8_t reg, uint16_t *out) {
    uint8_t b[2];
    if (!bus_write(CJ_ADDR, &reg, 1, true)) return false;
    if (!bus_read(CJ_ADDR, b, 2)) return false;
    *out = ((uint16_t)b[0] << 8) | b[1];
    return true;
}

bool cj_init(void) {
    uint16_t mfg = 0, dev = 0;
    if (!bus_lock(pdMS_TO_TICKS(100)))
        return false;
    bool ok = cj_read_reg(CJ_REG_MFG, &mfg) && cj_read_reg(CJ_REG_DEV, &dev);
    bus_unlock();
    return ok && mfg == 0x0054 && (dev >> 8) == 0x04;
}

bool cj_read(float *temp_c, TickType_t bus_wait) {
    uint16_t raw;
    if (!bus_lock(bus_wait))
        return false;
    bool ok = cj_read_reg(CJ_REG_TEMP, &raw);
    bus_unlock();
    if (!ok)
        return false;

    // top 3 bits are alert flags, bit 12 is sign, 1/16 C per count
    raw &= 0x1FFF;
    int16_t v = (int16_t)(raw & 0x0FFF);
    if (raw & 0x1000)
        v -= 4096;
    *temp_c = (float)v * 0.0625f;
    return true;
}


// ======== thermocouple ========
// tip = cold junction + polarity * (V - offset) / sensitivity
// Linear for now. Could switch to a lookup table later using solder melting
// points as calibration (63/37 = 183C, SAC305 ~217-220C).

float tip_temp(float tc_uv, float cj_c, const tc_cal_t *cal) {
    float sens = cal->sens_uv_per_c > 1.0f ? cal->sens_uv_per_c : 1.0f;    // avoid divide by 0
    return cj_c + (float)cal->polarity * (tc_uv - cal->offset_uv) / sens;
}
