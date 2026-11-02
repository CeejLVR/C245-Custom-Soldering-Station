#include "heater.h"

#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "pico/time.h"

#include "PinDefinitions.h"
#include "config.h"

// The heater is a plain GPIO now, not PWM. The control task does its own
// on/off timing so the ADC reading always lands in a window where the heater
// is off.
//
// The interlock works off timestamps: we remember when the heater last went
// off and when the relays last opened, and refuse to switch the other one
// until enough time has passed. Each check+switch happens with interrupts off
// so nothing can sneak in between.

static volatile bool locked_out = true;     // safety task releases this after the watchdog test passes
static volatile uint64_t heater_off_at;     // us, last time GP15 went low
static volatile uint64_t relays_open_at;    // us, last time GP6 went low

void heater_init(void) {
    gpio_init(PIN_HEATER_PWM);
    gpio_put(PIN_HEATER_PWM, 0);
    gpio_set_dir(PIN_HEATER_PWM, GPIO_OUT);

    gpio_init(PIN_SENSE_RELAY);
    gpio_put(PIN_SENSE_RELAY, 0);
    gpio_set_dir(PIN_SENSE_RELAY, GPIO_OUT);
    // both relay LEDs hang off this one pin, ~11mA total, so bump the drive
    gpio_set_drive_strength(PIN_SENSE_RELAY, GPIO_DRIVE_STRENGTH_12MA);

    heater_off_at = relays_open_at = time_us_64();
}

bool heater_on(void) {
#if !CFG_HEATER_ARMED
    return false;
#else
    uint32_t irq = save_and_disable_interrupts();
    bool allowed = !locked_out
                && !gpio_get_out_level(PIN_SENSE_RELAY)
                && (time_us_64() - relays_open_at) >= RELAY_OFF_US;
    if (allowed)
        gpio_put(PIN_HEATER_PWM, 1);
    restore_interrupts(irq);
    return allowed;
#endif
}

void heater_off(void) {
    uint32_t irq = save_and_disable_interrupts();
    if (gpio_get_out_level(PIN_HEATER_PWM)) {
        gpio_put(PIN_HEATER_PWM, 0);
        heater_off_at = time_us_64();
    }
    restore_interrupts(irq);
}

// No locking, no checks - just drive both pins low. This is what the fault
// paths and the RTOS hooks call.
void heater_kill(void) {
    gpio_put(PIN_HEATER_PWM, 0);
    gpio_put(PIN_SENSE_RELAY, 0);
    uint64_t now = time_us_64();
    heater_off_at = now;
    relays_open_at = now;
}

void heater_lockout(bool locked) {
    locked_out = locked;
    if (locked)
        heater_off();
}

bool heater_locked(void) {
    return locked_out;
}

bool relays_close(void) {
    uint32_t irq = save_and_disable_interrupts();
    // heater has to be off AND settled, otherwise the ADC sees 24V
    bool allowed = !gpio_get_out_level(PIN_HEATER_PWM)
                && (time_us_64() - heater_off_at) >= HEATER_OFF_SETTLE_US;
    if (allowed)
        gpio_put(PIN_SENSE_RELAY, 1);
    restore_interrupts(irq);
    return allowed;
}

void relays_open(void) {
    uint32_t irq = save_and_disable_interrupts();
    if (gpio_get_out_level(PIN_SENSE_RELAY)) {
        gpio_put(PIN_SENSE_RELAY, 0);
        relays_open_at = time_us_64();
    }
    restore_interrupts(irq);
}


// 74HC123 watchdog. 1A is tied low so a rising edge on 1B retriggers it.
// Pulse length = K*R*C ~= 0.5 * 220k * 1uF ~= 110ms (K from fig 6). Kicks
// closer together than ~10.5ms don't extend it, which is why the safety task
// runs at 25ms.

void hwdog_init(void) {
    gpio_init(PIN_WATCHDOG_KICK);
    gpio_put(PIN_WATCHDOG_KICK, 0);
    gpio_set_dir(PIN_WATCHDOG_KICK, GPIO_OUT);

    gpio_init(PIN_WATCHDOG_STATUS);
    gpio_set_dir(PIN_WATCHDOG_STATUS, GPIO_IN);
    gpio_pull_down(PIN_WATCHDOG_STATUS);    // a loose wire reads as tripped, not ok
}

void hwdog_kick(void) {
    gpio_put(PIN_WATCHDOG_KICK, 1);
    busy_wait_us_32(5);
    gpio_put(PIN_WATCHDOG_KICK, 0);
}

bool hwdog_running(void) {
    return gpio_get(PIN_WATCHDOG_STATUS);
}
