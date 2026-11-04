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
// We remember when the heater last went off so the control task can tell when
// the sense lines have settled. The check happens with interrupts off so
// nothing can sneak in between.

static volatile bool locked_out = true;     // safety task releases this after the watchdog test passes
static volatile uint64_t heater_off_at;     // us, last time GP15 went low

void heater_init(void) {
    gpio_init(PIN_HEATER_PWM);
    gpio_put(PIN_HEATER_PWM, 0);
    gpio_set_dir(PIN_HEATER_PWM, GPIO_OUT);

    heater_off_at = time_us_64();
}

bool heater_on(void) {
#if !CFG_HEATER_ARMED
    return false;
#else
    uint32_t irq = save_and_disable_interrupts();
    bool allowed = !locked_out;
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

// No locking, no checks - just drive the pin low. This is what the fault
// paths and the RTOS hooks call.
void heater_kill(void) {
    gpio_put(PIN_HEATER_PWM, 0);
    heater_off_at = time_us_64();
}

void heater_lockout(bool locked) {
    locked_out = locked;
    if (locked)
        heater_off();
}

bool heater_locked(void) {
    return locked_out;
}

bool heater_settled(void) {
    uint32_t irq = save_and_disable_interrupts();
    // off AND settled, otherwise the reading includes the return wire drop
    bool settled = !gpio_get_out_level(PIN_HEATER_PWM)
                && (time_us_64() - heater_off_at) >= HEATER_OFF_SETTLE_US;
    restore_interrupts(irq);
    return settled;
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
