#include "board.h"

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"

#include "semphr.h"
#include "queue.h"
#include "timers.h"

#include "quadrature_encoder.pio.h"     // from pico-examples, see lib/quadrature_encoder

#include "PinDefinitions.h"
#include "config.h"


// ---------- I2C bus ----------

#define I2C_PORT        i2c0
#define I2C_TIMEOUT_US  20000

static SemaphoreHandle_t bus_mutex;

void bus_init(void) {
    i2c_init(I2C_PORT, I2C_BAUD_HZ);
    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C);
    // the breakout boards have their own pullups, these are just backup
    gpio_pull_up(PIN_I2C_SDA);
    gpio_pull_up(PIN_I2C_SCL);

    bus_mutex = xSemaphoreCreateMutex();
    configASSERT(bus_mutex);
}

bool bus_lock(TickType_t wait) {
    return xSemaphoreTake(bus_mutex, wait) == pdTRUE;
}

void bus_unlock(void) {
    xSemaphoreGive(bus_mutex);
}

i2c_inst_t *bus_port(void) {
    return I2C_PORT;
}

bool bus_write(uint8_t addr, const uint8_t *data, size_t len, bool nostop) {
    return i2c_write_timeout_us(I2C_PORT, addr, data, len, nostop, I2C_TIMEOUT_US) == (int)len;
}

bool bus_read(uint8_t addr, uint8_t *data, size_t len) {
    return i2c_read_timeout_us(I2C_PORT, addr, data, len, false, I2C_TIMEOUT_US) == (int)len;
}


// ---------- encoder + buttons ----------

static QueueHandle_t input_q;

static void post(input_event_t ev) {
    xQueueSend(input_q, &ev, 0);
}

// The encoder is handled by the PIO quadrature program from pico-examples.
// It keeps a running step count in hardware (bounces cancel themselves out),
// we just read it every 5ms and turn every 4 steps into one click. It
// needs A and B on consecutive GPIOs, which GP10/GP11 already are.
#define ENC_PIO     pio0
#define ENC_SM      0

static int32_t enc_last;
static int32_t enc_steps;

static void poll_encoder(void) {
    // the PIO counts the opposite direction from the old interrupt code, so
    // negate it to keep ENCODER_REVERSE meaning the same thing
    int32_t now = -quadrature_encoder_get_count(ENC_PIO, ENC_SM);
    enc_steps += now - enc_last;
    enc_last = now;

    while (enc_steps >= ENCODER_TRANSITIONS_PER_DETENT) {
        enc_steps -= ENCODER_TRANSITIONS_PER_DETENT;
        post(ENCODER_REVERSE ? EV_ENC_CCW : EV_ENC_CW);
    }
    while (enc_steps <= -ENCODER_TRANSITIONS_PER_DETENT) {
        enc_steps += ENCODER_TRANSITIONS_PER_DETENT;
        post(ENCODER_REVERSE ? EV_ENC_CW : EV_ENC_CCW);
    }
}

// Buttons get polled from the same 5ms timer. The counter goes up while the
// button reads down and back toward 0 when it's up, so it has to be held for
// 4 polls in a row before it counts. Short press fires on release, long press
// fires while still held.
struct button {
    uint8_t pin;
    input_event_t press, hold;
    uint8_t count;
    bool down;
    bool hold_sent;
    uint32_t held_ms;
};

static struct button buttons[] = {
    { .pin = PIN_ENCODER_BUTTON, .press = EV_PSH, .hold = EV_PSH_LONG },
    { .pin = PIN_BUTTON_CON,     .press = EV_CON, .hold = EV_CON_LONG },
    { .pin = PIN_BUTTON_BAK,     .press = EV_BAK, .hold = EV_BAK_LONG },
};
#define NUM_BUTTONS (sizeof buttons / sizeof buttons[0])

static void poll_buttons(void) {
    for (unsigned i = 0; i < NUM_BUTTONS; i++) {
        struct button *b = &buttons[i];
        bool pressed_now = !gpio_get(b->pin);   // active low

        if (pressed_now && b->count < BUTTON_DEBOUNCE_SAMPLES) b->count++;
        if (!pressed_now && b->count > 0) b->count--;

        if (!b->down && b->count == BUTTON_DEBOUNCE_SAMPLES) {
            b->down = true;
            b->hold_sent = false;
            b->held_ms = 0;
        } else if (b->down && b->count == 0) {
            // let go. don't send a short press on top of a long one
            b->down = false;
            if (!b->hold_sent) post(b->press);
        } else if (b->down) {
            b->held_ms += BUTTON_POLL_MS;
            if (!b->hold_sent && b->held_ms >= BUTTON_LONG_PRESS_MS) {
                b->hold_sent = true;
                post(b->hold);
            }
        }
    }
}

static void input_timer(TimerHandle_t t) {
    (void)t;
    poll_encoder();
    poll_buttons();
}

void input_init(void) {
    input_q = xQueueCreate(16, sizeof(input_event_t));
    configASSERT(input_q);

    const uint pins[] = { PIN_ENCODER_BUTTON, PIN_BUTTON_CON, PIN_BUTTON_BAK };
    for (unsigned i = 0; i < sizeof pins / sizeof pins[0]; i++) {
        gpio_init(pins[i]);
        gpio_set_dir(pins[i], GPIO_IN);
        gpio_pull_up(pins[i]);
    }

    // the program has to sit at offset 0 (it uses computed jumps), nothing
    // else uses PIO0 so that's fine. init also sets the pullups on A and B
    pio_add_program(ENC_PIO, &quadrature_encoder_program);
    quadrature_encoder_program_init(ENC_PIO, ENC_SM, PIN_ENCODER_A, 0);
    enc_last = -quadrature_encoder_get_count(ENC_PIO, ENC_SM);

    TimerHandle_t t = xTimerCreate("input", pdMS_TO_TICKS(BUTTON_POLL_MS), pdTRUE, NULL, input_timer);
    configASSERT(t);
    xTimerStart(t, 0);
}

bool input_get(input_event_t *ev, TickType_t wait) {
    return xQueueReceive(input_q, ev, wait) == pdTRUE;
}
