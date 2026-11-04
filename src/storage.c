#include "storage.h"

#include <stddef.h>
#include <string.h>

#include "hardware/flash.h"
#include "hardware/watchdog.h"
#include "pico/flash.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "timers.h"

#include "config.h"
#include "heater.h"

// Settings live in the last 4KB sector of the 2MB flash. The sector is used
// as 16 slots of 256 bytes (one flash page each). Every save goes into the
// next blank slot and the sector only gets erased once all 16 are full, so
// it's one erase per 16 saves. On boot the valid slot with the highest
// sequence number wins.

#define STORE_OFFSET    (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)
#define NUM_SLOTS       (FLASH_SECTOR_SIZE / FLASH_PAGE_SIZE)
#define MAGIC           0xC245C246u

typedef struct {
    uint32_t magic;
    uint32_t seq;
    settings_t s;
    uint32_t crc;
} record_t;

_Static_assert(sizeof(record_t) <= FLASH_PAGE_SIZE, "settings record has to fit in one flash page");

static settings_t current;
static settings_t in_flash;     // what's actually saved right now
static uint32_t seq;
static SemaphoreHandle_t lock;
static TimerHandle_t save_timer;

// standard CRC-32 (same polynomial as zip/ethernet), bit at a time
static uint32_t crc32(const void *data, size_t len) {
    const uint8_t *p = data;
    uint32_t crc = 0xFFFFFFFFu;
    while (len--) {
        crc ^= *p++;
        for (int i = 0; i < 8; i++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xEDB88320u : crc >> 1;
    }
    return ~crc;
}

static const record_t *slot(int i) {
    return (const record_t *)(XIP_BASE + STORE_OFFSET + (uint32_t)i * FLASH_PAGE_SIZE);
}

static bool slot_valid(const record_t *r) {
    return r->magic == MAGIC && r->crc == crc32(r, offsetof(record_t, crc));
}

static bool slot_blank(int i) {
    const uint8_t *p = (const uint8_t *)slot(i);
    for (unsigned k = 0; k < FLASH_PAGE_SIZE; k++)
        if (p[k] != 0xFF)
            return false;
    return true;
}

static void load_defaults(settings_t *s) {
    memset(s, 0, sizeof *s);
    s->setpoint_c = SETPOINT_DEFAULT_C;
    s->preset_c[0] = 280;
    s->preset_c[1] = 320;
    s->preset_c[2] = 360;
    s->active_preset = 1;
    s->standby_c = STANDBY_TEMP_C;
    s->tc_sens_uv_per_c = TC_SENS_UV_PER_C;
    s->tc_offset_uv = TC_OFFSET_UV;
    s->tc_polarity = TC_POLARITY;
}

// fix anything out of range (garbage from an older struct layout, etc)
static void sanitize(settings_t *s) {
    if (s->setpoint_c < SETPOINT_MIN_C || s->setpoint_c > SETPOINT_MAX_C)
        s->setpoint_c = SETPOINT_DEFAULT_C;
    for (int i = 0; i < NUM_PRESETS; i++)
        if (s->preset_c[i] < SETPOINT_MIN_C || s->preset_c[i] > SETPOINT_MAX_C)
            s->preset_c[i] = SETPOINT_DEFAULT_C;
    if (s->active_preset >= NUM_PRESETS)
        s->active_preset = 0;
    if (s->standby_c < SETPOINT_MIN_C || s->standby_c > SETPOINT_MAX_C)
        s->standby_c = STANDBY_TEMP_C;
    if (!(s->tc_sens_uv_per_c > 5.0f && s->tc_sens_uv_per_c < 100.0f))
        s->tc_sens_uv_per_c = TC_SENS_UV_PER_C;
    if (s->tc_polarity != 1 && s->tc_polarity != -1)
        s->tc_polarity = TC_POLARITY;
}

struct flash_job {
    int slot;           // -1 = sector is full, erase first
    const uint8_t *page;
};

// runs inside flash_safe_execute() with interrupts off
static void write_slot(void *param) {
    struct flash_job *job = param;
    if (job->slot < 0) {
        flash_range_erase(STORE_OFFSET, FLASH_SECTOR_SIZE);
        job->slot = 0;
    }
    flash_range_program(STORE_OFFSET + (uint32_t)job->slot * FLASH_PAGE_SIZE, job->page, FLASH_PAGE_SIZE);
}

static void save_now(void) {
    settings_t snap;
    xSemaphoreTake(lock, portMAX_DELAY);
    snap = current;
    xSemaphoreGive(lock);

    if (memcmp(&snap, &in_flash, sizeof snap) == 0)
        return;     // nothing changed

    static uint8_t page[FLASH_PAGE_SIZE];
    memset(page, 0xFF, sizeof page);
    record_t *r = (record_t *)page;
    r->magic = MAGIC;
    r->seq = seq + 1;
    r->s = snap;
    r->crc = crc32(r, offsetof(record_t, crc));

    struct flash_job job = { .slot = -1, .page = page };
    for (int i = 0; i < (int)NUM_SLOTS; i++) {
        if (slot_blank(i)) {
            job.slot = i;
            break;
        }
    }

    // Interrupts are off during the write and an erase can take ~45ms, so the
    // heater goes off and both watchdogs get fed right before.
    heater_off();
    hwdog_kick();
    watchdog_update();

    // core 1 isn't used, so CMake sets PICO_FLASH_ASSUME_CORE1_SAFE and this
    // just disables interrupts around write_slot()
    if (flash_safe_execute(write_slot, &job, 1000) == PICO_OK) {
        seq++;
        in_flash = snap;
    }
}

static void save_timer_cb(TimerHandle_t t) {
    (void)t;
    save_now();
}

void storage_init(void) {
    lock = xSemaphoreCreateMutex();
    configASSERT(lock);

    // find the newest good save
    const record_t *best = NULL;
    for (int i = 0; i < (int)NUM_SLOTS; i++) {
        const record_t *r = slot(i);
        if (slot_valid(r) && (best == NULL || r->seq > best->seq))
            best = r;
    }

    if (best) {
        current = best->s;
        seq = best->seq;
    } else {
        load_defaults(&current);
        seq = 0;
    }
    sanitize(&current);
    in_flash = current;

    save_timer = xTimerCreate("save", pdMS_TO_TICKS(SETTINGS_SAVE_DELAY_MS), pdFALSE, NULL, save_timer_cb);
    configASSERT(save_timer);
}

void storage_get(settings_t *out) {
    xSemaphoreTake(lock, portMAX_DELAY);
    *out = current;
    xSemaphoreGive(lock);
}

void storage_update(const settings_t *in) {
    xSemaphoreTake(lock, portMAX_DELAY);
    current = *in;
    sanitize(&current);
    xSemaphoreGive(lock);

    // restarts the 2s countdown, so it only saves once you stop turning the knob
    xTimerReset(save_timer, 0);
}
