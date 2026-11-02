#ifndef HEATER_H
#define HEATER_H

#include <stdbool.h>

// Heater gate (GP15) and sense relays (GP6).
// The relays won't close while the heater is on and the heater won't turn on
// while the relays are closed. Software-only for now.

void heater_init(void);     // first thing in main
bool heater_on(void);       // false = not allowed right now
void heater_off(void);
void heater_kill(void);     // raw pin writes, also opens the relays. safe from anywhere
void heater_lockout(bool locked);
bool heater_locked(void);

bool relays_close(void);    // false = not allowed right now
void relays_open(void);

// 74HC123 hardware watchdog
void hwdog_init(void);
void hwdog_kick(void);
bool hwdog_running(void);   // 1Q level on GP22

#endif
