#ifndef HEATER_H
#define HEATER_H

#include <stdbool.h>

// Heater gate (GP15). The cartridge's heater/TC junction is grounded, so there
// are no sense relays to manage. Heater current still shares the return wire
// with the thermocouple, so readings only count once heater_settled() is true.

void heater_init(void);     // first thing in main
bool heater_on(void);       // false = not allowed right now
void heater_off(void);
void heater_kill(void);     // raw pin write. safe from anywhere
void heater_lockout(bool locked);
bool heater_locked(void);

bool heater_settled(void);  // true = heater off long enough to read the TC

// 74HC123 hardware watchdog
void hwdog_init(void);
void hwdog_kick(void);
bool hwdog_running(void);   // 1Q level on GP22

#endif
