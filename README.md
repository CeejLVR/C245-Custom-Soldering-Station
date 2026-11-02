# C245 Soldering Station
 
CSE321 project by Christian Jagiello and Ryan Luzi.
 
A homemade soldering station for JBC C245 tips. A Raspberry Pi Pico runs the show using FreeRTOS. This repo is the firmware only; the circuit diagrams are in the project docs.
 
## How to build
 
1. Clone the repo, then grab the libraries it needs:
```
   git submodule update --init --recursive
```
2. Open the folder in VS Code with the Raspberry Pi Pico extension.
3. Click **Compile**. To flash, hold BOOTSEL while plugging in the Pico and click **Run**.
The finished file is `build/solder_station.uf2`.
 
## How it works
 
The Pico does four jobs at once. FreeRTOS gives each job its own task and makes sure the important ones always go first.
 
| Task | How often | Job |
|---|---|---|
| Safety | every 25 ms | Checks that nothing has gone wrong and keeps the watchdog happy |
| Control | every 100 ms | Reads the tip temperature and decides how much to heat |
| UI | about 10 times a second | Draws the screen and reads the knob and buttons |
| Log | every 200 ms | Prints readings to the computer over USB |
 
The tip's temperature sensor shares wires with the heater, so the temperature can only be read while the heater is off. Every 100 ms the station:
 
1. turns the heater off
2. connects the sensor and takes a reading (about 8 ms)
3. disconnects the sensor
4. turns the heater back on for as long as the PID says
The code never lets the sensor and the heater be connected at the same time. Right now that rule only exists in software.
 
## Controls
 
- **Turn the knob:** change the temperature by 5 °C
- **Press the knob:** heater on/off (also clears an error)
- **CON:** next preset. Hold it to save the current temperature as a preset.
- **BAK:** switch between the number screen and the graph
"DRY" on the screen means the heater is disabled in the code. Settings save automatically 2 seconds after you stop changing them.
 
## Safety
 
- **Hardware watchdog (74HC123):** the Pico has to keep poking this chip. If it stops, the heater is cut off within about 0.1 s, even if the code has crashed. It is tested every time the station starts.
- **Pico watchdog:** if the safety task freezes, the Pico restarts itself.
- **Software checks:** the heater shuts off if the tip goes over 480 °C, if the temperature isn't rising while heating hard, or if a sensor goes missing.
- **Heater is off by default:** `CFG_HEATER_ARMED` is 0, so the heater pin never turns on until you change it. Power is also capped at 50% for now.
Warning: none of this protects against the GATE wire shorting to ground on the breadboard. Only the fuse does.
 
## Testing (in this order)
 
Keep `CFG_HEATER_ARMED` at 0 until step 5.
 
1. **Pico alone on USB.** It should show an error (nothing is connected yet), the buttons should work, and the log should print.
2. **Add the watchdog and logic chips.** The error should go away. Unplug the GP21 wire and the watchdog error should come back.
3. **Add the ADC and temperature sensor, heater power off.** Warm the tip with another iron. `tc_uv` in the log should go up; if it goes down, set `TC_POLARITY` to -1. Pull the tip out and the screen should say NO TIP.
4. **Scope GP6 and GP15.** GP6 should pulse briefly every 100 ms. GP15 should never go high.
5. **Set `CFG_HEATER_ARMED` to 1.** Use a 2.5 Ω dummy load and a current-limited bench supply at about 5 V. GP15 and GP6 should never be high together.
6. **Real tip, about 5 V, 1–2 A limit.** Keep the iron in its stand and stay with it. Raise the voltage slowly and tune the PID from the log.
7. **Calibrate** using solder melting points (63/37 melts at 183 °C, SAC305 at about 217–220 °C) by adjusting `TC_SENS_UV_PER_C` and `TC_OFFSET_UV`.
## Reading the log
 
```
# ms,state,tip_c,cj_c,tc_uv,set_c,demand,applied,tip,armed,fault
12400,HEATING,214.6,24.50,4006.3,320,0.500,0.500,1,1,none
```
 
`demand` is how much heat the PID wants. `applied` is how much actually went out (0 if the heater is disabled).
 
## Where things are
 
- `include/config.h`: every setting in one place (timings, limits, PID, calibration, heater on/off switch)
- `include/PinDefinitions.h`: which pin does what
- `src/control.c`: the control loop, plus the queues the tasks use to talk to each other
- `src/safety.c`: the safety task and watchdog self test
- `src/heater.c`: heater and sensor switching, the safety rule between them, and the 74HC123
- `src/sensors.c`: ADS1220, MCP9808 and the thermocouple math
- `src/ui.c`: screens and button handling
- `src/board.c`: shared I2C bus, encoder and buttons
- `src/storage.c`: settings saved to flash
- `src/main.c`: startup, RTOS error hooks and the USB log
## Libraries used
 
- `lib/pico-sdk`, `lib/FreeRTOS-Kernel`: git submodules
- `lib/ssd1306`: [pico-ssd1306](https://github.com/daschr/pico-ssd1306) by David Schramm (MIT). Changed a little so it can share the I2C bus: writes time out instead of hanging, the frame can be sent one page at a time, and it uses a static buffer. Every change is marked `C245:` in the source.
- `lib/pid`: C port of the [Arduino PID Library](https://github.com/br3ttb/Arduino-PID-Library) v1.2.1 by Brett Beauregard (MIT), with stronger anti-windup. See `pid.h` for the list of changes.
- `lib/quadrature_encoder`: the PIO encoder program from [pico-examples](https://github.com/raspberrypi/pico-examples) (BSD-3), unchanged.
- Flash writes go through the SDK's `flash_safe_execute()`.
## Still to do
 
- Hardware version of the sensor/heater safety rule
- Stand detection, buzzer, LEDs
- Calibration menu
- Changing PID values over USB without reflashing
The screen font comes with pico-ssd1306.
