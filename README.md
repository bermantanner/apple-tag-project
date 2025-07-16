# "YouLostIt" BLE Air Tag

This repository contains the second half of our personal BLE tag project, built with blood, sweat, and plenty of tears. As our first embedded project, we didn’t realize just how much time would be spent reading documentation, debugging mysteriously silent peripherals, and learning that “particular” is an understatement when it comes to hardware. We implemented the firmware on an STM32L4 **B‑L475E‑IOT01A1** discovery board while pair programming mostly in person. The first part of the code is in a separate private repository; this repo continues from there with a more organized layout and additional power optimizations.

## Project Overview

The goal was to build a very low‑power tracker similar to an AirTag. When the device detects that it has been dropped or left behind, it enters a "lost" mode. In that state the tag repeatedly transmits the owner ID and the elapsed time since the loss using Bluetooth LE advertisements and also flashes the information on its LEDs.

Major steps in development were:

1. **LED driver** – simple GPIO driver used for early testing.
2. **Timer driver** – using the low‑power LPTIM peripheral driven from LSI.
3. **I2C driver** – custom implementation for the STM32L4 to communicate with onboard sensors.
4. **Accelerometer driver** – reading motion data from the LSM6DSL.
5. **PrivTag application** – application logic to detect a lost device and output status.
6. **Bluetooth proximity** – custom BLE service to report the ID and duration since drop.
7. **Power optimization** – heavy clock/power gating and STOP2 mode for months of battery life.

After starting at roughly **110 mA** consumption during prototyping, we methodically disabled unused peripherals and switched to STOP2. The final firmware draws about **80 µA**, allowing something like a CR2032 cell battery to last well over three months.

## Repository Layout

```
youlostit-ble/
│   STM32L475VGTX_FLASH.ld    – linker script
│   youlostit-ble.ioc         – STM32CubeMX project
│   Core/
│       Inc/  – application headers
│       Src/  – application source files
│       Startup/ – startup assembly
│   Drivers/
│       CMSIS/ – vendor device headers
│       STM32L4xx_HAL_Driver/ – HAL sources
└── Debug/ – generated build files
```

The `Debug/` directory contains an auto‑generated `makefile` used to build the `youlostit-ble.elf` firmware image with GCC.

## Building

We are not sure if this works well because we just used Debug in STMCube IDE, but here is our best guess for using this code:

A GNU ARM toolchain is required. To compile run:

```bash
cd youlostit-ble/Debug
make
```

The resulting `youlostit-ble.elf` can be flashed to the board using ST‑Link tools.

## Important Code

### Low-power timer
LPTIM1 is configured to run from the LSI clock and generate periodic wakeups:

```c
RCC->CSR |= RCC_CSR_LSION;
while (!(RCC->CSR & RCC_CSR_LSIRDY));
// configure LPTIM1
LPTIM1->IER = LPTIM_IER_ARRMIE; // auto‑reload match interrupt
LPTIM1->CR |= LPTIM_CR_ENABLE;
LPTIM1->CR |= LPTIM_CR_CNTSTRT;
```
【F:youlostit-ble/Core/Src/timer.c†L15-L90】

### Accelerometer setup
The LSM6DSL is placed in high‑performance mode and its data‑ready interrupt is enabled:

```c
data[0] = CTRL1_XL;
data[1] = 0x60; // 16 Hz ODR, ±2 g
if (i2c_transaction(LSM6DSL_ADDR, 0, data, 2)) { /* error */ }

data[0] = INT1_CTRL;
data[1] = 0x01; // INT1_DRDY_XL
```
【F:youlostit-ble/Core/Src/lsm6dsl.c†L22-L37】

### Lost-mode logic
The main loop checks motion every timer tick. If movement stops for 60 seconds the tag becomes discoverable and starts reporting elapsed time via BLE:

```c
if (timer_flag) {
    lsm6dsl_read_xyz(&x, &y, &z);
    delta_x = abs(x - prev_x);
    delta_y = abs(y - prev_y);
    delta_z = abs(z - prev_z);
    total_movement = delta_x + delta_y + delta_z;
    // update state and send message when lost
}
```
【F:youlostit-ble/Core/Src/main.c†L171-L181】
When in lost mode the firmware formats a string with the device name and seconds lost and sends it over BLE every 10 seconds:

```c
unsigned char formatted_str[32];
snprintf((char*)formatted_str, sizeof(formatted_str), "%s %us", device_name, seconds_since_lost);
updateCharValue(NORDIC_UART_SERVICE_HANDLE, READ_CHAR_HANDLE, 0, str_len, formatted_str);
```
【F:youlostit-ble/Core/Src/main.c†L248-L257】
The MCU then enters STOP2 mode to conserve power until the next interrupt:

```c
HAL_SuspendTick();
HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
HAL_ResumeTick();
```
【F:youlostit-ble/Core/Src/main.c†L264-L266】

Large blocks of peripheral clock disabling can be found in `disableUselessShit()` which aggressively powers down unused hardware. (please excuse the naming, we were having fun with this project)

## Results

Our initial prototype consumed about 110 mA – a CR2032 would last under two hours. After optimizing the clocks and entering STOP2 mode, the idle current dropped to roughly 80 µA. This means over three months of battery life while still waking every few seconds to check the accelerometer and send BLE advertisements.

## License

The STM32 HAL sources are provided under ST’s BSD‑3‑Clause license. The rest of the project is released under the MIT license.