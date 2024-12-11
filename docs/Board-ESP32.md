
# ESP32-S3 / ESP32 Boards

## Arduino IDE Setup

madflight for ESP32-S3/ESP32 requires [Arduino-ESP32 v3.x.x](https://github.com/espressif/arduino-esp32)

madflight v1.1.2 and earlier requires Arduino-ESP32 v2.x.x

Start the Arduino IDE and select menu Tools->Board Manager to install this software.

## PlatformIO Setup

Clone or download the madflight repository from GitHub.

Start PlatformIO, press the "Import Arduino Project" button, and import a madflight example.

Adapt the platformio.ini file as follows:

```ini
; PlatformIO Project Configuration File for madflight

[env]
; Set this to the folder where madflight.h is located
; Default locations when installed via the Arduino IDE:
;    Windows: C:\Users\{username}\Documents\Arduino\libraries\madflight
;    macOS:   /Users/{username}/Documents/Arduino/libraries/madflight
;    Linux:   /home/{username}/Arduino/libraries/madflight

lib_extra_dirs = /ENTER/MADFLIGHT/FOLDER/HERE

[env:madflight_esp32]
platform = espressif32
board = esp32dev
framework = arduino
```

## Pinout ESP32-S3-DevKitC-1

This is the default pinout for ESP32-S3. It is optimized for the Espressif ESP32-S3-DevKitC-1 (44 pin) board. This pinout is defined in madflight_board_default_ESP32-S3.h, but can be modified with `#define HW_PIN_XXX` in your program.

NOTE: Many clones of this board exist, which use various ESP32-S3-VROOM modules and/or have different on-board hardware (LED, RGB LED, SDCARD, etc.) Set Arduino IDE Board settings and HW_PIN_XXX defines accordingly.

Note: Pin numbers refer to the GPIO numbers, not to the physical pin number on a board.

| Function | GPIO | Board | GPIO | Function |
| --: | :-- | :--: |--: | :-- |
3V3 out | 3V3 | Antenna | G | GND
3V3 out | 3V3 | | 43 | TX serial debug UART port
reset button | RST | | 44 | RX serial debug UART port
PWM1 | 4 | | 1 | -
PWM2 | 5 | | 2 | LED (not on all boards)
PWM3 | 6 | | 42 | -
PWM4 | 7 | | 41 | -
PWM5 | 15 | | 40 | SD_MMC_DATA
PWM6 | 16 | | 39 | SD_MMC_CLK
RCIN_TX | 17 | | 38 | SD_MMC_CMD
RCIN_RX | 18 | | 37 | PSRAM (on boards with Octal PSRAM)
I2C_SDA | 8 | | 36 | PSRAM (on boards with Octal PSRAM)
GPS_RX | 3 | | 35 | PSRAM (on boards with Octal PSRAM)
GPS_TX | 46 | | 0 | boot button
I2C_SCL | 9 | | 45| -
IMU_CS | 10 | | 48 | RGB_LED
SPI_MOSI | 11 | | 47 | -
SPI_MISO | 12 | | 21 | -
SPI_SCLK | 13 | | 20 | USB_D+ (serial debug alternate)
IMU_EXTI | 14 | | 19 | USB_D- (serial debug alternate)
5V in (*) | 5V | | G | GND
GND | G | USB connectors | G | GND

(*) 5V input via diode from BEC. Without a diode take care not connect USB and the battery at the same time!

![](img/ESP32-S3_DevKitC-1_pinlayout_v1.1.jpg)

## Setup ESP32-S3 in the Arduino IDE 

In the Aduino IDE select board: "ESP32S3 Dev Module" and set the following options in the Tools menu:

- Flash Mode: see table below
- Flash Size: see x in table below
- PSRAM: see table below

|ESP32-S3 module|Flash Mode|PSRAM|Notes
|-|-|-|-|
ESP32-S3-WROOM-1-Nx     | QIO 80MHz | - |
ESP32-S3-WROOM-1-Hx     | QIO 80MHz | - | High temperature version
ESP32-S3-WROOM-1-NxR2   | QIO 80MHz | QSPI PSRAM |
ESP32-S3-WROOM-1-NxR8   | QIO 80MHz | QSPI PSRAM | pins IO35, IO36, and IO37 are connected to the Octal SPI PSRAM and are not available
for other uses.
ESP32-S3-WROOM-1-NxR16V | QIO 80MHz | OPI PSRAM | 1.8V SPI, pins IO35, IO36, and IO37 are connected to the Octal SPI PSRAM and are not available
for other uses.
ESP32-S3-WROOM-2-NxR8V  | OPI 80MHz | OPI PSRAM | 120MHz SPI, 1.8V SPI, pins IO35, IO36, and IO37 are connected to the Octal SPI PSRAM and are not available
for other uses.

#### Setup Serial Port

Settings for 3 serial ports (Serial, Serial1 and Serial2), Serial connected to "USB-UART", Serial0 not connected (gives error 'Serial0' was not declared).

 - USB CDC On Boot: Disabled 

Settings for 4 serial ports (Serial, Serial0, Serial1 and Serial2), Serial connected to "USB-OTG", Serial0 connected to "USB-UART"

 - USB CDC On Boot: Enabled
 - USB DFU On Boot: Disabled
 - USB Mode: USB-OTG (TinyUSB)

#### Setup Programming Interface

Settings for programming via "USB-UART" usb port (programming works without pressing boot/reset buttons)

- Upload Mode: UART0 / Hardware CDC

Settings for programming via "USB-OTG" usb port (For programming: press boot, press+release reset, release boot, then upload; For serial monitor: press+release reset, then open serial monitor)

- Upload Mode: USB-OTG CDC (TinyUSB)

## Pinout ESP32 DevKitC

This is the default pinout for ESP32. It is optimized for the Espressiv ESP32 DevKitC (38 pin) board. This pinout is defined in madflight_board_default_ESP32.h, but can be modified with `#define HW_PIN_XXX` in your program.

Many clones of this board exist, which might have different ESP32 modules and/or different on-board hardware (LED, RGB LED, SDCARD, etc.) Set Arduino IDE Board settings and HW_PIN_XXX defines accordingly.

| Function | GPIO | Board | GPIO | Function |
| --: | :-- | :--: | --: | :-- |
| 3V3 out      | 3V3 | Antenna side            |  GND | GND
| reset button | EN |                            | 23 | I2C_SDA
| SPI_MISO     | VP 36 input only |              | 22 | I2C_SCL
| IMU_EXTI     | VN 39 input only |            | 1 TX | USB Serial Debug TX
| BAT_V        | 34 input only |               | 3 RX | USB Serial Debug RX
| RCIN_RX      | 35 input only |                 | 21 | SPI_MOSI
| RCIN_TX      | 32 |                           | GND | GND
| PWM1         | 33 |                            | 19 | SPI_SCLK
| PWM2         | 25 |                            | 18 | IMU_CS
| PWM3         | 26 |                       | strap 5 | GPS_TX
| PWM4         | 27 |                            | 17 | GPS_RX
| PWM5         | 14 |                            | 16 | PWM11
| PWM6         | 12 |                             | 4 | PWM10
| GND          | GND |                       | boot 0 | PWM9
| PWM7         | 13 |                       | strap 2 | LED     
| nc           | D2 9 flash |              | strap 15 | PWM8
| nc           | D3 10 flash |           | flash 8 D1 | nc
| nc           | CMD 11 flash |          | flash 7 D0 | nc
| 5V input (*) | 5V | USB connector     | flash 6 CLK | nc

(*) 5V input via diode from BEC. Without a diode take care not connect USB and the battery at the same time!

Note: During boot the input voltage levels (pull up/pull down) on strap pins have a configuration function, therefor these pins are used as output only.

ESP32 has 6 strapping pins:
- GPIO0: internal pull-up (boot button)
- GPIO2: internal pull-down
- GPIO4: internal pull-down
- GPIO5: internal pull-up
- GPIO12: internal pull-down
- GPIO15: internal pull-up

![](img/ESP32-DEV-KIT-DevKitC-v4-pinout-mischianti.png)

## ESP32-S3/ESP32 Hardware

ESP32-S3 and ESP32 are very similar chips. The ESP32-S3 is more recent: it has more pins, USB-OTG (3 UARTs plus 1 USB-OTG CDC UART), improved single block RAM structure. The ESP32 is better if you need a lot of PWM channels, it has 16 PWM (LEDC) outputs, versus 8 on ESP32-S3.

#### Dual Core / FPU

ESP32 and ESP32-S3 both have dual core CPU, but single core FPU. ESP-IDF implementation limits [float usage](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/freertos_idf.html#floating-point-usage) to a single core, and float can not be used in interrupts. FreeRTOS is always enabled and a watchdog limits interrupt execution time.

madflight uses float and is therefor limited to single core operation. The IMU loop runs as a high priorty task, triggered by the IMU interrupt.

#### FreeRTOS

FreeRTOS is enabled by default.

#### I2C

Arduino-ESP32 v2.0.17 has an [I2C bug](https://github.com/espressif/esp-idf/issues/4999) which causes the bus to hang for 1 second after a failed read, which can happen a couple times per minute. This makes Wire I2C for IMU not a real option...

A workaround is to use #define USE_ESP32_SOFTWIRE which enables software I2C, but this does not work well with all sensors.
  
(!) So, until a better I2C solution is available: use an SPI IMU sensor on ESP32.

NOTE: as of June 2024 this bug is apparently fixed, but not yet confirmed with madflight.
