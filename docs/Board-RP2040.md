# RP2040 / RP2350 Boards

## Arduino IDE Setup

_madflight_ for RP2040/RP2350 requires [arduino-pico v4.x.x](https://github.com/earlephilhower/arduino-pico)

Start the Arduino IDE and select menu Tools->Board Manager to install this software.

## PlatformIO Setup

Clone or download the _madflight_ repository from GitHub.

Start PlatformIO, press the "Import Arduino Project" button, and import a _madflight_ example.

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

; RP2040/RP2350 is not merged into mainline PlatformIO, see 
; https://arduino-pico.readthedocs.io/en/latest/platformio.html
; on Windows you need to enable long path names, see above link for instructions

[env:madflight_pico]
platform = https://github.com/maxgerhardt/platform-raspberrypi.git
board = pico
framework = arduino
board_build.core = earlephilhower

[env:madflight_pico2]
platform = https://github.com/maxgerhardt/platform-raspberrypi.git
board = rpipico2
framework = arduino
board_build.core = earlephilhower
```

## Pinout Raspberry Pi Pico / Pico2

This is the default pinout for RP2040 and RP2350. It is optimized for Raspberry Pi Pico / Pico2 (40 pin) boards. This pinout is defined in madflight_board__RP2040.h, but can be modified with _madflight_ pin_xxx configuration settings.

| Module <-> HW_PIN_xxx | GPIO | Board | GPIO | HW_PIN_xxx <-> Module |
| --: | :-- | :--: | --: | :-- |
_pin_ser0_tx_ (connect to radio rx) | 0   | USB connector | VBUS     | not connected
_pin_ser0_rx_ (connect to radio tx) | 1   |               | VSYS     | 5V input via diode (*)
GND | GND |               | GND      | GND
_pin_out0_ (connect to motor/servo1) | 2   |               | EN       | not connected
_pin_out1_ (connect to motor/servo2) | 3   |               | 3.3V out | 3V3
_pin_out2_ (connect to motor/servo3) | 4   |               | VREF     | not connected
_pin_out3_ (connect to motor/servo4) | 5   |               | 28_A2    | _pin_bat_v_ (connect to battery voltage divider)
GND | GND |               | GND      | GND
_pin_out4_ (connect to motor/servo5) | 6   |               | 27_A1    | free for future use
_pin_out5_ (connect to motor/servo6) | 7   |               | 26_A0    | free for future use
_pin_ser1_tx_ (connect to gps rx)  | 8   |               | RUN      | reset button to GND
_pin_ser1_rx_ (connect to gps tx) | 9   |               | 22       | _pin_imu_int_ (connect to SPI/I2C gyro interrupt out)
GND | GND |               | GND      | GND
_pin_i2c1_sda_ (connect to I2C gyro sda) | 10  |               | 21       | _pin_i2c0_scl_ (connect to scl pins of barometer, magnetometer, etc.)
_pin_i2c1_scl_ (connect to I2C gyro scl) | 11  |               | 20       | _pin_i2c0_sda_ (connect to sda pins of barometer, magnetometer, etc.)
_pin_spi1_miso_ (connect to sdcard miso) | 12  |               | 19       | _pin_spi0_mosi_ (connect to SPI gyro mosi) 
_pin_bbx_cs _ (connect to sdcard cs)| 13  |               | 18       |  _pin_spi0_sclk_ (connect to SPI gyro sclk)
GND | GND |               | GND      | GND
_pin_spi1_sclk_ (connect to sdcard sclk) | 14  |               | 17       | _pin_imu_cs_ (connect to SPI gyro cs)
_pin_spi1_mosi_ (connect to sdcard mosi) | 15  | JTAG pins     | 16       | _pin_spi0_miso_ (connect to SPI gyro miso)

(*) 5V input via diode from BEC. Without a diode take care not connect USB and the battery at the same time!

![](img/Raspberry-Pi-Pico-rp2040-pinout-mischianti.png)
![](img/Raspberry-Pi-Pico-W-rp2040-WiFi-pinout-mischianti.png)

## RP2040/RP2350 Hardware

RP2350 (Raspberry Pi Pico2) is newer and a lot more powerful than RP2040 (Raspberry Pi Pico).

#### Dual Core / FPU

RP2350 has dual core processor, plus dual single precision FPUs.

RP2040 has dual core processor, but no FPUs.

_madflight_ uses float and is much happier with RP2350 than with RP2040 !

#### PWM

Consecutive even/odd PWM pins (e.g. pins 2,3 or 10,11) share the same timer and have the same frequency.

#### Serial

_madflight_ uses a custom high performance SerialIRQ library, because the default Arduino Serial transmitter blocks after sending a couple bytes. Something we don't want.

#### Dual Core Usage

_madflight_ uses FreeRTOS and executes the 1000Hz IMU loop on the second core. The first core is used for the other sensors.

## Available madflight Modules

All _madflight_ modules are available, exept:

- BBX: SDCARD with MMC interface (use SPI interface)
