# STM32 Boards

## Arduino IDE Setup

_madflight_ for STM32 requires:

- [STM32duino Arduino Core for STM32 v2.x.x](https://github.com/stm32duino/Arduino_Core_STM32) Start the Arduino IDE and select menu Tools->Board Manager to install this software.

- [STM32duino FreeRTOS](https://github.com/stm32duino/STM32FreeRTOS) Select menu Tools->Manage Libaries, then search for "STM32duino FreeRTOS".

## PlatformIO Setup

1. Clone or download a madflight release to your harddisk

2. Start PlatformIO and open folder `madflight/extras/PlatformIO_madflight`

## Pinout STM32 Commercial Flight Controllers

In the `src` directory you'll find 400+ Betaflight configuration files for commercial flight controllers. Include the madflight_board_XXX.h header file of your board, and in your program override settings as needed to match your board. 

The Betaflight configuration files are manually generated by a Python script in the 'extras' folder. If your favorite board is missing, run the script to add it.

## Pinout STM32F411 Black Pill

This is the default pinout for STM32. It is optimized for the WeAct STM32F411 Black Pill (40 pin) board. This pinout is defined in madflight_board__STM32.h,  but can be modified with _madflight_ _pin_xxx_ configuration settings in your program.

| Function | GPIO | Board | GPIO | Function |
| --: | :-- | :--: | --: | :-- |
not connected                               | VB  |   SWD pins    | 3V3 | 3V3 out
 _pin_led_                                  | C13 |               | G   | GND
free for future use                         | C14 |               | 5V  | 5V input (*)
free for future use                         | C15 |               | B9  | _pin_i2c1_sda_ (connect to I2C gyro sda)
not connected                               | R   |               | B8  | free for future use
free for future use                         | A0  |               | B7  | _pin_i2c0_scl_ (connect to scl pins of barometer, magnetometer, etc. sensors)
free for future use                         | A1  |               | B6  | _pin_i2c0_sda_ (connect to sda pins of barometer, magnetometer, etc. sensors)
_pin_ser1_tx_ (connect to gps rx)           | A2  |               | B5  | free for future use
_pin_ser1_rx_ (connect to gps tx)           | A3  |               | B4  | free for future use
_pin_imu_cs_ (connect to SPI gyro cs)       | A4  |               | B3  | _pin_ser0_rx_ (connect to radio tx)
 _pin_spi0_sclk_ (connect to SPI gyro sclk) | A5  |               | A15 | _pin_ser0_tx_ (connect to radio rx)
_pin_spi0_miso_ (connect to SPI gyro miso)  | A6  |               | A12 | USB D+
_pin_spi0_mosi_ (connect to SPI gyro mosi)  | A7  |               | A11 | USB D-
_pin_bat_i_ (connect to battery current sensor) | B0  |           | A10 | _pin_out5_ (connect to motor/servo6)
_pin_bat_v_ (connect to battery voltage divider) | B1  |          | A9  | _pin_out4_ (connect to motor/servo5)
_pin_imu_int_ (connect to SPI/I2C gyro interrupt out) | B2  |     | A8  | _pin_out3_ (connect to motor/servo4)
_pin_i2c1_scl_ (connect to I2C gyro scl)    | B10 |               | B15 | _pin_out2_ (connect to motor/servo3)
3V3 out                                     | 3V3 |               | B14 | _pin_out1_ (connect to motor/servo2)
GND                                         | G   |               | B13 | _pin_out0_ (connect to motor/servo1)
5V input (*)                                | 5V  | USB connector | B12 | free for future use

Internally connected: C13 - _pin_led_, A0 - key button

(*) 5V input via diode from BEC. Without a diode take care not connect USB and the battery at the same time!

![](img/STM32-STM32F4-STM32F411-STM32F411CEU6-pinout-high-resolution.png)

## STM32 Hardware

The STM32 MCU family is a huge collection of chips. _madflight_ will run on most stm32duino supported F4, F7, H7 and similar chips, preferred are STM32H743 and STM32F405.

Most supported STM32 targets have a single core MCU with FPU. 

FreeRTOS is required for _madflight_ v1.3 and later.

_madflight_ for STM32 runs the IMU loop by default in interrupt context.

## madflight Limitiations

- Serial Ports: Standard 64 buffer is too small for GPS/MAVLink -> Set `#define SERIAL_RX_BUFFER_SIZE 256` and `#define SERIAL_TX_BUFFER_SIZE 256` in HardwareSerial.h
- OUT: STM32duino PWM drivers have 1 us resolution, too coarse for OneShot
- OUT: PWM pins with shared timer have same frequency
- BBX: SDCARD (not implemented yet)
