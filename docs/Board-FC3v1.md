# madflight FC3 - RP2350B Flight Controller

![](img/madflight-FC3-1.png){: style="width:27.97%"} ![](img/madflight-FC3-2.png){: style="width:21.54%"}  ![](img/madflight-FC3-3.png){: style="width:20.53%"}  ![](img/madflight-FC3-4.png){: style="width:21.95%"}

[Schematic](/img/madflight-FC3.pdf)

[Buy](https://www.tindie.com/products/madflight/flight-controller-raspberry-pi-rp2350b/)

This is a Raspberry Pi Pico2 on steroids. Use it as a flight controller, or for a project that requires precise orientation sensing with the on-board 10DOF best-in-class sensors. 

The layout of the pins is similar to the Raspberry Pi Pico2, but with added 5V power pins to make it easy to connect external components like RC receivers, GPS with compass, 4-in-1 ESC, Servos, Radars, etc. 

Easy mounting: Use the standard 30.5mm x 30.5mm mounting holes to fit your drone frame. Or use double sided tape to mount the board, the board has no components on the back.

Power the board directly from 3 ~ 6V (eg. USB or 1S LiPo), or use the included DC-DC converter to connect larger batteries. For added flexibility the DC-DC converter board can be top or bottom mounted.

Even when using all interal functions, the external pins can be used for:

- 22 PWM pins
- 8 DSHOT pins
- 6 Serial Ports (2 Hardware + 4 PIO UARTs)
- 1 I2C Port
- 1 SPI Port
- 6 ADC pins

## Specifications

#### madflight FC3

- RP2350B with 48 GPIO
- 16MB Flash
- 30 pins accessible via 2.54mm pinheader
- 9 power and 9 ground pins
- 5 additional GPIOs accessible via SMD pads
- Gyro/Acc: ICM-42688-P Precision Gyro/Accelerometer
- QMC5883P Magnetometer
- BMP580 Precision Barometer (2cm noise @ 85Hz sample rate)
- INA226 16-bit Battery Monitor (36V, 40A with 1.25mV, 1.25mA resolution)
- WS2812B RGB LED
- SDCARD with click mechanism and fast 4-bit SDIO interface
- Optional PSRAM or second Flash (empty SO-8 socket on back of board)
- Standard 30.5 x 30.5 mm mounting with 4x Φ4mm holes, with grommets Φ3mm
- Dimensions: 50.7 x 41.6 mm (4.2 mm max height)
- Weight: 5.6 gr (without DC-DC)

#### DC-DC Converter (included)

- Output: 5V 2A continous / 3A peak
- Input: 6.5 ~ 20V
- Dimensions: 20 x 10 x 5 mm
- Weight: 1.8 gr

## Arduino IDE and PlatformIO Setup

See [here](Board-RP2040.md)

## Betaflight Setup

See [here](Board-FC3.md)

## Pinout FC3

Set `#define MF_BOARD "brd/madflight_FC3.h"` to use this configuration, it can be modified with `madflight_config` configuration settings in your program.

| GPIO |Default External Pin Function | Internal Function |
|:-:|:-|:-|
 0 | SER0_TX (connect to radio receiver RX) | 
 1 | SER0_RX (connect to radio receiver TX) | 
 2 | I2C1_SDA (connect to GPS SDA) | 
 3 | I2C1_SCL (connect to GPS SCL) | 
 4 | SER1_TX (connect to GPS RX) | 
 5 | SER1_RX (connect to GPS TX) | 
 6 | OUT0 (connect to ESC for motor 1) | 
 7 | OUT1 (connect to ESC for motor 2) | 
 8 | OUT2 (connect to ESC for motor 3) | 
 9 | OUT3 (connect to ESC for motor 4) | 
10 | free | 
11 | free | 
12 | free | 
14 | free | 
15 | free | 
16 | free | 
17 | free | 
18 | free | 
19 | free | 
20 | free | 
21 | free | 
22 | free | 
23 | free | 
24 | free | 
25 | free | 
26 |  | IMU_CLKIN
27 |  | IMU_INT
28 |  | IMU_SPI1_MISO
29 |  | IMU_CS
30 |  | IMU_SPI1_SCLK
31 |  | IMU_SPI1_MOSI
32 |  | I2C0_SDA (bar,bat,mag)
33 |  | I2C0_SCL (bar,bat,mag)
34 |  | SD_SDIO_CLK or SD_SPI0_SLCK
35 |  | SD_SDIO_CMD or SD_SPI0_MOSI
36 |  | SD_SDIO_D0 or SD_SPI0_MISO
37 |  | SD_SDIO_D1
38 |  | SD_SDIO_D2
39 |  | SD_SDIO_D3 or SD_SPI0_CS
40 | free (ADC0) | 
41 | free (ADC1) | 
42 | free (ADC2) | 
43 | free (ADC3) | 
44 | free (SMD pad ADC4) | 
45 | free (SMD pad ADC5) | 
46 |  | RBGLED
47 |  | QMI_CS1 (for optional PSRAM)

SMD Pads

- GPIO 33 (internal SDA)
- GPIO 32 (internal SCL)
- GPIO 44 (free)
- GPIO 45 (free)
- GPIO 46 (internal RGB LED)
- BAT_INT Battery sensor interrupt pin
- BAR_INT Barometer interrupt pin

## RP2350B Hardware

RP2350B is the 80 pin / 48 GPIO variant of the Raspberry Pi RP2350 processor.

RP2350 has dual core processors with dual single precision FPUs.

_madflight_ uses a custom Serial library, because the default Arduino Serial transmitter blocks after sending a couple bytes. Something we don't want.

_madflight_ uses FreeRTOS and executes the IMU loop on the second core. The first core is used for the other sensors.

## madflight Limitiations

- OUT: Consecutive even/odd PWM pins (e.g. pins 2,3 or 10,11) share the same timer and have the same frequency.

## Previous Versions of this Board

[madflight FC1](Board-FC1.md)
