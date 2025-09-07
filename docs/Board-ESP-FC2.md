# madflight FC2 - ESP32-S3 Flight Controller

![](img/madflight-ESP-FC2-1.png){: style="width:30%"} ![](img/madflight-ESP-FC2-2.png){: style="width:30%"} ![](img/madflight-ESP-FC2-3.png){: style="width:30%"}

[Schematic](/img/madflight-ESP-FC2.pdf)

[Buy](https://https://www.tindie.com/products/madflight/esp32-flight-controller-esp-fc2/)

This is a flight controller based on the ESP32-S3 dual core microcontroller.

Made with DIY in mind: easy accessible 2.54mm pin headers, standard mounting points to fit quadcopter frames, but can also be used for non-drone projects.

For additional flexibility: the included DC-DC board can be mounted at the top or bottom (see pictures). Or can be omitted when you use an external BEC or if want to build an ultralight a 1S battery setup.

The ESP32-S3 has 3 hardware UARTs that can be connected to any of the GPIOs.

## Specifications

### Specifications FC2 Flight Controller

- Processor: ESP32-S3 with 4 MB flash and 2 MB psram
- Gyro/Acc: ICM-42688-P
- Magnetometer: QMC6309
- Barometer: HP203B
- Battery Monitor: INA226 with 16 bit accuracy
- Micro SDCARD with MMC interface
- RGB LED
- Operates with 1-4S LiPo/LiIon battery
- Standard M4 mounting holes with 30.5 mm spacing
- 18 GPIOs accessible via 2.54mm pin headers
- 6 GND and 6 Power pins on pin header
- 4 brushed motor drivers
- 4 ESC pins
- Reset (EN) and Boot (BT) buttons
- Dimensions: 40.5 x 40.5 x 5 mm
- Weight: 5.1 gr (without pins, DC-DC)

### Specifications DC-DC Converter

- Output: 5V 2A continous / 3A peak
- Input: 6.5~20V (for 2-4S battery)
- Dimensions: 20 x 10 x 5 mm
- Weight: 1.8 gr

## Suggested Pinouts

### Pinout for a Quadcopter

| Row | Pin1 | Pin2 | Pin3 | Pin4 | Pin5 | Pin6 | Pin7 | Pin8 | Pin9 |
|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
|**RowD**| | | | | | |48|47|46|
|**RowC**|GND|+5V|5|8|26|38|41|GND|+5V|
|**RowB**|GND|+5V|6 RX1 <- GPS TX|9 TX1 -> GPS RX|33 SDA|39 SCL|42 free|GND|+5V|
|**RowA** (board edge)|GND|+5V|7 RX0 <- Receiver TX|21 TX0 -> Receiver TX|34|40|45 LED Strip|GND|+5V|

Connect 4 ESCs to S1-S4 (round), + (round) and - (square) pads at the corners

Or connect brushed motors to M1-M4 (square) and the + (round) pads at the corners

### Pinout for a Airplane

| Row | Pin1 | Pin2 | Pin3 | Pin4 | Pin5 | Pin6 | Pin7 | Pin8 | Pin9 |
|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
|**RowD**| | | | | | |48|47|46|
|**RowC**|GND|+5V|5 Rudder Servo|8|26|38|41 Elevator Servo|+5V|GND|
|**RowB**|GND|+5V|6 RX1 <- GPS TX|9 TX1 -> GPS RX|33 SDA|39 SCL|42 Aileron Servo|+5V|GND|
|**RowA** (board edge)|GND|+5V|7 RX0 <- Receiver TX|21 TX0 -> Receiver TX|34|40|45 Motor ESC|+5V|GND|

## Arduino IDE Setup

_madflight_ for ESP32-S3/ESP32 requires [Arduino-ESP32 v3.x.x](https://github.com/espressif/arduino-esp32)

Start the Arduino IDE and select menu Tools->Board Manager to install this software.

## PlatformIO Setup

1. Clone or download a madflight release to your harddisk

2. Start PlatformIO and open folder `madflight/examples`

Note:

Espressiv stopped Arduino framework support for PlatformIO. Arduino 2 is the lastest Espressiv supported framework version, _madflight_ should still compile with Arduino 2.

If you need Arduino 3, you could try [pioarduino](https://github.com/pioarduino/platform-espressif32)

## Pinout FC2

This is the default ESP-FC2 configuration, the external pins can be reconfigured.

Use `#define MF_BOARD "brd/madflight_FC2.h"` instead of `#define MF_BOARD "brd/default.h"` to include this pinout.


| GPIO | External Pin Function | Internal Function |
|:-:|:-|:-|
 0 | [SMD Pad] | BT (Boot) button
 1 | [SMD Pad] S1 ESC output / [SMD Pad] M1 brushed motor | 
 2 | [SMD Pad] S2 ESC output / [SMD Pad] M2 brushed motor | 
 3 | [SMD Pad] S3 ESC output / [SMD Pad] M3 brushed motor | 
 4 | [SMD Pad] S4 ESC output / [SMD Pad] M4 brushed motor | 
 5 | [RowC,Pin3] | 
 6 | [RowB,Pin3] RX1 <- GPS TX | 
 7 | [RowA,Pin3] RX0 <- Receiver TX | 
 8 | [RowC,Pin4] | 
 9 | [RowB,Pin4] TX1 -> GPS RX | 
10 | | I2C0_SDA (bar,bat,mag)
11 | | I2C0_SCL (bar,bat,mag)
12 | | RGBLED
13 | | IMU_INT
14 | | SPI0_MISO (imu)
15 | | SPI0_MOSI (imu)
16 | | SPI0_SCLK (imu)
17 | | IMU_CS
18 | | IMU_CLKIN
21 | [RowA,Pin4] | 
26 | [RowC,Pin5] RX0 <- Receiver TX| 
33 | [RowB,Pin5] I2C1_SDA | 
34 | [RowC,Pin6] | 
35 |  | SD_CMD (bbx)
36 |  | SD_CLK (bbx)
37 |  | SD_DAT (bbx)
38 | [RowC,Pin6] | 
39 | [RowB,Pin6] I2C1_SCL | 
40 | [RowA,Pin6] | 
41 | [RowC,Pin7] | 
42 | [RowB,Pin7] | 
43 | [SMD Pad] Programmer TX | 
44 | [SMD Pad] Programmer RX | 
45 | [RowC,Pin7] | 
46 | [RowD,Pin9] | 
47 | [RowD,Pin8] | 
48 | [RowD,Pin7] | 
