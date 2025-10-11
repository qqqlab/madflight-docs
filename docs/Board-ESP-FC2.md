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
|**RowD**| | | | | | |G48|G47|G46|
|**RowC**|GND|+5V|G5|G8|G26|G38|G41|GND|+5V|
|**RowB**|GND|+5V|G6 RX1 <- GPS TX|G9 TX1 -> GPS RX|G33 SDA|G39 SCL|G42|GND|+5V|
|**RowA** (board edge)|GND|+5V|G7 RX0 <- Receiver TX|G21 TX0 -> Receiver TX|G34|G40|G45|GND|+5V|

Connect 4 ESCs to the round SMD pads marked S1-S4 at the corners (GPIO1-4), and the battery+ (round) and battery- (square) pads.

Or connect brushed motors to the square SMD pads marked M1-M4 at the corners (GPIO1-4), and the round battery+ pads.

### Pinout for a Airplane

| Row | Pin1 | Pin2 | Pin3 | Pin4 | Pin5 | Pin6 | Pin7 | Pin8 | Pin9 |
|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
|**RowD**| | | | | | |G48|G47|G46|
|**RowC**|GND|+5V|G5 Rudder Servo|G8|G26|G38|G4 Elevator Servo|+5V|GND|
|**RowB**|GND|+5V|G6 RX1 <- GPS TX|G9 TX1 -> GPS RX|G33 SDA|G39 SCL|G42 Aileron Servo|+5V|GND|
|**RowA** (board edge)|GND|+5V|G7 RX0 <- Receiver TX|G21 TX0 -> Receiver TX|G34|G40|G45 Motor ESC|+5V|GND|

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

Set `#define MF_BOARD "brd/madflight_FC2.h"` to use this configuration, it can be modified with `madflight_config` configuration settings in your program.

| GPIO | External Pin Function | Internal Function |
|:-:|:-|:-|
 0 | [SMD Pad] | BT (Boot) button
 1 | [SMD Pad] OUT0 - S1 ESC output / M1 brushed motor | 
 2 | [SMD Pad] OUT1 - S2 ESC output / M2 brushed motor | 
 3 | [SMD Pad] OUT2 - S3 ESC output / M3 brushed motor | 
 4 | [SMD Pad] OUT3 - S4 ESC output / M4 brushed motor | 
 5 | [RowC,Pin3] OUT7| 
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
41 | [RowC,Pin7] OUT6| 
42 | [RowB,Pin7] OUT5| 
43 | [SMD Pad] Programmer TX | 
44 | [SMD Pad] Programmer RX | 
45 | [RowA,Pin7] OUT4| 
46 | [RowD,Pin9] | 
47 | [RowD,Pin8] | 
48 | [RowD,Pin7] | 


Use `#define MF_BOARD "brd/madflight_FC2.h"` instead of `#define MF_BOARD "brd/default.h"` to include this pinout, and use `madflight_config` to override the settings. 

Or, just copy the following config to your program.

```
//Config for madflight FC2

const char madflight_board[] = R""(

//==========================================//
// external components - modify accordingly //
//==========================================//

//--- RCL --- Remote Controller Link  (use serial bus -OR- ppm pin)
rcl_gizmo      NONE  // options: NONE, MAVLINK, CRSF, SBUS, DSM, PPM
rcl_num_ch     8     // number of channels
rcl_deadband   0     // center stick deadband
rcl_ser_bus    0
pin_rcl_ppm   -1

//--- GPS ---
gps_gizmo      NONE  // options: NONE, UBLOX
gps_baud       0   // use 0 for auto baud
gps_ser_bus    1

//--- RDR --- Radar (use serial bus -OR- trig+echo pins)
rdr_gizmo      NONE  // options: NONE, LD2411S, LD2413, USD1, SR04
rdr_baud       0
rdr_ser_bus   -1
pin_rdr_trig  -1
pin_rdr_echo  -1

//--- AHR --- AHRS (keep MAHONY, unless you want to experiment)
ahr_gizmo      MAHONY  // options: MAHONY, MAHONY_BF, MADGWICK, VQF

//--- Serial bus 0 ---
pin_ser0_rx    7
pin_ser0_tx   21

//--- Serial bus 1 ---
pin_ser1_rx    6
pin_ser1_tx    9 

//--- SPI bus 1 ---
pin_spi1_miso -1
pin_spi1_mosi -1
pin_spi1_sclk -1

//--- I2C Bus 1 ---
pin_i2c1_sda  33
pin_i2c1_scl  39

//--- OUT Pins ---
pin_out0       1 //S1 SMD pad at corner
pin_out1       2 //S2 SMD pad at corner
pin_out2       3 //S3 SMD pad at corner
pin_out3       4 //S4 SMD pad at corner
pin_out4      45 //pin with 5v and gnd next to it
pin_out5      42 //pin with 5v and gnd next to it
pin_out6      41 //pin with 5v and gnd next to it
pin_out7       5 //pin with 5v and gnd next to it
pin_out8      -1
pin_out9      -1
pin_out10     -1
pin_out11     -1
pin_out12     -1
pin_out13     -1
pin_out14     -1
pin_out15     -1

//=====================================//
// internal components - do not modify //
//=====================================//

//--- IMU --- Inertial Measurement Unit  (use spi -OR- i2c bus)
imu_gizmo      ICM42688    // options: NONE, BMI270, MPU6000, MPU6050, MPU6500, MPU9150, MPU9250, ICM45686, ICM42688
imu_bus_type   SPI     // options: SPI, I2C (not all combinations of gizmo and bus_type are supported)
imu_align      CW0     //board edge with pins is forward direction - options: CW0, CW90, CW180, CW270, CW0FLIP, CW90FLIP, CW180FLIP, CW270FLIP
imu_spi_bus     0 //spi
pin_imu_cs     17 //spi
pin_imu_int    13 //spi and i2c
imu_i2c_bus    -1 //i2c
imu_i2c_adr     0 //i2c: enter decimal i2c address, not hex (use 0 for default i2c address)

// IMPORTANT: the IMU sensor should be the ONLY sensor on the selected bus


//--- BAR --- Barometer
bar_gizmo      HP203B  // options: NONE, BMP390, BMP388, BMP280, MS5611, HP203B
bar_i2c_adr    118  //always 118 (0x76) for HP203B
bar_i2c_bus    0

//--- MAG --- Magnetometer
mag_gizmo      QMC6309  // options: NONE, QMC5883, QMC6309, RM3100
mag_align      CW180   //board edge with pins is forward direction - options: CW0, CW90, CW180, CW270, CW0FLIP, CW90FLIP, CW180FLIP, CW270FLIP
mag_i2c_adr    124   //always 124 (0x7C) for QMC6309
mag_i2c_bus    0

//--- BAT --- Battery Monitor  (use i2c bus -OR- adc pins)
bat_gizmo      INA226  // options: NONE, ADC, INA226, INA228
bat_i2c_adr    64
bat_i2c_bus    0
pin_bat_i     -1
pin_bat_v     -1
bat_cal_v      1 //adc voltage scale, value is: actual_voltage_in_v / adc_reading
bat_cal_i      0.002 //adc current scale, value is: actual_current_in_a / adc_reading; for ina226/228: rshunt value in ohm

//--- BBX --- Black Box Data Logger  (use spi -OR- mmc)
bbx_gizmo      SDMMC  // options: NONE, SDSPI, SDMMC
pin_bbx_cs    -1  // spi
bbx_spi_bus   -1  // spi
pin_mmc_dat   37  // mmc
pin_mmc_clk   36  // mmc
pin_mmc_cmd   35  // mmc

//--- LED ---
led_gizmo       RGB // options: NONE, HIGH_IS_ON, LOW_IS_ON, RGB
pin_led         12

//--- SPI bus 0 ---
pin_spi0_miso 14
pin_spi0_mosi 15
pin_spi0_sclk 16

//--- I2C Bus 0 ---
pin_i2c0_sda  11
pin_i2c0_scl  10

)""; //end of madflight_board
```
