# Sensor Boards

An overview of available sensor boards.

* Table last updated: July 2025
* Parts marked with &#x2705; are supported by _madflight_
* Parts with &#x1F6A7; are work in progress, see Pull Requests on Github
* The "Breakout Board?" column lists price if board is cheap & easy to procure
* Prices in USD from online platforms incl shipping

## 6-axis IMU

| Part    | Breakout Board? | Rate (Hz) | Gyro Noise (mdps/&radic;Hz) | Acc Noise (µg/&radic;Hz) | Notes |
|-|:-:|:-:|:-:|:-:|-|
MPU-6000 &#x2705;| yes | 8k gyro, 1k acc ||| Released 2011, EOL
MPU-6050 &#x2705;| yes | 8k gyro, 1k acc ||| I2C only, Released 2011, EOL
MPU-6500 &#x2705;| $2 | 8k gyro, 4k acc | 10 | 300 | upgraded MPU-6000, Released 2014, NRND
BMI160     | $2 | 3.2k gyro, 1.6k acc | 8 | 180 |
BMI270  &#x2705;| no | 6k gyro | 7 | 160 | Used in current commercial FC as replacement for MPU-6000/6500
LSM6DS3    | $2 | | 7 | 180 |
LSM6DSO    | $10 | 6.7k | 3.8 | 110 |
LSM6DSL    | no | 6.7k | 4 | 80 |
LSM6DSM    | no | 6.7k | 3.8 | 130 |
LSM6DSV    | no | 7.68k | 2.8 | 60 |
ICM-20602  | $5 | | 4 | 100 | NRND
ICM-20649  | no |  | 17.5 | 285 | NRND
ICM-40609-D| no | 32k gyro+acc | 4.5 | 100 | Marketed for drones, replacement of ICM-20602 and MPU-6500
ICM-42670-P| $3 | 1.6k gyro+acc | 7 | 100 | Marketed for drones
ICM-42688-P &#x2705;| $6 | 32k gyro | 2.8 | 70 | Used in current high performance FC
ICM-45686 &#x2705;| no | 6.4k gyro+acc | 3.8 | 80 |

## 9-axis IMU

| Part    | Breakout Board? | Interface | Notes |
|-|:-:|-|-|
MPU-9150 &#x2705;| $7 | I2C | 8k gyro, MPU6050 + AK8975, EOL
MPU-9250 &#x2705;| $7 | SPI 20MHz, I2C | 8k gyro, 4k acc, 100Hz mag, MPU6500 + AK8963, Released 2014 (EOL) Note: many fake or relabelled MPU-6500 chips sold as MPU-9250 on the market...
MPU-9255 &#x2705;| $9 | SPI 20MHz, I2C | 8k gyro, MPU-6000 + AK8963, EOL
ICM-20948 &#x1F6A7;| $9 | SPI 7MHz, I2C | noise: 15 mdps/&radic;Hz 230 µg/&radic;Hz, replacement of MPU-9250/9255, ICM-20649 + AK09916, Released 2018
LSM9DSO  | no

## Magnetometer

Earth's magnetic field strength at its surface ranges from 25 to 65 μT (0.25 to 0.65 G).

| Part    | Breakout Board? | Sampling Rate | LSB Resolution | ADC | Full Scale | Notes |
|-|:-:|-|-|-|-|-|
QMC5883L &#x2705;| $2 | 200 Hz | 8nT @ ±200µT | 16-bit | ±200 or ±800µT | chip marking "5883"
HMC5883L  | $2 | 160 Hz| 70nT @ ±100µT | 12-bit | ±100 to ±800µT | chip marking "L883"
HMC5983   | $2 | 220 Hz | 70nT @ ±100µT | 12-bit | ±100 to ±800µT
MMC5983MA |$15 | 1000 Hz| 6nT @ ±800µT | 18-bit | ±800µT
AK8963 &#x2705;|    | 100 Hz| 150nT | 16-bit | | integraded in MPU9250
AK8975 &#x2705;|    | 100 Hz| 300nT | 13-bit | | integraded in MPU9150
AK09916   |    | 100 Hz| 150nT | 16-bit | | integraded in ICM-20948
IST8310   |    | 200 Hz| 300nT | 14-bit
LIS2MDL   |    | 100 Hz| 15nT | 16-bit
BMM150    | $7 | 300 Hz| 300nT | | ±1300µT | NRND
BMM350    | $7 | 400 Hz| 100nT | 16-bit | ±2000µT |
RM3100 &#x2705;|$25 | 500 Hz| 13nT @ 150 Hz | no ADC | ±800µT | Uses coils, not hall sensors
QMC6309 &#x2705;| | 1500 Hz| 25nT | 16-bit | ±800µT or ±3200µT

## Barometer

When used as altimeter: approx 12 Pa (0.12 mbar) per meter at sea level.

| Part    | Breakout Board? | Relative Precision "Marketing Bla Bla" | ADC Resolution | Max Measurement Rate | RMS Noise | Notes |
|-|:-:|-|-|-|-|-|
DPS310    | $3 | "relative accuracy 6Pa 50cm" | 0.06Pa | 128Hz | 0.35Pa @35Hz | 2.5x2x1 mm, 8 pin, pitch 0.65mm 
SPL06-001 | $2 | 6Pa 50cm | 0.06Pa | 128Hz | 1.2Pa @35Hz | Registers identical to DSP310, but noisier
BMP180    | $2
BMP280 &#x2705;| $2 | "relative accuracy 12Pa 100cm" | 0.25Pa raw 20 bit ADC value | 26-167Hz OSR=16-1 | 2.5Pa 20cm @125Hz | 2.5x2x1mm, 8 pin, pitch 0.65mm
BMP388 &#x2705;| $2 | 8Pa 66cm | | 200Hz
BMP390 &#x2705;| $3 | "relative accuracy 3Pa 25cm" | 2.6Pa 16bit OSR=1 to 0.8Pa 21bit OSR=32 | 25-200Hz OSR=32-1| 0.9Pa @25Hz OSR=32, 5Pa  @100Hz | 2x2x0.75mm, 10 pi, pitch 0.5mm
MS5611 &#x2705;| $4 | "high resolution 10cm" | 0.014Pa raw 24 bit ADC value | 120-2000Hz OSR=4096-256 | | 5x3x1 mm, 8 pin, pitch 1.25mm
MS5607    | | "high resolution 20cm" | 0.014Pa raw 24 bit ADC value | 120-2000Hz OSR=4096-256 | | 5x3x1 mm, 8 pin, pitch 1.25mm
MPL3115A2 | $5 |  | 0.25Pa | 166Hz | 19Pa @166Hz, 1.5Pa @2Hz
LPS22HB   | $6 | 10Pa 80cm | 0.025Pa | 75Hz
LPS22DF   | $14 | 1Pa 8cm | 0.025Pa | 200Hz
ILPS22QS  | $14 | 1.5Pa 12cm | 0.025Pa | 200Hz
HP203B &#x2705;| | "Altitude Resolution down to 0.1 meter" | 0.01Pa 20 bit value (24bit ADC) | 8-240Hz OSR=4096-128| | 3.8×3.6×1.2 mm, 8 pin, pitch 0.9mm

## Multi Sensor Modules

| Module | Price | DOF | Sensors |
|-|:-:|-|-|
GY-85 | | 9DOF | ITG3205, ADXL345, HMC5883L
GY-86 | | 10DOF | MPU6050, HMC5883L, MS5611 &#x2705;
GY-87 | $4 | 10DOF | MPU6050 &#x2705;, HMC5883L, BMP180
GY-91 | $8 | 10DOF | MPU9255 &#x2705; (not MPU9250), BMP280 &#x2705;
GY-521 | $2 | 6DOF | MPU6050 &#x2705;
GY-912 | $11 | 10DOF | ICM20948 &#x1F6A7;, BMP388 &#x2705;

## Current Sensors

| Part | Breakout Board? | Notes |
|-|:-:|-|
INA219 | $1 | 12-bit ADC, 0-26V
INA226 &#x2705;| $1.5 | 16-bit ADC, 0-36V, 1LSB = 1.25mV; ±32A with 2.5m&ohm; 3W shunt, 1LSB = 1mA
INA228 &#x2705;| $5 | 20-bit ADC, 0-85V, 1LSB = 0.2mV; ±80A with 0.5m&ohm; 3W shunt, 1LSB = 0.16mA

## Radar Modules

| Part | Module Price | Manufacturer | Max Range | Resolution | Sample Rate | Notes |
|-|:-:|-|-|-|-|-|
LD2411S &#x2705;|$4|Hi-link|6m|1cm (accuracy 5%)|10Hz|24G Radar, Distance detection - Tested stationary indoor approx 1m above floor: gives very erratic values and appears not to react on change in height... Not useful for _madflight_...
LD2413 &#x2705;|$11|Hi-link|10m|1mm (accuracy 3mm)|20Hz|24G Radar, Liquid level - Tested stationary indoor approx 1m above floor: works but about 2 second delay reporting change in height... Not useful for _madflight_...
LD2451|$7|Hi-link|100m|1m||24G Radar, Vehicle detection, tracks relative speed, angle and distance of multiple targets
RD-03E|$3|Ai-tinker|6m|1cm (accuracy 5%)||24G Radar, Ranging Firmware (also exist with Gesture Firmware)
SR04M-2 &#x2705;|$3|no-name|6m||20Hz|Waterproof Ultrasonic, pwm output. - Tested stationary indoor approx 1m above floor: works but sensor axis needs to be perpendicular (approx. +/-15 degrees) to floor to give correct readings.
