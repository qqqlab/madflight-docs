# Sensor Boards

An overview of available sensor boards.

* The "Breakout Board?" column lists price if board is cheap & easy to procure.
* Prices in USD from online platforms incl shipping.
* Table last updated: Mar 2025

## 6-axis IMU

| Part    | Breakout Board? | Rate (Hz) | Gyro Noise (mdps/&radic;Hz) | Acc Noise (µg/&radic;Hz) | Notes |
|-|:-:|:-:|:-:|:-:|-|
MPU-6000   | yes | 8k gyro, 1k acc ||| Released 2011, EOL
MPU-6050   | yes | 8k gyro, 1k acc ||| I2C only, Released 2011, EOL
MPU-6500   | $2 | 8k gyro, 4k acc | 10 | 300 | upgraded MPU-6000, Released 2014, NRND
BMI160     | $2 | 3.2k gyro, 1.6k acc | 8 | 180 |
BMI270     | no | 6k gyro | 7 | 160 | Used in current commercial FC as replacement for MPU-6000/6500
LSM6DS3    | $2 | | 7 | 180 |
LSM6DSO    | $10 | 6.7k | 3.8 | 110 |
LSM6DSL    | no | 6.7k | 4 | 80 |
LSM6DSM    | no | 6.7k | 3.8 | 130 |
LSM6DSV    | no | 7.68k | 2.8 | 60 |
ICM-20602  | $5 | | 4 | 100 | NRND
ICM-20649  | no |  | 17.5 | 285 | NRND
ICM-40609-D| no | 32k gyro+acc | 4.5 | 100 | Marketed for drones, replacement of ICM-20602 and MPU-6500
ICM-42670-P| $3 | 1.6k gyro+acc | 7 | 100 | Marketed for drones
ICM-42688-P| $6 | 32k gyro | 2.8 | 70 | Used in current high performance FC
ICM-45686  | no | 6.4k gyro+acc | 3.8 | 80 |

## 9-axis IMU

| Part    | Breakout Board? | Interface | Notes |
|-|:-:|-|-|
MPU-9150  | $7 | I2C | 8k gyro, MPU6050 + AK8975, EOL
MPU-9250  | $7 | SPI 20MHz, I2C | 8k gyro, 4k acc, 100Hz mag, MPU6500 + AK8963, Released 2014 (EOL) Note: many fake or relabelled MPU-6500 chips sold as MPU-9250 on the market...
MPU-9255  | $9 | SPI 20MHz, I2C | 8k gyro, MPU-6000 + AK8963, EOL
ICM-20948 | $9 | SPI 7MHz, I2C | noise: 15 mdps/&radic;Hz 230 µg/&radic;Hz, replacement of MPU-9250/9255, ICM-20649 + AK09916, Released 2018
LSM9DSO  | no

## Magnetometer

Earth's magnetic field strength at its surface ranges from 25 to 65 μT (0.25 to 0.65 G).

| Part    | Breakout Board? | Sampling Rate | LSB Resolution | ADC | Full Scale | Notes |
|-|:-:|-|-|-|-|-|
QMC5883L  | $2 | 200 Hz | 8nT @ ±200µT | 16-bit | ±200 or ±800µT | chip marking "5883"
HMC5883L  | $2 | 160 Hz| 70nT @ ±100µT | 12-bit | ±100 to ±800µT | chip marking "L883"
HMC5983   | $2 | 220 Hz | 70nT @ ±100µT | 12-bit | ±100 to ±800µT
MMC5983MA |$15 | 1000 Hz| 6nT @ ±800µT | 18-bit | ±800µT
AK8963    |    | 100 Hz| 150nT | 16-bit | | integraded in MPU9250
AK8975    |    | 100 Hz| 300nT | 13-bit | | integraded in MPU9150
AK09916   |    | 100 Hz| 150nT | 16-bit | | integraded in ICM-20948
IST8310   |    | 200 Hz| 300nT | 14-bit
LIS2MDL   |    | 100 Hz| 15nT | 16-bit
BMM150    | $7 | 300 Hz| 300nT | | ±1300µT | NRND
BMM350    | $7 | 400 Hz| 100nT | 16-bit | ±2000µT |
RM3100    |$25 | 500 Hz| 13nT @ 150 Hz | no ADC | ±800µT | Uses coils, not hall sensors

## Barometer

| Part    | Breakout Board? | Relative Precision | Resolution | Max Measurement Rate | RMS Noise | Notes |
|-|:-:|-|-|-|-|-|
DPS310    | $3 | 6Pa 50cm | 0.06Pa | 128Hz | 0.35Pa @35Hz | Used in current commercial FC
SPL06-001 | $2 | 6Pa 50cm | 0.06Pa | 128Hz | 1.2Pa @35Hz | Registers identical to DSP310, but noisier
BMP180    | $2
BMP280    | $2 | 12Pa 100cm | 1.3Pa @125Hz | 157Hz | 2.5Pa 20cm @125Hz | Used in current commercial FC
BMP388    | $2 | 8Pa 66cm | | 200Hz
BMP390    | $3 | 3Pa 25cm | | 200Hz | 0.9Pa @25Hz, 5Pa 36cm @100Hz
MS5611    | $4 |  | 1.2Pa @120Hz | 1000Hz
MPL3115A2 | $5 |  | 0.25Pa | 166Hz | 19Pa @166Hz, 1.5Pa @2Hz
LPS22HB   | $6 | 10Pa 80cm | 0.025Pa | 75Hz
LPS22DF   | $14 | 1Pa 8cm | 0.025Pa | 200Hz
ILPS22QS  | $14 | 1.5Pa 12cm | 0.025Pa | 200Hz


## Multi Sensor Modules

| Module | Price | DOF | Sensors |
|-|:-:|-|-|
GY-85 | | 9DOF | ITG3205 ADXL345 HMC5883L
GY-86 | | 10DOF | MPU6050 HMC5883L MS5611
GY-87 | $4 | 10DOF | MPU6050 HMC5883L BMP180
GY-91 | $8 | 10DOF | MPU9255 (not MPU9250) BMP280
GY-521 | $2 | 6DOF | MPU6050
GY-912 | $11 | 10DOF | ICM20948 BMP388

## Current Sensors

| Part | Breakout Board? | Notes |
|-|:-:|-|
INA219 | $1 | 12-bit ADC, 0-26V
INA226 | $1.5 | 16-bit ADC, 0-36V, 1LSB = 1.25mV; ±16A with 5m&ohm; shunt, 1LSB = 0.5mA
INA228 | $5 | 20-bit ADC, 0-85V, 1LSB = 0.2mV; ±16A with 10m&ohm; shunt, 1LSB = 0.031mA

## Radar Modules

| Part | Module Price | Manufacturer | Max Range | Resolution | Sample Rate | Notes |
|-|:-:|-|-|-|-|-|
LD2411S|$4|Hi-link|6m|1cm (accuracy 5%)|10Hz|24G Radar, Distance detection - Tested stationary indoor approx 1m above floor: gives very erratic values and appears not to react on change in height... Not useful for _madflight_...
LD2413|$11|Hi-link|10m|1mm (accuracy 3mm)|20Hz|24G Radar, Liquid level - Tested stationary indoor approx 1m above floor: works but about 2 second delay reporting change in height... Not useful for _madflight_...
LD2451|$7|Hi-link|100m|1m||24G Radar, Vehicle detection, tracks relative speed, angle and distance of multiple targets
RD-03E|$3|Ai-tinker|6m|1cm (accuracy 5%)||24G Radar, Ranging Firmware (also exist with Gesture Firmware)
SR04M-2|$3|no-name|6m||20Hz|Waterproof Ultrasonic, pwm output. - Tested stationary indoor approx 1m above floor: works but sensor axis needs to be perpendicular (approx. +/-15 degrees) to floor to give correct readings.
