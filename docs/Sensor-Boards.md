# Sensor Boards

An overview of available sensor boards.

* Table last updated: January 2026
* Parts marked with &#x2705; are supported by _madflight_
* Parts with &#x1F6A7; are work in progress, see Pull Requests on Github
* The "Module Price" column lists price of a breakout board (if it is cheap & easy to procure)
* Prices in USD from online platforms incl shipping
* EOL = End Of Life, NRND = Not Recommended for New Designs

## 6-axis IMU

| Part    | Flight Controller | Module Price | Rate (Hz) | Gyro Noise (mdps/&radic;Hz) | Acc Noise (µg/&radic;Hz) | Notes |
|-|:-:|:-:|:-:|:-:|:-:|-|
BMI055 | `A----3` | 
BMI088 | `A-I--3` | 
BMI160 | `ABI--4` | $2 | 3.2k gyro, 1.6k acc | 8 | 180 |
BMI270 &#x2705;| `ABIM-3` |  | 6k gyro | 7 | 160 | Used in current commercial FC as replacement for MPU-6000/6500
BMI330 | `------` | | 6.4k gyro+acc | 7 | 180 |
ICM-20602  | `A---P-` | $5 | | 4 | 100 | NRND
ICM-20649  | `AB--P-` |  |  | 17.5 | 285 | NRND
ICM-20948  | `-----3`
ICM-40609-D| `AB--P-` |  | 32k gyro+acc | 4.5 | 100 | Marketed for drones, replacement of ICM-20602 and MPU-6500
ICM-42605 &#x2705;| `ABIMP-` | | 8k gyro+acc | 3.8 | 70 | Similar to ICM-42688-P
ICM-42670-P| `A---P-` | $3 | 1.6k gyro+acc | 7 | 100 | Marketed for drones
ICM-42688-P &#x2705;| `ABIMP3` | $6 | 32k gyro | 2.8 | 70 | Used in current high performance FC
ICM-45605 | `ABI---` | 
ICM-45686 &#x2705;| `ABIMP1` | no | 6.4k gyro+acc | 3.8 | 80 |
IMM-42652 | `AB--P-` |
IMM-42653 &#x2705; | `-B-MP-` | | 32k gyro+acc | 5 | 70 |
LSM6DS3    | `--I---` | $2 | | 7 | 180 |
LSM6DSL    | `--I---` |  | 6.7k | 4 | 80 |
LSM6DSM    | `------` |  | 6.7k | 3.8 | 130 |
LSM6DSO    | `-BI--2` | $10 | 6.7k | 3.8 | 110 |
LSM6DSR    | `-----1` |  | 6.7k | 5 | 60 |
LSM6DSV    | `-B---1` |  | 7.68k | 2.8 | 60 |
MPU-6000 &#x2705;| `ABIMP4` | $2 | 8k gyro, 1k acc ||| Released 2011, EOL
MPU-6050 &#x2705;| `ABIMP4` | $2 | 8k gyro, 1k acc ||| I2C only, Released 2011, EOL
MPU-6500 &#x2705;| `ABIMP4` | $2 | 8k gyro, 4k acc | 10 | 300 | Upgraded MPU-6000, Released 2014, NRND

*Flight Controller*  
A: ArduPilot  
B: BetaFlight  
I: INAV  
M: Madflight  
P: PX4  
1-4: [SlimeVR IMU ranking](https://docs.slimevr.dev/diy/imu-comparison.html) 1=best 4=worst  

## 9-axis IMU

| Part    | Module Price | Interface | Notes |
|-|:-:|-|-|
ICM-20948 &#x1F6A7;| $9 | SPI 7MHz, I2C | noise: 15 mdps/&radic;Hz 230 µg/&radic;Hz, replacement of MPU-9250/9255, ICM-20649 + AK09916, Released 2018
LSM9DSO  | 
MPU-9150 &#x2705;| $7 | I2C | 8k gyro, MPU6050 + AK8975, EOL
MPU-9250 &#x2705;| $7 | SPI 20MHz, I2C | 8k gyro, 4k acc, 100Hz mag, MPU6500 + AK8963, Released 2014, EOL. Note: many fake or relabelled MPU-6500 chips sold as MPU-9250 on the market...
MPU-9255 &#x2705;| $9 | SPI 20MHz, I2C | 8k gyro, MPU-6000 + AK8963, EOL

## Magnetometer

Earth's magnetic field strength at its surface ranges from 25 to 65 μT (0.25 to 0.65 G).

| Part    | Flight Controller | Module Price | Max Sampling Rate | LSB Resolution | ADC | Full Scale | Notes |
|-|:-:|:-:|-|-|-|-|-|
AK09916 &#x1F6A7; | `A---P` |    | 100 Hz| 1.5mG | 16-bit | | integraded in ICM-20948
AK09918 | `-----` | | 120 Hz | 1.5mG | 16-bit | ±50G
AK8963 &#x2705; | `ABIMP` |    | 100 Hz| 1.5mG | 16-bit | | integraded in MPU9250
AK8975 &#x2705; | `-BIM-` |    | 100 Hz| 3mG | 13-bit | | integraded in MPU9150
BMM150    | `A---P` | $7 | 300 Hz| 3mG | | ±13G | NRND
BMM350    | `A---P` | $7 | 400 Hz| 1mG | 16-bit | ±20G |
HMC5883L  | `-BI-P` | $2 | 160 Hz| 0.7mG | 12-bit | ±1G to ±8G | chip marking "L883"
HMC5983   | `-----` | $2 | 220 Hz | 0.7mG | 12-bit | ±1G to ±8G
IST8310   | `ABI-P` |    | 200 Hz| 3mG | 14-bit
LIS2MDL   | `-B--P` |    | 100 Hz| 0.15mG | 16-bit
MMC5603NJ | `-----` |    | 1000 Hz| 0.06mG | 20-bit | ±30G | linearity: 0.5%FS, noise: 1.5mG @ 75Hz
MMC5983MA | `A---P` | $15 | 1000 Hz| 0.06mG | 18-bit | ±8G | linearity: 0.1%FS, noise: 0.6mG @ 100Hz
QMC6309 &#x2705; | `---M-` | | 1500 Hz| 0.25mG | 16-bit | ±8G or ±32G | linearity: 0.6%FS, noise: 7mG @ 200Hz
QMC5883L &#x2705; | `ABIMP` | $2 | 200 Hz | 0.08mG | 16-bit | ±2G or ±8G | linearity: 0.1%FS, noise: 2mG, chip marking "5883"
QMC5883P &#x2705; | `A-IMP` |    | 1500 Hz | 0.08mG | 16-bit | ±2G to ±32G | linearity: 0.5%FS
RM3100 &#x2705; | `A--MP` | $25 | 500 Hz| 0.13mG @ 150 Hz | no ADC | ±8G | Uses coils, not hall sensors

## Barometer

When used as altimeter: approx 12 Pa (0.12 mbar) per meter at sea level.

| Part    | Flight Controller | Module Price | Relative Precision "Marketing Bla Bla" | ADC Resolution | Max Measurement Rate | RMS Noise | Notes |
|-|:-:|:-:|-|-|-|-|-|
BMP180   | `-----` | $2
BMP280 &#x2705;| `ABIMP` | $2 | "relative accuracy 12Pa 100cm" | 0.25Pa raw 20 bit ADC value | 26-167Hz OSR=16-1 | 2.1Pa @ 87Hz | 2.5x2x1mm, 8 pin, pitch 0.65mm
BMP388 &#x2705;| `ABIMP` | $2 | 8Pa 66cm | | 200Hz | 3.2Pa @ 92Hz
BMP390 &#x2705;| `---M-` | $3 | "relative accuracy 3Pa 25cm" | 2.6Pa 16bit OSR=1 to 0.8Pa 21bit OSR=32 | 25-200Hz OSR=32-1| 2.0Pa @ 92Hz | 2x2x0.75mm, 10 pi, pitch 0.5mm
BMP580 / BMP581 / BMP585 &#x2705;| `A--MP` | | "Relative pressure accuracy: 6 Pa per 10kPa step" | 0.016Pa (1/64Pa) 24bit | 622 Hz | 0.21Pa @ 87Hz | 2x2x0.8mm, 10 pin, pitch 0.5mm
DPS310 &#x2705;| `ABIMP` | $3 | "relative accuracy 6Pa 50cm" | 0.06Pa | 128Hz | 0.35Pa @35Hz | 2.5x2x1 mm, 8 pin, pitch 0.65mm, EOL
HP203B &#x2705;| `---M-` | | "Altitude Resolution down to 0.1 meter" | 0.01Pa 20 bit value (24bit ADC) | 8-240Hz OSR=4096-128| | 3.8×3.6×1.2 mm, 8 pin, pitch 0.9mm
ILPS22QS  | `-----` | $14 | 1.5Pa 12cm | 0.025Pa | 200Hz
LPS22HB   | `----P` | $6 | 10Pa 80cm | 0.025Pa | 75Hz
LPS22DF   | `-B---` | $14 | 1Pa 8cm | 0.025Pa | 200Hz
MPL3115A2 | `-----` | $5 |  | 0.25Pa | 166Hz | 19Pa @166Hz, 1.5Pa @2Hz
MS5607    | `--I--` | | "high resolution 20cm" | 0.014Pa raw 24 bit ADC value | 120-2000Hz OSR=4096-256 | | 5x3x1 mm, 8 pin, pitch 1.25mm
MS5611 &#x2705; | `ABIMP` | $4 | "high resolution 10cm" | 0.014Pa raw 24 bit ADC value | 120-2000Hz OSR=4096-256 | | 5x3x1 mm, 8 pin, pitch 1.25mm
SPL06-001 | `ABIMP` | $2 | 6Pa 50cm | 0.06Pa | 128Hz | 1.2Pa @35Hz | Registers identical to DSP310, but noisier

The following tests were performed to get some experimental data at approximately 100Hz sample rate using the best possible pressure oversampling. Clear winner is the BMP580 with a standard deviation of less than 2 cm. 
```
TEST RESULTS with madflight 2.1.2
with CLI calinfo stationary sampling for 3 seconds

BMP580 PRES_16X_TEMP_1X
=== Barometer - Sample rate: 85 Hz===
Altitude[m]     mean:+536.401306    stdev:0.016890  min:+536.355164    max:+536.452942    n:257
Pressure[Pa]    mean:+95045.796875  stdev:0.192236  min:+95045.218750  max:+95046.328125  n:257
Temperature[C]  mean:+24.054352     stdev:0.010603  min:+24.028427     max:+24.079422     n:257

BMP390 PRES_2X_TEMP_1X
=== Barometer - Sample rate: 100 Hz===
Altitude[m]     mean:+536.515625    stdev:0.219643  min:+535.979980    max:+537.145203    n:301
Pressure[Pa]    mean:+95044.492188  stdev:2.504872  min:+95037.312500  max:+95050.609375  n:301
Temperature[C]  mean:+24.873934	    stdev:0.004969  min:+24.859617     max:+24.887152     n:301

BMP280 PRES_4X_TEMP_1X
=== Barometer - Sample rate: 105 Hz===
Altitude[m]   	mean:+526.812683    stdev:0.186374  min:+526.335693    max:+527.411072    n:315
Pressure[Pa]  	mean:+95155.210938	stdev:2.126955  min:+95148.382812  max:+95160.664062  n:315
Temperature[C]	mean:+25.984348     stdev:0.008128  min:+25.959999     max:+26.000000     n:315

HP203B OSR=256
=== Barometer - Sample rate: 100 Hz===
Altitude[m]     mean:+539.022583    stdev:1.616964  min:+533.578125    max:+543.663696    n:300
Pressure[Pa]    mean:+95015.906250	stdev:18.437220 min:+94963.000000  max:+95078.000000  n:300
Temperature[C]  mean:+27.717833     stdev:0.010926  min:+27.700001     max:+27.740000     n:300
```

## Multi Sensor Modules

| Module | Module Price | DOF | Sensors |
|-|:-:|-|-|
GY-85 | | 9DOF | ITG3205, ADXL345, HMC5883L
GY-86 | | 10DOF | MPU6050, HMC5883L, MS5611 &#x2705;
GY-87 | $4 | 10DOF | MPU6050 &#x2705;, HMC5883L, BMP180
GY-91 | $8 | 10DOF | MPU9255 &#x2705; (not MPU9250), BMP280 &#x2705;
GY-521 | $2 | 6DOF | MPU6050 &#x2705;
GY-912 | $11 | 10DOF | ICM20948 &#x1F6A7;, BMP388 &#x2705;

## Current Sensors

| Part | Module Price | Notes |
|-|:-:|-|
INA219 | $1 | 12-bit ADC, 0-26V
INA226 &#x2705;| $1.5 | 16-bit ADC, 0-36V, 1LSB = 1.25mV; ±32A with 2.5m&ohm; 3W shunt, 1LSB = 1mA
INA228 &#x2705;| $5 | 20-bit ADC, 0-85V, 1LSB = 0.2mV; ±80A with 0.5m&ohm; 3W shunt, 1LSB = 0.16mA

## Radar / Lidar / Ultrasonic Sensors

| Part | Module Price | Manufacturer | Max Range | Resolution | Sample Rate | Notes |
|-|:-:|-|-|-|-|-|
LD2411S &#x2705;|$4|Hi-link|6m|1cm (accuracy 5%)|10Hz|24G Radar, Distance detection - Tested stationary indoor approx 1m above floor: gives very erratic values and appears not to react on change in height. Not useful for _madflight_...
LD2413 &#x2705;|$11|Hi-link|10m|1mm (accuracy 3mm)|20Hz|24G Radar, Liquid level - Tested stationary indoor approx 1m above floor: works but about 2 second delay reporting change in height. Not useful for _madflight_...
LD2451 |$7|Hi-link|100m|1m||24G Radar, Vehicle detection, tracks relative speed, angle and distance of multiple targets. Not useful for _madflight_...
RD-03E |$3|Ai-tinker|6m|1cm (accuracy 5%)||24G Radar, Ranging Firmware (also exist with Gesture Firmware)
SR04M-2 &#x2705;|$3|no-name|6m||20Hz|Waterproof Ultrasonic, pwm output. - Tested stationary indoor approx 1m above floor: works but sensor axis needs to be perpendicular (approx. +/-15 degrees) to floor to give correct readings.
DTS6012M &#x2705;|$15|Senkylaser|20m (12m in full sun)|1mm (accuracy: 6cm, 1%)|250Hz (default 100Hz)|Lidar with UART and I2C interface, 1.35 gram, 330 mW. - Tested stationary indoor approx 1m above floor: works very well, also at an angle.

## Optical Flow Sensors

| Part | Module Price | Notes |
|-|:-:|-|
|PMW3901 &#x2705;|$8|SPI interface (4 GPIO), Frame rate: 120Hz, Power: 30 mW, Max speed: 7.4 rad/s (7.4 m/s at 1 m height), Field of view: 42°, Range: ~80mm to infinity
|PMW3901U &#x2705;||UART interface. This is a PMW3901 with external microcontroller for SPI to UART conversion
|[ESP32-Optical-Flow](https://github.com/qqqlab/ESP32-Optical-Flow) &#x2705;|$6|ESP32-Cam module with OV2640 or OV3660 camera
