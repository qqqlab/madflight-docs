# Getting Started

For additional help see [_madflight Discussions](https://github.com/qqqlab/madflight/discussions)

1. Connect the required hardware (see below) to your controller board: 
    - See [RP2350/RP2040 pinout and instructions](Board-RP2040.md)
    - -or- [ESP32-S3/ESP32 pinout and instructions](Board-ESP32.md)
    - -or- [STM32 pinout and instructions](Board-STM32.md)
    - Connect your IMU (gyro/acceleration) sensor as shown [below](#connecting-the-imu-sensor).
    - Connect your radio receiver according to the configured pins.
2. Install the madflight library in Arduino IDE. (Menu *Tools->Manage Libraries*, then search for _madflight_)
3. Open *Examples for custom libraries->madflight->Quadcopter.ino* in the Arduino IDE.
4. Edit the HARDWARE CONFIG section in madflight_config.h to enable the connected peripherals.
5. If you're not using the default pinout then setup your board pinout in the CUSTOM PINS CONFIG section.
6. Compile Quadcopter.ino and upload it to your board. Connect the Serial Monitor at 115200 baud and check the messages. Type `help` to see the available CLI commands.
7. Type `calradio` and follow the prompts to setup your RC radio receiver.
8. IMPORTANT: Use CLI `calimu` and `calmag` to calibate the sensors.
9. Use CLI commands `pimu`, `pahr`, `prcl`, `pmot`, etc. and check that IMU sensor, AHRS and RC Receiver are working correctly. 
10. Connect motors (no props) and battery and check that motors are spinning correctly.
11. Mount props, go to an wide open space, and FLY!

## Required Hardware

- [Development board](Controller-Boards.md): 
    - RP2350/RP2040 (e.g. Raspberry Pi Pico2)
    - ESP32-S3/ESP32 (e.g. Espressiv ESP32-S3 DevKitC)
    - STM32 (e.g. Black Pill or a commercial flight controller)
- [SPI IMU sensor](Sensor-Boards.md) (BMI270, MPU9250, MPU6500, MPU6000, ICM45686), if not available then use an I2C IMU sensor (MPU6050, MPU9150) 
- RC Receiver: MAVLink, ELRS, CRSF, SBUS, DMSX, or PPM
- BEC or DC-DC converter to power your board from a battery
- ESC (OneShot125 or 50-490Hz PWM) and/or servos (50-490Hz PWM)

## Optional Hardware

- GPS Module (Serial)
- Barometer (I2C BMP280, BMP388, BMP390, MS5611)
- Magnetometer (I2C QMC5883L)
- Current/Voltage Sensor (ADC or I2C INA226, INA228)
- Radar/Lidar/Ultrasonic distance sensor
- [Optical Flow Sensor](https://github.com/qqqlab/ESP32-Optical-Flow) (I2C)

## Connecting SPI IMU Sensor 

_madflight_ requires the interrupt pin _pin_imu_int_ connected.

| Sensor   |  |  Dev Board |
|-:|:-:|:-|
SCL/SCLK |<---| _pin_spiX_sclk_
SDA/SDI  |<---| _pin_spiX_mosi_
ADD/SDO  |--->| _pin_spiX_miso_
NCS      |<---| _pin_imu_cs_
INT      |--->| _pin_imu_int_
VCC      |----| 3V3 or 5V (depending on your sensor board)
GND      |----| GND

_X_ is the SPI bus number, set with _imu_spi_bus_

## Connecting I2C IMU Sensor

Only use I2C if you really have to, better use SPI: no hanging busses - no crashes of your craft because of that.

_madflight_ requires the interrupt pin _pin_imu_int_ connected.

| Sensor   |  |  Dev Board |
|-:|:-:|:-|
SCL |<-->| _pin_i2cY_scl_
SDA |<-->| _pin_i2cY_scl_
INT |--->| _pin_imu_int_
VCC |<-->| 3V3 or 5V (depending on your sensor board)
GND |<-->| GND

_Y_ is the I2C bus number, set with _imu_i2c_bus_
