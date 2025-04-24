# Getting Started

For additional help see [madflight Discussions](https://github.com/qqqlab/madflight/discussions)

1. Get the required hardware (see below), and have a look at the hardware specific instructions: 
    - [madflight FC1 instructions](Board-FC1.md)
    - -or- [RP2350/RP2040 pinout and instructions](Board-RP2040.md)
    - -or- [ESP32-S3/ESP32 pinout and instructions](Board-ESP32.md)
    - -or- [STM32 pinout and instructions](Board-STM32.md)
2. Install the madflight library in Arduino IDE. (Menu *Tools->Manage Libraries*, then search for _madflight_)
3. Open *Examples for custom libraries->madflight->Quadcopter.ino* in the Arduino IDE.
4. Edit the HARDWARE CONFIG section in madflight_config.h to enable the peripherals:
    - Connect IMU (gyro/acceleration) sensor (see below)
    - Connect radio receiver (see below)
6. Compile Quadcopter.ino and upload it to your board. Connect the Serial Monitor at 115200 baud and check the startup messages. Type `help` to see the available CLI commands.
7. IMPORTANT: Use CLI `calimu` and `calmag` to calibate the sensors.
8. Type `calradio` and follow the prompts to setup your RC radio receiver.
9. Use CLI commands `pacc`, `pgyr`, `pahr`, `prcl`, `pmot`, etc. and check that IMU sensor, AHRS and RC Receiver are working correctly. 
10. Connect battery and motor ESCs (no props) to `pin_out0-3`, arm the quad and check that motors are spinning correctly.
11. Mount props, go to an wide open space, and FLY!

## Required Hardware

- [Development board](Controller-Boards.md): 
    - RP2350/RP2040 (e.g. Raspberry Pi Pico2)
    - ESP32-S3/ESP32 (e.g. Espressiv ESP32-S3 DevKitC)
    - STM32 (e.g. Black Pill)
- [SPI IMU sensor](Sensor-Boards.md) (BMI270, MPU9250, MPU6500, MPU6000, ICM-45686, ICM-42688-P), if not available then use an I2C IMU sensor (MPU6050, MPU9150) 
- RC Receiver: MAVLink, ELRS, CRSF, SBUS, DMSX, or PPM
- BEC or DC-DC converter to power your board from a battery
- ESC (OneShot125 or 50-490Hz PWM) and/or servos (50-490Hz PWM)

Or a commercial flight controller which includes some or all items on a single board.

## Optional Hardware

- GPS Module (Serial)
- Barometer (I2C BMP280, BMP388, BMP390, MS5611)
- Magnetometer (I2C QMC5883L)
- Current/Voltage Sensor (ADC or I2C INA226, INA228)
- Radar/Lidar/Ultrasonic distance sensor
- [Optical Flow Sensor](https://github.com/qqqlab/ESP32-Optical-Flow) (I2C)

## Connect IMU Sensor

### Connect a SPI IMU Sensor

Set the following in madflight_config:
```
imu_gizmo      ICM42688 // select your sensor type here
imu_bus_type   SPI
imu_align      CW0 // options: CW0, CW90, CW180, CW270, CW0FLIP, CW90FLIP, CW180FLIP, CW270FLIP

//only uncomment the following if you do not want to use the default settings
//pin_imu_int        <gpio>
//pin_imu_cs         <gpio>
//imu_spi_bus        <bus>
//pin_spi<bus>_sclk  <gpio>
//pin_spi<bus>_mosi  <gpio>
//pin_spi<bus>_miso  <gpio>
```

_madflight_ requires the interrupt pin _pin_imu_int_ connected.

Connect the IMU sensor:

| Sensor   |  |  Dev Board |
|-:|:-:|:-|
SCL/SCLK |<---| `pin_spi<bus>_sclk`
SDA/SDI  |<---| `pin_spi<bus>_mosi`
ADD/SDO  |--->| `pin_spi<bus>_miso`
NCS      |<---| `pin_imu_cs`
INT      |--->| `pin_imu_int`
VCC      |----| 3V3 or 5V (depending on your sensor board)
GND      |----| GND

`<bus>` is the SPI bus number, set with `imu_spi_bus <bus>`

### Connect an I2C IMU Sensor

Only use I2C if you really have to, better use SPI: no hanging busses - no crashes of your craft because of that.

Set the following in madflight_config:
```
imu_gizmo      MPU6050 // select your sensor type here
imu_bus_type   I2C
imu_align      CW0 // options: CW0, CW90, CW180, CW270, CW0FLIP, CW90FLIP, CW180FLIP, CW270FLIP

//only uncomment the following if you do not want to use the default settings
//pin_imu_int        <gpio>
//imu_i2c_bus        <bus>
//pin_i2c<bus>_sda   <gpio>
//pin_i2c<bus>_scl   <gpio>
```

_madflight_ requires the interrupt pin _pin_imu_int_ connected.

Connect the IMU sensor:

| Sensor   |  |  Dev Board |
|-:|:-:|:-|
SCL |<-->| `pin_i2c<bus>_scl`
SDA |<-->| `pin_i2c<bus>_sda`
INT |--->| `pin_imu_int`
VCC |<-->| 3V3 or 5V (depending on your sensor board)
GND |<-->| GND

`<bus>` is the I2C bus number, set with `imu_i2c_bus <bus>`

### Configure the IMU Sensor Orientation

`imu_align` sets the sensor orientation. The label is yaw / roll (in that order) needed to rotate the sensor from it's normal position to it's mounted position.

IMPORTANT: Check that `imu_align` is correct, your craft will immediately crash otherwise. Use CLI `pahr` to check that rolling right side down gives positive ahr.roll, pitching nose up gives positive ahr.pitch, and yawing right (turning clockwise from as seen from above) gives positive ahr.yaw. Using `pacc` you should get [ax,ay,az] = [1,0,0] when nose down, [0,1,0] when right side down, and [0,0,1] when level. If not, adjust the `imu_align` parameter, re-upload and try again.

## Connect Radio Receiver

### Connect a Serial Receiver

Set the following in madflight_config:
```
rcl_gizmo      CRSF // select your radio receiver type here: MAVLINK, CRSF, SBUS, DSM
rcl_num_ch     8     // number of channels
rcl_deadband   0     // center stick deadband, set to 0 for serial receivers

//only uncomment the following if you do not want to use the default settings
//imu_ser_bus        <bus>
//pin_ser<bus>_tx    <gpio>
//pin_ser<bus>_rx    <gpio>
```
### Connect a PPM Receiver

Set the following in madflight_config:
```
rcl_gizmo      PPM 
rcl_num_ch     8      // number of channels
rcl_deadband   10     // center stick deadband in [us], a value around 10 will probably work fine
pin_rcl_ppm    <gpio> // select the pin here
```

### Configure Radio Channels

Either use CLI `calradio`, or modify the following parameters, or set your radio transmitter to match the default parameters:

```
rcl_thr_ch        1 // 1-based channel number for throttle
rcl_thr_pull   1100 // pwm for stick pulled toward you, i.e. idle throttle
rcl_thr_mid    1500
rcl_thr_push   1900 // pwm for stick pushed away, i.e. full throttle

rcl_rol_ch        2 // roll channel
rcl_rol_left   1100
rcl_rol_mid    1500
rcl_rol_right  1900

rcl_pit_ch        3 // pitch channel
rcl_pit_pull   1100 // pwm for stick pulled toward you, i.e. pitch-up
rcl_pit_mid    1500
rcl_pit_push   1900 // pwm for stick pushed away, i.e. pitch-down

rcl_yaw_ch        4 // yaw channel
rcl_yaw_left   1100
rcl_yaw_mid    1500
rcl_yaw_right  1900

rcl_arm_ch        5 // arm switch channel, set to 0 to use stick commands for arming
rcl_arm_min    1600 // armed pwm range min
rcl_arm_max    2500 // armed pwm range max

// flightmode 6 position switch - Ardupilot switch pwm: 1165,1295,1425,1555,1685,1815 (spacing 130)
// EdgeTx 3-pos SA + 2-pos SB setup:
//   Source:SA Weight:52 Offset:0 + Source:SB Weight:13 Offset:-1 Multiplex: add
//   -OR- Source:SA Weight:26 Offset:-40 Switch:SBdown + Source:SA Weight:26 Offset:36 Switch:SBup Multiplex:Replace

rcl_flt_ch        6
rcl_flt_min    1165 // 6-pos switch lowest pwm (flight mode 0)
rcl_flt_max    1815 // 6-pos switch lowest pwm (flight mode 5)
```
