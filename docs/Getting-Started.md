# Getting Started

1. Get the required hardware
2. Setup PlatformIO or Arduino IDE development environment
3. Open Quadcopter example
4. Configure hardware
5. Compile Quadcopter
6. Check and calibrate
7. FLY!

For additional help see [madflight Discussions](https://github.com/qqqlab/madflight/discussions)

## Configuration Tools

You can use the following tools for configuring _madflight_.

### CLI Command Line Interface

The CLI is the preferred and most powerful configuration tool. The CLI can be used for calibration, modify settings, and to view the drone's state.

Connect your favorite terminal via USB to the flight controller. Type `help` to see the available commands, `diff` and `dump` to see the current settings.

### Betaflight Configurator

You can use the Betaflight Configurator to check your settings, but you can not use it to configure madflight.

Connect your board by USB and open [app.betaflight.com](https://app.betaflight.com) with a Chrome browser or install the [PC Configurator v10.10.0](https://github.com/betaflight/betaflight-configurator/releases/tag/10.10.0). Then use the Setup, Receiver, Motors, or Sensors Tabs to check your settings. The CLI Tab can also be used, but using the CLI with an USB terminal program is more responsive.

### Mission Planner

You can use MP to check attitude and GPS position, change parameters, and to analyze madflight log files.

Start Mission Planner and go to the Config->Planner tab, and check both "Reset on USB Connect (toggle DTR)" and "Disable RTS reset on ESP SerialUSB". Then connect your board by USB and press the CONNECT button. You can also use a MavLink receiver to connect.

## 1. Required Hardware

- [Development board](Controller-Boards.md): 
    - RP2350/RP2040 (e.g. Raspberry Pi Pico2)
    - ESP32-S3/ESP32 (e.g. Espressiv ESP32-S3 DevKitC)
    - STM32 (e.g. Black Pill)
- [SPI IMU sensor](Sensor-Boards.md) (e.g. BMI270, MPU9250, MPU6500, MPU6000, ICM-45686, ICM-42688-P, LSM6DSV), if not available then use an I2C IMU sensor (MPU6050, MPU9150) 
- RC Receiver: MAVLink, ELRS, CRSF, SBUS, DMSX, or PPM
- BEC or DC-DC converter to power your board from a battery
- ESC (OneShot125 or 50-490Hz PWM) and/or servos (50-490Hz PWM)

Or a commercial flight controller which includes some or all items on a single board.

### Optional Hardware

- GPS Module (Serial)
- Barometer (I2C, e.g. BMP280, BMP388, BMP390, MS5611)
- Magnetometer (I2C e.g. QMC5883L, QMC8309, MMC5603)
- Current/Voltage Sensor (ADC/I2C, e.g. INA226, INA228)
- Radar/Lidar/Ultrasonic distance sensor
- [Optical Flow Sensor](https://github.com/qqqlab/ESP32-Optical-Flow) (I2C)

## 2. Development Envirionment

Setup your favorite development environment (PlatformIO or Arduino IDE) for your board:

 - [ESP32-S3/ESP32](Board-ESP32.md)
 - [RP2350/RP2040](Board-RP2040.md)
 - [STM32](Board-STM32.md)
 - [madflight FC1 RP2350B](Board-FC1.md)
 - [madflight FC2 ESP32-S3](Board-ESP-FC2.md)
 - [madflight FC3 RP2350B](Board-FC3.md)

## 3. Open Quadcopter example

### Arduino IDE

Open the Quadcopter example: use menu **File->Examples->Examples for custom libraries->madflight->Quadcopter.ino**

### PlatformIO

Open `platformio.ini` and read the instructions in the file to compile the Quadcopter example

## 4. Configure Hardware

Edit the `madflight_config` multiline string in `madflight_config.h` to enable the hardware peripherals.

### Configure a SPI IMU Sensor (IMU)

Set the following in madflight_config:
```
imu_gizmo      ICM42688 // change to match your sensor type
imu_bus_type   SPI
pin_imu_int    14       // enter gpio number for interrupt pin here
pin_imu_cs     15       // enter gpio number for chip-select pin here
imu_spi_bus    0        // select bus SPI0

//setup SPI0 bus (bus setup is done in the board file, but you can override it here as required)
pin_spi0_sclk  11       // enter gpio number for SPI0 SCLK pin here
pin_spi0_mosi  12       // enter gpio number for SPI0 MOSI pin here
pin_spi0_miso  13       // enter gpio number for SPI0 MISO pin here
```

Connect the IMU sensor:

| SPI Sensor   |  | Flight Controller |
|-:|:-:|:-|
SCL/SCLK |<---| `pin_spi0_sclk`
SDA/SDI  |<---| `pin_spi0_mosi`
ADD/SDO  |--->| `pin_spi0_miso`
NCS      |<---| `pin_imu_cs`
INT      |--->| `pin_imu_int`
VCC      |----| 3V3 or 5V (depending on your sensor board)
GND      |----| GND

If you want to use a different SPI bus than SPI0, change imu_spi_bus and pin_spiX_YYY accordingly.

_madflight_ requires the interrupt pin _pin_imu_int_ connected.

### Configure an I2C IMU Sensor (IMU)

Only use I2C if you really have to, better use SPI: no hanging busses - no crashes of your craft because of that.

Set the following in madflight_config:
```
imu_gizmo      MPU6050 // select your sensor type here
imu_bus_type   I2C
pin_imu_int    24      // enter gpio number for interrupt pin here
imu_i2c_bus    0       // select bus I2C0

//setup I2C0 bus (bus setup is done in the board file, but you can override it here as required)
pin_i2c0_sda  21       // enter gpio number for I2C0 SDA pin here
pin_i2c1_scl  22       // enter gpio number for I2C0 SCL pin here
```

Connect the IMU sensor:

| I2C Sensor   |  | Flight Controller |
|-:|:-:|:-|
SCL |<-->| `pin_i2c0_scl`
SDA |<-->| `pin_i2c0_sda`
INT |--->| `pin_imu_int`
VCC |<-->| 3V3 or 5V (depending on your sensor board)
GND |<-->| GND

If you want to use a different I2C bus than I2C0, change imu_i2c_bus and pin_i2cX_YYY accordingly.

_madflight_ requires the interrupt pin _pin_imu_int_ connected.

### Configure a Serial Receiver (RCL)

Set the following in madflight_config:
```
rcl_gizmo      CRSF  // select your radio receiver type here: MAVLINK, CRSF, SBUS, DSM, IBUS
rcl_num_ch     8     // number of channels
rcl_ser_bus    0     // select bus SER0

//setup SER0 bus (bus setup is done in the board file, but you can override it here as required)
pin_ser0_tx    31       // enter gpio number for SER0 TX pin here
pin_ser0_rx    32       // enter gpio number for SER0 RX pin here
```

If you want to use a different Serial bus than SER0, change rcl_ser_bus and pin_serX_YYY accordingly.

### Configure a PPM Receiver (RCL)

Set the following in madflight_config:
```
rcl_gizmo      PPM 
rcl_num_ch     8      // number of channels
rcl_deadband   10     // center stick deadband in [us], a value around 10 will probably work fine
pin_rcl_ppm    31     // select the PPM pin here
```

### Configure Motors (OUT)

Connect the 4 ESCs of the motors to `pin_out0` - `pin_out3`

The motors are connected in BetaFlight order:

| BetaFlight Motor | madflight OUT | Position | Rotation Direction (as seen from above) |
|:-:|:-:|:-:|:-:|
Motor 1|pin_out0| Right Back | Clockwise
Motor 2|pin_out1| Right Front | Counter-Clockwise
Motor 3|pin_out2| Left Back | Counter-Clockwise
Motor 4|pin_out3| Left Front | Clockwise

```
      front
 CW -->   <-- CCW
    m4     m2 
      \ ^ /
       |X|
      / - \
    m3     m1 
CCW -->   <-- CW
```

## 6. Compile Quadcopter.ino

Compile Quadcopter.ino and upload it to your board.

Connect the Serial Monitor at 115200 baud, type `help` to see the available CLI (Command Line Interface) commands.

## 7. Check and Calibrate

COMPLETE THIS SECTION OR YOUR CRAFT WILL CRASH (you have been warned :-)

First check the startup messages for errors/warnings, and fix those before continuing, it will save you time and headaches.

### Calibrate Gyro/Accelerometer (IMU)

Place vehicle horizontal and stationary, then type `calimu` and wait for calibration to complete.

### Calibrate Magnetometer (MAG)

Type `calmag`, and keep moving the vehicle in all directions until completed. 

The calibration routine tries to capture the maximum and minimum magnetic flux on each axis. One way to achieve this is:

 - Point the vehicle horizontally toward the magnetic North
 - Pitch up until inverted, then back to horizontal
 - Pitch down until inverted, then back to horizontal
 - Yaw vehicle 90 degrees, i.e. point left or right side towards magnetic North
 - Roll right until inverted, then back to horizontal
 - Roll left until inverted, then back to horizontal

### Set Gyro/Accelerometer Orientation (IMU)

The `imu_align` parameter sets the sensor orientation. The label is yaw / roll (in that order) needed to rotate the sensor from its normal position to its mounted position. The normal sensor position is NED (North East Down), i.e. x-axis points forward (N), y-axis points right (E), z-axis points down (D).

Type `pacc` to display the IMU accelerometer outputs, and check that:

 - Horizontal gives az = 1, for example: `ax:-0.02  ay:-0.01  az:+1.03`

 - Nose pointing to ground gives ay = 1

 - Right side pointing to ground gives ax = 1

If not: use `dump imu_align` to show current setting, and `set imu_align <new_value>` to change, and then `pacc` to until this matches.

Note: you can also use the Setup Tab in the BF Configurator and check that the displayed drone orientation follows your roll/pitch/yaw movements.

### Set Magnetometer Orientation (MAG)

The procedure is similar to the IMU procedure: type `pmag` to check, adjust the `mag_align` parameter until you get compass = 0 when pointing North and compass = +90 when pointing East.

### Check AHRS

Type `pahr` to check that the calculated AHRS roll and pitch angles are correct. From horizontal do:

 - Pitch up (nose up) and the AHRS pitch should be positive and according to the pitch angle.

 - Roll right (right side down) and the AHRS roll should be positive and according to the roll angle.

 - Yaw right (rotate clockwise) and the AHRS yaw should increase according to yaw angle.

 - If you have a magnetometer, also check that the AHRS yaw compass direction correct. You should get yaw = 0 when pointing North and yaw = +90 when pointing East.

IMPORTANT: Take your time to check this. If anything does not match up, repeat the calibration/orientation steps. If AHRS does not work 100% you won't be able to fly.

Type `save` to store the settings. (The flight controller will reboot after `save`)

### Setup Radio Link (RCL)

Connect your receiver and power up your transmitter. Type `ppwm` to check that you have connection. `ppwm` gives the received pwm values per channel, each value should be between 800 and 2200. 

Type `calradio` and follow the prompts to setup your RC radio receiver.

Type `prcl` to check the configuration:

- Throttle should be between 0 (idle) and 1 (full throttle). 
- Roll and yaw should be 0 for center, -1 for left and +1 for right.
- Pitch should be 0 for center, -1 for pitch down, +1 for pitch up.
- Armed should be 0 when the switch is in disarmed position and 1 in armed position
- Flightmode should be 0,1,2,3,4,5 representing the 2/3/6-position flight mode switch.

If something does not look right, fix it before continuing.

Type `save` to store the settings. (The flight controller will reboot after `save`)

### Test Arming (RCL,OUT)

Type `pout` to display the motor outputs.

Check the arming mechanism: upon arming `out.armed` changes from 0 to 1.

With arm switch configured (parameter `rcl_arm_ch > 0`)
 - ARMING: Set throttle idle, then flip arm switch to armed
 - DISARMING: Flip arm switch to disarmed

Without arm switch configured (parameter `rcl_arm_ch == 0`)
 - ARMING: Pull both sticks toward you, yaw full right, and roll full left and keep sticks there for 2 sec
 - DISARMING: Pull both sticks toward you, yaw full left, and roll full right and keep sticks there for 2 sec

### Test Quadcopter Motor Mixer (OUT)

LEAVE BATTERY DISCONNECTED

Use CLI commands `pout` to display the motor outputs.

Keep the quad horizontal and stationary.

Now set throttle to mid position, the outputs should go to around 50%: `out.armed:1  M0%:45  M1%:49  M2%:52  M3%:53`

The values will change slowly as the PID integrators build up. This is normal, reduce the throttle to idle and then back to mid position to reset the integrators.

Move pitch stick forward and out0,2 should go up, out1,3 down: `out.armed:1  M0%:71  M1%:43  M2%:72  M3%:44`

Move roll stick right and out2,3 should go up, out0,1 down: `out.armed:1  M0%:41  M1%:47  M2%:69  M3%:75`

Move yaw stick right and out1,2 should go up, out0,3 down: `out.armed:1  M0%:3  M1%:103  M2%:104  M3%:7`

If any of the checks fail -> re-check your IMU, RCL and OUT configuration settings.

### Test Quadcopter Motor Direction and Order (OUT)

Connect the battery but REMOVE PROPELLERS

Use the Motors Tab in the BF Configurator.

Or, in the CLI type `spinmotors` and then `go`. This will spin each motor in order. Check that the correct motor spins, and that the motor spins in the correct direction.

If the incorrect motor spins, change the pin_out<x> parameters to correct this.

If a motor spins in the wrong direction: exchange any 2 of the 3 wires of a brushless motor, or exchange the 2 wires of a brushed motor. Then check again!

### Check Flight Modes and PID

READ the [notes](Parameters.md#pid) for the selected PID controller. 

Type `diff pid` to check your PID settings match.

Type `dump rcl_flt` to ensure that your selected flight modes are correct.

For first attempts start with RATE flight mode. If RATE works, then try ANGLE. If ANGLE works, try other modes.

It helps to set your radio to show the flight mode telemetry value.

## 8. FLY

Again, only continue if all calibrations and checks passed!!!

Have a look at [Quadcopter Example](Example-Quadcopter.md) for details on the quadcopter program.

Mount props, go to a wide open space, and FLY!
