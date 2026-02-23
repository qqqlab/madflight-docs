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

Connect your favorite terminal by USB. The CLI is the most powerful tool to view the drone's state, calibrate it, and to modify settings. Type 'help' to see the available commands, 'diff' to see the current settings.

### Betaflight Configurator

You can use the Betaflight Configurator to check your settings, but you can not use it to configure madflight. Connect your board by USB and open [app.betaflight.com](https://app.betaflight.com) with a Chrome browser or install the [PC Configurator v10.10.0](https://github.com/betaflight/betaflight-configurator/releases/tag/10.10.0). Then use the Setup, Receiver, Motors, or Sensors Tabs to check your settings. Also the CLI Tab can be used but direct terminal CLI connection is more responsive.

### Mission Planner

Start Mission Planner and go to the Config->Planner tab, and check both "Reset on USB Connect (toggle DTR)" and "Disable RTS reset on ESP SerialUSB". Then connect your board by USB and press the CONNECT button. You can also use a MavLink receiver to connect.

You can use MP to check attitude and GPS position, change parameters, and to analyze madflight log files.

## 1. Required Hardware

- [Development board](Controller-Boards.md): 
    - RP2350/RP2040 (e.g. Raspberry Pi Pico2)
    - ESP32-S3/ESP32 (e.g. Espressiv ESP32-S3 DevKitC)
    - STM32 (e.g. Black Pill)
- [SPI IMU sensor](Sensor-Boards.md) (BMI270, MPU9250, MPU6500, MPU6000, ICM-45686, ICM-42688-P), if not available then use an I2C IMU sensor (MPU6050, MPU9150) 
- RC Receiver: MAVLink, ELRS, CRSF, SBUS, DMSX, or PPM
- BEC or DC-DC converter to power your board from a battery
- ESC (OneShot125 or 50-490Hz PWM) and/or servos (50-490Hz PWM)

Or a commercial flight controller which includes some or all items on a single board.

### Optional Hardware

- GPS Module (Serial)
- Barometer (I2C BMP280, BMP388, BMP390, MS5611)
- Magnetometer (I2C QMC5883L)
- Current/Voltage Sensor (ADC or I2C INA226, INA228)
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

Open the Quadcopter example: use menu **File->Examples->Examples for custom libraries->madflight->01.Quadcopter.ino**

### PlatformIO

Open `platformio.ini` and read the instructions in the file to compile the Quadcopter example

## 4. Configure Hardware

Edit the `madflight_config` multiline string in `madflight_config.h` to enable the hardware peripherals.

### Configure a SPI IMU Sensor (IMU)

Set the following in madflight_config:
```
imu_gizmo      ICM42688 // select your sensor type here
imu_bus_type   SPI

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

### Configure an I2C IMU Sensor (IMU)

Only use I2C if you really have to, better use SPI: no hanging busses - no crashes of your craft because of that.

Set the following in madflight_config:
```
imu_gizmo      MPU6050 // select your sensor type here
imu_bus_type   I2C

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

`<bus>` is the I2C bus number (0 or 1), set with `imu_i2c_bus <bus>`

### Configure a Serial Receiver (RCL)

Set the following in madflight_config:
```
rcl_gizmo      CRSF // select your radio receiver type here: MAVLINK, CRSF, SBUS, DSM, IBUS
rcl_num_ch     8     // number of channels
rcl_deadband   0     // center stick deadband, set to 0 for serial receivers

//only uncomment the following if you do not want to use the default settings
//imu_ser_bus        <bus>
//pin_ser<bus>_tx    <gpio>
//pin_ser<bus>_rx    <gpio>
```
### Configure a PPM Receiver (RCL)

Set the following in madflight_config:
```
rcl_gizmo      PPM 
rcl_num_ch     8      // number of channels
rcl_deadband   10     // center stick deadband in [us], a value around 10 will probably work fine
pin_rcl_ppm    <gpio> // select the PPM pin here
```
### Configure Radio Channels (RCL)

Set your radio transmitter to match the default parameters (AERT) listed below. Or modify the parameters to match your radio setup.

Or skip these settings and see below to setup the parameters interactively. 

```
rcl_rol_ch        1 // roll (aileron) channel number
rcl_rol_left   1100
rcl_rol_mid    1500
rcl_rol_right  1900

rcl_pit_ch        2 // pitch (elevator) channel
rcl_pit_pull   1100 // pwm for stick pulled toward you, i.e. pitch-up
rcl_pit_mid    1500
rcl_pit_push   1900 // pwm for stick pushed away, i.e. pitch-down

rcl_yaw_ch        3 // yaw (rudder) channel
rcl_yaw_left   1100
rcl_yaw_mid    1500
rcl_yaw_right  1900

rcl_thr_ch        4 // throttle channel
rcl_thr_pull   1100 // pwm for stick pulled toward you, i.e. idle throttle
rcl_thr_mid    1500
rcl_thr_push   1900 // pwm for stick pushed away, i.e. full throttle

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

### Check Gyro/Accelerometer Orientation (IMU)

The `imu_align` parameter sets the sensor orientation. The label is yaw / roll (in that order) needed to rotate the sensor from its normal position to its mounted position. The normal sensor position is NED (North East Down), i.e. x-axis points forward (N), y-axis points right (E), z-axis points down (D). 

Use the Setup Tab in the BF Configurator and check that the displayed drone orientation follows your roll/pitch/yaw movements.

Or, use CLI `pacc` to display the IMU accelerometer outputs.

Holding the quad horizontal should give ax:0 ay:0 az:1, for example: `ax:-0.02  ay:-0.00  az:+1.00` 

Holding the quad nose down should give ax:1 ay:0 az:0, for example: `ax:+0.96  ay:-0.04  az:-0.07` 

Holding right side down should give ax:0 ay:1 az:0, for example: `ax:+0.05  ay:+1.00  az:+0.06`

If not, adjust the parameter `imu_align` and re-upload until this matches. 

### Calibrate Gyro/Accelerometer (IMU)

Now calibrate the IMU: place it horizontal and stationary, then type `calimu`, and `save` to store the settings. (The quad will reboot after `save`)

After calibration, use `pahr` to check that the calculated roll and pitch angles are correct.

### Test Radio Link (RCL)

If you did not setup the radio parameters in the previous steps, then type `calradio` and follow the prompts to setup your RC radio receiver.

Check the Radio Link with `ppwm` and `prcl`. 

`ppwm` gives the received pwm values per channel, each value should be between 800 and 2200. 

`prcl` gives the adjusted madflight values:
- Throttle should be between 0 (idle) and 1 (full throttle). 
- Roll and yaw should be 0 for center, -1 for left and +1 for right.
- Pitch should be 0 for center, -1 for pitch down, +1 for pitch up.
- Armed should be 0 when the switch is in disarmed position and 1 in armed position
- Flightmode should be 0,1,2,3,4,5 representing the 6-position flight mode switch. (Flight mode is not used in the Quadcopter example, so can be ignored for now.)

If something does not look right, check/modify your RCL config and upload again. 

### Test Arming (RCL,OUT)

Use CLI commands `pout` to display the motor outputs.

Check the arming mechanism: upon arming `out.armed` changes from 0 to 1.

With arm switch configured (parameter `rcl_arm_ch > 0`)
 - ARMING: Set throttle idle, then flip arm switch to armed
 - DISARMING: Flip arm switch to disarmed

Without arm switch configured (parameter `rcl_arm_ch == 0`)
 - ARMING: Pull both sticks toward you, yaw full right, and roll full left and keep sticks there for 2 sec
 - DISARMING: Pull both sticks toward you, yaw full left, and roll full right and keep sticks there for 2 sec

### Test Motor Mixer (OUT)

LEAVE BATTERY DISCONNECTED

Use CLI commands `pout` to display the motor outputs.

Keep the quad horizontal and stationary.

Now set throttle to mid position, the outputs should go to around 50%: `out.armed:1  M0%:45  M1%:49  M2%:52  M3%:53`

The values will change slowly as the PID integrators build up. This is normal, reduce the throttle to idle and then back to mid position to reset the integrators.

Move pitch stick forward and out0,2 should go up, out1,3 down: `out.armed:1  M0%:71  M1%:43  M2%:72  M3%:44`

Move roll stick right and out2,3 should go up, out0,1 down: `out.armed:1  M0%:41  M1%:47  M2%:69  M3%:75`

Move yaw stick right and out1,2 should go up, out0,3 down: `out.armed:1  M0%:3  M1%:103  M2%:104  M3%:7`

If any of the checks fail -> re-check your IMU, RCL and OUT configuration settings.

### Test Motor Direction and Order (OUT)

Connect the battery but REMOVE PROPELLERS

Use the Motors Tab in the BF Configurator.

Or, in the CLI type `spinmotors` and then `go`. This will spin each motor in order. Check that the correct motor spins, and that the motor spins in the correct direction.

If the incorrect motor spins, change the pin_out<x> parameters to correct this.

If a motor spins in the wrong direction: exchange any 2 of the 3 wires of a brushless motor, or exchange the 2 wires of a brushed motor. Then check again!

## 8. FLY

Again, only continue if all checks and calibrations passed!!!

Have a look at [Quadcopter Example](Example-Quadcopter.md) for details on the quadcopter program.

Mount props, go to a wide open space, and FLY!
