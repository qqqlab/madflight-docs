# Quadcopter

Quadcopter demo program.

## Required Hardware

- IMU sensor (SPI or I2C)
- RC receiver with 6 channels for Roll, Pitch, Throttle, Yaw, Arm Switch, Flight Mode Switch
- 4 brushless motors with ESCs -OR- 4 brushed motors with MOSFET drivers

## Connecting Hardware

See [Getting Started](Getting-Started.md)

## Flight Modes

Default flight mode is RATE. The mode can be changed to ANGLE with the flight mode channel of the radio controller. Important: calibrate the accelometer before using ANGLE.

## RATE Mode

The roll/pitch stick inputs control the rate of change. Keeping the sticks centered will keep turning at the current rate.

## ANGLE mode

The roll/pitch stick inputs control the roll/pitch angle. Keeping the sticks centered will keep the quadcopter horizontal.

## Arming / Disarming

With a dedicated switch channel:

- Arming: Set throttle low, then flip arm switch from DISARMED to ARMED
- Disarming: Flip arm switch from ARMED to DISARMED, at any throttle position. "Kill switch"

With stick commands (when parameter `rcl_arm_ch == 0`):

- Arming: throttle pulled, yaw right, pitch pulled, roll left, and keep sticks for 2 seconds
- Disarming: throttle pulled, yaw left, pitch pulled, roll right, and keep sticks for 2 seconds

## LED Status

- OFF - not powered
- ON blue - startup, running gyro calibration (don't move)
- Blinking green - long OFF short ON - DISARMED
- Blinking red - long ON short OFF - ARMED
- Blink interval longer than 1 second - imu_loop() is taking too much time - Use CLI `ps` to check
- Fast blinking orange - something is wrong, connect USB serial for info

## Building and Flying

These are some of the quads I used for testing...

### RP2350 DJI Phantom 1 P330

Swapped existing NAZA flight controller with a madflight FC3 RP2350. The existing ESCs are recycled and driven in PWM 400Hz mode.

Specs

 - DJI Phantom 1 P330
 - madflight FC3 RP2350 Flight Controller
 - ELRS Nano Receiver
 - Short USB-C cable for programming

<img src="../img/dji_phantom1_p330_h1024.jpg"/>


### ESP32-S3 5" sub 250 gram

Lightweight 6mm bambu + printed PETG motor mounts

Specs

 - madflight FC1 ESP32-S3 Flight Controller
 - ELRS Nano Receiver
 - ESCs: Favorite LittleBee Spring 20A with BlueJay bidir DSHOT
 - 1503 Motors
 - Folding 5" 120mm props
 - Weight: 201 gr (including 2S 18650 battery)

<img src="../img/esp32_drone_h1024.jpg"/>


### RP2350 Quad with 9" Props

Specs

 - Raspberry Pi Pico2
 - MPU6500 Gyro/Acc Module
 - BME280 Barometer Module
 - INA226 Current Sensor Module
 - Micro SD Card Module
 - Mini DC-DC 12-20V to 5V 3A Buck Converter
 - uBlox M8 GPS with QMC5883L Compass Compass
 - ELRS receiver
 - DJI E300 Propulsion System (9.4x4.3 props, 2212 920KV motors, 15A ESCs)
 - Frame of a Ideafly IFLY-4 Quadcopter

<img src="../img/ex-qa1.jpg" width="19%" /> <img src="../img/ex-qa2.jpg" width="15%" /> <img src="../img/ex-qa3.jpg" width="36%" />


### ESP32 Dualsky Hornet 460

This build does not use the default board for ESP32. And a custom pinout is used, so that the MPU6500 board can be soldered directly with pins to the ESP32 board, and just requires the red wire for 3V. The I2C sensor boards are also soldered directly with pins, plus one black ground wire.

Specs

 - Lolin ESP32 Lite
 - MPU6500 SPI Gyro/Acc Module
 - ELRS receiver
 - Dualsky Hornet 460 Quadcopter minus original brain
 - BME280 Barometer Module
 - QMC5883L Magnetometer Module

<img src="../img/ex-q1.jpg" width="19.9%" /> <img src="../img/ex-q2.jpg" width="25%" /> <img src="../img/ex-q3.jpg" width="25%" />
