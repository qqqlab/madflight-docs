# Quadcopter

Minimal quadcopter demo program

## Required Hardware

- IMU sensor (SPI or I2C)
- RC receiver with 5 channels (CRSF/ELRS preferred)
- 4 brushless motors with ESCs

## Connecting Hardware

- SPI IMU: connect _pin_imu_int_, _pin_imu_cs_, _pin_spi0_miso_, _pin_spi0_mosi_, _pin_spi0_sclk_
- or for I2C IMU: connect _pin_imu_int_, _pin_i2c1_scl_, _pin_i2c1_sda_
- RC receiver: connect _pin_ser0_rx_ to receiver TX pin
- ESCs: _pin_out0_ ... _pin_out3_ to the ESC inputs of motor1 ... motor4

Motor order diagram (Betaflight order)

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

Default flight mode is RATE. The mode can be changed to ANGLE by changing the PID controller in imu_loop(). Important: calibrate the accelometer before using ANGLE.

## RATE Mode

The roll/pitch stick inputs control the rate of change. Keeping the sticks centered will keep turning at the current rate.

## ANGLE Mode

The roll/pitch stick inputs control the roll/pitch angle. Keeping the sticks centered will keep the quadcopter horizontal.

## Arming / Disarming

With a dedicated switch channel:

- Arming: Set throttle low, then flip arm switch from DISARMED to ARMED
- Disarming: Flip arm switch from ARMED to DISARMED, at any throttle position. "Kill switch"

With stick commands (when parameter cfg.rcl_arm_ch == 0):

- Arming: throttle pulled, yaw right, pitch pulled, roll left, and keep sticks for 2 seconds
- Disarming: throttle pulled, yaw left, pitch pulled, roll right, and keep sticks for 2 seconds

## LED Status

- OFF - not powered
- startup - a couple blinks then ON while running gyro calibration (don't move)
- blinking long OFF short ON - DISARMED
- blinking long ON short OFF - ARMED
- blink interval longer than 1 second - imu_loop() is taking too much time
- fast blinking - something is wrong, connect USB serial for info

## Building and Flying

This is one of the quads I use for testing. Note that it does not use the default board for ESP32. And a custom pinout is used, so that the MPU6500 board can be soldered directly with pins to the ESP32 board, and just requires the red wire for 3V. The I2C sensor boards are also soldered directly with pins, plus one black ground wire.

<img src="../img/ex-q1.jpg" width="19.9%" /> <img src="../img/ex-q2.jpg" width="25%" /> <img src="../img/ex-q3.jpg" width="25%" />

### Bill of Material

|Part|Price|
|-|-:|
Lolin ESP32 Lite | $2.9
MPU6500 SPI Gyro/Acc Module | $1.3
ELRS receiver | $7.2
Dualsky Hornet 460 Quadcopter minus original brain
BME280 Barometer Module (not used in this example) | $0.8
QMC5883L Magnetometer Module (not used in this example) | $1.8
