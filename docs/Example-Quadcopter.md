# Quadcopter

Minimal quadcopter demo program

## Required Hardware

- IMU sensor (SPI or I2C)
- RC receiver with 5 channels (CRSF/ELRS preferred)
- 4 brushless motors with ESCs

## Connecting Hardware

- IMU: connect IMU_EXTI, IMU_CS, SPI_MISO, SPI_MOSI, SPI_CLK for SPI (or IMU_EXTI, I2C_SDA, I2C_SCL for I2C)
- RC receiver: connect RCIN_RX to receiver TX pin
- ESCs: PWM1-4 to the ESC inputs

Motor order diagram (Betaflight order)

```
      front
 CW -->   <-- CCW
     4     2 
      \ ^ /
       |X|
      / - \
     3     1 
CCW -->   <-- CW
```

Default flight mode is RATE. The mode can be changed to ANGLE by changing the PID controller in imu_loop(). Important: calibrate the accelometer before using ANGLE.

## RATE Mode

The roll/pitch stick inputs control the rate of change. Keeping the sticks centered will keep turning at the current rate.

## ANGLE mode

The roll/pitch stick inputs control the roll/pitch angle. Keeping the sticks centered will keep the quadcopter horizontal.

## Arming / Disarming

- Arming: Set throttle low, then flip arm switch from DISARMED to ARMED.
- Disarming: Flip arm switch from ARMED to DISARMED, at any throttle position. "Kill switch".

## LED Status

- OFF - not powered
- startup - a couple blinks then ON while running gyro calibration (don't move)
- blinking long OFF short ON - DISARMED
- blinking long ON short OFF - ARMED
- blink interval longer than 1 second - imu_loop() is taking too much time
- fast blinking - something is wrong, connect USB serial for info
