# Quadcopter Advanced

Advanced quadcopter demo program. Adds flight modes selectable by radio controller, telemetry, barometer, gps, and logging.

## Required Hardware

- IMU sensor (SPI or I2C)
- RC receiver with 6 channels (CRSF/ELRS preferred)
- 4 brushless motors with ESCs -OR- 4 brushed motors with MOSFETs

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

With stick commands (when parameter cfg.rcl_arm_ch == 0):

- Arming: throttle pulled, yaw right, pitch pulled, roll left, and keep sticks for 2 seconds
- Disarming: throttle pulled, yaw left, pitch pulled, roll right, and keep sticks for 2 seconds

## LED Status

- OFF - not powered
- ON - startup, running gyro calibration (don't move)
- blinking long OFF short ON - DISARMED
- blinking long ON short OFF - ARMED
- blink interval longer than 1 second - imu_loop() is taking too much time
- fast blinking - something is wrong, connect USB serial for info

## Building and Flying

Bill of Materials

|Part|Price|
|-|-:|
Raspberry Pi Pico | $4.0
MPU6500 Gyro/Acc Module | $1.3
BME280 Barometer Module | $0.8
INA226 Current Sensor Module | $1.0
Micro SD Card Module | $0.6 
Mini DC-DC 12-20V to 5V 3A Buck Converter | $0.4
uBlox M8 GPS with QMC5883L Compass Compass | $10.0
ELRS receiver | $7.2
DJI E300 Propulsion System (9.4x4.3 props, 2212 920KV motors, 15A ESCs)
Frame of a Ideafly IFLY-4 Quadcopter

<img src="../img/ex-qa1.jpg" width="19%" /> <img src="../img/ex-qa2.jpg" width="15%" /> <img src="../img/ex-qa3.jpg" width="36%" />
