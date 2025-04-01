![](img/madflight_logo_2000x538.png){ .off-glb }

**madflight** is a toolbox to build high performance flight controllers with Aduino IDE or PlatformIO for ESP32-S3 / ESP32 / RP2350 / RP2040 / STM32. A functional DIY flight controller can be build for under $10 from readily available [development boards](Controller-Boards.md) and [sensor breakout boards](Sensor-Boards.md).

Flight tested example programs for quadcopter and airplane are included. The example programs are only a couple hundred lines long, but contain the full flight controller logic. The nitty-gritty low-level sensor and input/output management is done by the madflight library.

## Your Feedback is Welcome

If you like <b>madflight</b>, give it a [&star; star on GitHub](https://github.com/qqqlab/madflight), or fork it and contribute. Thanks!

<img src="img/madflight RP2040 flight controller.jpeg" title="madflight RP2040 flight controller" width="25%" /> <img src="img/madflight drone.jpeg" title="madflight drone" width="19.6%" /> <img src="img/madflight ESP32 flight controller.jpeg" title="madflight ESP32 flight controller" width="19.1%" />

## Required Hardware

- [Development board](Controller-Boards.md): 
    - [RP2350/RP2040](Board-RP2040.md) (e.g. Raspberry Pi Pico2)
    - [ESP32-S3/ESP32](Board-ESP32.md) (e.g. Espressiv ESP32-S3 DevKitC)
    - [STM32](Board-STM32.md) (e.g. Black Pill or a commercial flight controller)
- [SPI IMU sensor](Sensor-Boards.md) (BMI270, MPU9250, MPU6500, MPU6000, ICM45686), if not available then use an I2C IMU sensor (MPU6050, MPU9150) 
- RC Receiver: MAVLink, ELRS, CRSF, SBUS, DMSX, or PPM
- BEC or DC-DC converter to power your board from a battery
- ESC (OneShot125 or 50-490Hz PWM) and/or servos (50-490Hz PWM)

## Optional Hardware

- GPS Module (Serial)
- Barometer (I2C BMP280, BMP388, BMP390, MS5611)
- Magnetometer (I2C QMC5883L)
- Current/Voltage Sensor (ADC or I2C INA226, INA228)
- [Optical Flow Sensor](https://github.com/qqqlab/ESP32-Optical-Flow) (I2C)

## Getting Started

1. Connect the required hardware to your controller board: 
    - See [RP2350/RP2040 pinout and instructions](Board-RP2040.md)
    - -or- [ESP32-S3/ESP32 pinout and instructions](Board-ESP32.md)
    - -or- [STM32 pinout and instructions](Board-STM32.md)
    - Connect your IMU (gyro/acceleration) sensor as shown [below](#connecting-the-imu-sensor).
    - Connect your radio receiver according to the configured pins.
2. Install the madflight library in Arduino IDE. (Menu *Tools->Manage Libraries*, then search for **madflight**)
3. Open *Examples for custom libraries->madflight->Quadcopter.ino* in the Arduino IDE.
4. Edit the HARDWARE section in madflight_config.h to enable the connected peripherals.
5. If you're not using the default pinout then setup your board pinout in the CUSTOM PINS section.
6. Compile Quadcopter.ino and upload it to your board. Connect the Serial Monitor at 115200 baud and check the messages. Type `help` to see the available CLI commands.
7. Type `calradio` and follow the prompts to setup your RC radio receiver.
8. IMPORTANT: Use CLI `calimu` and `calmag` to calibate the sensors.
9. Use CLI commands `pimu`, `pahr`, `prcl`, `pmot`, etc. and check that IMU sensor, AHRS and RC Receiver are working correctly. 
10. Connect motors (no props) and battery and check that motors are spinning correctly.
11. Mount props, go to an wide open space, and FLY!

## Safety First!!!

By default madflight has these safety features enabled:

- Motors only rotate when armed.
- Arming Procedure: set throttle low then flip the arm switch from disarmed to armed.
- Kill Switch: when the arm switch is in the disarm position, disarm and stop motors until re-armed.
- Failsafe: when radio connection is lost, disarm and stop motors until re-armed.
- Armed Low Throttle: motors run at low speed, to give visible armed indication.
- LED armed/disarmed indicator.

## Software Design

- Keep it simple!!!
- Based on [dRehmFlight](https://github.com/nickrehm/dRehmFlight)
- Coded primarily for readability, then for speed and code size.
- No external dependencies, all modules are included in the `src/madflight` directory.
- The madflight flight controller runs standard `setup()` and `loop()`.
- It mainly uses plain Arduino functionality: Serial, Wire, and SPI. One custom hardware dependent library is used for PWM. Therefor, it can fairly easily ported to other 32 bit microcontrollers that support the Arduino framework. Also porting to other build environments like PlatformIO or CMake should not be a huge effort.
- The following modules are used:
    - `ahr` Attitude Heading Reference System, estimates roll, yaw, pitch
    - `bar` Barometer sensor
    - `bat` Battery monitor
    - `bbx` Black Box data logger
    - `cfg` Read and save configuration to eeprom (flash)
    - `cli` Command Line Interface for debugging, configuration and calibration
    - `gps` GPS receiver
    - `hal` Hardware Abstraction Layer, specific code for STM32, RP2040 and ESP32
    - `imu` Inertial Measurement Unit, retrieves accelerometer, gyroscope, and magnetometer sensor data
    - `led` LED driver
    - `mag` Magnetometer sensor (external)
    - `out` Output to motors and servos
    - `rcl` Remote Control Link, retrieves RC receiver data and sends telemetry data
- Most modules are interfaced through a global object, for example the `imu` object has property `imu.gx` which is the current gyro x-axis rate in degrees per second for the selected IMU chip.
- The module implementations are in subdirectories of the `src/madflight` directory. Here you find the module header file, e.g. `src/madflight/imu/imu.h`.
- The module files are usually subdivided in gizmos, which can be selected in _madflight_ config. For example: `imu_gizmo MPU6500`

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

## Changes from dRehmFlight

- Add support for RP2350, RP2040, ESP32, ESP32-S3, and STM32
- Dropped Teensy support, but could be re-added by creating a hal files. (I just don't have the hardware to test on)
- Moved all hardware specific code to the hal directory and added hardware specific libraries
- Reduced the number of global variables
- Oneshot is implemented as PWM up to 3.9kHz
- New libs for IMU sensors
- Changed arming logic
- Loop rate set to 1kHz to match IMU sensor rate
- Interrupt driven IMU operation

## Flight Controllers on Github

In (approximate) increasing order of complexity.

| Flight Controller | Features | Development Environment | Microcontroller |
|:-|:-|:-|:-|
[drone-flight-controller](https://github.com/lobodol/drone-flight-controller) | Single 700 line ino file, no libs | Arduino | ATmega328P
[dRehmFlight](https://github.com/nickrehm/dRehmFlight) | Quad, Plane, VTOL | Arduino | Arduino Teensy 4
[madflight](https://github.com/qqqlab/madflight) | Quad, Plane, VTOL, based on dRehmFlight | Arduino | ESP32, RP2040, and STM32
[esp-fc](https://github.com/rtlopez/esp-fc) | FPV Quad | PlatformIO | ESP32
[Crazyflie](https://github.com/bitcraze/crazyflie-firmware) | FPV Quad | | STM32F405
[esp-drone](https://github.com/espressif/esp-drone) | FPV Quad, based on Crazyflie | | ESP32 
[Betaflight](https://github.com/betaflight/betaflight) | FPV Quad, based on cleanflight | | STM32 F4/F7/H7
[EmuFlight](https://github.com/emuflight/EmuFlight) | Multi-rotor, based on cleanflight | | STM32 F4/F7
[inav](https://github.com/iNavFlight/inav) | Plane, based on cleanflight | | STM32 F4/F7/H7
[Ardupilot](https://github.com/ArduPilot/ardupilot) | Quad, Plane, VTOL | Linux waf | STM32 F4/F7/H7 or Linux based
[PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) | Quad, Plane, VTOL | | STM32 F4/F7/H7 |


<img src="https://api.star-history.com/svg?repos=qqqlab/madflight,rtlopez/esp-fc,lobodol/drone-flight-controller,emuflight/EmuFlight,espressif/esp-drone,nickrehm/dRehmFlight&type=Date" width="48%" /> <img src="https://api.star-history.com/svg?repos=bitcraze/crazyflie-firmware,iNavFlight/inav,PX4/PX4-Autopilot,betaflight/betaflight,ArduPilot/ardupilot&type=Date" width="48%" />

## Disclaimer

This code is a shared, open source flight controller for small micro aerial vehicles and is intended to be modified to suit your needs. It is NOT intended to be used on manned vehicles. I do not claim any responsibility for any damage or injury that may be inflicted as a result of the use of this code. Use and modify at your own risk. More specifically put:

THIS SOFTWARE IS PROVIDED BY THE CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Logo image copyright (c) 1975 Deutsches MAD Magazine. This project is not associated with MAD Magazine.
