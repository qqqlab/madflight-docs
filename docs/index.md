![](img/madflight_logo_2000x538.png){ .off-glb }

**madflight** is a toolbox to build high performance flight controllers with Aduino IDE or PlatformIO for ESP32-S3 / ESP32 / RP2350 / RP2040 / STM32. A functional DIY flight controller can be build for under $10 from readily available [development boards](Controller-Boards.md) and [sensor breakout boards](Sensor-Boards.md).

Flight tested example programs for quadcopter and airplane are included. The example programs are only a couple hundred lines long, but contain the full flight controller logic. The nitty-gritty low-level sensor and input/output management is done by the madflight library.

If you like _madflight_, please give it a [&star; star on GitHub](https://github.com/qqqlab/madflight)

## Getting Started

Click [here](Getting-Started.md)

<img src="/img/madflight RP2040 flight controller.jpeg" title="madflight RP2040 flight controller" width="25%" /> <img src="/img/madflight drone.jpeg" title="madflight drone" width="19.6%" /> <img src="/img/madflight ESP32 flight controller.jpeg" title="madflight ESP32 flight controller" width="19.1%" />

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
- Inspired by dRehmFlight
- Coded primarily for readability, then for speed and code size.
- No external dependencies, all modules are included as subdirectories of the `src` directory.
- The madflight flight controller runs standard `setup()` and `loop()`.
- It mainly uses plain Arduino functionality: Serial, Wire, and SPI. One custom hardware dependent library is used for PWM. Therefor, it can fairly easily ported to other 32 bit microcontrollers that support the Arduino framework. Also porting to other build environments like CMake should not be a huge effort.
- The following modules are used:
    - `ahr` Attitude Heading Reference System, estimates roll, yaw, pitch
    - `alt` Altitude estimators
    - `bar` Barometer sensor
    - `bat` Battery monitor
    - `bbx` Black Box data logger
    - `brd` Board definition headers
    - `cfg` Read and save configuration to eeprom (flash)
    - `cli` Command Line Interface for debugging, configuration and calibration
    - `gps` GPS receiver
    - `hal` Hardware Abstraction Layer, specific code for STM32, RP2040 and ESP32
    - `imu` Inertial Measurement Unit, retrieves accelerometer, gyroscope, and magnetometer sensor data
    - `led` LED driver
    - `mag` Magnetometer sensor (external)
    - `nav` Waypoint navigation
    - `out` Output to motors and servos
    - `pid` PID controller
    - `rcl` Remote Control Link, retrieves RC receiver data and sends telemetry data
    - `tbx` Toolbox, common tools
    - `veh` Vehicle info
- Most modules are interfaced through a global object, for example the `imu` object has property `imu.gx` which is the current gyro x-axis rate in degrees per second for the selected IMU chip.
- The module implementations are in subdirectories of the `src` directory. Here you'll find the module header file, e.g. `src/imu/imu.h`.
- The module files are usually subdivided in gizmos, which can be selected in _madflight_ config. For example: `imu_gizmo MPU6500`

## Disclaimer

This code is a shared, open source flight controller for small micro aerial vehicles and is intended to be modified to suit your needs. It is NOT intended to be used on manned vehicles. I do not claim any responsibility for any damage or injury that may be inflicted as a result of the use of this code. Use and modify at your own risk. More specifically put:

THIS SOFTWARE IS PROVIDED BY THE CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Logo image copyright (c) 1975 Deutsches MAD Magazine. This project is not associated with MAD Magazine.
