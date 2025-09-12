<img src="/img/desk.jpg" width="100%" />

# Developer Guide

This page tries to give some insights in the internal workings of _madflight_.

_madflight_ targets ESP32, RP2 and STM32 platforms with the Arduino framework. _madflight_ can be compiled with the Arduino IDE or with PlatformIO.

## Design Goals

1. Keep the code readable
2. High performance
3. Portability

_madflight_ is a collection of code modules from which you can pick and choose to build flight controllers. The examples are there to show some possibilities. The madflight.h header is the glue between the main program (examples) and the modules.

## Modules

Each module lives in a separate subdirectory, for example `gps`. For each module a global variable is defined, for example `gps`. Even if the underlying peripheral is not present, the global variable is defined as a placeholder object. This helps to declutter code: 
```C++
#ifdef USE_GPS
  gps.update();
#endif
```
becomes
```C++
gps.update();
```
which is a no-op when gps is not used.

The `gps.h` header file defines the interface, the actual implementation is in `gps.cpp`. Some modules have a `cfg_cpp.h` instead of `cfg.cpp`, this is because these files contain compile time options which can be set with #define in the main.cpp program.

The modules have as little as possible cross-connections to other modules, the actual fusion of the modules takes place in the main.cpp program.

Most modules have an update() method which needs to be called periodically from your program. This keeps the implementation flexible: you decide when (polling, interrupt, timer) and from which thread the updates take place.

## What is a 'gizmo'

>> Something, generally a device, for which one does not know the proper term.

In _madflight_ gizmo is the underlying device of a module, for example a BMP390 barometer sensor for the `bar` module, or a MAVLink radio receiver for the `rcl` module, or a Mahony complentary filter for the `ahr` module.

## Threads / Tasks / Interrupts

_madflight_ uses FreeRTOS, and uses the following threads / tasks / interrupts:

|Priority|Description|
|-|-|
highest | IMU interrupt, which wakes up imu_loop() task
high    | imu_loop() task
low     | loop(), blackbox, and lua tasks
lowest  | idle task

The [BBX] blackbox SDCARD logging module is thread-safe, and runs as a separate task so that slow SDCARD operations do not block other tasks.

The other modules are not thread-safe. Care must be taken to only access a module from a single thread. But even if this rule is broken, the effects should be limited as long as the variables involved are at most 32 bits. For example: when reading the location from the gps module from a different thread as where `gps.update()` is called, one might get the longitude from the previous sample and the latitude from the current sample, but each value itself is correct. At least I hope so, maybe memory alignment plays a role here? Anyway, you have been warned, add a mutex as required.

## Hardware Abstraction Layer (HAL)

You would hope that Arduino for ESP32 is the same as Arduino for STM32. Wrong!!! As it turns out, the Arduino implementations for the different platforms differ greatly. How to handle the fact that the class for the Serial peripheral is either a `SerialUART`, `HardwareSerial`, or a `MyFancySerial`, which might or might not be derived from `HardwareSerial` ?

One way to do this is to use templates to abstract these objects. I tried this, but it did not make me happy. After spending many hours trying to rewrite the modules to `template<class SerialType> MyGpsDriver` I gave up this route: too much rewriting needed, and I was spending too much time on cryptic compiler/linker errors messages.

A second solution is to place the full implementation of the module in the header file, and make it refer a peripheral global object `gps_Serial`. The type of `gps_Serial` can be anything, as long as it implements the serial methods used by the module. If things don't work, you get a clear error message pointing out that method `availableForWrite()` is missing for `gps_Serial`. This basically makes C++ a scripting language. Disadvantage is that only one module driver can be active, as the driver is accessing the global peripheral object directly.

The third option is to define the drivers as abstract, and inherit the driver to implement the peripheral interface. The peripheral implementation is done in the module header file, thus having the "scripting" advantage, but also allows for multiple driver instances.

A 4th way, and chosen way, is to define an abstract class `MF_Serial` and use this as basis for the modules. But this results in a lot of extra code to derive a `MF_Serial` for each Serial peripheral class one wishes to use. However, for Arduino we can use a template for this. The Arduino classes have different types, but all classes implement methods with the same name, for example `begin(baud)`. This results in the following implementation in _madflight_:

```C++
class MF_Serial {
  public:
    virtual void begin(int baud) = 0;
    ...
};

template<class T>
class MF_SerialPtrWrapper : public MF_Serial {
  protected:
    T _serial;
  public:
    MF_SerialPtrWrapper(T serial) {
       _serial = serial;
    }
    void begin(int baud) override {
      _serial->begin(baud);
    }
    ...
};

...

// Now instantiate a MF_Serial serial port in two steps:

// First create an instance of the Arduino-like serial class
auto *serial = &Serial1;
// -or-
auto *serial = new SerialUART(uart0, PIN_TX, PIN_RX);
// -or-
auto *serial = new MyFancySerial(PIN_RX, PIN_TX);

// Then use the wrapper to create the madflight serial instance, use decltype to get the type of the auto variable
MF_Serial *mf_serial = new MF_SerialPtrWrapper< decltype(serial) >(serial);
```

Apart from the differences in the Arduino class structure, there are also differences in the actual implementation of the interface classes. What happens when I call Serial.write("1234567890") ? Does it block while 10 chars are written directly to the UART, does it write to a buffer and use interrupts, does it use DMA??? For _madflight_ the following assumptions are made for the interfaces:

**Serial:** Reading and writing is non-blocking, not thread-safe. At least 255 byte input and output buffers. Attempting to write more than fits in the free buffer space fails gracefully (no bytes are written to the buffer).

**I2C (Wire):** Reading and writing is blocking, not thread-safe.

**SPI:** Reading and writing is blocking, not thread-safe.



## Creating a new Gizmo for a Module

Let's assume we want to add the SEEALL radar gizmo to the `rdr` module.

### Anatomy of a Module/Gizmo

The module header `rdr/rdr.h` defines the module and gizmo interfaces:

- `struct RdrState` is the state info, in this case the measured distance by the radar gizmo
- `struct RdrConfig` is the config for the module, the config always contains which gizmo to use (Cfg::rdr_gizmo_enum gizmo)
- `class RdrGizmo` is the abstract base class for the rdr gizmos
- `class Rdr : public RdrState` is the module class, which usually has setup() and update() methods. It inherits RdrState, so that we can access the state variables directly like: rdr.dist
- `extern Rdr rdr` declares the global instance for the module

### Steps to Create Gizmo SEEALL

#### cfg/cfg.h

Edit file `cfg/cfg.h` and append mf_SEEALL to the option list of the rdr_gizmo parameter.

#### rdr/RdrGizmoSEEALL.h

Create file `rdr/RdrGizmoSEEALL.h` for `class RdrGizmoSEEALL : public RdrGizmo` and implement:

- `static RdrGizmoSEEALL* create(RdrConfig *c, RdrState *s)` which returns a pointer to the created gizmo on success, or nullptr on failure.
- `bool update() override` which updates RdrState *s (i.e. distance).

The update() method should be non-blocking. So instead of: "trigger measurement, wait for completion, report result", do "exit if busy, report result if measurement received, trigger next measurement".

Have a look at the other gizmos for inspiration, or use one as template for your new gizmo.

#### External libraries

Optional: if your gizmo uses an external library, copy the external library to folder `rdr/SEEALL`. Copy only the required source files and create a single readme.txt file with a link to the external lib and the lib's license info. Do not copy examples and other  optional files. I know, this copying feels wrong, but it guarantees that _madflight_ will always compile, even if the external lib changes or disappears.

#### rdr/rdr.cpp

Edit file `rdr/rdr.cpp` and add the SEEALL gizmo to switch(config.gizmo) in Rdr::setup()

#### Publish your work

That's it. Now test,test,test by setting `rdr_gizmo SEEALL` in your madflight config. When you're confident that it works, create a Pull Request on Github and let others profit from your work.
