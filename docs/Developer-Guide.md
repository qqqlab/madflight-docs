# Developer Guide

This page tries to give some insights in the internal workings of _madflight_.

_madflight_ targets ESP32, RP2 and STM32 platforms with the Arduino framework. _madflight_ can be compiled with the Arduino IDE or with PlatformIO.

## Design Goals

1) Keep the code readable
2) High performance
3) Portability

_madflight_ is split up in modules. Each module lives in a separate subdirectory, for example `gps`. 

For each module a global variable is defined, for example `gps`. Even if the underlying peripheral is not present, the global variable is defined as a placeholder object. This helps to declutter code: 
```C++
#ifdef USE_GPS
  gps.update();
#endif
```
becomes
```C++
gps.update();
```
which is compiled away to a no-op when gps is not used.

The `gps_interface.h` header file defines the interface, the actual implementation is in `gps.h`.

The modules have as little as possible cross-connections to other modules, the actual fusion of the modules takes place in the main .ino program.

## Threads / Tasks / Interrupts

_madflight_ uses the following threads / tasks / interrupts:

|Priority|Description|
|-|-|
highest | IMU interrupt, which wakes up imu_loop() task
high| imu_loop() task
low| loop() and blackbox tasks
lowest| idle task

The [BB] blackbox SDCARD logging module is thread-safe, and runs as a separate task so that slow SDCARD operations do not block other tasks.

The other modules are not thread-safe. Care must be taken to only access a module from a single thread. But even if this rule is broken, the effects should be limited as long as the variables involved are at most 32 bits. For example: when reading the location from the gps module from a different thread as where `gps.update()` is called, one might get the longitude from the previous sample and the latitude from the current sample, but each value itself is correct. At least I hope so, maybe memory alignment plays a role here? Anyway, you have been warned, add a mutex as required.

## C++ as Scripting Language?

As it turns out, the Arduino implementations for the different platforms differ greatly. How to handle the fact that the class for the Serial peripheral is either a `SerialUART`, `HardwareSerial`, or a `MyFancySerial`, which might or might not be derived from `HardwareSerial` ?

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

// now instantiate the serial port it with:

auto *rcin_ser = &Serial1;
// -or-
auto *rcin_ser = new SerialUART(uart0, HW_PIN_RCIN_TX, HW_PIN_RCIN_RX);
// -or-
auto *rcin_ser = new MyFancySerial(HW_PIN_RCIN_RX, HW_PIN_RCIN_TX);

MF_Serial *rcin_Serial = new MF_SerialPtrWrapper< decltype(rcin_ser) >(rcin_ser);
```
