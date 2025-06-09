# Lua Scripting

The Lua 5.4.7 scripting is included in _madflight_. If the file `madflight.lua` is present in the root of the SDCARD, and `#define MF_LUA_ENABLE 1` is set, then this Lua script file will be executed in a separate RTOS task with the same priority as loop().

## Resources

[Lua 5.4 Manual](https://www.lua.org/manual/5.4/)
