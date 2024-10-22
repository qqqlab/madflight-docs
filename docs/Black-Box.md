# Black Box Data Logging

The black box (BB) module logs flight data for later analysis. The log format is ArduPilot Binary Log. 

Web Tools for Log Analysis:

 - [UAV Log Viewer](https://plot.ardupilot.org/#/)
 - [Dronee Plotter](https://plot.dronee.aero/)
 - [ArduPilot WebTools](https://firmware.ardupilot.org/Tools/WebTools/)

PC Tools for Log Analysis:

 - [MAVExplorer](https://ardupilot.org/dev/docs/using-mavexplorer-for-log-analysis.html)
 - [MissionPLanner](https://ardupilot.org/planner/)
 - [QGroundControl](http://qgroundcontrol.com/)

see also:
https://ardupilot.org/dev/docs/common-logs.html


## File Format of ArduPilot Binary Logs

ArduPilot Binary Log format is a self-describing log format. Messages have fixed length per type.

File Structure:

 - FMT for FMT (required?)
 - FMT (required) for each message type used in the file (can be just before first use of that type, does not need to be in the header section.)
 - UNIT,MULT,FMTU (optional) to further define the meaning of the fields
 - PARM (optional) but some log analyzers expect at least one PARM message
 - MSG (optional) but some log analyzers expect "ArduPilot", "ArduCopter", etc. to identify craft type
 - any other messages

System Messages:

|Name|Description|
|:-:|-|
FMT | Message format for a message - Defines message Name, Type, and Length. Plus field datatypes and field names for up to 16 fields 
UNIT | Units definitions, which can be referenced by FMTU messages
MULT | Multipliers definitions, which can be referenced by FMTU messages
FMTU | Associates Unit (e.g. centimeters/second/second) and Multiplier to FMT message fields
PARM | Parameters (name, value, default_value)
MSG | Text message (max 64 char)

Each message has the following format:

|Name|Bytes|Description|
|:-:|:-:|-|
Header|2|0xA3 0x95
Type|1| As defined in the FMT message
Payload|up to 252| Message data, fixed length per message as defined in FMT message, little endian

The format of each message Type is defined in a FMT message:

|Name|Bytes|Description|
|:-:|:-:|-|
Header|2|0xA3 0x95
FMT Type|1|0x80 (FMT is the only message with a fixed identifier)
Type   |  1 | unique-to-this-log identifier for message being defined
Length  |  1 | the number of bytes taken up by this message (including all headers)
Name    |  4 | name of the message being defined
Format   | 16 | character string defining the C-storage-type of the fields in this message
Columns  | 64 | a comma separated list of column labels of the message being defined

Meaning of format characters in the FMT format string:

|Character|Meaning|
|:-:|-|
  a | int16_t[32]
  b | int8_t
  B | uint8_t
  h | int16_t
  H | uint16_t
  i | int32_t
  I | uint32_t
  f | float
  d | double
  n | char[4]
  N | char[16]
  Z | char[64]
  c | int16_t * 100
  C | uint16_t * 100
  e | int32_t * 100
  E | uint32_t * 100
  L | int32_t latitude/longitude in 10e-7 degrees
  M | uint8_t flight mode (intrepetation depends on verhicle type)
  q | int64_t
  Q | uint64_t


### Example Binary Messages

Example format (FMT) message for Attitude (ATT), the message is 89 bytes long:

```
A3 95 80 64 1C 41 54 54 00 51 63 63 63 63 43 43 43 43 42 00  
00 00 00 00 00 54 69 6D 65 55 53 2C 44 65 73 52 6F 6C 6C 2C  
52 6F 6C 6C 2C 44 65 73 50 69 74 63 68 2C 50 69 74 63 68 2C  
44 65 73 59 61 77 2C 59 61 77 2C 45 72 72 52 50 2C 45 72 72  
59 61 77 2C 41 45 4B 46 00
```

|Name|Meaning|Hex Data|
|:-:|-|-|
Header | | A3 95 
FMT Type | This is a FMT (type 0x80) message | 80 
Type: | 0x64 is the type for the ATT message | 64 
Length | ATT messages are 28 bytes long | 1C 
Name | "ATT" | 41 54 54 00
Format | "QccccCCCCB" field column is 'Q' (8 bytes uint64_t), second field is 'c' (2 bytes int16_t * 100), etc. | 51 63 63 63 63 43 43 43 43 42 00 00 00 00 00 00
Columns | "TimeUS,DesRoll,Roll,DesPitch,Pitch,DesYaw,Yaw,ErrRP,ErrYaw,AEKF" | 54 69 6D 65 55 53 2C 44 65 73 52 6F 6C 6C 2C 52 6F 6C 6C 2C 44 65 73 50 69 74 63 68 2C 50 69 74 63 68 2C 44 65 73 59 61 77 2C 59 61 77 2C 45 72 72 52 50 2C 45 72 72 59 61 77 2C 41 45 4B 46 00

Example Attitude message, the message is 28 bytes long:

```
A3 95 64 CE 85 E1 0A 00 00 00 00 00 00 55 02 3C FF DF FF 00 00 5B 09 01 00 01 00 03
```

|Name|Type|Meaning|Hex Data|
|:-:|-|-|-|
Header | | A3 95 
Type | 0x64 Attitude | 64 
TimeUS | Q = uint64_t | 0x000000000AE185CE = 182.552014 seconds since start | CE 85 E1 0A 00 00 00 00
DesRoll | c = int16_t * 100 | 0x0000 = 0.00 | 00 00
Roll | c = int16_t * 100 | 0x0255 = 5.97 | 55 02
DesPitch | c = int16_t * 100 | 0xFF3C = -1.96 | 3C FF
Pitch | c = int16_t * 100 | 0xFFDF = -0.33 | DF FF
DesYaw | C = uint16_t * 100 | 0x0000 = 0.00 | 00 00
Yaw | C = uint16_t * 100 | 0x095B = 23.95 | 5B 09
ErrRP | C = uint16_t * 100 | 0x0001 = 0.01 | 01 00
ErrYaw | C = uint16_t * 100 | 0x0001 = 0.01 | 01 00
AEKF |B = uint8_t | 3 | 03

### File Structure



