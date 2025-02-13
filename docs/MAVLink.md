# MAVLink

madflight includes the MAVLink protocol. 

## ExpressLRS Setup

- Flash ELRS version 3.5.3 or later to your transmitter and receiver
- Use the ExpressLRS lua script or webinterface to set the link to MAVLink.
- In madflight set `#define RCIN_USE RCIN_USE_MAVLINK`

Optional: Telemetry Script

- Install [https://github.com/iNavFlight/OpenTX-Telemetry-Widget](INAV Lua Telemetry) on the transmitter

Optional: Ground Control

- Update the backpack to version 1.5.1 or later 
- and use [https://ardupilot.org/planner/](Misson Planner) 
- or [https://qgroundcontrol.com/](QGroundControl) via UDP

Optional: Yaapu Telemetry Script

- Use the ExpressLRS Configurator in Expert Mode and flash ELRS with git pull request [https://github.com/ExpressLRS/ExpressLRS/pull/3077](Mavlink to Ardupilot Custom Telemetry CRSF translation for Yaapu Telemetry Script #3077) to your transmitter -AND- receiver
- Install (https://github.com/yaapu/FrskyTelemetryScript)[yaapu FrskyTelemetryScript] on your transmitter

## Implemented MAVLink Messages

### Receiver

MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE
MAVLINK_MSG_ID_PARAM_REQUEST_LIST
MAVLINK_MSG_ID_PARAM_REQUEST_READ
MAVLINK_MSG_ID_PARAM_SET

### Transmitter

MAVLINK_MSG_ID_HEARTBEAT
MAVLINK_MSG_ID_ATTITUDE
MAVLINK_MSG_ID_GLOBAL_POSITION_INT
MAVLINK_MSG_ID_GPS_RAW_INT
MAVLINK_MSG_ID_BATTERY_STATUS
MAVLINK_MSG_ID_PARAM_VALUE
MAVLINK_MSG_ID_STATUSTEXT
