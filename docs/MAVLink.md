# MAVLink

madflight includes the MAVLink protocol. 

## ExpressLRS MAVLink Setup

- Flash ELRS version 3.5.3 or later to your transmitter and receiver
- Use the ExpressLRS lua script or webinterface to set the link to MAVLink.
- In madflight set `rcl_gizmo MAVLINK`

Optional: Telemetry Script

- Install [INAV Lua Telemetry](https://github.com/iNavFlight/OpenTX-Telemetry-Widget) on the transmitter

Optional: Ground Control

- Update the backpack to version 1.5.1 or later 
- Connect [Misson Planner](https://ardupilot.org/planner/) or [QGroundControl](https://qgroundcontrol.com/) via UDP

Optional: Yaapu Telemetry Script

- Use the ExpressLRS Configurator in Expert Mode and flash ELRS with git pull request [Mavlink to Ardupilot Custom Telemetry CRSF translation for Yaapu Telemetry Script #3077](https://github.com/ExpressLRS/ExpressLRS/pull/3077) to your transmitter -AND- receiver
- Install [yaapu FrskyTelemetryScript](https://github.com/yaapu/FrskyTelemetryScript) on your transmitter

## Implemented MAVLink Messages

### Received by madflight

- MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE
- MAVLINK_MSG_ID_PARAM_REQUEST_LIST
- MAVLINK_MSG_ID_PARAM_REQUEST_READ
- MAVLINK_MSG_ID_PARAM_SET

### Transmitted by madflight

- MAVLINK_MSG_ID_HEARTBEAT
- MAVLINK_MSG_ID_ATTITUDE
- MAVLINK_MSG_ID_GLOBAL_POSITION_INT
- MAVLINK_MSG_ID_GPS_RAW_INT
- MAVLINK_MSG_ID_BATTERY_STATUS
- MAVLINK_MSG_ID_PARAM_VALUE
- MAVLINK_MSG_ID_STATUSTEXT
