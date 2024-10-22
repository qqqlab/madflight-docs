# Plane

This program is an airplane controller, it has 3 flight modes: MANUAL, ROLL and FBWA.

NOTE: This program is experimental - it was only flight-tested in a couple of flights.

## MANUAL Mode

Regular RC control, no stabilization. All RC inputs are passed through to the servo outputs.

## ROLL Mode

Stabilize roll angle

## FBWA Fly By Wire A Mode (inspired by ArduPilot)

This is the most popular mode for assisted flying, and is the best mode for inexperienced flyers. In this mode the
plane will hold the roll and pitch specified by the control sticks. So if you hold the aileron stick hard right then the 
plane will hold its pitch level and will bank right by the angle specified in the roll limit parameter. It is not possible 
to roll the plane past the roll limit, and it is not possible to pitch the plane beyond the pitch limit settings.

Note that holding level pitch does not mean the plane will hold altitude. How much altitude a plane gains or loses at a 
particular pitch depends on its airspeed, which is primarily controlled by throttle. So to gain altitude you should raise 
the throttle, and to lose altitude you should lower the throttle.

In FBWA mode the rudder is under manual control.

## Setup Procedure

First edit sections "PINS", "BOARD", "HARDWARE", "RC RECEIVER", "OUTPUTS". Use CLI to verify things work as expected.

Calibrate gyro, accelerometer, and magnetometer --- important!!!

Connect power and then let plane sit for 15 seconds, during this time the gyro biases are re-calibrated.

Do a dry run:

Set to MANUAL and power up the plane. Move the rc controls and make sure that the aileron, elevator, and rudder move in 
the correct direction. Arm the plane, and carefully test the motor, then disarm.
If incorrect: modify the #define OUT_ELEVATOR_DOWN etc. statements.

Then set to FBWA flight mode, keep the radio sticks centered, and move the plane around, to make sure that the control 
surfaces work to oppose the move, that is: pitching the plane down should move elevator up, banking right should deflect 
the right aileron down, left aileron up. 

Another thing that needs to be set are the PID parameters. Set to ROLL or FBWA mode and adjust the PID parameters so that the 
control surfaces react quickly, but don't oscillate, on changes in attitude.

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