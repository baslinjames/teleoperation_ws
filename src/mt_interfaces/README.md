# Meditel-Interfaces

This package provides interfaces for the MediTel project.

## Messages (.msg)
**Note: All messages are stamped unless there exists both a non-stamped and stamped version.**
- [CartesianCoordinateArray](msg/CartesianCoordinateArray.msg): Array of cartesian coordinates.
- [Drive](msg/Drive.msg): Mode, drives and speed_scalar for driving DCE's X2 platform.
- [Gripper](msg/Gripper.msg): Gripper values. 
- [HandAnalogPoses](msg/HapticArmsPoses.msg): Poses of the haptic arms.
- [MotorState](msg/MotorState.msg): Status of the motors on DCE's X2 platform.
- [Pulse](msg/Pulse.msg): Data from the piezo pulse package.
- [Radar](msg/Radar.msg): Radar distance array.
- [RobotArmPoses](msg/RobotArmPoses.msg): The current and calculated poses of a robot arm.
- [StatusMessage](msg/StatusMessage.msg): A variable severity status message.
- [ThermalImage](msg/ThermalImage.msg): Thermal camera image.

## Services (.srv)
- [Latency](srv/Latency.srv): Service to calculate message latency.
- [SetMode](srv/SetMode.srv): Service to set the mode.
- [SetUInt8](srv/SetUInt8.srv): Service to set a uint8.

