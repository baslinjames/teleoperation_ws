# The status of the motors on DCE's X2 platform

# Motor 1 error codes
# Converted from byte to uint8
uint8 MOTOR_1_NO_ERROR = 16 # 0x10
uint8 MOTOR_1_NOT_STARTED = 17 # 0x11
uint8 MOTOR_1_NOT_HOMED = 18 # 0x12
uint8 MOTOR_1_CONTROL_ERROR_CHANNEL_DISABLED = 19 # 0x13
uint8 MOTOR_1_WRONG_MODE = 20 # 0x14
uint8 MOTOR_1_READBACK_COMMAND_NOT_RECOGNISED = 21 # 0x15
uint8 MOTOR_1_SIGNAL_LOST = 22 # 0x16

# Motor 2 error codes
uint8 MOTOR_2_NO_ERROR = 32 # 0x20
uint8 MOTOR_2_NOT_STARTED = 33 # 0x21
uint8 MOTOR_2_NOT_HOMED = 34 # 0x22
uint8 MOTOR_2_CONTROL_ERROR_CHANNEL_DISABLED = 35 # 0x23
uint8 MOTOR_2_WRONG_MODE = 36 # 0x24
uint8 MOTOR_2_READBACK_COMMAND_NOT_RECOGNISED = 37 # 0x25
uint8 MOTOR_2_SIGNAL_LOST = 38 # 0x26

# Motor control mode
uint8 LOCAL_MODE = 0 # 0x00
uint8 REMOTE_MODE = 1 # 0x01

std_msgs/Header header # Mostly for timestamp but can use frame_id if needed

int32 motor_1_encoder_counter
int32 motor_1_speed
uint8 motor_1_error

int32 motor_2_encoder_counter
int32 motor_2_speed
uint8 motor_2_error

uint8 control_mode