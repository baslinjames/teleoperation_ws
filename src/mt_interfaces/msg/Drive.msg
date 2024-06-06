# The data required to drive DCE's X2 platform

# Constants for the drive mode
uint8 VECTOR_DRIVE_MODE = 1
uint8 DIRECT_DRIVE_MODE = 2

std_msgs/Header header # Mostly for timestamp but can use frame_id if needed

uint8 drive_mode # The control scheme expected by the ROS2 node controlling the X2

# Range: [-1.0, 1.0]
# Direct: Positive is forwards, negative is backwards
# Vector: Positive is forwards, negative is backwards
float32 drive_1

# Range: [-1.0, 1.0]
# Direct: Positive is forwards, negative is backwards
# Vector: Positive is right, negative is left
float32 drive_2

# Range: [0.0, 100.0]
float32 speed_scalar # Scales the input drive values as a percentage of the max value
