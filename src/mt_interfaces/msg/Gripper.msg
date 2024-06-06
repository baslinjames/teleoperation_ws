# Gripper actuator message

std_msgs/Header header # Mostly for timestamp but can use frame_id if needed

# Robot arm side
uint8 LEFT = 0
uint8 RIGHT = 1

uint8 side # The robot the gripper is attached to

# Open/close constants
float32 CLOSE_TWO_FINGER = 0.0
float32 OPEN_TWO_FINGER = 0.024

float32 CLOSE_THREE_FINGER = 0.0
float32 OPEN_THREE_FINGER = 0.04

float32 actuation_distance # In m

# Ideal gripper force for the Franka Emika Panda gripper
float32 IDEAL_GRIPPER_FORCE = 40.0

float32 actuation_force # In N

# Ideal gripper actuation speed for the Franka Emika Panda gripper
float32 IDEAL_GRIPPER_SPEED = 0.1

float32 actuation_speed # In m/s
