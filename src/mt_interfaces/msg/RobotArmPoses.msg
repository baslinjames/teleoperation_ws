# Current and calculated end effector pose of a robot arm

# Robot arm side
uint8 LEFT = 0
uint8 RIGHT = 1

std_msgs/Header header # Mostly for timestamp but can use frame_id if needed

uint8 side # The side of the UGV the robot arm sits on

geometry_msgs/Pose current    # Current pose
geometry_msgs/Pose calculated # Calculated pose
