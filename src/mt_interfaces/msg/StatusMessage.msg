# Status messages from ROS2 nodes

# Possible status levels
uint8 DEBUG=0
uint8 INFO=1
uint8 WARN=2
uint8 ERROR=3

std_msgs/Header header # Mostly for timestamp but can use frame_id if needed

uint8 status_level # Level of the status
string message_origin # Origin of the message e.g. node name, class name or script name
string message # Status message
