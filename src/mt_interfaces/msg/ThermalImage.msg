# Message contains uncompressed thermal image

std_msgs/Header header # Mostly for timestamp but can use frame_id if needed

uint32 height # Image height. Number of rows
uint32 width  # Image width. Number of columns

uint32 step   # Full row length
uint16[] data # Actual data