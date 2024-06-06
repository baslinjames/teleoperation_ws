# Message for the radar data

std_msgs/Header header # Mostly for timestamp but can use frame_id if needed

float64 maximum_value # Maximum value within the range of radar values
float64 minimum_value # Minaximum value within the range of radar values
float64 average_value # Average value of the radar values

float64[] radar_range # Array of ranges