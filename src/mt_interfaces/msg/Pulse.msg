# Message for the data from the piezo pulse sensor package
# Data types reflect what comes from the microcontroller

std_msgs/Header header # Mostly for timestamp but can use frame_id if needed

uint32 time_elapsed                     # Time elapsed data field
float32 analog_input                    # Analog value from the piezo sensor
sensor_msgs/BatteryState battery_state  # Battery data
sensor_msgs/Imu imu                     # IMU data
