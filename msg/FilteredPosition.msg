# Based on MAVLink 1.0 Messages: GLOBAL_POSITION_INT
# See common.xml in MAVLink Messages

Header header

# Latitude and Longitude, Altitude and Relative Altitude
float64 latitude
float64 longitude
float64 altitude
float64 relative_altitude

# Ground Speed along X, Y, and Z expressed as m/s * 100
int16 ground_x_speed
int16 ground_y_speed
int16 ground_z_speed

# Heading in degrees * 100 (set to 65536 if false)
uint16 heading
