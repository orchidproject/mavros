uint8 waypoint_type
uint8 autocontinue

float32 latitude
float32 longitude
float32 altitude
# parameters as speisifed by MAVLink
float32[4] params


uint8 TYPE_NAV = 1
uint8 TYPE_TAKEOFF = 2
uint8 TYPE_LAND = 3
uint8[4] DEFAULT = [0, 0, 0, 0]
