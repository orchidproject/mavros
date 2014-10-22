# if autocontinue is 1, MAV should continue to next waypoint
# otherwise it should be set to zero, to prevent auto continuation
# to next waypoint
uint8 autocontinue

# Range (in metres) that MAV must get within waypoint to consider
# it reached. Required because reaching waypoint exactly is 
# impossible
float32 radius 

# wait time to hold at waypoint before continuing
duration waitTime

# Coordinate frame in which x,y,z coordinates are specified
# Valid values are defined below
uint8 frame

# Valid values for coordinate frame are:
uint8 FRAME_INVALID = 0    # used for comparision only

uint8 FRAME_GLOBAL  = 1    # WGS84 system with relative altitude
                           #    x: latitude,
                           #    y: longitude,
                           #    z: relative altitude above ground

uint8 FRAME_LOCAL   = 2    # local ENU (east/north/up) coordinates
                           #    x: east
                           #    y: north
                           #    z: up

# Waypoint coordinates specified in coordinate frame 
float32 x
float32 y
float32 z
