uint8 mode   # control mode from enumeration below

uint8 UNKNOWN   = 0 # used to indicate that the current control mode is unknown
uint8 AUTO      = 1 # set control mode to AUTO (for waypoint following)
uint8 MANUAL    = 2 # set control mode for direct control of velocity
uint8 EMERGENCY = 3 # put UAV in emergency mode
