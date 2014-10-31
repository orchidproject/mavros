# Message type used for communicating errors that may have occurred
# In particular, this is used as part of return time for services defined
# in the mavros package

uint8 code  # type of error from enumeration below

uint8 SUCCESS = 0    # no error occurred
uint8 FAILURE = 1    # generic failure occurred
uint8 UNSUPPORTED_MODE = 2     # requested MAV mode is unsupported 
uint8 UNSUPPORTED_FRAME = 3    # requested coordinate frame is unsupported
uint8 COORDS_OUT_OF_RANGE = 4  # specified coordinates are out of range
uint8 MAV_TIMEOUT = 5          # attempt to communicate with MAV timed out
uint8 PARAM_NOT_SET = 6        # requested parameter is undefined
uint8 BAD_PARAM_VALUE = 7      # requested parameter has wrong value
uint8 KEY_VALUE_COUNT_MISMATCH = 8  # number of keys and values don't match
uint8 MAV_COMMAND_ERROR = 9  # MAV reported an error in command acknowledgement
uint8 UNSUPPORTED_COMMAND = 10 # attempt to execute unsupported command
uint8 UNDEFINED_COMMAND = 11   # attempt to execute undefined command
uint8 INTERNAL = 12            # Internal error (indicates a runtime bug)
uint8 UNDEFINED_WAYPOINT = 13  # Attempt to set mission to undefined waypoint
uint8 WAYPOINT_VERIFICATION_FAILURE = 14  # Failed to verify waypoints on drone
uint8 NO_GPS_FIX = 15  # Can't complete operation because there is no GPS fix
