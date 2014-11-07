
#******************************************************************************
#   Import mavlink dialect
#******************************************************************************
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)),
                            '../mavlink/pymavlink'))
from mavutil import mavlink

#******************************************************************************
#   System state mapping extracted from mavlink
#******************************************************************************
SYSTEM_STATES = {
    "UNINIT" : mavlink.MAV_STATE_UNINIT,
    "BOOT" : mavlink.MAV_STATE_BOOT,
    "CALIBRATING" : mavlink.MAV_STATE_CALIBRATING,
    "STANDBY" : mavlink.MAV_STATE_STANDBY,
    "ACTIVE" : mavlink.MAV_STATE_ACTIVE,
    "CRITICAL" : mavlink.MAV_STATE_CRITICAL,
    "EMERGENCY" : mavlink.MAV_STATE_EMERGENCY,
    "POWEROFF" : mavlink.MAV_STATE_POWEROFF
}

SYSTEM_STATE_NAME = dict( zip(SYSTEM_STATES.values(), SYSTEM_STATES.keys()) )

def get_system_status_name(system_state_id):
    """Returns meaningful name for system state"""
    if system_state_id in SYSTEM_STATE_NAME:
       return SYSTEM_STATE_NAME[system_state_id]
    else:
       return "Unknown state: %d" % system_state_id

#******************************************************************************
#   Base mode mapping extracted from mavlink 
#******************************************************************************
BASE_MODE_VALUE = {
    "PREFLIGHT" : mavlink.MAV_MODE_PREFLIGHT,
    "MANUAL_DISARMED" : mavlink.MAV_MODE_MANUAL_DISARMED,
    "TEST_DISARMED" : mavlink.MAV_MODE_TEST_DISARMED,
    "STABILIZE_DISARMED" : mavlink.MAV_MODE_STABILIZE_DISARMED,
    "GUIDED_DISARMED" : mavlink.MAV_MODE_GUIDED_DISARMED,
    "AUTO_DISARMED" : mavlink.MAV_MODE_AUTO_DISARMED,
    "MANUAL_ARMED" : mavlink.MAV_MODE_MANUAL_ARMED,
    "TEST_ARMED" : mavlink.MAV_MODE_TEST_ARMED,
    "STABILIZE_ARMED" : mavlink.MAV_MODE_STABILIZE_ARMED,
    "GUIDED_ARMED" : mavlink.MAV_MODE_GUIDED_ARMED,
    "AUTO_ARMED" : mavlink.MAV_MODE_AUTO_ARMED
}

BASE_MODE_NAME = dict( zip(BASE_MODE_VALUE.values(), BASE_MODE_VALUE.keys()) )

def get_base_mode_name(mode_id):
    """Returns meaningful name for system base mode"""
    if mode_id in BASE_MODE_NAME:
       return BASE_MODE_NAME[mode_id]
    else:
       return "Unknown mode: %d" % mode_id
        


# not sure what this is used for if anything
BASE_MODES = {
    0: 0,
    1: 80,
    2: 208,
    3: 64,
    4: 192,
    5: 88,
    6: 216,
    7: 92,
    8: 220,
    9: 66,
    10: 194
}

#******************************************************************************
# CUSTOM_MODES contains for all vehicles the corresponding numbers for that
# vehicle specific custom modes as required by MAVLink
# If you require add ones for your own type of commands
#******************************************************************************
CUSTOM_MODES = {
    "ArduCopter": {
        "STABILIZE": 0,  # hold level position
        "ACRO": 1,  # rate control
        "ALT_HOLD": 2,  # AUTO control
        "AUTO": 3,  # AUTO control
        "GUIDED": 4,  # AUTO control
        "LOITER": 5,  # Hold a single location
        "RTL": 6,  # AUTO control
        "CIRCLE": 7,  # AUTO control
        "POSITION": 8,  # AUTO control,
        "LAND": 9,  # AUTO control
        "OF_LOITER": 10,  # Hold a single location using optical flow
        "DRIFT": 11,
        "SPORT": 13,
        "HYBRID": 16
    },
    "ArduRover": {
        "MANUAL": 0,
        "LEARNING": 2,
        "STEERING": 3,
        "HOLD": 4,
        "AUTO": 10,
        "RTL": 11,
        "GUIDED": 15,
        "INITIALISING": 16
    },
    "ARDrone": {
        "STABILIZE": 0,  # hold level position
        "ACRO": 1,  # rate control
        "ALT_HOLD": 2,  # AUTO control
        "AUTO": 3,  # AUTO control
        "GUIDED": 4,  # AUTO control
        "LOITER": 5,  # Hold a single location
        "RTL": 6,  # AUTO control
        "CIRCLE": 7,  # AUTO control
        "POSITION": 8,  # AUTO control,
        "LAND": 9,  # AUTO control
        "OF_LOITER": 10,  # Hold a single location using optical flow
        "APPROACH": 11,
        "MANUAL": 12
    }}

def get_custom_mode_name(system,custom_mode):
    """Returns meaningful name for system custom mode"""

    return "%d" % custom_mode
