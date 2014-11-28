
#******************************************************************************
#   Import mavlink dialect
#******************************************************************************
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)),
                            '../mavlink/pymavlink'))
from mavutil import mavlink


#******************************************************************************
# Adhoc manual mode - apparently used to request manual control of MAVs that
# might not support manual mode
#******************************************************************************
ADHOC_MANUAL = 99

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

def is_emergency_enabled(system_state_id):
    """Returns true of the system is in emergency mode
    """
    if system_state_id == mavlink.MAV_STATE_EMERGENCY:
        return True
    else:
        return False

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

#******************************************************************************
#  Utility functions for testing which modes are enabled
#******************************************************************************
def is_custom_mode_enabled(base_mode,custom_mode):
    """Returns true if MAV is in custom mode

       Parameters
       base_mode - system base mode bit field (as integer)
       custom_mode - custom mode bit field (as integer)
    """
    if base_mode & mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED:
        return True
    else:
        return False

def is_adhoc_manual_mode_enabled(base_mode,custom_mode):
    """Returns true if MAV is in ARProxy defined ADHOC_MANUAL mode.

       This was added, because the generic quadrotor type in mavlink
       does not define a MANUAL mode.

       Parameters
       base_mode - system base mode bit field (as integer)
       custom_mode - custom mode bit field (as integer)
    """
    if is_custom_mode_enabled(base_mode,custom_mode) and \
        (custom_mode == ADHOC_MANUAL):
        return True
    else:
        return False

def is_auto_mode_enabled(base_mode,custom_mode):
    """Returns true if MAV is in auto mode

       Parameters
       base_mode - system base mode bit field (as integer)
       custom_mode - custom mode bit field (as integer)
    """
    if base_mode & mavlink.MAV_MODE_FLAG_AUTO_ENABLED:
        return True
    else:
        return False

def is_guided_mode_enabled(base_mode,custom_mode):
    """Returns true if MAV is in guided mode

       System automatically follows specified waypoints

       Parameters
       base_mode - system base mode bit field (as integer)
       custom_mode - custom mode bit field (as integer)
    """
    if base_mode & mavlink.MAV_MODE_FLAG_GUIDED_ENABLED:
        return True
    else:
        return False

def is_stabilize_mode_enabled(base_mode,custom_mode):
    """Returns true if MAV is in stabilized mode

       System stabilizes electronically its attitude (and optionally
       position). However, it needs further input to move around.

       You'd expect this to be enabled all the time for unstable
       airframes, such as quadrotors.

       Parameters
       base_mode - system base mode bit field (as integer)
       custom_mode - custom mode bit field (as integer)
    """
    if base_mode & mavlink.MAV_MODE_FLAG_STABILIZE_ENABLED:
        return True
    else:
        return False

def is_hil_mode_enabled(base_mode,custom_mode):
    """Returns true if system is running hardware in the loop.

       Motors are blocked, but internal software is running
       hardware in the loop simulation.

       Parameters
       base_mode - system base mode bit field (as integer)
       custom_mode - custom mode bit field (as integer)
    """
    if base_mode & mavlink.MAV_MODE_FLAG_HIL_ENABLED:
        return True
    else:
        return False

def is_manual_mode_enabled(base_mode,custom_mode):
    """Returns true if manual input is enabled

       System will accept remote control input

       Also returns true if system is in alternative "ADHOC_MANUAL"
       custom mode.

       Parameters
       base_mode - system base mode bit field (as integer)
       custom_mode - custom mode bit field (as integer)
    """
    if base_mode & mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED:
        return True
    elif is_adhoc_manual_mode_enabled(base_mode,custom_mode):
        return True
    else:
        return False

def is_safety_mode_enabled(base_mode,custom_mode):
    """Returns true if safety is armed.

       MAV safety set to armed. Motors are enabled / running / can
       start. Ready to fly.

       Parameters
       base_mode - system base mode bit field (as integer)
       custom_mode - custom mode bit field (as integer)
    """
    if base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        return True
    else:
        return False

def is_preflight_mode_enabled(base_mode,custom_mode):
    """Returns true if system is in preflight mode

       System is not ready, no flags are set.

       Parameters
       base_mode - system base mode bit field (as integer)
       custom_mode - custom mode bit field (as integer)
    """
    if base_mode==0:
        return True
    else:
        return False


