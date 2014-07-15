#!/usr/bin/env python

# CUSTOM_MODES contains for all vehicles the corresponding numbers for that
# vehicle specific custom modes as required by MAVLink
# If you require add ones for your own type of commands

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

