#!/usr/bin/env python
import rospy
import mavros.msg
import mavros.srv
import sys


def construct_waypoints_global(n, command_switch):
    if n == 1:
        start = 0
        inc = 1
    elif n == 2:
        start = 3
        inc = -1

    while not rospy.is_shutdown():
        # Starting position should be 50.930042,-1.407951
        lat = [50.9298172, 50.9299545, 50.9299545, 50.9298172]
        lon = [-1.4078220, -1.4078220, -1.4075130, -1.4075130]
        instructions = mavros.msg.InstructionList()

        if command_switch:
            i = mavros.msg.Instruction()
            i.type = mavros.msg.Instruction.TYPE_TAKEOFF
            instructions.inst.append(i)

        i = mavros.msg.Instruction()
        i.type = mavros.msg.Instruction.TYPE_GOTO
        i.frame = mavros.msg.Instruction.FRAME_GLOBAL
        i.latitude = lat[start]
        i.longitude = lon[start]
        i.altitude = 1.5
        instructions.inst.append(i)
        start += inc

        i = mavros.msg.Instruction()
        i.type = mavros.msg.Instruction.TYPE_GOTO
        i.frame = mavros.msg.Instruction.FRAME_GLOBAL
        i.latitude = lat[start]
        i.longitude = lon[start]
        i.altitude = 1.5
        instructions.inst.append(i)
        start += inc

        # if command_switch:
        #     i = mavros.msg.Instruction()
        #     i.type = mavros.msg.Instruction.TYPE_LAND
        #     instructions.inst.append(i)
        #
        #     i = mavros.msg.Instruction()
        #     i.type = mavros.msg.Instruction.TYPE_TAKEOFF
        #     instructions.inst.append(i)

        i = mavros.msg.Instruction()
        i.type = mavros.msg.Instruction.TYPE_GOTO
        i.frame = mavros.msg.Instruction.FRAME_GLOBAL
        i.latitude = lat[start]
        i.longitude = lon[start]
        i.altitude = 1.5
        instructions.inst.append(i)
        start += inc

        i = mavros.msg.Instruction()
        i.type = mavros.msg.Instruction.TYPE_GOTO
        i.frame = mavros.msg.Instruction.FRAME_GLOBAL
        i.latitude = lat[start]
        i.longitude = lon[start]
        i.altitude = 1.5
        instructions.inst.append(i)

        if command_switch:
            i = mavros.msg.Instruction()
            i.type = mavros.msg.Instruction.TYPE_LAND
            instructions.inst.append(i)

        return instructions


def construct_waypoints_local(n, command_switch):
    if n == 1:
        start = 0
        inc = 1
    elif n == 2:
        start = 3
        inc = -1

    lat = [0,  5,  5,  0]
    lon = [5,  5,  0,  0]
    instructions = mavros.msg.InstructionList()

    if command_switch:
        i = mavros.msg.Instruction()
        i.type = mavros.msg.Instruction.TYPE_TAKEOFF
        i.waitTime = 5
        i.frame = mavros.msg.Instruction.FRAME_LOCAL
        instructions.inst.append(i)

    i = mavros.msg.Instruction()
    i.type = mavros.msg.Instruction.TYPE_GOTO
    i.frame = mavros.msg.Instruction.FRAME_LOCAL
    i.latitude = lat[start]
    i.longitude = lon[start]
    i.altitude = 1.5
    instructions.inst.append(i)
    start += inc

    i = mavros.msg.Instruction()
    i.type = mavros.msg.Instruction.TYPE_GOTO
    i.frame = mavros.msg.Instruction.FRAME_LOCAL
    i.latitude = lat[start]
    i.longitude = lon[start]
    i.altitude = 1.5
    instructions.inst.append(i)
    start += inc

    # if command_switch:
    #     i = mavros.msg.Instruction()
    #     i.type = mavros.msg.Instruction.TYPE_LAND
    #     i.waitTime = 5
    #     i.frame = mavros.msg.Instruction.FRAME_LOCAL
    #     instructions.inst.append(i)
    #
    #     i = mavros.msg.Instruction()
    #     i.type = mavros.msg.Instruction.TYPE_TAKEOFF
    #     i.waitTime = 5
    #     i.frame = mavros.msg.Instruction.FRAME_LOCAL
    #     instructions.inst.append(i)

    i = mavros.msg.Instruction()
    i.type = mavros.msg.Instruction.TYPE_GOTO
    i.frame = mavros.msg.Instruction.FRAME_LOCAL
    i.latitude = lat[start]
    i.longitude = lon[start]
    i.altitude = 1.5
    instructions.inst.append(i)
    start += inc

    i = mavros.msg.Instruction()
    i.type = mavros.msg.Instruction.TYPE_GOTO
    i.frame = mavros.msg.Instruction.FRAME_LOCAL
    i.latitude = lat[start]
    i.longitude = lon[start]
    i.altitude = 1.5
    instructions.inst.append(i)

    if command_switch:
        i = mavros.msg.Instruction()
        i.type = mavros.msg.Instruction.TYPE_LAND
        i.waitTime = 5
        i.frame = mavros.msg.Instruction.FRAME_LOCAL
        instructions.inst.append(i)

    return instructions


def construct_spiral_sweep(command_switch):
    instructions = mavros.msg.InstructionList()

    if command_switch:
        i = mavros.msg.Instruction()
        i.type = mavros.msg.Instruction.TYPE_TAKEOFF
        i.waitTime = 5
        instructions.inst.append(i)

    i = mavros.msg.Instruction()
    i.type = mavros.msg.Instruction.TYPE_SPIRAL_SWEEP
    i.frame = mavros.msg.Instruction.FRAME_LOCAL
    i.waitTime = 3
    i.range = 8
    i.latitude = 10
    i.longitude = 10
    i.altitude = 1.5
    instructions.inst.append(i)

    i = mavros.msg.Instruction()
    i.type = mavros.msg.Instruction.TYPE_SPIRAL_SWEEP
    i.frame = mavros.msg.Instruction.FRAME_LOCAL
    i.waitTime = 5
    i.range = 2
    i.latitude = 2
    i.longitude = 2
    i.altitude = 1.5
    instructions.inst.append(i)

    if command_switch:
        i = mavros.msg.Instruction()
        i.type = mavros.msg.Instruction.TYPE_LAND
        i.waitTime = 0
        instructions.inst.append(i)

    return instructions


def construct_rect_sweep(command_switch):
    instructions = mavros.msg.InstructionList()

    if command_switch:
        i = mavros.msg.Instruction()
        i.type = mavros.msg.Instruction.TYPE_TAKEOFF
        i.waitTime = 5
        instructions.inst.append(i)

    i = mavros.msg.Instruction()
    i.type = mavros.msg.Instruction.TYPE_RECT_SWEEP
    i.frame = mavros.msg.Instruction.FRAME_LOCAL
    i.waitTime = 3
    i.range = 8
    i.latitude = 2
    i.longitude = 2
    i.altitude = 2
    instructions.inst.append(i)

    i = mavros.msg.Instruction()
    i.type = mavros.msg.Instruction.TYPE_RECT_SWEEP
    i.frame = mavros.msg.Instruction.FRAME_LOCAL
    i.waitTime = 5
    i.range = 2
    i.latitude = 18
    i.longitude = 18
    i.altitude = 2
    instructions.inst.append(i)

    if command_switch:
        i = mavros.msg.Instruction()
        i.type = mavros.msg.Instruction.TYPE_LAND
        i.waitTime = 0
        instructions.inst.append(i)

    return instructions


# *******************************************************************************
# Parse any arguments that follow the node command
# *******************************************************************************
# from optparse import OptionParser
#
# parser = OptionParser("mosaic_node.py [options]")
# parser.add_option("-n", "--number", dest="n", default=1,
#                   help="Number of drone")
# parser.add_option("-i", "--instructions", action="store_true", dest="instructions", default=False,
#                   help="Using Landing/Takeoff instructions")
# (opts, args) = parser.parse_args()
#
# if __name__ == '__main__':
#     try:
#         rospy.wait_for_service('/mosaic/queue')
#         queue = rospy.ServiceProxy('/mosaic/queue', mavros.srv.Queue)
#         wps = construct_waypoints_global(opts.n, opts.instructions)
#         resp = queue(1, wps)
#         print("Waypoints Sent. Response: " + str(resp.result))
#         rospy.sleep(1)
#         resp = queue(4, [])
#         print("Execute Response: " + str(resp.result))
#
#     except rospy.ROSInterruptException:
#         pass
