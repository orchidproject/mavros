#!/usr/bin/env python
"""
    ROS Node for high-level UAV Control.

    Provides high-level UAV Control functionality via the
    following topics and services

    Services
    --------
    In future, we could implement some of these using the action library,
    but don't feel the extra complexity is warranted right now.

    /uav_name/set_mode, mavros/SetMode, switch between manual and control mode

    /uav_name/clear_queue,  clears the waypoint queue
    /uav_name/pause_queue, pauses the waypoint queue
    /uav_name/resume_queue, resumes the waypoint queue if paused
    /uav_name/add_waypoints, adds the specified waypoints to the queue

    /uav_name/land, commands the UAV to land
    /uav_name/takeoff, commands the UAV to takeoff
    /uav_name/emergency, commands the UAV to perform emergency shutdown

    The following services control more complex path plans, and adds
    the resulting waypoints to the queue.
    /uav_name/sweep, tells the uav to perform sweep search
    /uav_name/spiral_out, tells the uav to spiral out
    /uav_name/spiral_in, tells the uav to spiral in

    Publications
    ------------
    /diagnostics - provides status information via the ROS diagnostics package

    Subscriptions
    -------------
    /uav_name/manual_control - when in manual mode, listens for velocity
                               messages to control UAV directly

    The following are used to issue commands to all UAVs at once, but
    not response or acknowledgement is given in response.
    /all_uavs/takeoff - tell all UAVs to take off
    /all_uavs/emergency - tell all UAV to enter emergency mode
    /all_uavs/land - tell all UAVs to land at once


"""
import rospy
import mavros.srv, mavros.msg
from utm import from_latlon, to_latlon
from uav_utils import sweeps
import diagnostic_updater
import diagnostic_msgs


