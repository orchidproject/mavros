#!/usr/bin/env python
"""
    ROS Node for high-level UAV Control.

    Provides high-level UAV Control functionality via the
    following topics and services

    Services
    --------
    In future, we could implement some of these using the action library,
    but don't feel the extra complexity is warranted right now.

    /uav_name/control/set_mode, mavros/SetMode, switch between emergency, auto
        and manual control.

    /uav_name/control/set_origin_here, tells the uav to set the SHARED origin
        for all uavs at its own current position

    /uav_name/control/clear_queue,  clears the waypoint queue
    /uav_name/control/pause_queue, pauses the waypoint queue
    /uav_name/control/resume_queue, resumes the waypoint queue if paused
    /uav_name/control/add_waypoints, adds the specified waypoints to the queue

    /uav_name/control/land, commands the UAV to land
    /uav_name/control/takeoff, commands the UAV to takeoff

    The following services control more complex path plans, and adds
    the resulting waypoints to the queue.
    /uav_name/control/add_sweep, tells the uav to perform sweep search
    /uav_name/control/add_spiral_out, tells the uav to spiral out
    /uav_name/control/add_spiral_in, tells the uav to spiral in

    Publications
    ------------
    /diagnostics - provides status information via the ROS diagnostics package

    Subscriptions
    -------------
    /uav_name/control/manual_control - when in manual mode, listens for velocity
        messages to control UAV directly

    The following are used to issue commands to all UAVs at once, but
    not response or acknowledgement is given in response.
    /all/control/takeoff - tell all UAVs to take off
    /all/control/emergency - tell all UAV to enter emergency mode
    /all/control/land - tell all UAVs to land at once
    /all/control/set_origin - tell all UAVs to set specified GPS waypoint as
        origin for local coordinate frame.
"""
from Queue import Queue
import rospy
import mavros.srv as srv
import mavros.msg as msg
from utm import from_latlon, to_latlon
from uav_utils import sweeps
import diagnostic_updater
import diagnostic_msgs
from std_msgs.msg import Empty as EmptyMsg

#*******************************************************************************
#   Constants
#*******************************************************************************

# prefix for things subscribed to by all UAV controllers
MULTI_UAV_CONTROL_PREFIX = "/all/control/"

#*******************************************************************************
#   Classes
#*******************************************************************************
class Controller:
    """Main class --- implements high level control for drones
    """

    def __init__(self,uav_name):
        """Constructs a new Controller

           Parameters
           uav_name - name of UAV this object will control
        """

        # Name of UAV we will control
        self.uav_name = uav_name 

        # ROS namespace prefix for services provided by driver
        self.driver_prefix = '/' + uav_name + '/' 

        # ROS namespace prefix for services provided by this controller
        self.control_prefix = '/' + uav_name + '/control/' 

        # prefix used by this controller for logging messages
        self.log_prefix = "[CONTROL %s] " % uav_name

        # current mode we believe the drone is in
        self.uav_mode = srv.SetMode.UNKNOWN

        # queue of waypoints still to be executed
        self.queue = Queue()

        # if true, waypoints are not being executed and uav_mode is AUTO then
        # the drone is put in a hold pattern, until this becomes false
        self.queue_is_paused = True

        # 

    def __logerr(self,msg):
        """Used for logging error messages
           
           Parameters
           msg - string to log
        """
        rospy.logerr(self.log_prefix + msg)

    def __logwarn(self,msg):
        """Used for logging warning messages
           
           Parameters
           msg - string to log
        """
        rospy.logwarn(self.log_prefix + msg)

    def __loginfo(self,msg):
        """Used for logging information messages
           
           Parameters
           msg - string to log
        """
        rospy.loginfo(self.log_prefix + msg)

    def __logdebug(self,msg):
        """Used for logging debug messages
           
           Parameters
           msg - string to log
        """
        rospy.logdebug(self.log_prefix + msg)

    def __get_global_waypoint(self,wp):
        """Puts waypoints into global coordinate frame

           Local frame waypoints are in UTM offset by local origin.

           Parameter:
           wp - a mavros/Waypoint message in either the local or global
               coordinate frame.

           Returns the equivalent waypoint in the global frame (i.e. no change
               if already in the correct frame)
        """
        pass

    def __set_waypoints_from_queue(self):
        """Syncs the waypoints on the drone with the currently queued waypoints
        """

        #**********************************************************************
        # If any waypoints are in the local coordinate frame, convert them
        # to the global coordinate frame now
        #**********************************************************************
        # TODO - USE METHOD ABOVE

    def __ros_init(self):
        """Initialises ROS services, publications and subscriptions"""

    #***************************************************************************
    #   Initialise proxy functions for remote services we call.
    #   These are all provided by the mavros driver node 
    #***************************************************************************
    self.mav_command_srv = rospy.ServiceProxy(self.prefix + "mav_command",
            srv.MAVCommand)

    self.set_waypoints_srv = rospy.ServiceProxy(self.prefix + "set_waypoints",
            srv.SetWaypoints)

    self.get_waypoints_srv = rospy.ServiceProxy(self.prefix + "get_waypoints",
            srv.GetWaypoints)

    self.set_params_srv = rospy.ServiceProxy(self.prefix + "set_params",
            srv.SetParameters)

    self.get_params_srv = rospy.ServiceProxy(self.prefix + "get_params",
            srv.GetParameters)

    self.set_mission_srv = rospy.ServiceProxy(self.prefix + "set_mission",
            srv.SetMission)

    #***************************************************************************
    #   Initialise ROS services we provide
    #***************************************************************************
    rospy.Service(self.control_prefix + "set_mode", srv.SetMode,
            self.set_mode_cb)

    rospy.Service(self.control_prefix + "set_origin_here", srv.SimpleCommand,
            self.set_origin_here_cb)

    rospy.Service(self.control_prefix + "clear_queue", srv.SimpleCommand,
            self.clear_queue_cb)

    rospy.Service(self.control_prefix + "pause_queue", srv.SimpleCommand,
            self.pause_queue_cb)

    rospy.Service(self.control_prefix + "resume_queue", srv.SimpleCommand,
            self.resume_queue_cb)

    rospy.Service(self.control_prefix + "land", srv.SimpleCommand,
            self.land_cb)

    rospy.Service(self.control_prefix + "takeoff", srv.SimpleCommand,
            self.takeoff_cb)

    rospy.Service(self.control_prefix + "add_waypoints", srv.AddWaypoints,
            self.add_waypoints_cb)

    rospy.Service(self.control_prefix + "add_sweep", srv.AddSweep,
            self.add_sweep_cb)

    rospy.Service(self.control_prefix + "add_spiral_out", srv.AddSpiral,
            self.add_spiral_out_cb)

    rospy.Service(self.control_prefix + "add_spiral_in", srv.AddSpiral,
            self.add_spiral_in_cb)

    #***************************************************************************
    #   Register call back functions to for topics we subscribe to
    #***************************************************************************
    rospy.Subscriber(self.control_prefix + "manual_control", msg.Velocity,
        self.manual_control_cb)

    rospy.Subscriber(MULTI_UAV_CONTROL_PREFIX + "takeoff", std_msgs.Empty,
            self.takeoff_cb)

    rospy.Subscriber(MULTI_UAV_CONTROL_PREFIX + "land", std_msgs.Empty,
            self.land_cb)

    rospy.Subscriber(MULTI_UAV_CONTROL_PREFIX + "emergency", std_msgs.Empty,
            self.emergency_cb)

    rospy.Subscriber(MULTI_UAV_CONTROL_PREFIX + "set_origin", msg.Waypoint,
            self.set_origin_cb)

    #***************************************************************************
    #   Advertise messages that we publish
    #***************************************************************************
    # None right now -- apart from diagnostics
    # In future, might want to publish something about the state of the queue?

    def set_mode_cb(self,req):
        """Callback for mavros/SetMode service"""
        pass

    def set_origin_here_cb(self,req=None):
        """Tells the UAV to set the origin for ALL UAVs.

           The origin for all UAVs will be set to the current location of this
           UAV.
        """
        pass

    def set_origin_cb(self,origin_wp):
        """Accepts a new origin for the local coordinate frame for this UAV"""
        pass

    def clear_queue_cb(self,req=None):
        """Callback for clearing the queue

           Parameters
           req - is an empty service request (no input required)

           Returns
           mavros/Error message indicating success or failure
        """
        pass

    def pause_queue_cb(self,req=None):
        """Callback for pausing execution of the queue

           Parameters
           req - is an empty service request (no input required)

           Returns
           mavros/Error message indicating success or failure
        """
        pass

    def resume_queue_cb(self,req=None):
        """Callback for resuming execution of the queue

           Parameters
           req - is an empty service request (no input required)

           Returns
           mavros/Error message indicating if the command was sent successfully
        """
        pass

    def land_cb(self,req=None):
        """Callback for landing the drone

           Parameters
           req - is an empty service request (no input required)

           Returns
           mavros/Error message indicating if the command was sent successfully
        """
        pass

    def takeoff_cb(self,req=None):
        """Callback for telling the drone to take-off

           Parameters
           req - is an empty service request (no input required)

           Returns
           mavros/Error message indicating if the command was sent successfully
        """
        pass

    def emergency_cb(self,req=None):
        """Callback for telling the drone to enter emergency mode

           Parameters
           req - is an empty service request (no input required)

           Returns
           mavros/Error message indicating if the command was sent successfully
        """
        pass

    def add_waypoints_cb(self,req):
        """Callback for adding a set of waypoints to the queue
           Implements mavros/AddWaypoints ROS service.
           See service definition for details
        """
        pass

    def add_sweep_cb(self,req):
        """Callback for adding a set of waypoints to the queue
           Implements mavros/AddSweep ROS service.
           See service definition for details
        """
        pass

    def add_spiral_out_cb(self,req):
        """Callback for adding waypoints to spiral out"""
        pass

    def add_spiral_in_cb(self,req):
        """Callback for adding waypoints to spiral in"""
        pass

    def manual_control_cb(self,vel):
        """Callback for directly controlling drones velocity

           Parameters
           vel - mavros/Velocity.msg specifying velocity vector
        """
        pass

#*******************************************************************************
#   Parse any arguments that follow the node command
#*******************************************************************************
from optparse import OptionParser

parser = OptionParser("mosaic_node.py [options]")
parser.add_option("-n", "--name", dest="name", default="parrot",
    help="Name of the prefix for the mavros node")
parser.add_option("-r", "--ros", action="store_true", dest="ros",
    help="Use ROS parameter server", default=False)
(opts, args) = parser.parse_args()

#*******************************************************************************
#   If the named UAV is active then run the node until we're interrupted
#   Otherwise, bail out now --- we're not needed.
#*******************************************************************************
if __name__ == '__main__':
    try:
        if not opts.ros or opts.name in rospy.get_param("/drones_active"):
            rospy.init_node("mavros_controller")
            rospy.loginfo("[CONTROL %s] waiting for driver services" %
                    opts.name)
            rospy.wait_for_service("/" + opts.name + "/mav_command")
            rospy.wait_for_service("/" + opts.name + "/set_waypoints")
            rospy.wait_for_service("/" + opts.name + "/get_waypoints")
            rospy.wait_for_service("/" + opts.name + "/set_params")
            rospy.wait_for_service("/" + opts.name + "/get_params")
            rospy.wait_for_service("/" + opts.name + "/set_mission")
            rospy.loginfo("[CONTROL %s] initialising high-level control" %
                    opts.name)
            node = Controller(opts.name)
            node.start()
    except rospy.ROSInterruptException:
        pass

