#!/usr/bin/python

from Tkinter import *
import tkFont
import rospy
from std_msgs.msg import Empty as EmptyMsg
import mavros.msg
import mavros.srv
import queue_node as q
from waypoint_tester import *
import uav_utils.sweeps as sweep
from tools import *

# prefix for things subscribed to by all UAV controllers
MULTI_UAV_CONTROL_PREFIX = "/all/control/"

class NavGUI:
    def __init__(self, uav_name, command_switch=False):
        """Constructs a new NavGUI"""
        self.command_switch = command_switch
        self.root = Tk()
        self.root.title("AR Drone: " + uav_name)
        self.font = tkFont.Font(family="Helvetica", size=15)
        self.control_prefix = '/' + uav_name + '/control/'
        self.log_prefix = "[GUI %s] " % uav_name

        # Lets assume current camera is front for toggling
        # Worst, case we'll have to hit the button twice
        self.current_camera = mavros.srv.SelectCameraRequest.FRONT

    def start(self):
        """Set up GUI components and ROS services"""

        #**********************************************************************
        #   Wait for mavros controller services to intialise
        #**********************************************************************
        self.__loginfo("Waiting for controller services")
        rospy.wait_for_service(self.control_prefix + "select_camera")
        rospy.wait_for_service(self.control_prefix + "set_mode")
        rospy.wait_for_service(self.control_prefix + "set_origin_here")
        rospy.wait_for_service(self.control_prefix + "clear_queue")
        rospy.wait_for_service(self.control_prefix + "pause_queue")
        rospy.wait_for_service(self.control_prefix + "resume_queue")
        rospy.wait_for_service(self.control_prefix + "add_waypoints")
        rospy.wait_for_service(self.control_prefix + "land")
        rospy.wait_for_service(self.control_prefix + "takeoff")
        rospy.wait_for_service(self.control_prefix + "add_sweep")
        rospy.wait_for_service(self.control_prefix + "add_spiral_out")
        rospy.wait_for_service(self.control_prefix + "add_spiral_in")

        #**********************************************************************
        #   Setup proxies for mavros controller services
        #**********************************************************************
        self.select_camera_srv = rospy.ServiceProxy(self.control_prefix +
            "select_camera", mavros.srv.SelectCamera)

        self.set_mode_srv = rospy.ServiceProxy(self.control_prefix +
            "set_mode", mavros.srv.SetMode)

        self.set_origin_here_srv = rospy.ServiceProxy(self.control_prefix +
            "set_origin_here", mavros.srv.SimpleCommand)

        self.clear_queue_srv = rospy.ServiceProxy(self.control_prefix +
            "clear_queue", mavros.srv.SimpleCommand)

        self.pause_queue_srv = rospy.ServiceProxy(self.control_prefix +
            "pause_queue", mavros.srv.SimpleCommand)

        self.resume_queue_srv = rospy.ServiceProxy(self.control_prefix +
            "resume_queue", mavros.srv.SimpleCommand)

        self.add_waypoints_srv = rospy.ServiceProxy(self.control_prefix +
            "add_waypoints", mavros.srv.AddWaypoints)

        self.land_srv = rospy.ServiceProxy(self.control_prefix +
            "land", mavros.srv.SimpleCommand)

        self.takeoff_srv = rospy.ServiceProxy(self.control_prefix +
            "takeoff", mavros.srv.SimpleCommand)

        self.add_sweep_srv = rospy.ServiceProxy(self.control_prefix +
            "add_sweep", mavros.srv.AddSweep)

        self.add_spiral_out_srv = rospy.ServiceProxy(self.control_prefix +
            "add_spiral_out", mavros.srv.AddSweep)

        self.add_spiral_in_srv = rospy.ServiceProxy(self.control_prefix +
            "add_spiral_in", mavros.srv.AddSweep)

        self.__loginfo("Controller services activated")

        #**********************************************************************
        #   Advertise ROS topics we publish to
        #**********************************************************************
        self.pub_vel = rospy.Publisher(self.control_prefix + "manual_control",
            mavros.msg.Velocity, queue_size=1000)

        self.pub_takeoff_all = rospy.Publisher(MULTI_UAV_CONTROL_PREFIX +
            "takeoff", EmptyMsg, queue_size=10)

        self.pub_land_all = rospy.Publisher(MULTI_UAV_CONTROL_PREFIX +
            "land", EmptyMsg, queue_size=10)

        self.pub_emergency_all = rospy.Publisher(MULTI_UAV_CONTROL_PREFIX +
            "emergency", EmptyMsg, queue_size=10)

        #**********************************************************************
        #  Setup and start executing GUI
        #**********************************************************************
        self.setup_gui()
        self.setup_keyboard()
        self.setup_help()
        self.root.mainloop()

    def setup_gui(self):
        main_frame = Frame(self.root)
        control_frame = Frame(main_frame)
        frame1 = Frame(main_frame)
        frame2 = Frame(main_frame)
        frame3 = Frame(main_frame)
        frame4 = Frame(main_frame)
        frame5 = Frame(main_frame)

        info1 = Label(control_frame, text="Roll/Pitch", font=self.font)
        info1.grid(row=1, column=1)
        pitch_forward = Button(control_frame, text="/\\", command=lambda: self.pub_vel.publish([0, -1, 0, 0]),
                               font=self.font)
        pitch_forward.grid(row=0, column=1)
        pitch_back = Button(control_frame, text="\\/", command=lambda: self.pub_vel.publish([0, 1, 0, 0]),
                            font=self.font)
        pitch_back.grid(row=2, column=1)
        roll_left = Button(control_frame, text="<", command=lambda: self.pub_vel.publish([-1, 0, 0, 0]), font=self.font)
        roll_left.grid(row=1, column=0)
        roll_right = Button(control_frame, text=">", command=lambda: self.pub_vel.publish([1, 0, 0, 0]), font=self.font)
        roll_right.grid(row=1, column=2)

        info2 = Label(control_frame, text="Yaw/Alt", font=self.font)
        info2.grid(row=1, column=5)
        alt_up = Button(control_frame, text="/\\", command=lambda: self.pub_vel.publish([0, 0, 1, 0]), font=self.font)
        alt_up.grid(row=0, column=5)
        alt_down = Button(control_frame, text="\\/", command=lambda: self.pub_vel.publish([0, 0, -1, 0]), font=self.font)
        alt_down.grid(row=2, column=5)
        yaw_left = Button(control_frame, text="<", command=lambda: self.pub_vel.publish([0, 0, 0, -1]), font=self.font)
        yaw_left.grid(row=1, column=4)
        yaw_right = Button(control_frame, text=">", command=lambda: self.pub_vel.publish([0, 0, 0, 1]), font=self.font)
        yaw_right.grid(row=1, column=6)

        manual = Button(frame1, text="Manual", command=lambda: self.__set_manual_mode(), font=self.font)
        manual.pack(side=LEFT)
        auto = Button(frame1, text="Auto", command=lambda: self.__set_auto_mode(), font=self.font)
        auto.pack(side=RIGHT)

        takeoff = Button(frame2, text="Takeoff", command=lambda: self.__takeoff(), font=self.font)
        takeoff.pack(side=LEFT)
        camera = Button(frame2, text="Camera", command=lambda: self.__toggle_camera(), font=self.font)
        camera.pack(side=LEFT)
        land = Button(frame2, text="Land", command=lambda: self.__land(), font=self.font)
        land.pack(side=RIGHT)

        spiral = Button(frame3, text="Spiral Sweep", command=lambda: self.__add_spiral_in(),
                        font=self.font)
        spiral.pack(side=LEFT)
        rect = Button(frame3, text="Rectangular Sweep", command=lambda: self.__add_sweep(),
                        font=self.font)
        rect.pack(side=RIGHT)

        send = Button(frame4, text="Send LOCAL Waypoints",
                      command=lambda: self.__add_waypoints(), font=self.font)
        send.pack(side=LEFT)
        clear = Button(frame4, text="Clear", command=lambda: self.__clear_queue(), font=self.font)
        clear.pack(side=RIGHT)

        run = Button(frame5, text="Execute", command=lambda: self.__resume_queue(), font=self.font)
        run.pack(side=LEFT)
        pause = Button(frame5, text="Pause", command=lambda: self.__pause_queue(), font=self.font)
        pause.pack(side=LEFT)
        origin = Button(frame5, text="Set Origin", command=lambda: self.__set_origin_here(), font=self.font)
        origin.pack(side=RIGHT)

        takeoff_all = Button(main_frame, text="Takeoff all", command=lambda: self.__takeoff_all(), font=self.font)
        takeoff_all.pack(side=BOTTOM)

        control_frame.pack()
        frame1.pack(side=TOP)
        frame2.pack(side=TOP)
        frame5.pack(side=BOTTOM)
        frame4.pack(side=BOTTOM)
        frame3.pack(side=BOTTOM)
        main_frame.grid(row=0, column=0, rowspan=3, sticky="nesw")

        red_button = Button(self.root, text="LAND ALL", command=lambda: self.__land_all(),
                            font=self.font, height=2, width=10, bg="red")
        red_button.grid(row=0, column=2, sticky="nesw")
        red_button_all = Button(self.root, text="KILL ALL", command=lambda: self.__kill_all(),
                                font=self.font, height=2, width=10, bg="red")
        red_button_all.grid(row=2, column=2, sticky="nesw")

    def setup_keyboard(self):

        # pitch
        self.root.bind("<w>", lambda (event): self.pub_vel.publish([0, -1, 0, 0]))
        self.root.bind("<s>", lambda (event): self.pub_vel.publish([0, 1, 0, 0]))

        # roll
        self.root.bind("<a>", lambda (event): self.pub_vel.publish([-1, 0, 0, 0]))
        self.root.bind("<d>", lambda (event): self.pub_vel.publish([1, 0, 0, 0]))

        # up/down
        self.root.bind("<Up>", lambda (event): self.pub_vel.publish([0, 0, 1, 0]))
        self.root.bind("<Down>", lambda (event): self.pub_vel.publish([0, 0, -1, 0]))

        # yaw
        self.root.bind("<Left>", lambda (event): self.pub_vel.publish([0, 0, 0, -1]))
        self.root.bind("<Right>", lambda (event): self.pub_vel.publish([0, 0, 0, 1]))

        self.root.bind("<q>", lambda (event): self.__set_manual_mode() )
        self.root.bind("<e>", lambda (event): self.__set_auto_mode() )
        self.root.bind("<r>", lambda (event): self.__takeoff() )
        self.root.bind("<f>", lambda (event): self.__land() )
        self.root.bind("<c>", lambda (event): self.__clear_queue() )
        self.root.bind("<v>", lambda (event): self.__add_waypoints() )
        self.root.bind("<k>", lambda (event): self.__resume_queue() )
        self.root.bind("<l>", lambda (event): self.__pause_queue() )
        self.root.bind("<j>", lambda (event): self.__toggle_camera() )
        self.root.bind("<space>", lambda (event): self.__land_all() )
        self.root.bind("<F11>", lambda (event): self.__takeoff_all() )
        self.root.bind("<F12>", lambda (event): self.__kill_all())

    def setup_help(self):
        frame = Frame(self.root)
        label1 = Label(frame, text="Key Bindings:", font=self.font)
        label1.grid(row=0)
        label2 = Label(frame, text="Roll/Pitch - A/S/D/W", font=self.font)
        label2.grid(row=1)
        label3 = Label(frame, text="Yaw/Alt - Arrow Keys", font=self.font)
        label3.grid(row=2)
        label4 = Label(frame, text="Manual - Q", font=self.font)
        label4.grid(row=3)
        label5 = Label(frame, text="Auto - E", font=self.font)
        label5.grid(row=4)
        label6 = Label(frame, text="Takeoff - R", font=self.font)
        label6.grid(row=5)
        label7 = Label(frame, text="Land - F", font=self.font)
        label7.grid(row=6)
        label8 = Label(frame, text="Toggle Camera - J", font=self.font)
        label8.grid(row=7)
        label9 = Label(frame, text="Clear Waypoints - C", font=self.font)
        label9.grid(row=8)
        label10 = Label(frame, text="Send LOCAL waypoints - V", font=self.font)
        label10.grid(row=9)
        label11 = Label(frame, text="Execute - K", font=self.font)
        label11.grid(row=10)
        label12 = Label(frame, text="Pause - L", font=self.font)
        label12.grid(row=11)
        label13 = Label(frame, text="Land All - Space", font=self.font)
        label13.grid(row=12)
        label14 = Label(frame, text="TAKEOFF ALL - F11", font=self.font)
        label14.grid(row=13)
        label14 = Label(frame, text="!!!KILL ALL!!! - F12", font=self.font)
        label14.grid(row=14)
        frame.grid(row=0, column=1, rowspan=3, sticky="nesw")

    #**************************************************************************
    #   Functions for controlling multiple drones
    #**************************************************************************
    def __takeoff_all(self):
        self.pub_takeoff_all.publish( EmptyMsg() )
        self.__loginfo("TAKEOFF ALL")

    def __kill_all(self):
        self.pub_emergency_all.publish( EmptyMsg() )
        self.__loginfo("KILL ALL")

    def __land_all(self):
        self.pub_land_all.publish( EmptyMsg() )
        self.__loginfo("LAND ALL")

    #**************************************************************************
    #   Wrappers for safely calling ROS services
    #**************************************************************************
    def __toggle_camera(self):
        """Toggles the current active camera"""

        #**********************************************************************
        #   Decide which camera we want to make active - i.e. the currently
        #   inactive one
        #**********************************************************************
        if mavros.srv.SelectCameraRequest.FRONT == self.current_camera:
            target_camera = mavros.srv.SelectCameraRequest.BOTTOM
        else:
            target_camera = mavros.srv.SelectCameraRequest.FRONT

        #**********************************************************************
        #   Ask the controller to change camera to the one we want
        #**********************************************************************
        try:
            response = self.select_camera_srv(target_camera)
        except rospy.ServiceException as e:
            self.__logerr("Exception occurred while trying to toggle "
                " camera: %s" % e)
            return

        if SUCCESS_ERR == response.status:
            self.__loginfo("Camera toggled")
        else:
            self.__logerr("Failed to toggle camera with error code: %d" %
                    response.status)

        #**********************************************************************
        #   If successful, note which camera is active now
        #**********************************************************************
        self.current_camera = target_camera

    def __set_manual_mode(self):
        """Set the drone to MANUAL mode"""
        status = self.__set_mode(mavros.srv.SetModeRequest.MANUAL)
        if SUCCESS_ERR != status:
            self.__logerr("Failed to enter MANUAL mode with error code %d" %
                    status)
        else:
            self.__loginfo("Now in MANUAL mode")

    def __set_auto_mode(self):
        """Set the drone to AUTO mode"""
        status = self.__set_mode(mavros.srv.SetModeRequest.AUTO)
        if SUCCESS_ERR != status:
            self.__logerr("Failed to enter AUTO mode with error code %d" %
                    status)
        else:
            self.__loginfo("Now in AUTO mode")

    def __set_emergency_mode(self):
        """Set the drone to EMERGENCY mode"""
        status = self.__set_mode(mavros.srv.SetModeRequest.EMERGENCY)
        if SUCCESS_ERR != status:
            self.__logerr("Failed to enter EMERGENCY mode with error code %d" %
                    status)
        else:
            self.__loginfo("Now in EMERGENCY mode")

    def __set_mode(self,mode):
        """Put drone in specified mode"""
        try:
            response = self.set_mode_srv(mode)
        except rospy.ServiceException as e:
            self.__logerr("Exception caught while trying to set mode: %s" % e)
            return SERVICE_CALL_FAILED_ERR
        return response.status

    def __set_origin_here(self):
        try:
            response = self.set_origin_here_srv()
        except rospy.ServiceException as e:
            self.__logerr("Exception caught while trying to set origin: %s" % e)
            return

        if SUCCESS_ERR != response.status:
            self.__logerr("Error code %d returned while setting origin" %
                    response.status)
        else:
            self.__loginfo("Origin set to this drone's position.")

    def __clear_queue(self):
        try:
            response = self.clear_queue_srv()
        except rospy.ServiceException as e:
            self.__logerr("Exception caught while trying to clear queue: %s" % e)
            return

        if SUCCESS_ERR != response.status:
            self.__logerr("Error code %d returned while clearing queue" %
                    response.status)
        else:
            self.__loginfo("Queue cleared.")

    def __pause_queue(self):
        try:
            response = self.pause_queue_srv()
        except rospy.ServiceException as e:
            self.__logerr("Exception caught while trying to pause queue: %s" % e)
            return

        if SUCCESS_ERR != response.status:
            self.__logerr("Error code %d returned while pausing queue" %
                    response.status)
        else:
            self.__loginfo("Queue paused.")

    def __resume_queue(self):
        try:
            response = self.resume_queue_srv()
        except rospy.ServiceException as e:
            self.__logerr("Exception caught while trying to resume queue: %s" % e)
            return

        if SUCCESS_ERR != response.status:
            self.__logerr("Error code %d returned while resume queue" %
                    response.status)
        else:
            self.__loginfo("Queue resumed.")

    def __land(self):
        try:
            response = self.land_srv()
        except rospy.ServiceException as e:
            self.__logerr("Exception caught while trying to land: %s" % e)
            return

        if SUCCESS_ERR != response.status:
            self.__logerr("Error code %d returned while landing" %
                    response.status)
        else:
            self.__loginfo("landing.")

    def __takeoff(self):
        try:
            response = self.takeoff_srv()
        except rospy.ServiceException as e:
            self.__logerr("Exception caught while trying to takeoff: %s" % e)
            return

        if SUCCESS_ERR != response.status:
            self.__logerr("Error code %d returned while taking off" %
                    response.status)
        else:
            self.__loginfo("Taking off.")

    def __add_waypoints(self):

        #**********************************************************************
        #   Construct waypoint 5m east, 2m north
        #**********************************************************************
        wp1 = mavros.msg.Waypoint()
        wp1.autocontinue = True
        wp1.waitTime = rospy.Duration(secs=1.0)
        wp1.radius = 2.0
        wp1.frame = mavros.msg.Waypoint.FRAME_LOCAL
        wp1.x = 5.0
        wp1.y = 2.0
        wp1.z = 1.0

        #**********************************************************************
        #   Construct waypoint 5m west, 2m south
        #**********************************************************************
        wp2 = mavros.msg.Waypoint()
        wp2.autocontinue = True
        wp2.waitTime = rospy.Duration(secs=1.0)
        wp2.radius = 2.0
        wp2.frame = mavros.msg.Waypoint.FRAME_LOCAL
        wp2.x = -5.0
        wp2.y = -2.0
        wp2.z = 1.0

        #**********************************************************************
        #   Construct waypoint at origin
        #**********************************************************************
        wp3 = mavros.msg.Waypoint()
        wp3.autocontinue = False
        wp3.waitTime = rospy.Duration(secs=1.0)
        wp3.radius = 2.0
        wp3.frame = mavros.msg.Waypoint.FRAME_LOCAL
        wp3.x = 0.0
        wp3.y = 0.0
        wp3.z = 0.0

        #**********************************************************************
        #   Try to add waypoints to the queue
        #**********************************************************************
        waypoints = [wp1, wp2, wp3]
        try:
            response = self.add_waypoints_srv(waypoints)
        except rospy.ServiceException as e:
            self.__logerr("Exception occurred while trying to add"
                " waypoints: %s" % e)
            return

        if SUCCESS_ERR == response.status:
            self.__loginfo("Waypoints added")
        else:
            self.__logerr("Failed to add waypoints with error code: %d" %
                    response.status)

    def __add_sweep(self):
        self.__logwarn("Adding sweep not yet implemented!")

    def __add_spiral_out(self):
        self.__logwarn("Adding spiral out not yet implemented!")
        
    def __add_spiral_in(self):
        self.__logwarn("Adding spiral in not yet implemented!")

    #**************************************************************************
    #   Wrapper methods for logging errors and debug messages
    #**************************************************************************
    def __logfatal(self,msg):
        """Used for logging fatal error messages
           
           Parameters
           msg - string to log
        """
        rospy.logfatal(self.log_prefix + msg)

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


#******************************************************************************
# Parse any arguments that follow the node command
#******************************************************************************
from optparse import OptionParser

parser = OptionParser("nav_gui.py [options]")
parser.add_option("-n", "--name", dest="name", default="parrot",
                  help="Name of the prefix for the mavros node")
parser.add_option("-c", "--commands", action="store_true", dest="command_switch", default=False,
                  help="Should we have the command switch on the waypoint_tester")
parser.add_option("-r", "--ros", action="store_true", dest="ros", help="Use ROS parameter server", default=False)
(opts, args) = parser.parse_args()

if __name__ == '__main__':
    try:
        if not opts.ros or opts.name in rospy.get_param("/drones_active"):
            rospy.init_node("nav_gui")
            gui = NavGUI(opts.name, True)
            gui.start()
    except rospy.ROSInterruptException:
        gui.root.quit()
