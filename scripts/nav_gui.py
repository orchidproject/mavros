#!/usr/bin/python

from Tkinter import *
import tkFont
import rospy
import mavros.msg
import mavros.srv
import mosaic_node as q
from waypoint_tester import construct_waypoints_global, construct_waypoints_local


class NavGUI:
    def __init__(self, prefix):
        self.root = Tk()
        self.root.title("AR Drone Controller")
        self.font = tkFont.Font(family="Helvetica", size=15)
        self.prefix = prefix
        self.queue = None
        self.manual = None
        self.inst = None
        self.origin = None

    def start(self):
        rospy.init_node("nav_gui")
        self.manual = rospy.Publisher(self.prefix + "velocity", mavros.msg.Velocity, queue_size=1000)
        self.inst = rospy.Publisher(self.prefix + "instructions", mavros.msg.InstructionList, queue_size=100)
        self.queue = rospy.ServiceProxy(self.prefix + "queue", mavros.srv.Queue)
        rospy.Subscriber("/apm/filtered_pos", mavros.msg.FilteredPosition, self.gps_cb)
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

        info1 = Label(control_frame, text="Roll/Pitch", font=self.font)
        info1.grid(row=1, column=1)
        pitch_forward = Button(control_frame, text="/\\", command=lambda: self.manual.publish([0, -1, 0, 0]),
                               font=self.font)
        pitch_forward.grid(row=0, column=1)
        pitch_back = Button(control_frame, text="\\/", command=lambda: self.manual.publish([0, 1, 0, 0]),
                            font=self.font)
        pitch_back.grid(row=2, column=1)
        roll_left = Button(control_frame, text="<", command=lambda: self.manual.publish([-1, 0, 0, 0]), font=self.font)
        roll_left.grid(row=1, column=0)
        roll_right = Button(control_frame, text=">", command=lambda: self.manual.publish([1, 0, 0, 0]), font=self.font)
        roll_right.grid(row=1, column=2)

        info2 = Label(control_frame, text="Yaw/Alt", font=self.font)
        info2.grid(row=1, column=5)
        alt_up = Button(control_frame, text="/\\", command=lambda: self.manual.publish([0, 0, 1, 0]), font=self.font)
        alt_up.grid(row=0, column=5)
        alt_down = Button(control_frame, text="\\/", command=lambda: self.manual.publish([0, 0, -1, 0]), font=self.font)
        alt_down.grid(row=2, column=5)
        yaw_left = Button(control_frame, text="<", command=lambda: self.manual.publish([0, 0, 0, -1]), font=self.font)
        yaw_left.grid(row=1, column=4)
        yaw_right = Button(control_frame, text=">", command=lambda: self.manual.publish([0, 0, 0, 1]), font=self.font)
        yaw_right.grid(row=1, column=6)

        manual = Button(frame1, text="Manual", command=lambda: self.queue(q.CMD_MANUAL), font=self.font)
        manual.pack(side=LEFT)
        auto = Button(frame1, text="Auto", command=lambda: self.queue(q.CMD_AUTO), font=self.font)
        auto.pack(side=RIGHT)

        takeoff = Button(frame2, text="Takeoff", command=lambda: self.queue(q.CMD_MANUAL_TAKEOFF), font=self.font)
        takeoff.pack(side=LEFT)
        camera = Button(frame2, text="Camera", command=lambda: self.queue(q.CMD_SWITCH_CAMERA), font=self.font)
        camera.pack(side=LEFT)
        land = Button(frame2, text="Land", command=lambda: self.queue(q.CMD_MANUAL_LAND), font=self.font)
        land.pack(side=RIGHT)

        send = Button(frame3, text="Send Waypoints",
                      command=lambda: self.inst.publish(construct_waypoints_global(1, False)), font=self.font)
        send.pack(side=LEFT)
        clear = Button(frame3, text="Clear Waypoints", command=lambda: self.queue(q.CMD_CLEAR), font=self.font)
        clear.pack(side=RIGHT)

        run = Button(frame4, text="Execute", command=lambda: self.queue(q.CMD_EXECUTE), font=self.font)
        run.pack(side=LEFT)
        pause = Button(frame4, text="Pause", command=lambda: self.queue(q.CMD_PAUSE), font=self.font)
        pause.pack(side=LEFT)
        origin = Button(frame4, text="Set Origin", command=lambda: self.inst.publish(self.origin) if self.origin else None, font=self.font)
        origin.pack(side=RIGHT)

        emergency = Button(main_frame, text="Emergency Land", command=lambda: self.emergency(2), font=self.font)
        emergency.pack(side=BOTTOM)

        control_frame.pack()
        frame1.pack(side=TOP)
        frame2.pack(side=TOP)
        frame4.pack(side=BOTTOM)
        frame3.pack(side=BOTTOM)
        main_frame.grid(row=0, column=0, rowspan=3, sticky="nesw")

        red_button = Button(self.root, text="KILL", command=lambda: self.queue(q.CMD_EMERGENCY),
                            font=self.font, height=2, width=10, bg="red")
        red_button.grid(row=1, column=2, sticky="nesw")

    def setup_keyboard(self):
        self.root.bind("<w>", lambda (event): self.manual.publish([0, -1, 0, 0]))
        self.root.bind("<s>", lambda (event): self.manual.publish([0, 1, 0, 0]))
        self.root.bind("<a>", lambda (event): self.manual.publish([-1, 0, 0, 0]))
        self.root.bind("<d>", lambda (event): self.manual.publish([1, 0, 0, 0]))

        self.root.bind("<Up>", lambda (event): self.manual.publish([0, 0, 1, 0]))
        self.root.bind("<Down>", lambda (event): self.manual.publish([0, 0, -1, 0]))
        self.root.bind("<Left>", lambda (event): self.manual.publish([0, 0, 0, -1]))
        self.root.bind("<Right>", lambda (event): self.manual.publish([0, 0, 0, 1]))

        self.root.bind("<q>", lambda (event): self.queue(q.CMD_MANUAL))
        self.root.bind("<e>", lambda (event): self.queue(q.CMD_AUTO))
        self.root.bind("<r>", lambda (event): self.queue(q.CMD_MANUAL_TAKEOFF))
        self.root.bind("<f>", lambda (event): self.queue(q.CMD_MANUAL_LAND))
        self.root.bind("<c>", lambda (event): self.queue(q.CMD_CLEAR))
        self.root.bind("<v>", lambda (event): self.inst.publish(construct_waypoints_local(1, True)))
        self.root.bind("<k>", lambda (event): self.queue(q.CMD_EXECUTE))
        self.root.bind("<l>", lambda (event): self.queue(q.CMD_PAUSE))
        self.root.bind("<j>", lambda (event): self.queue(q.CMD_SWITCH_CAMERA))
        self.root.bind("<space>", lambda (event): self.emergency(2))
        self.root.bind("<F12>", lambda (event): self.queue(q.CMD_EMERGENCY))

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
        label10 = Label(frame, text="Send waypoints - V", font=self.font)
        label10.grid(row=9)
        label11 = Label(frame, text="Execute - K", font=self.font)
        label11.grid(row=10)
        label12 = Label(frame, text="Pause - L", font=self.font)
        label12.grid(row=11)
        label13 = Label(frame, text="Emergency Land - Space", font=self.font)
        label13.grid(row=12)
        label14 = Label(frame, text="!!!KILL!!! - F12", font=self.font)
        label14.grid(row=13)
        frame.grid(row=0, column=1, rowspan=3, sticky="nesw")

    def emergency(self, times):
        for i in range(times):
            self.queue(q.CMD_PAUSE)
            self.queue(q.CMD_CLEAR)
            self.queue(q.CMD_MANUAL_LAND)
            rospy.sleep(0.1)

    def gps_cb(self, req):
        if not self.origin:
            self.origin = mavros.msg.InstructionList()
            self.origin.inst.append(mavros.msg.Instruction())
            self.origin.inst[0].type = mavros.msg.Instruction.TYPE_ORIGIN
        self.origin.inst[0].latitude = req.latitude
        self.origin.inst[0].longitude = req.longitude
        self.origin.inst[0].altitude = req.relative_altitude


# *******************************************************************************
# Parse any arguments that follow the node command
# *******************************************************************************
from optparse import OptionParser

parser = OptionParser("mosaic_node.py [options]")
parser.add_option("-p", "--prefix", dest="prefix", default="/mosaic/",
                  help="prefix of the mavros node")
(opts, args) = parser.parse_args()

if __name__ == '__main__':
    try:
        rospy.wait_for_service(opts.prefix + "queue")
        gui = NavGUI(opts.prefix)
        gui.start()
    except rospy.ROSInterruptException:
        gui.root.quit()
