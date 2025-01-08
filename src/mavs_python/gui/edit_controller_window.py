'''
Non-Commercial License - Mississippi State University Autonomous Vehicle Software (MAVS)

ACKNOWLEDGEMENT:
Mississippi State University, Center for Advanced Vehicular Systems (CAVS)

*NOTICE*
Thank you for your interest in MAVS, we hope it serves you well!  We have a few small requests to ask of you that will help us continue to create innovative platforms:
-To share MAVS with a friend, please ask them to visit https://gitlab.com/cgoodin/msu-autonomous-vehicle-simulator to register and download MAVS for free.
-Feel free to create derivative works for non-commercial purposes, i.e. academics, U.S. government, and industry research.  If commercial uses are intended, please contact us for a commercial license at otm@msstate.edu or jonathan.rudd@msstate.edu.
-Finally, please use the ACKNOWLEDGEMENT above in your derivative works.  This helps us both!

Please visit https://gitlab.com/cgoodin/msu-autonomous-vehicle-simulator to view the full license, or email us if you have any questions.

Copyright 2018 (C) Mississippi State University
'''
from tkinter import *
from Tooltip import Tooltip
import mavs_defs as mavs

class EditControllerWindow():
    def __init__(self, master, controller):
        self.controller_window = Toplevel(master,background=mavs.darkgrey)
        self.controller_window.title("Edit Controller Properties")

        self.wheelbase_lab = Label(self.controller_window,text='Wheelbase',background=mavs.darkgrey,foreground='white')
        self.wheelbase_lab.grid(column=0,row=0)
        self.wheelbase_ent = Entry(self.controller_window,width=5,background=mavs.lightgrey)
        self.wheelbase_ent.grid(column=0,row=1)
        self.wheelbase_ent.insert(0,controller.wheelbase)
        Tooltip(self.wheelbase_ent,text=('Set the vehicle wheelbase in meters'))

        self.steering_lab = Label(self.controller_window,text='Steering Coefficient',background=mavs.darkgrey,foreground='white')
        self.steering_lab.grid(column=1,row=0)
        self.steering_ent = Entry(self.controller_window,width=5,background=mavs.lightgrey)
        self.steering_ent.grid(column=1,row=1)
        self.steering_ent.insert(0,controller.steering_coeff)
        Tooltip(self.steering_ent,text=('Set the steering proportional coefficient'))

        self.speed_lab = Label(self.controller_window,text='Max Speed',background=mavs.darkgrey,foreground='white')
        self.speed_lab.grid(column=2,row=0)
        self.speed_ent = Entry(self.controller_window,width=5,background=mavs.lightgrey)
        self.speed_ent.grid(column=2,row=1)
        self.speed_ent.insert(0,controller.desired_speed)
        Tooltip(self.speed_ent,text=('Set the desired speed in m/s'))

        self.sa_lab = Label(self.controller_window,text='Steering Angle',background=mavs.darkgrey,foreground='white')
        self.sa_lab.grid(column=3,row=0)
        self.sa_ent = Entry(self.controller_window,width=5,background=mavs.lightgrey)
        self.sa_ent.grid(column=3,row=1)
        self.sa_ent.insert(0,controller.max_steering_angle)
        Tooltip(self.sa_ent,text=('Set the max steering angle in radians'))

        def finish_editing():
            controller.SetSteeringScale(float(self.steering_ent.get()))
            controller.SetWheelbase(float(self.wheelbase_ent.get()))
            controller.SetMaxSteerAngle(float(self.sa_ent.get()))
            controller.SetDesiredSpeed(float(self.speed_ent.get()))
            self.controller_window.destroy()

        self.finish_button = Button(self.controller_window,text='Done', command=finish_editing, background=mavs.lightgrey)
        self.finish_button.grid(column=0,row=7,columnspan=4)
        Tooltip(self.finish_button,text=('Select when done editing controller properties'))
        master.wait_window(self.controller_window)