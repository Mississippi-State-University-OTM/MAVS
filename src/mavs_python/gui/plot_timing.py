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
import sys
from Tooltip import Tooltip
import mavs_defs as mavs

class TimingPlot():
    def __init__(self,master,data):
        self.initialized = True
        self.Init(master, data)

    def Init(self,master,data):
        self.window = Toplevel(master,background=mavs.darkgrey)
        self.window.title("Timing")
        self.entries = []
        dnum = 0
        keys = data.keys()
        for k in keys:
            label = Label(self.window,text=k,background=mavs.darkgrey,foreground='white')
            label.grid(column=dnum,row=0)
            entry = Entry(self.window,width=5,background=mavs.lightgrey)
            entry.grid(column=dnum,row=1)
            entry.insert(0,data[k])
            Tooltip(entry,text=('Shows the ratio of sim-time to real-time. >1 is slower than real time, <1 is faster than real time'))
            self.entries.append(entry)
            dnum = dnum + 1
        self.window.protocol('WM_DELETE_WINDOW', self.CloseWindow)

    def Update(self,master,data):
        if not self.initialized:
            self.Init(master,data)
            self.initialized = True
        if self.initialized and self.window:
            dnum = 0
            keys = data.keys()
            for k in keys:
                self.entries[dnum].delete(0,END)
                self.entries[dnum].insert(0,data[k])
                dnum = dnum + 1

    def CloseWindow(self):
        self.initialized = False
        self.window.destroy()


class SpeedPlot():
    def __init__(self,master,speed):
        self.initialized = True
        self.Init(master, speed)

    def Init(self,master,speed):
        self.window = Toplevel(master,background=mavs.darkgrey)
        self.window.title("Vehicle Speed")
        label = Label(self.window,text='Vehicle Speed (m/s):',background=mavs.darkgrey,foreground='white')
        label.grid(column=0,row=0)
        self.entry = Entry(self.window,width=5,background=mavs.lightgrey)
        self.entry.grid(column=1,row=0)
        self.entry.insert(0,speed)
        self.window.protocol('WM_DELETE_WINDOW', self.CloseWindow)

    def Update(self,master,speed):
        if not self.initialized:
            self.Init(master,speed)
            self.initialized = True
        if self.initialized and self.window:
            self.entry.delete(0,END)
            self.entry.insert(0,('%0.3f'%speed))

    def CloseWindow(self):
        self.initialized = False
        self.window.destroy()
