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
#tkinter modules
import tkinter as tk
from tkinter import *
from tkinter import filedialog
#third party modules
from Tooltip import Tooltip
import mavs_defs as mavs

class EnvironmentWindow():
    def __init__(self):
        self.window = None
    def EditProperties(self,master,env):
        if self.window is None:
            self.window = Toplevel(master,background=mavs.darkgrey)
            self.window.title("Environmental Properties")

            self.turbid_slider = Scale(self.window, from_=10.0, to=2.0,background=mavs.darkgrey,foreground='white')
            self.turbid_slider.set(env.turbidity)
            self.turbid_label = Label(self.window,text="Turbidity",background=mavs.darkgrey,foreground='white')
            self.turbid_label.grid(column=0,row=0)
            self.turbid_slider.grid(column=0,row=1)
            Tooltip(self.turbid_slider,text=('Turbidity (aerosol content) of the air'))

            self.cloud_slider = Scale(self.window, from_=100, to=0,background=mavs.darkgrey,foreground='white')
            self.cloud_slider.set(100*env.cloud_cover)
            self.cloud_label = Label(self.window,text="Cloud Cover",background=mavs.darkgrey,foreground='white')
            self.cloud_label.grid(column=1,row=0)
            self.cloud_slider.grid(column=1,row=1)
            Tooltip(self.cloud_slider,text=('Cloud cover percent'))

            self.rain_slider = Scale(self.window, from_=25, to=0,background=mavs.darkgrey,foreground='white')
            self.rain_label = Label(self.window,text="Rain Rate",background=mavs.darkgrey,foreground='white')
            self.rain_label.grid(column=2,row=0)
            self.rain_slider.set(env.rain_rate)
            self.rain_slider.grid(column=2,row=1)
            Tooltip(self.rain_slider,text=('Rain rate in mm/h'))

            self.fog_slider = Scale(self.window, from_=100, to=0,background=mavs.darkgrey,foreground='white')
            self.fog_slider.set(env.fog)
            self.fog_label = Label(self.window,text="Fog",background=mavs.darkgrey,foreground='white')
            self.fog_label.grid(column=3,row=0)
            self.fog_slider.grid(column=3,row=1)
            Tooltip(self.fog_slider,text=('Fogginess in relative optical depth units'))

            self.time_slider = Scale(self.window, from_=23, to=0,background=mavs.darkgrey,foreground='white')
            self.time_slider.set(env.hour)
            self.time_label = Label(self.window,text="Time",background=mavs.darkgrey,foreground='white')
            self.time_label.grid(column=4,row=0)
            self.time_slider.grid(column=4,row=1)
            Tooltip(self.time_slider,text=('Adjust the time of day (hours in military time)'))

            self.albedo_label = Label(self.window,text="Albedo",background=mavs.darkgrey,foreground='white')
            self.albedo_label.grid(column=5,row=0)
            self.albedo_slider = Scale(self.window,from_=100,to=0,background=mavs.darkgrey,foreground='white')
            self.albedo_slider.set(100*env.albedo)
            self.albedo_slider.grid(column=5,row=1)
            Tooltip(self.albedo_slider,text=('Local albedo (global surface reflectance)'))

            self.snow_slider = Scale(self.window, from_=25, to=0,background=mavs.darkgrey,foreground='white')
            self.snow_label = Label(self.window,text="Snow Rate",background=mavs.darkgrey,foreground='white')
            self.snow_label.grid(column=6,row=0)
            self.snow_slider.set(env.snow_rate)
            self.snow_slider.grid(column=6,row=1)
            Tooltip(self.snow_slider,text=('Snow rate in mm/h'))

            self.window.protocol('WM_DELETE_WINDOW', self.removewindow)
    def removewindow(self):
        self.window.destroy()
        self.window = None
    def GetFog(self):
        return self.fog_slider.get()
    def GetRain(self):
        return self.rain_slider.get()
    def GetTurbidity(self):
        return self.turbid_slider.get()
    def GetCloudCover(self):
        return 0.01*self.cloud_slider.get()
    def GetTime(self):
        return self.time_slider.get()
    def GetAlbedo(self):
        return (0.01*self.albedo_slider.get())
    def GetSnow(self):
        return (self.snow_slider.get())
