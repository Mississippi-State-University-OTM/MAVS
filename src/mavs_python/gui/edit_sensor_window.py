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
import sys

sys.path.append(mavs.data_path_)

class SensorTypePopUp():
    def __init__(self,master,type):
        self.window = Toplevel(master,background=mavs.darkgrey)
        if type=='lidar':
            types = mavs.lidar_models
        if type=='camera':
            types = mavs.camera_models
        if type=='radar':
            types = mavs.radar_models
        self.s = StringVar()
        for t in types:
            b = Radiobutton(self.window,text=t,variable=self.s,value=t)
            b.config(bg=mavs.darkgrey)
            b.pack(anchor=W)
        done_button = Button(self.window,text='Done',command=self.Done, background=mavs.lightgrey)
        done_button.pack()
        Tooltip(done_button,text=('Select when done specifying sensor'))
        master.wait_window(self.window)
    def Done(self):
        self.window.destroy()
        return self.s.get()

def AddBaseSensorStuff(self,sensor):
    self.off_ent_lab_x = Label(self.sensor_window,text='Offset (x)',background=mavs.darkgrey,foreground='white')
    self.off_ent_lab_x.grid(column=0,row=0,sticky='w')
    self.off_ent_lab_y = Label(self.sensor_window,text='Offset (y)',background=mavs.darkgrey,foreground='white')
    self.off_ent_lab_y.grid(column=1,row=0,sticky='w')
    self.off_ent_lab_z = Label(self.sensor_window,text='Offset (z)',background=mavs.darkgrey,foreground='white')
    self.off_ent_lab_z.grid(column=2,row=0,sticky='w')
    self.off_ent_x = Entry(self.sensor_window,width=5,background=mavs.lightgrey)
    self.off_ent_x.grid(column=0,row=1,sticky='w')
    self.off_ent_y = Entry(self.sensor_window,width=5,background=mavs.lightgrey)
    self.off_ent_y.grid(column=1,row=1,sticky='w')
    self.off_ent_z = Entry(self.sensor_window,width=5,background=mavs.lightgrey)
    self.off_ent_z.grid(column=2,row=1,sticky='w')
    self.off_ent_x.insert(0,sensor.offset[0])
    self.off_ent_y.insert(0,sensor.offset[1])
    self.off_ent_z.insert(0,sensor.offset[2])
    Tooltip(self.off_ent_x,text=('Enter the offset from the CG in the X direction'))
    Tooltip(self.off_ent_y,text=('Enter the offset from the CG in the Y direction'))
    Tooltip(self.off_ent_z,text=('Enter the offset from the CG in the Z direction'))
    self.sensor_name_lab = Label(self.sensor_window,text='Name',background=mavs.darkgrey,foreground='white')
    self.sensor_name_lab.grid(column=3,row=0,sticky='w')
    self.sensor_name_ent = Entry(self.sensor_window,width=20,background=mavs.lightgrey)
    self.sensor_name_ent.grid(column=3,row=1,sticky='w')
    self.sensor_name_ent.insert(0,sensor.name)
    Tooltip(self.sensor_name_ent,text=('Edit the sensor name'))

    self.relor_w_lab = Label(self.sensor_window,text='Orientation (w)',background=mavs.darkgrey,foreground='white')
    self.relor_w_lab.grid(column=0,row=2,sticky='w')
    self.relor_x_lab = Label(self.sensor_window,text='Orientation (x)',background=mavs.darkgrey,foreground='white')
    self.relor_x_lab.grid(column=1,row=2,sticky='w')
    self.relor_y_lab = Label(self.sensor_window,text='Orientation (y)',background=mavs.darkgrey,foreground='white')
    self.relor_y_lab.grid(column=2,row=2,sticky='w')
    self.relor_z_lab = Label(self.sensor_window,text='Orientation (z)',background=mavs.darkgrey,foreground='white')
    self.relor_z_lab.grid(column=3,row=2,sticky='w')
    self.relor_w_ent = Entry(self.sensor_window,width=5,background=mavs.lightgrey)
    self.relor_w_ent.grid(column=0,row=3,sticky='w')
    self.relor_x_ent = Entry(self.sensor_window,width=5,background=mavs.lightgrey)
    self.relor_x_ent.grid(column=1,row=3,sticky='w')
    self.relor_y_ent = Entry(self.sensor_window,width=5,background=mavs.lightgrey)
    self.relor_y_ent.grid(column=2,row=3,sticky='w')
    self.relor_z_ent = Entry(self.sensor_window,width=5,background=mavs.lightgrey)
    self.relor_z_ent.grid(column=3,row=3,sticky='w')
    self.relor_w_ent.insert(0,sensor.rel_or[0])
    self.relor_x_ent.insert(0,sensor.rel_or[1])
    self.relor_y_ent.insert(0,sensor.rel_or[2])
    self.relor_z_ent.insert(0,sensor.rel_or[3])
    Tooltip(self.relor_w_ent,text=('Set the relative orientation from vehicle LookTo'))
    Tooltip(self.relor_x_ent,text=('Set the relative orientation from vehicle LookTo'))
    Tooltip(self.relor_y_ent,text=('Set the relative orientation from vehicle LookTo'))
    Tooltip(self.relor_z_ent,text=('Set the relative orientation from vehicle LookTo'))

    self.rate_label = Label(self.sensor_window,text='Update Rate',background=mavs.darkgrey,foreground='white')
    self.rate_label.grid(column=0,row=4,sticky='w')
    self.rate_ent = Entry(self.sensor_window,width=5,background=mavs.lightgrey)
    self.rate_ent.grid(column=0,row=5,sticky='w')
    self.rate_ent.insert(0,sensor.update_rate)
    Tooltip(self.rate_ent,text=('Set the sensor refresh rate in Hz'))

class EditCameraWindow():
    def __init__(self, master, camera):
        self.sensor_window = Toplevel(master,background=mavs.darkgrey)
        self.sensor_window.title("Edit Camera Properties")
        AddBaseSensorStuff(self,camera)

        self.gamma_label = Label(self.sensor_window,text='Gamma',background=mavs.darkgrey,foreground='white')
        self.gamma_label.grid(column=1,row=4,sticky='w')
        self.gamma_ent = Entry(self.sensor_window,width=5,background=mavs.lightgrey)
        self.gamma_ent.grid(column=1,row=5,sticky='w')
        self.gamma_ent.insert(0,camera.gamma)
        Tooltip(self.gamma_ent,text=('Set the camera compression exponent, gamma. Typically ranges from 0-1'))

        self.gain_label = Label(self.sensor_window,text='Gain',background=mavs.darkgrey,foreground='white')
        self.gain_label.grid(column=2,row=4,sticky='w')
        self.gain_ent = Entry(self.sensor_window,width=5,background=mavs.lightgrey)
        self.gain_ent.grid(column=2,row=5,sticky='w')
        self.gain_ent.insert(0,camera.gain)
        Tooltip(self.gain_ent,text=('Set the camera gain factor. Must be positive'))

        self.aa_label = Label(self.sensor_window,text='Anti-aliasing Factor',background=mavs.darkgrey,foreground='white')
        self.aa_label.grid(column=3,row=4,sticky='w')
        self.aa_ent = Entry(self.sensor_window,width=5,background=mavs.lightgrey)
        self.aa_ent.grid(column=3,row=5,sticky='w')
        self.aa_ent.insert(0,camera.aa_fac)
        Tooltip(self.aa_ent,text=('Set the pixel-oversampling factor'))

        self.shadows_var = BooleanVar()
        self.shadows_check = Checkbutton(self.sensor_window,text=('Render Shadows?'),
                                      variable=self.shadows_var,onvalue=True,
                                      offvalue=False,
                                      selectcolor=mavs.maroon,
                                      background=mavs.darkgrey,
                                      foreground='white')
        self.shadows_check.grid(column=0,row=6)
        self.shadows_var.set(camera.render_shadows)
        Tooltip(self.shadows_check,text=('Render shadows?'))

        self.lensdrops_var = BooleanVar()
        self.lensdrops_check = Checkbutton(self.sensor_window,text=('Raindrops on Lens?'),
                                      variable=self.lensdrops_var,onvalue=True,
                                      offvalue=False,
                                      selectcolor=mavs.maroon,
                                      background=mavs.darkgrey,
                                      foreground='white')
        self.lensdrops_check.grid(column=1,row=6)
        self.lensdrops_var.set(camera.raindrop_lens)
        Tooltip(self.lensdrops_check,text=('Show raindrops on camera lens?'))

        def finish_editing():
            camera.SetDropsOnLens(self.lensdrops_var.get())
            camera.RenderShadows(self.shadows_var.get())
            camera.update_rate = float(self.rate_ent.get())
            camera.SetAntiAliasingFactor(int(self.aa_ent.get()))
            camera.offset = [float(self.off_ent_x.get()),float(self.off_ent_y.get()),float(self.off_ent_z.get())]
            camera.rel_or = [float(self.relor_w_ent.get()),float(self.relor_x_ent.get()),float(self.relor_y_ent.get()),float(self.relor_z_ent.get())]
            camera.SetGammaAndGain(float(self.gamma_ent.get()),float(self.gain_ent.get()))
            camera.SetOffset(camera.offset,camera.rel_or)
            camera.name = self.sensor_name_ent.get()
            self.sensor_window.destroy()

        self.finish_button = Button(self.sensor_window,text='Done', command=finish_editing, background=mavs.lightgrey)
        self.finish_button.grid(column=0,row=7,columnspan=4)
        Tooltip(self.finish_button,text=('Select when done editing sensor properties'))
        master.wait_window(self.sensor_window)

class EditLidarWindow():
    def __init__(self, master, lidar):
        self.sensor_window = Toplevel(master,background=mavs.darkgrey)
        self.sensor_window.title("Edit lidar Properties")
        AddBaseSensorStuff(self,lidar)

        def finish_editing():
            lidar.update_rate = float(self.rate_ent.get())
            lidar.offset = [float(self.off_ent_x.get()),float(self.off_ent_y.get()),float(self.off_ent_z.get())]
            lidar.rel_or = [float(self.relor_w_ent.get()),float(self.relor_x_ent.get()),float(self.relor_y_ent.get()),float(self.relor_z_ent.get())]
            lidar.SetOffset(lidar.offset,lidar.rel_or)
            lidar.name = self.sensor_name_ent.get()
            self.sensor_window.destroy()

        self.finish_button = Button(self.sensor_window,text='Done', command=finish_editing, background=mavs.lightgrey)
        self.finish_button.grid(column=0,row=7,columnspan=4)
        Tooltip(self.finish_button,text=('Select when done editing sensor properties'))
        master.wait_window(self.sensor_window)
        

class EditRadarWindow():
    def __init__(self, master, radar):
        self.sensor_window = Toplevel(master,background=mavs.darkgrey)
        self.sensor_window.title("Edit lidar Properties")
        AddBaseSensorStuff(self,radar)

        self.range_label = Label(self.sensor_window,text='Range',background=mavs.darkgrey,foreground='white')
        self.range_label.grid(column=1,row=4,sticky='w')
        self.range_ent = Entry(self.sensor_window,width=5,background=mavs.lightgrey)
        self.range_ent.grid(column=1,row=5,sticky='w')
        self.range_ent.insert(0,'250.0')
        Tooltip(self.range_ent,text=('Set the radar range in meters'))

        self.res_label = Label(self.sensor_window,text='Gain',background=mavs.darkgrey,foreground='white')
        self.res_label.grid(column=2,row=4,sticky='w')
        self.res_ent = Entry(self.sensor_window,width=5,background=mavs.lightgrey)
        self.res_ent.grid(column=2,row=5,sticky='w')
        self.res_ent.insert(0,0.25)
        Tooltip(self.res_ent,text=('Set the radar angular resolution (degrees). Must be positive'))

        def finish_editing():
            radar.update_rate = float(self.rate_ent.get())
            radar.offset = [float(self.off_ent_x.get()),float(self.off_ent_y.get()),float(self.off_ent_z.get())]
            radar.rel_or = [float(self.relor_w_ent.get()),float(self.relor_x_ent.get()),float(self.relor_y_ent.get()),float(self.relor_z_ent.get())]
            radar.SetOffset(radar.offset,radar.rel_or)
            radar.name = self.sensor_name_ent.get()
            self.sensor_window.destroy()

        self.finish_button = Button(self.sensor_window,text='Done', command=finish_editing, background=mavs.lightgrey)
        self.finish_button.grid(column=0,row=7,columnspan=4)
        Tooltip(self.finish_button,text=('Select when done editing sensor properties'))
        master.wait_window(self.sensor_window)