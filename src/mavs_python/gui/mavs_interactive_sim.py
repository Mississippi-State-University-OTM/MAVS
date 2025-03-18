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
#python modules
from multiprocessing.managers import ValueProxy
import sys
import os
import math
import time
#tkinter modules
import tkinter as tk
from tkinter import *
from tkinter import filedialog
from tkinter import simpledialog
from tkinter import messagebox
#third party modules
from Tooltip import Tooltip

# Load the mavs python modules
if (len(sys.argv)>1):
    python_directory = sys.argv[1]
sys.path.append(python_directory)
import mavs_interface
import mavs_python_paths
import mavs_defs as mavs
mavs.data_path_ = python_directory
from edit_sensor_window import EditCameraWindow,EditLidarWindow,EditRadarWindow,SensorTypePopUp
from edit_controller_window import EditControllerWindow
from edit_waypoint_window import EditWaypointsWindow
from environment_window import EnvironmentWindow
from plot_timing import TimingPlot,SpeedPlot
from menu_bar import MavsMenuBar

class MavsInteractiveSim(MavsMenuBar):
    def __init__(self,master):
        self.master = master
        self.master.configure(background=mavs.darkgrey)
        self.master.title("MAVS Simulation")
        self.free_drive_var = BooleanVar()
        self.free_drive_var.set(True)
        self.show_timing_var = BooleanVar()
        self.show_timing_var.set(False)
        self.plot_speed_var = BooleanVar()
        self.plot_speed_var.set(False)
        self.map_displayed_var = BooleanVar()
        self.map_displayed_var.set(False)
        self.time_since_last_map = 0.0
        menubar = MavsMenuBar
        menubar.AddMenuBar(self)
        self.env_window = EnvironmentWindow()
        self.simulation = mavs_interface.MavsSimulation()
        self.mavs_data_path = mavs_python_paths.mavs_data_path
        self.sim_state = 'paused'
        self.dt = 1.0/30.0 
        self.total_wall = 0.0
        self.loaded = False
        # Create a camera for looking at the vehicle
        self.cam = mavs_interface.MavsCamera()
        #self.cam.Initialize(960,640,(1920.0/1080.0)*0.0035,0.0035,0.0035)
        self.cam.Initialize(768,432,(1920.0/1080.0)*0.0035,0.0035,0.0035)
        self.cam.SetOffset([-10.0,0.0,3.0],[1.0,0.0,0.0,0.0])
        self.cam.RenderShadows(False)

        # Start button 
        self.start_button_text = StringVar()
        start_button = Button(self.master,textvariable=self.start_button_text, command=self.StartSim,background=mavs.lightgrey)
        start_button.grid(column=0,row=3)
        self.start_button_text.set('Start Simulation')
        Tooltip(start_button,text=('Start the simulation'))
        
        # End button 
        self.end_button_text = StringVar()
        end_button = Button(self.master,textvariable=self.end_button_text, command=self.EndSim,background=mavs.lightgrey)
        end_button.grid(column=0,row=4)
        self.end_button_text.set('End Simulation')
        Tooltip(end_button,text=('Exit the current simulation'))

        # Display elapsed time
        self.elapsed_time_label = Label(self.master,text='Simulation Time:',background=mavs.darkgrey,foreground='white')
        self.elapsed_time_label.grid(column=2,row=3)
        self.elapsed_time_ent = Entry(self.master,width=5,background=mavs.lightgrey)
        self.elapsed_time_ent.grid(column=3,row=3)
        self.elapsed_time_ent.insert(0,0.0)
        Tooltip(self.elapsed_time_ent,text=('Simulation time (seconds)'))

        #---- sensor listbox
        self.sensor_listbox = Listbox(self.master,background=mavs.lightgrey)
        self.sensor_listbox.grid(column=1,row=0,rowspan=6)
        def edit_sensor_event(event):
            self.EditSensor()
        self.sensor_listbox.bind('<Double-Button>',edit_sensor_event)
        self.sensor_listbox.bind('<Return>',edit_sensor_event)
        def delete_sensor_event(event):
            self.DeleteSensor()
        self.sensor_listbox.bind('<Delete>',delete_sensor_event)
        def do_popup_event(event):
            self.SensorPopupMenu(event)
        self.sensor_listbox.bind("<Button-3>",do_popup_event)
        Tooltip(self.sensor_listbox,text='List of all sensors on the vehicle. Double click to edit. Right click to turn on/off. Delete key to remove')

        self.viewer = None 

        cavs_logo = PhotoImage(file=self.mavs_data_path+'/../src/mavs_python/gui/cavs.gif')
        cavs_label = Label(master,image=cavs_logo,background=mavs.darkgrey)
        cavs_label.image = cavs_logo
        cavs_label.grid(column=0,row=8,sticky='w')
        msu_logo = PhotoImage(file=self.mavs_data_path+'/../src/mavs_python/gui/msu.gif')
        msu_label = Label(master,image=msu_logo,background=mavs.darkgrey)
        msu_label.image = msu_logo
        msu_label.grid(column=3,row=8,sticky='e')

        self.cam_time = 0.0
        self.map_time = 0.0
        self.timing_plot = None
        self.speed_plot = None
    #------ Done with create new sensor menu -----------------------------#
    def SensorPopupMenu(self,event):
        popup = Menu(self.master, tearoff=0)
        popup.add_command(label="Turn On", command=self.ActivateSensor)
        popup.add_command(label="Turn Off", command=self.DeactivateSensor)
        popup.add_command(label="Save Raw", command=self.TurnOnSaveRawSensorData)
        popup.add_command(label="Save Labeled", command=self.TurnOnSaveLabeledSensorData)
        popup.add_command(label="Stop Saving", command=self.TurnOffSaveSensorData)
        try:
            popup.tk_popup(event.x_root,event.y_root,0)
        finally:
            popup.grab_release()
    def GetNewDt(self):
        new_dt = simpledialog.askfloat("Input","New time step in seconds?", parent=self.master, minvalue=0.001, maxvalue=1.0,initialvalue=self.dt)
        if (new_dt):
            self.dt = new_dt
    def LoadNewWaypoints(self):
        wpfile = filedialog.askopenfilename(initialdir=self.mavs_data_path+'/waypoints', title='Select Waypoints File (.vprp)')
        self.simulation.LoadNewWaypoints(wpfile)
        self.PauseSim()
    def LoadNewVehicle(self):
        vehdynfile = filedialog.askopenfilename(initialdir=self.mavs_data_path+'/vehicles/rp3d_vehicles', title='Select Vehicle Dynamics File')
        vehvizfile = filedialog.askopenfilename(initialdir=self.mavs_data_path+'/actors/actors', title='Select Vehicle Visualization File')
        self.simulation.LoadNewVehicle(vehdynfile,vehvizfile)
        self.PauseSim()
    def LoadNewScene(self):
        self.PauseSim()
        scenefile = filedialog.askopenfilename(initialdir=self.mavs_data_path+'/scenes', title='Select Scene File')
        self.simulation.LoadNewScene(scenefile)
        self.PauseSim()
    def PauseSim(self):
        self.sim_state = 'paused'
        self.start_button_text.set('Resume Simulation')
    def StartSim(self):
        if not self.loaded:
            messagebox.showinfo('No Simulation Loaded.', 'Please Load a Simulation')
            return
        if self.sim_state=='paused':
            self.sim_state='running'
            self.start_button_text.set('Pause Simulation')
        elif self.sim_state=='running':
            self.sim_state = 'paused'
            self.start_button_text.set('Resume Simulation')
    def EndSim(self):
        self.__init__(self.master)
    def SaveSim(self):
        save_name = filedialog.asksaveasfilename(initialdir=self.mavs_data_path+'/sims/sensor_sims', title='Select Simulation File')
        self.simulation.WriteToJson(save_name)
    def SetSaveDirectory(self):
        self.simulation.save_location = filedialog.askdirectory(initialdir='./',title='Select a directory to save sensor data')
    def InitMapViewer(self):
        if self.map_displayed_var.get():
            #self.viewer = mavs_interface.MavsOrthoViewer()
            pos = self.simulation.vehicle.GetPosition()
            self.viewer = mavs_interface.MavsMapViewer(pos[0]-256.0, pos[1]-256.0, pos[0]+256.0, pos[1]+256.0, 0.5)
            self.viewer.AddWaypoints(self.simulation.waypoints.waypoints)
            self.viewer.AddCircle([pos[0],pos[1]], 2.0)
            self.viewer.Display(self.simulation.env)
            #self.viewer.SetWaypoints(self.simulation.waypoints)
        elif not self.map_displayed_var.get():
            self.viewer = None
    def EditController(self):
        EditControllerWindow(self.master,self.simulation.controller)
    def EditWaypoints(self):
        pos = self.simulation.vehicle.GetPosition()
        EditWaypointsWindow(self.master,self.simulation.waypoints, python_directory, pos, self.simulation.env)
    def AddSensor(self,new_sensor_type):
        get_type = SensorTypePopUp(self.master,new_sensor_type)
        spec_type = get_type.s.get()
        if new_sensor_type=='lidar':
            new_sens = mavs_interface.MavsLidar(spec_type)
            EditLidarWindow(self.master,new_sens)
        elif new_sensor_type=='camera':
            new_sens = mavs_interface.MavsCamera()
            new_sens.Model(spec_type)
            EditCameraWindow(self.master,new_sens)
        elif new_sensor_type=='radar':
            new_sens = mavs_interface.MavsRadar()
            EditRadarWindow(self.master, new_sens)
        elif new_sensor_type=='rtk':
            new_sens = mavs_interface.MavsRtk()
            #EditRtkGpsWindow(self.master, new_sens)
        self.simulation.sensors.append(new_sens)
        self.simulation.sensor_times.append(0.0)
        self.sensor_listbox.insert(END,new_sens.name)
    def DeleteSensor(self):
        answer = messagebox.askyesno("Question","Really Delete Sensor")
        if answer:
            sensnum = self.sensor_listbox.curselection()[0]
            self.simulation.sensors.pop(sensnum)
            self.simulation.sensor_times.pop(sensnum)
            self.sensor_listbox.delete(sensnum)
    def EditSensor(self):
        sensnum = self.sensor_listbox.curselection()[0]
        if self.simulation.sensors[sensnum].type=='camera':
            EditCameraWindow(self.master,self.simulation.sensors[sensnum])
        if self.simulation.sensors[sensnum].type=='lidar':
            EditLidarWindow(self.master,self.simulation.sensors[sensnum])
    def EditEnvironment(self):
        self.env_window.EditProperties(self.master,self.simulation.env)
    def ActivateSensor(self):
        sensnum = self.sensor_listbox.curselection()[0]
        self.simulation.TurnOnSensor(sensnum,display=True)
        self.sensor_listbox.itemconfig(sensnum,{'bg':'green'})
    def DeactivateSensor(self):
        sensnum = self.sensor_listbox.curselection()[0]
        self.simulation.TurnOffSensor(sensnum)
        self.sensor_listbox.itemconfig(sensnum,{'bg':'red'})
    def TurnOnSaveRawSensorData(self):
        sensnum = self.sensor_listbox.curselection()[0]
        self.simulation.TurnOnSensor(sensnum,display=True,save_raw=True)
        self.sensor_listbox.itemconfig(sensnum,{'bg':'blue'})
    def TurnOnSaveLabeledSensorData(self):
        sensnum = self.sensor_listbox.curselection()[0]
        self.simulation.TurnOnSensor(sensnum,display=True,save_raw=True,labeling=True)
        self.sensor_listbox.itemconfig(sensnum,{'bg':'purple'})
    def TurnOffSaveSensorData(self):
        sensnum = self.sensor_listbox.curselection()[0]
        if (self.simulation.sensors[sensnum].is_active):
            self.simulation.TurnOnSensor(sensnum,display=True,save_raw=False,labeling=False)
            self.sensor_listbox.itemconfig(sensnum,{'bg':'green'})
        else:
            self.simulation.TurnOffSensor(sensnum)
            self.sensor_listbox.itemconfig(sensnum,{'bg':'red'})
    def LoadSimFromFile(self,sim_name):
        self.simulation.Load(sim_name)
        self.loaded = True
        for s in self.simulation.sensors:
            self.sensor_listbox.insert(END,s.name)
        for i in range (0,len(self.simulation.sensors)):
            self.sensor_listbox.itemconfig(i,{'bg':'red'})
        self.UpdateSimCam(0.0)
    def LoadSimulation(self):
        sim_name = filedialog.askopenfilename(initialdir=self.mavs_data_path+'/sims/sensor_sims', title='Select Simulation File')
        if (sim_name):
            self.LoadSimFromFile(sim_name)
    def UpdateSimCam(self,dt):
        self.cam.SetEnvironmentProperties(self.simulation.env.obj)
        self.cam.SetPose(self.simulation.vehicle.GetPosition(),self.simulation.vehicle.GetOrientation())
        self.cam.Update(self.simulation.env,1.0/self.cam.update_rate)
        self.cam.Display()
        self.cam.elapsed_since_last = 0.0
    def UpdateEnvironmentParams(self):
        if self.simulation.env.turbidity != self.env_window.GetTurbidity():
            self.simulation.env.SetTurbidity(self.env_window.GetTurbidity())
        if self.simulation.env.cloud_cover != self.env_window.GetCloudCover():
            self.simulation.env.SetCloudCover(self.env_window.GetCloudCover())
        if self.simulation.env.rain_rate != self.env_window.GetRain():
            self.simulation.env.SetRainRate(self.env_window.GetRain())
        if self.simulation.env.snow_rate != self.env_window.GetSnow():
            self.simulation.env.SetSnow(self.env_window.GetSnow())
        if self.simulation.env.fog != self.env_window.GetFog():
            self.simulation.env.SetFog(self.env_window.GetFog())
        if self.simulation.env.hour != self.env_window.GetTime():
            self.simulation.env.SetTime(self.env_window.GetTime())
        if self.simulation.env.albedo != self.env_window.GetAlbedo():
            self.simulation.env.SetAlbedo(self.env_window.GetAlbedo())
    def PlotTimeGraph(self):
        timing_dict = {}
        timing_dict['Sim Window'] = self.cam_time
        timing_dict['Map Window'] = self.map_time
        timing_dict['Vehicle'] = self.simulation.veh_time
        timing_dict['Environment'] = self.simulation.env_time
        ns = 0 
        for s in self.simulation.sensors:
            timing_dict[s.name] = self.simulation.sensor_times[ns]
            ns = ns + 1
        timing_dict['Total'] = self.total_wall/self.simulation.elapsed_time
        if not self.timing_plot:
            self.timing_plot = TimingPlot(self.master,timing_dict)
        self.timing_plot.Update(self.master,timing_dict)
    def PlotSpeedGraph(self):
        if not self.speed_plot:
            self.speed_plot = SpeedPlot(self.master,self.simulation.vehicle.GetSpeed())
        self.speed_plot.Update(self.master,self.simulation.vehicle.GetSpeed())
    def Update(self):
        # update the state monitors
        self.elapsed_time_ent.delete(0,'end')
        self.elapsed_time_ent.insert(0,self.simulation.elapsed_time)

        #update the sim state, if it's running
        if (self.sim_state=='running'):
            tw0 = time.time()
            self.simulation.free_driving = self.free_drive_var.get()
            if self.free_drive_var.get():
                dc = self.cam.GetDrivingCommand()
                self.simulation.Update(self.dt,throttle=dc.throttle, steering=dc.steering, braking=dc.braking)
            else:
                self.simulation.Update(self.dt)
            if (self.env_window.window):
                self.UpdateEnvironmentParams()
            self.cam.elapsed_since_last = self.cam.elapsed_since_last + self.dt
            
            #update the simulation display window
            if self.cam.elapsed_since_last > (1.0/self.cam.update_rate):
                t0 = time.time()
                if len(self.simulation.sensors)==0:
                    self.simulation.time_to_update_actor = True
                self.UpdateSimCam(self.dt)
                t1 = time.time()
                self.cam_time = (t1-t0)*self.cam.update_rate

            # Update the map if it is being viewed
            if (self.map_displayed_var.get()):
                if self.viewer == None:
                    self.InitMapViewer()
                self.time_since_last_map = self.time_since_last_map + self.dt
                if (self.time_since_last_map>1.0):
                    t2 = time.time()
                    pos = self.simulation.vehicle.GetPosition()
                    self.viewer.AddWaypoints(self.simulation.waypoints.waypoints)
                    self.viewer.AddCircle([pos[0],pos[1]], 2.0)
                    self.viewer.Display(self.simulation.env)
                    #self.viewer.Update(self.simulation.env,self.simulation.vehicle.GetPosition())
                    t3 = time.time()
                    self.map_time = t3-t2
                    self.time_since_last_map = 0.0
            elif not self.map_displayed_var.get():
                if self.viewer:
                    self.InitMapViewer()
            # update the timing window
            if self.show_timing_var.get():
                self.PlotTimeGraph()
            elif self.timing_plot:
                self.timing_plot.CloseWindow()

            #update the speed plot window
            if self.plot_speed_var.get():
                self.PlotSpeedGraph()
            elif self.speed_plot:
                self.speed_plot.CloseWindow()
            # Add to the elapsed wall time
            tw1 = time.time()
            wall_dt = tw1-tw0
            if (wall_dt<self.dt):
                time.sleep(self.dt-wall_dt)
            self.total_wall = self.total_wall+(tw1-tw0)
