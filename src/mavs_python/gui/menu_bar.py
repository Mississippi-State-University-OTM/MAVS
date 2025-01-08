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
from tkinter import messagebox
import webbrowser
from tkinter import Menu
from Tooltip import Tooltip
import mavs_defs as mavs

class MavsMenuBar:
    def AddMenuBar(self):
        menubar = Menu(self.master,foreground='white',background=mavs.maroon)
        #def dummy():
        #    pass
        #simulation menu
        sim_menu = Menu(menubar, tearoff=0)
        sim_menu.add_command(label="Load Simulation", command=self.LoadSimulation)
        sim_menu.add_command(label="Save Simulation", command=self.SaveSim)
        sim_menu.add_command(label="Set Output Directory", command=self.SetSaveDirectory)
        sim_menu.add_command(label="Set Time Step", command=self.GetNewDt)
        sim_menu.add_checkbutton(label='Keyboard Vehicle Control?',onvalue=True, offvalue=False, variable=self.free_drive_var)
        sim_menu.add_separator()
        sim_menu.add_command(label="Exit", command=self.master.destroy)
        menubar.add_cascade(label="Simulation", menu=sim_menu)

        # edit menus
        editmenu = Menu(menubar, tearoff=0)

        edit_menu = Menu(editmenu,tearoff=0)
        edit_menu.add_command(label="Edit Environment", command=self.EditEnvironment)
        edit_menu.add_command(label="Edit Controller", command=self.EditController)
        edit_menu.add_command(label="Edit Waypoints", command=self.EditWaypoints)
        editmenu.add_cascade(label="Edit", menu=edit_menu)

        loadmenu = Menu(editmenu, tearoff=0)
        loadmenu.add_command(label="Load New Scene", command=self.LoadNewScene)
        loadmenu.add_command(label="Load New Vehicle", command=self.LoadNewVehicle)
        loadmenu.add_command(label="Load New Waypoints", command=self.LoadNewWaypoints)
        editmenu.add_cascade(label="Load", menu=loadmenu)

        display_menu = Menu(editmenu, tearoff=0)
        display_menu.add_checkbutton(label='Display Map?',onvalue=True, offvalue=False, variable=self.map_displayed_var)
        display_menu.add_checkbutton(label='Display Vehicle Speed?',onvalue=True, offvalue=False, variable=self.plot_speed_var)
        display_menu.add_checkbutton(label='Show Timing Info?',onvalue=True, offvalue=False, variable=self.show_timing_var)
        editmenu.add_cascade(label="Display", menu=display_menu)

        menubar.add_cascade(label="Edit", menu=editmenu)

        # sensor menus
        def add_lidar():
            self.AddSensor('lidar')
        def add_camera():
            self.AddSensor('camera')
        def add_radar():
            self.AddSensor('radar')
        def add_rtk_gps():
            self.AddSensor('rtk')
        sensormenu = Menu(menubar, tearoff=0)
        add_sensor_menu = Menu(sensormenu,tearoff=0)
        add_sensor_menu.add_command(label="Add Lidar", command=add_lidar)
        add_sensor_menu.add_command(label="Add Radar", command=add_radar)
        add_sensor_menu.add_command(label="Add Camera", command=add_camera)
        add_sensor_menu.add_command(label="Add RTK-GPS", command=add_rtk_gps)
        sensormenu.add_cascade(label="Add Sensor", menu=add_sensor_menu)
        menubar.add_cascade(label="Sensors", menu=sensormenu)

        def gui_help():
            webbrowser.open_new('https://mavs-documentation.readthedocs.io/en/latest/Gui/RunningMavsGUI/')
        def mavs_help():
            webbrowser.open_new('https://mavs-documentation.readthedocs.io/en/latest/')
        def api_help():
            webbrowser.open_new('https://cgoodin.gitlab.io/msu-autonomous-vehicle-simulator/')

        helpmenu = Menu(menubar, tearoff=0)
        helpmenu.add_command(label="GUI Instructions", command=gui_help)
        helpmenu.add_command(label="MAVS Overview", command=mavs_help)
        helpmenu.add_command(label="MAVS C++ API Documentation", command=api_help)
        menubar.add_cascade(label="Help", menu=helpmenu)

        self.master.config(menu = menubar)