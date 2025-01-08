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
import sys
from tkinter import *
from Tooltip import Tooltip
import mavs_defs as mavs

class EditWaypointsWindow():
    def __init__(self, master, waypoints, python_directory, pos, env):
        sys.path.append(python_directory)
        import mavs_interface

        self.waypoints_window = Toplevel(master,background=mavs.darkgrey)
        self.waypoints_window.title("Edit Waypoints")

        self.listbox = Listbox(self.waypoints_window,background=mavs.lightgrey)
        self.listbox.grid(column=1,row=0,rowspan=6,columnspan=2)

        for wp in waypoints.waypoints:
            self.listbox.insert(END,wp)
        
        viewer = mavs_interface.MavsMapViewer(pos[0]-256.0, pos[1]-256.0, pos[0]+256.0, pos[1]+256.0, 0.5)
        
        def redraw():
            for wp in waypoints.waypoints:
            #self.viewer.AddWaypoints(self.simulation.waypoints.waypoints)
                viewer.AddCircle([wp[0],wp[1]], 0.25)
            viewer.Display(env)

        redraw()
        
        def add_waypoint():
            wp_win = Toplevel(self.waypoints_window,background=mavs.darkgrey)
            wp_win.title("Add Waypoint")
            wpnum = self.listbox.curselection()[0]
            lab_x = Label(wp_win,text='X',background=mavs.darkgrey,foreground='white')
            lab_x.grid(column=0,row=0,sticky='w')
            lab_y = Label(wp_win,text='Y',background=mavs.darkgrey,foreground='white')
            lab_y.grid(column=1,row=0,sticky='w')
            lab_z = Label(wp_win,text='Z',background=mavs.darkgrey,foreground='white')
            lab_z.grid(column=2,row=0,sticky='w')
            ent_x = Entry(wp_win,width=5,background=mavs.lightgrey)
            ent_x.grid(column=0,row=1,sticky='w')
            ent_y = Entry(wp_win,width=5,background=mavs.lightgrey)
            ent_y.grid(column=1,row=1,sticky='w')
            ent_z = Entry(wp_win,width=5,background=mavs.lightgrey)
            ent_z.grid(column=2,row=1,sticky='w')
            ent_x.insert(0,0.0)
            ent_y.insert(0,0.0)
            ent_z.insert(0,0.0)
            Tooltip(ent_x,text=('Enter the X coordinate of the waypoint'))
            Tooltip(ent_y,text=('Enter the Y coordinate of the waypoint'))
            Tooltip(ent_z,text=('Enter the Z coordinate of the waypoint'))
            def finish_wp():
                waypoints.waypoints.insert(wpnum,[float(ent_x.get()),float(ent_y.get()),float(ent_z.get())])
                self.listbox.insert(wpnum,waypoints.waypoints[wpnum])
                wp_win.destroy()
            wp_done = Button(wp_win,text='Done', command=finish_wp, background=mavs.lightgrey)
            wp_done.grid(column=0,row=7,columnspan=4)
            Tooltip(wp_done,text=('Select when done editing the waypoint'))
            self.waypoints_window.wait_window(wp_win)
            redraw()

        self.add_wp_btn = Button(self.waypoints_window,text='Add Waypoint', command=add_waypoint, background=mavs.lightgrey)
        self.add_wp_btn.grid(column=0,row=0,sticky='w')
        Tooltip(self.add_wp_btn,text=('Add a waypoint above the selected.'))

        def EditSelectedWaypoint():
            wp_win = Toplevel(self.waypoints_window,background=mavs.darkgrey)
            wp_win.title("Edit Selected Waypoint")
            wpnum = self.listbox.curselection()[0]
            lab_x = Label(wp_win,text='X',background=mavs.darkgrey,foreground='white')
            lab_x.grid(column=0,row=0,sticky='w')
            lab_y = Label(wp_win,text='Y',background=mavs.darkgrey,foreground='white')
            lab_y.grid(column=1,row=0,sticky='w')
            lab_z = Label(wp_win,text='Z',background=mavs.darkgrey,foreground='white')
            lab_z.grid(column=2,row=0,sticky='w')
            ent_x = Entry(wp_win,width=5,background=mavs.lightgrey)
            ent_x.grid(column=0,row=1,sticky='w')
            ent_y = Entry(wp_win,width=5,background=mavs.lightgrey)
            ent_y.grid(column=1,row=1,sticky='w')
            ent_z = Entry(wp_win,width=5,background=mavs.lightgrey)
            ent_z.grid(column=2,row=1,sticky='w')
            ent_x.insert(0,waypoints.waypoints[wpnum][0])
            ent_y.insert(0,waypoints.waypoints[wpnum][1])
            ent_z.insert(0,waypoints.waypoints[wpnum][2])
            Tooltip(ent_x,text=('Enter the X coordinate of the waypoint'))
            Tooltip(ent_y,text=('Enter the Y coordinate of the waypoint'))
            Tooltip(ent_z,text=('Enter the Z coordinate of the waypoint'))
            def finish_wp():
                waypoints.waypoints[wpnum] = [float(ent_x.get()),float(ent_y.get()),float(ent_z.get())]
                self.listbox.delete(wpnum)
                self.listbox.insert(wpnum,waypoints.waypoints[wpnum])
                wp_win.destroy()
            wp_done = Button(wp_win,text='Done', command=finish_wp, background=mavs.lightgrey)
            wp_done.grid(column=0,row=7,columnspan=4)
            Tooltip(wp_done,text=('Select when done editing selected waypoint'))
            self.waypoints_window.wait_window(wp_win)
            redraw()

        self.edit_wp_btn = Button(self.waypoints_window,text='Edit Waypoint', command=EditSelectedWaypoint, background=mavs.lightgrey)
        self.edit_wp_btn.grid(column=0,row=1,sticky='w')
        Tooltip(self.edit_wp_btn,text=('Edit the selected waypoint.'))

        def delete_waypoint():
            wpnum = self.listbox.curselection()[0]
            self.listbox.delete(wpnum)
            waypoints.waypoints.pop(wpnum)
            redraw()
        self.del_wp_btn = Button(self.waypoints_window,text='Delete Waypoint', command=delete_waypoint, background=mavs.lightgrey)
        self.del_wp_btn.grid(column=0,row=2,sticky='w')
        Tooltip(self.del_wp_btn,text=('Remove the selected waypoint.'))

        def FinishEditing():
            redraw()
            self.waypoints_window.destroy()

        self.finish_button = Button(self.waypoints_window,text='Done', command=FinishEditing, background=mavs.lightgrey)
        self.finish_button.grid(column=0,row=7,columnspan=4)
        Tooltip(self.finish_button,text=('Select when done editing waypoints'))
        master.wait_window(self.waypoints_window)


    
