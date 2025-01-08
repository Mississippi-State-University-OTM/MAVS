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
import time
import numpy as np
import os.path
import math

# Set the path to the mavs python api, mavs_interface.py
# You will have to change this on your system.
sys.path.append(r'C:\Users\cgoodin\Desktop\vm_shared\shared_repos\mavs\src\mavs_python')

# Load the mavs python modules
import mavs_interface
import mavs_python_paths

import tkinter as tk
from tkinter import *
from tkinter import filedialog
#third party modules
from Tooltip import Tooltip
import mavs_defs as mavs

class MaterialWindow():
    def __init__(self):
        self.window = None
        self.material_viewer = mavs_interface.MavsMaterialViewer()
        meshfile = filedialog.askopenfilename(initialdir=mavs_python_paths.mavs_data_path+'/scenes/meshes', title='Select Mesh File', filetypes=[('Mesh files', '*.obj')])
        self.materials = self.material_viewer.LoadMaterialsFromObj(meshfile)
        self.spectrum_plot = mavs_interface.MavsPlot()
    def EditProperties(self,master):
        if self.window is None:
            self.window = master #Toplevel(master,background=mavs.darkgrey)
            self.window.configure(background=mavs.darkgrey)
            self.window.title("Material Properties")

            self.lb_label = Label(self.window,text="Materials",background=mavs.darkgrey,foreground='white')
            self.lb_label.grid(column=0,row=0)
            self.material_listbox = Listbox(self.window,background=mavs.lightgrey)
            self.material_listbox.grid(column=0,row=1,rowspan=4)
            for m in self.materials:
                self.material_listbox.insert(END,m)
            def select_mat_event(event):
                self.SelectMaterial()
            self.material_listbox.bind('<Double-Button>',select_mat_event)
            self.material_listbox.bind('<Return>',select_mat_event)
            Tooltip(self.material_listbox,text=('Double click a material to render to the sphere'))

            #specular exponent
            self.ns_label = Label(self.window,text='Ns',background=mavs.darkgrey,foreground='white')
            self.ns_label.grid(column=1,row=1)
            self.ns_ent = Entry(self.window,width=5,background=mavs.lightgrey)
            self.ns_ent.grid(column=2,row=1)
            self.ns_ent.insert(0,0.0)
            Tooltip(self.ns_ent,text=('Specular exponent'))

            # specular color
            self.spec_label = Label(self.window,text='Specular',background=mavs.darkgrey,foreground='white')
            self.spec_label.grid(column=1,row=2)
            self.spec_r_ent = Entry(self.window,width=10,background=mavs.lightgrey)
            self.spec_r_ent.grid(column=2,row=2)
            self.spec_g_ent = Entry(self.window,width=10,background=mavs.lightgrey)
            self.spec_g_ent.grid(column=3,row=2)
            self.spec_b_ent = Entry(self.window,width=10,background=mavs.lightgrey)
            self.spec_b_ent.grid(column=4,row=2)
            self.spec_r_ent.insert(0,0.0)
            self.spec_g_ent.insert(0,0.0)
            self.spec_b_ent.insert(0,0.0)
            Tooltip(self.spec_r_ent,text=('Red channel, specular'))
            Tooltip(self.spec_g_ent,text=('Green channel, specular'))
            Tooltip(self.spec_b_ent,text=('Blue channel, specular'))

            # diffuse color
            self.diff_label = Label(self.window,text='Diffuse',background=mavs.darkgrey,foreground='white')
            self.diff_label.grid(column=1,row=3)
            self.diff_r_ent = Entry(self.window,width=10,background=mavs.lightgrey)
            self.diff_r_ent.grid(column=2,row=3)
            self.diff_g_ent = Entry(self.window,width=10,background=mavs.lightgrey)
            self.diff_g_ent.grid(column=3,row=3)
            self.diff_b_ent = Entry(self.window,width=10,background=mavs.lightgrey)
            self.diff_b_ent.grid(column=4,row=3)
            self.diff_r_ent.insert(0,0.0)
            self.diff_g_ent.insert(0,0.0)
            self.diff_b_ent.insert(0,0.0)
            Tooltip(self.diff_r_ent,text=('Red channel, diffuse'))
            Tooltip(self.diff_g_ent,text=('Green channel, diffuse'))
            Tooltip(self.diff_b_ent,text=('Blue channel, diffuse'))

            # RGB calculated from spectrum
            self.wl_label = Label(self.window,text='Spectral Color',background=mavs.darkgrey,foreground='white')
            self.wl_label.grid(column=1,row=4)
            self.wl_r_ent = Entry(self.window,width=10,background=mavs.lightgrey)
            self.wl_r_ent.grid(column=2,row=4)
            self.wl_g_ent = Entry(self.window,width=10,background=mavs.lightgrey)
            self.wl_g_ent.grid(column=3,row=4)
            self.wl_b_ent = Entry(self.window,width=10,background=mavs.lightgrey)
            self.wl_b_ent.grid(column=4,row=4)
            self.wl_r_ent.insert(0,0.0)
            self.wl_g_ent.insert(0,0.0)
            self.wl_b_ent.insert(0,0.0)
            Tooltip(self.wl_r_ent,text=('Red channel, from spectrum'))
            Tooltip(self.wl_g_ent,text=('Green channel, from spectrum'))
            Tooltip(self.wl_b_ent,text=('Blue channel, from spectrum'))

            self.window.protocol('WM_DELETE_WINDOW', self.removewindow)
    def removewindow(self):
        self.window.destroy()
        self.window = None
    def SelectMaterial(self):
        matnum = self.material_listbox.curselection()[0]
        self.material_viewer.SetMaterial(matnum)
        self.ns_ent.delete(0,'end')
        self.ns_ent.insert(0,self.material_viewer.avail_materials[matnum].ns)
        self.spec_r_ent.delete(0,'end')
        self.spec_r_ent.insert(0,self.material_viewer.avail_materials[matnum].ks[0])
        self.spec_g_ent.delete(0,'end')
        self.spec_g_ent.insert(0,self.material_viewer.avail_materials[matnum].ks[1])
        self.spec_b_ent.delete(0,'end')
        self.spec_b_ent.insert(0,self.material_viewer.avail_materials[matnum].ks[2])
        self.diff_r_ent.delete(0,'end')
        self.diff_r_ent.insert(0,self.material_viewer.avail_materials[matnum].kd[0])
        self.diff_g_ent.delete(0,'end')
        self.diff_g_ent.insert(0,self.material_viewer.avail_materials[matnum].kd[1])
        self.diff_b_ent.delete(0,'end')
        self.diff_b_ent.insert(0,self.material_viewer.avail_materials[matnum].kd[2])
        self.PlotSpectrum(self.material_viewer.avail_materials[matnum].refl,matnum)
        
    def PlotSpectrum(self,spec_file,matnum):
        if os.path.exists(spec_file):
            data = np.loadtxt(spec_file)
            wl = data[:,0]
            rho = data[:,1]
            rgb = self.GetSpecRgb(wl,rho)
            self.wl_r_ent.delete(0,'end')
            self.wl_r_ent.insert(0,rgb[0])
            self.wl_g_ent.delete(0,'end')
            self.wl_g_ent.insert(0,rgb[1])
            self.wl_b_ent.delete(0,'end')
            self.wl_b_ent.insert(0,rgb[2])
            self.spectrum_plot.PlotTrajectory(wl,rho)
        else:
            print('Could not load spectrum file '+spec_file)
            sys.stdout.flush()
    def GetSpecRgb(self,wavelengths,reflectance):
        rgb = [0.0,0.0,0.0]
        nr = 0.0
        ng = 0.0
        nb = 0.0
        ## get rgb using CIE response 
        # https://en.wikipedia.org/wiki/CIE_1931_color_space#/media/File:CIE_1931_XYZ_Color_Matching_Functions.svg
        for i in range(0,len(wavelengths)):
            #blue
            xb = wavelengths[i]-0.45 
            nb = nb+1.0
            fac = 1.8*math.exp(-600.0*xb*xb)
            rgb[2] = rgb[2] + fac*reflectance[i]
            #green
            xg =wavelengths[i]-0.55 
            ng = ng+1.0
            fac = 1.0*math.exp(-400.0*xg*xg)
            rgb[1] = rgb[1] + fac*reflectance[i]
            #red
            xr1 = wavelengths[i]-0.605
            xr2 = wavelengths[i]-0.445
            nr = nr+1.0
            fac = 1.1*math.exp(-600.0*xr1*xr1) + 0.35*math.exp(-1640*xr2*xr2)
            rgb[0] = rgb[0] + fac*reflectance[i]
        rgb[0] = rgb[0]/100.0
        rgb[1] = rgb[1]/100.0
        rgb[2] = rgb[2]/100.0
        return rgb



master = Tk()

mat_editor = MaterialWindow()
mat_editor.EditProperties(master)

while True:
    mat_editor.material_viewer.Update()

    master.update_idletasks()
    master.update()