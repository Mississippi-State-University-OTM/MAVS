import json
import os
import sys
from pathlib import Path
import tkinter as tk
from tkinter import ttk
from tkinter import filedialog
import Tooltip as tt
import webbrowser 

root = tk.Tk()
root.title("MAVS RP3D Vehicle Editor")
root.geometry("600x300")

def SuspensionTab(tab_in,):
    long_offset_label = ttk.Label(tab_in, text='Longitudinal Offset from CG (m)')
    long_offset_label.grid(column=0, row=0,sticky='w')
    long_offset_entry = tk.Entry(tab_in)
    long_offset_entry.grid(column=1, row=0,sticky='w')
    long_offset_entry.insert(0,"168")
    tt.Tooltip(long_offset_label,text=('Growth simulation time step in hours'))
    tt.Tooltip(long_offset_entry,text=('Growth simulation time step in hours'))
    
    track_width_label = ttk.Label(tab_in, text='Track Width (m)')
    track_width_label.grid(column=0, row=1,sticky='w')
    track_width_entry = tk.Entry(tab_in)
    track_width_entry.grid(column=1, row=1,sticky='w')
    track_width_entry.insert(0,"168")
    tt.Tooltip(track_width_label,text=('Growth simulation time step in hours'))
    tt.Tooltip(track_width_entry,text=('Growth simulation time step in hours'))
    
    spring_const_label = ttk.Label(tab_in, text='Spring Constant (N/m)')
    spring_const_label.grid(column=0, row=2, sticky='w')
    spring_const_entry = tk.Entry(tab_in)
    spring_const_entry.grid(column=1, row=2,sticky='w')
    spring_const_entry.insert(0,"168")
    tt.Tooltip(spring_const_label,text=('Growth simulation time step in hours'))
    tt.Tooltip(spring_const_entry,text=('Growth simulation time step in hours'))
    
    damp_const_label = ttk.Label(tab_in, text='Damping Constant (Ns/m)')
    damp_const_label.grid(column=0, row=3, sticky='w')
    damp_const_entry = tk.Entry(tab_in)
    damp_const_entry.grid(column=1, row=3,sticky='w')
    damp_const_entry.insert(0,"168")
    tt.Tooltip(damp_const_label,text=('Suspension spring damping constant'))
    tt.Tooltip(damp_const_entry,text=('Suspension spring damping constant'))
    
    spring_len_label = ttk.Label(tab_in, text='Spring Length (m)')
    spring_len_label.grid(column=0, row=4, sticky='w')
    spring_len_entry = tk.Entry(tab_in)
    spring_len_entry.grid(column=1, row=4,sticky='w')
    spring_len_entry.insert(0,"168")
    tt.Tooltip(spring_len_label,text=('Growth simulation time step in hours'))
    tt.Tooltip(spring_len_entry,text=('Growth simulation time step in hours'))
    
    steer_angle_label = ttk.Label(tab_in, text='Maximum Steer Angle (deg)')
    steer_angle_label.grid(column=0, row=5, sticky='w')
    steer_angle_entry = tk.Entry(tab_in)
    steer_angle_entry.grid(column=1, row=5,sticky='w')
    steer_angle_entry.insert(0,"168")
    tt.Tooltip(steer_angle_label,text=('Growth simulation time step in hours'))
    tt.Tooltip(steer_angle_entry,text=('Growth simulation time step in hours'))
    
    unsprung_label = ttk.Label(tab_in, text='Unsprung Mass (kg)')
    unsprung_label.grid(column=0, row=5, sticky='w')
    unsprung_entry = tk.Entry(tab_in)
    unsprung_entry.grid(column=1, row=5,sticky='w')
    unsprung_entry.insert(0,"168")
    tt.Tooltip(unsprung_label,text=('Growth simulation time step in hours'))
    tt.Tooltip(unsprung_entry,text=('Growth simulation time step in hours'))
    
    steered_var = tk.IntVar()
    steered_var.set(True)
    steered_checkbox = tk.Checkbutton(tab_in, text="Steered?", variable=steered_var)
    steered_checkbox.grid(column=1,row=6,sticky='w')
    tt.Tooltip(steered_checkbox,text=('Stop simlation after max number of years?'))
    
    powered_var = tk.IntVar()
    powered_var.set(True)
    powered_checkbox = tk.Checkbutton(tab_in, text="Powered?", variable=powered_var)
    powered_checkbox.grid(column=1,row=7,sticky='w')
    tt.Tooltip(powered_checkbox,text=('Stop simlation after max number of years?'))
    
def TireTab(tab_in,):
    spring_cons_label = ttk.Label(tab_in, text='Spring Constant (N/m)')
    spring_cons_label.grid(column=0, row=0,sticky='w')
    spring_cons_entry = tk.Entry(tab_in)
    spring_cons_entry.grid(column=1, row=0,sticky='w')
    spring_cons_entry.insert(0,"168")
    tt.Tooltip(spring_cons_label,text=('Growth simulation time step in hours'))
    tt.Tooltip(spring_cons_entry,text=('Growth simulation time step in hours'))
    
    damp_const_label = ttk.Label(tab_in, text='Damping Constant (Ns/m)')
    damp_const_label.grid(column=0, row=1, sticky='w')
    damp_const_entry = tk.Entry(tab_in)
    damp_const_entry.grid(column=1, row=1,sticky='w')
    damp_const_entry.insert(0,"168")
    tt.Tooltip(damp_const_label,text=('Tire damping constant'))
    tt.Tooltip(damp_const_entry,text=('Tire damping constant'))
    
    tire_rad_label = ttk.Label(tab_in, text='Tire Radius (m)')
    tire_rad_label.grid(column=0, row=2, sticky='w')
    tire_rad_entry = tk.Entry(tab_in)
    tire_rad_entry.grid(column=1, row=2,sticky='w')
    tire_rad_entry.insert(0,"168")
    tt.Tooltip(tire_rad_label,text=('Tire damping constant'))
    tt.Tooltip(tire_rad_entry,text=('Tire damping constant'))
    
    tire_wid_label = ttk.Label(tab_in, text='Tire Width (m)')
    tire_wid_label.grid(column=0, row=3, sticky='w')
    tire_wid_entry = tk.Entry(tab_in)
    tire_wid_entry.grid(column=1, row=3,sticky='w')
    tire_wid_entry.insert(0,"168")
    tt.Tooltip(tire_wid_label,text=('Tire damping constant'))
    tt.Tooltip(tire_wid_entry,text=('Tire damping constant'))
    
    tire_sh_label = ttk.Label(tab_in, text='Section Height (m)')
    tire_sh_label.grid(column=0, row=4, sticky='w')
    tire_sh_entry = tk.Entry(tab_in)
    tire_sh_entry.grid(column=1, row=4,sticky='w')
    tire_sh_entry.insert(0,"168")
    tt.Tooltip(tire_sh_label,text=('Tire damping constant'))
    tt.Tooltip(tire_sh_entry,text=('Tire damping constant'))
    
    hsca_label = ttk.Label(tab_in, text='High Slip Crossover Angle (deg)')
    hsca_label.grid(column=0, row=5, sticky='w')
    hsca_entry = tk.Entry(tab_in)
    hsca_entry.grid(column=1, row=5,sticky='w')
    hsca_entry.insert(0,"168")
    tt.Tooltip(hsca_label,text=('Tire damping constant'))
    tt.Tooltip(hsca_entry,text=('Tire damping constant'))
    
    vf_label = ttk.Label(tab_in, text='Viscous Friction Coefficient')
    vf_label.grid(column=0, row=5, sticky='w')
    vf_entry = tk.Entry(tab_in)
    vf_entry.grid(column=1, row=5,sticky='w')
    vf_entry.insert(0,"168")
    tt.Tooltip(vf_label,text=('Tire damping constant'))
    tt.Tooltip(vf_entry,text=('Tire damping constant'))

# create the tab panes
tabControl = ttk.Notebook(root)
tab1 = ttk.Frame(tabControl)
tab2 = ttk.Frame(tabControl)
tab3 = ttk.Frame(tabControl)
tab4 = ttk.Frame(tabControl)
tab5 = ttk.Frame(tabControl)
tabControl.add(tab1, text='Chassis')
tabControl.add(tab2, text='Powertrain')
tabControl.add(tab3, text='Suspension')
tabControl.add(tab4, text='Tires')
tabControl.add(tab5, text='Meshes')
tabControl.grid(row=0,column=0,rowspan=7,columnspan=6,sticky='nw')

# tab 1, Chassis properties ----------------------------------------------------------------------#
sprungmass_label = ttk.Label(tab1, text='Sprung Mass (kg)')
sprungmass_label.grid(column=0, row=0,sticky='w')
sprungmass_entry = tk.Entry(tab1)
sprungmass_entry.grid(column=1, row=0,sticky='w')
sprungmass_entry.insert(0,"300")
tt.Tooltip(sprungmass_label,text=('Maximum length of the simulation in years'))
tt.Tooltip(sprungmass_entry,text=('Maximum length of the simulation in years'))

cg_vert_offset_label = ttk.Label(tab1, text='CG Vertical Offset (m)')
cg_vert_offset_label.grid(column=0, row=1,sticky='w')
cg_vert_offset_entry = tk.Entry(tab1)
cg_vert_offset_entry.grid(column=1, row=1,sticky='w')
cg_vert_offset_entry.insert(0,"168")
tt.Tooltip(cg_vert_offset_label,text=('Growth simulation time step in hours'))
tt.Tooltip(cg_vert_offset_entry,text=('Growth simulation time step in hours'))

cg_lat_offset_label = ttk.Label(tab1, text='CG Lateral Offset (m)')
cg_lat_offset_label.grid(column=0, row=2,sticky='w')
cg_lat_offset_entry = tk.Entry(tab1)
cg_lat_offset_entry.grid(column=1, row=2,sticky='w')
cg_lat_offset_entry.insert(0,"168")
tt.Tooltip(cg_lat_offset_label,text=('Growth simulation time step in hours'))
tt.Tooltip(cg_lat_offset_entry,text=('Growth simulation time step in hours'))

chass_dim_label = ttk.Label(tab1, text='Chassis Dimensions (m)')
chass_dim_label.grid(column=0, row=3,sticky='w')
chass_x_entry = tk.Entry(tab1)
chass_x_entry.grid(column=1, row=3,sticky='w')
chass_x_entry.insert(0,"0")
chass_y_entry = tk.Entry(tab1)
chass_y_entry.grid(column=2, row=3,sticky='w')
chass_y_entry.insert(0,"0")
chass_z_entry = tk.Entry(tab1)
chass_z_entry.grid(column=3, row=3,sticky='w')
chass_z_entry.insert(0,"0")
tt.Tooltip(chass_dim_label,text=('Number of years to skip growth at the beginning (Usually 0)'))
tt.Tooltip(chass_x_entry,text=('Number of years to skip growth at the beginning (Usually 0)'))

drag_coeff_label = ttk.Label(tab1, text='Drag Coefficient')
drag_coeff_label.grid(column=0, row=4,sticky='w')
drag_coeff_entry = tk.Entry(tab1)
drag_coeff_entry.grid(column=1, row=4,sticky='w')
drag_coeff_entry.insert(0,"168")
tt.Tooltip(drag_coeff_label,text=('Growth simulation time step in hours'))
tt.Tooltip(drag_coeff_entry,text=('Growth simulation time step in hours'))

# Get desired number of axles
def add_tabs():
    new_tab = ttk.Frame(susp_notebook)
    new_tire_tab = ttk.Frame(tire_notebook)
    susp_notebook.add(new_tab, text=f"Axle {1+susp_notebook.index('end')}")
    tire_notebook.add(new_tire_tab, text=f"Tire, Axle {1+susp_notebook.index('end')}")
    SuspensionTab(new_tab)
    TireTab(new_tire_tab)

def remove_tabs():
    if susp_notebook.index('end') > 2:  # Prevent removing the last 2 tabs
        susp_notebook.forget(susp_notebook.select())
    if tire_notebook.index('end') > 2:  # Prevent removing the last 2 tabs
        tire_notebook.forget(tire_notebook.select())
        
def num_axles_changed(*args):
    if (susp_notebook.index('end')<int(num_axle_var.get())):
        num_to_add = int(num_axle_var.get()) - susp_notebook.index('end')
        for i in range(num_to_add):
            add_tabs()
    elif (susp_notebook.index('end')>int(num_axle_var.get())):
        num_to_rmv = susp_notebook.index('end') - int(num_axle_var.get())
        for i in range(num_to_rmv):
            remove_tabs()
            
num_axle_label = ttk.Label(tab1, text='Number of Axles')
num_axle_label.grid(column=0, row=5,sticky='w')
num_axle_var = tk.StringVar(root)
num_axle_var.set("2")  # Default value
num_axle_options = ["2", "3", "4", "5", "6", "7", "8"]
num_axle_dropdown = tk.OptionMenu(tab1, num_axle_var, *num_axle_options, command=num_axles_changed)
num_axle_dropdown.grid(row=5,column=1,columnspan=1,sticky='ew')
tt.Tooltip(num_axle_label,text=("Set the logging-level of the simulation. \n   DEBUG will display images to the screen. \n    INFO will save images to the output directory."))
tt.Tooltip(num_axle_dropdown,text=("Set the logging-level of the simulation. \n    DEBUG will display images to the screen. \n     INFO will save images to the output directory."))
#-------------------------------------------------------------------------------------------------#

# tab 2, powertrain properties --------------------------------------------------------------------#
fdr_label = ttk.Label(tab2, text='Final Drive Ratio')
fdr_label.grid(column=0, row=0,sticky='w')
fdr_entry = tk.Entry(tab2)
fdr_entry.grid(column=1, row=0,sticky='w')
fdr_entry.insert(0,"3.3")
tt.Tooltip(fdr_label,text=('File that lists all the species present in the ecosystem. Must be located in the data/ecosystem directory.'))
tt.Tooltip(fdr_entry,text=('File that lists all the species present in the ecosystem. Must be located in the data/ecosystem directory.'))

max_torque_label = ttk.Label(tab2, text='Max Engine Torque')
max_torque_label.grid(column=0, row=1,sticky='w')
max_torque_entry = tk.Entry(tab2)
max_torque_entry.grid(column=1, row=1,sticky='w')
max_torque_entry.insert(0,"3.3")
tt.Tooltip(max_torque_label,text=('File that lists all the species present in the ecosystem. Must be located in the data/ecosystem directory.'))
tt.Tooltip(max_torque_entry,text=('File that lists all the species present in the ecosystem. Must be located in the data/ecosystem directory.'))

max_rpm_label = ttk.Label(tab2, text='Max Engine Rpm')
max_rpm_label.grid(column=0, row=2,sticky='w')
max_rpm_entry = tk.Entry(tab2)
max_rpm_entry.grid(column=1, row=2,sticky='w')
max_rpm_entry.insert(0,"3.3")
tt.Tooltip(max_rpm_label,text=('File that lists all the species present in the ecosystem. Must be located in the data/ecosystem directory.'))
tt.Tooltip(max_rpm_entry,text=('File that lists all the species present in the ecosystem. Must be located in the data/ecosystem directory.'))

brake_torque_label = ttk.Label(tab2, text='Max Braking Torque')
brake_torque_label.grid(column=0, row=3,sticky='w')
brake_torque_entry = tk.Entry(tab2)
brake_torque_entry.grid(column=1, row=3,sticky='w')
brake_torque_entry.insert(0,"3.3")
tt.Tooltip(brake_torque_label,text=('File that lists all the species present in the ecosystem. Must be located in the data/ecosystem directory.'))
tt.Tooltip(brake_torque_entry,text=('File that lists all the species present in the ecosystem. Must be located in the data/ecosystem directory.'))

idle_rpm_label = ttk.Label(tab2, text='Max Engine Rpm')
idle_rpm_label.grid(column=0, row=4,sticky='w')
idle_rpm_entry = tk.Entry(tab2)
idle_rpm_entry.grid(column=1, row=4,sticky='w')
idle_rpm_entry.insert(0,"3.3")
tt.Tooltip(idle_rpm_label,text=('File that lists all the species present in the ecosystem. Must be located in the data/ecosystem directory.'))
tt.Tooltip(idle_rpm_entry,text=('File that lists all the species present in the ecosystem. Must be located in the data/ecosystem directory.'))
#-------------------------------------------------------------------------------------------------#

# tab 3, suspension properties ----------------------------------------------------------------------#
susp_notebook = ttk.Notebook(tab3)
susp1 = ttk.Frame(susp_notebook)
susp2 = ttk.Frame(susp_notebook)
susp_notebook.add(susp1, text='Axle 1')
susp_notebook.add(susp2, text='Axle 2')
SuspensionTab(susp1)
SuspensionTab(susp2)
susp_notebook.grid(row=0,column=0,sticky='nw')
#-------------------------------------------------------------------------------------------------#

# tab 4, tire properties ----------------------------------------------------------------#
tire_notebook = ttk.Notebook(tab4)
tire1 = ttk.Frame(tire_notebook)
tire2 = ttk.Frame(tire_notebook)
tire_notebook.add(tire1, text='Tire, Axle 1')
tire_notebook.add(tire2, text='Tire, Axle 2')
TireTab(tire1)
TireTab(tire2)
tire_notebook.grid(row=0,column=0,sticky='nw')

#-------------------------------------------------------------------------------------------------#

# tab 5, meshes -----------------------------------------------------------------#
def TireMeshCallBack():
   home_dir = os.environ.get('HOME')
   tm_file = tk.filedialog.askopenfilename(initialdir = home_dir)
   tm_file_entry.delete(0, tk.END)
   tm_file_entry.insert(0,(os.path.basename(tm_file)))
tm_file_label = ttk.Label(tab5, text='Tire Mesh')
tm_file_label.grid(column=0, row=0,sticky='w')
tm_file_entry = tk.Entry(tab5)
tm_file_entry.grid(column=1, row=0,sticky='w')
tm_file_entry.insert(0,"1kHM.png")
tm_file_select_button = tk.Button(tab5, text ="Select Tire Mesh File", command = TireMeshCallBack)
tm_file_select_button.grid(row=0,column=2,columnspan=1,sticky='ew')
tt.Tooltip(tm_file_label,text=('Heightmap file (png). Must be located in the data/heightmaps directory.'))
tt.Tooltip(tm_file_entry,text=('Heightmap file (png. Must be located in the data/masks directory.'))
tt.Tooltip(tm_file_select_button,text=('Select existing heightmap from database. Must be located in the data/heightmaps directory.'))

def VehicleMeshCallBack():
   home_dir = os.environ.get('HOME')
   vm_file = tk.filedialog.askopenfilename(initialdir = home_dir)
   vm_file_entry.delete(0, tk.END)
   vm_file_entry.insert(0,(os.path.basename(vm_file)))
vm_file_label = ttk.Label(tab5, text='Vehicle Mesh')
vm_file_label.grid(column=0, row=1,sticky='w')
vm_file_entry = tk.Entry(tab5)
vm_file_entry.grid(column=1, row=1,sticky='w')
vm_file_entry.insert(0,"1kHM.png")
vm_file_select_button = tk.Button(tab5, text ="Select Vehicle Mesh File", command = VehicleMeshCallBack)
vm_file_select_button.grid(row=1,column=2,columnspan=1,sticky='ew')
tt.Tooltip(vm_file_label,text=('Heightmap file (png). Must be located in the data/heightmaps directory.'))
tt.Tooltip(vm_file_entry,text=('Heightmap file (png. Must be located in the data/masks directory.'))
tt.Tooltip(vm_file_select_button,text=('Select existing heightmap from database. Must be located in the data/heightmaps directory.'))
#-------------------------------------------------------------------------------------------------#
    
def ExportFile():
    data={}
    eco_data = {}
    eco_data["Ecosystem File"] = ecofile_entry.get()
    eco_data["Species File"] = species_file_entry.get()
    eco_data["Max Stem Density (1/m2)"] = float(maxdens_entry.get())
    eco_data["Seeds Per Plant"] = int(seeds_entry.get())
    eco_data["Init Number Plants"] = int(init_entry.get()) 
    data["Ecosystem"] = eco_data
    sim_data = {}
    sim_data["Sim Length (years)"] = int(simlen_entry.get())
    sim_data["Time Step (hours)"] = int(simstep_entry.get())
    sim_data["Time Bound"] = bool(timebound_var.get())
    sim_data["Initial Year Skip"] = int(yearskip_entry.get())
    sim_data["Basal Area Start, Stop, Step"] = [int(target_ba_entry_lo.get()), int(target_ba_entry_hi.get()), int(target_ba_entry_step.get())]
    data["Simulation Control"] = sim_data
    moist_data = {}
    moist_data["Lower Left Corner (ENU)"] = [float(llc_x_entry.get()), float(llc_y_entry.get())]
    moist_data["Width (m)"] = float(size_x_entry.get())
    moist_data["Length (m)"] = float(size_y_entry.get())
    moist_data["Resolution (m)"] = float(res_entry.get())
    moist_data["Time Step (days)"] = int(timestep_entry.get())
    data["Moisture Grid"] = moist_data
    surf_data = {}
    surf_data["Heightmap"] = hm_file_entry.get()
    surf_data["Resolution (m)"] = float(surf_res_entry.get())
    surf_data["Low Freq Mag (m)"] = float(surf_lfm_entry.get())
    surf_data["Low Freq Length (m)"] = float(surf_lfw_entry.get())
    surf_data["High Freq Mag (m)"] = float(surf_hfm_entry.get())
    surf_data["High Freq Length (m)"] = float(surf_hfw_entry.get())
    data["Surface Mesh"] = surf_data
    mask_data = {}
    mask_data["Road Mask"] = mask_file_entry.get()
    mask_data["Lower Left (ENU)"] = [float(mask_llc_x_entry.get()), float(mask_llc_y_entry.get())]
    mask_data["Pixel Dim (m)"] = float(mask_res_entry.get())    
    data["Masks"] = mask_data
    trail_data = {}
    trail_data["Trail Width (m)"] = float(trail_width_entry.get())
    trail_data["Wheelbase (m)"] = float(track_width_entry.get())
    trail_data["Track Width (m)"] = float(rut_width_entry.get())
    trail_data["Path Type"] = path_type_entry.get()  
    data["Trail"] = trail_data
    weather_data = {}
    weather_data["Rain Per Year (mm)"] = float(rain_entry.get())
    weather_data["Rain Days Per Year"] = int(rain_days_entry.get())            
    weather_data["Temperature (C)"] = float(temperature_entry.get())   
    data["Weather"] = weather_data
    
    file_out = tk.filedialog.asksaveasfilename(initialdir="./inputs", filetypes=[("JSON","*.json")])
    with open(file_out, 'w') as fout:
        json_dumps_str = json.dumps(data, indent=4)
        print(json_dumps_str, file=fout)
    return file_out

def ResetEntry(entry_field, new_string):
    entry_field.delete(0, tk.END)
    entry_field.insert(0,str(new_string))
    
def ImportFile():
   file = tk.filedialog.askopenfilename(initialdir="./inputs", filetypes=[("JSON","*.json")])
   f = open(file)
   data = json.load(f)
   f.close()
   
   if "Ecosystem" in data:
        if "Ecosystem File" in data["Ecosystem"]:
            ResetEntry(ecofile_entry, data["Ecosystem"]["Ecosystem File"])
        if "Species File" in data["Ecosystem"]:
            ResetEntry(species_file_entry, data["Ecosystem"]["Species File"])
        if "Max Stem Density (1/m2)" in data["Ecosystem"]:
            ResetEntry(maxdens_entry, data["Ecosystem"]["Max Stem Density (1/m2)"])
        if "Seeds Per Plant" in data["Ecosystem"]:
            ResetEntry(seeds_entry, data["Ecosystem"]["Seeds Per Plant"])
        if "Init Number Plants" in data["Ecosystem"]:
            ResetEntry(init_entry, data["Ecosystem"]["Init Number Plants"])
                        
   if "Simulation Control" in data:
        if "Sim Length (years)" in data["Simulation Control"]:
            ResetEntry(simlen_entry, data["Simulation Control"]["Sim Length (years)"])
        if "Time Step (hours)" in data["Simulation Control"]:
            ResetEntry(simstep_entry, data["Simulation Control"]["Time Step (hours)"])
        if "Time Bound" in data["Simulation Control"]:
            timebound_var.set(data["Simulation Control"]["Time Bound"])
        if "Initial Year Skip" in data["Simulation Control"]:
            ResetEntry(yearskip_entry, data["Simulation Control"]["Initial Year Skip"])
        if "Basal Area Start, Stop, Step" in data["Simulation Control"]:
            ResetEntry(target_ba_entry_lo, data["Simulation Control"]["Basal Area Start, Stop, Step"][0])
            ResetEntry(target_ba_entry_hi, data["Simulation Control"]["Basal Area Start, Stop, Step"][1])
            ResetEntry(target_ba_entry_step, data["Simulation Control"]["Basal Area Start, Stop, Step"][2])
        
   if "Moisture Grid" in data:
        if "Lower Left Corner (ENU)" in data["Moisture Grid"]:
            ResetEntry(llc_x_entry, data["Moisture Grid"]["Lower Left Corner (ENU)"][0])
            ResetEntry(llc_y_entry, data["Moisture Grid"]["Lower Left Corner (ENU)"][1])
        if "Width (m)" in data["Moisture Grid"]:
            ResetEntry(size_x_entry, data["Moisture Grid"]["Width (m)"])
        if "Length (m)" in data["Moisture Grid"]:
            ResetEntry(size_y_entry, data["Moisture Grid"]["Length (m)"])
        if "Resolution (m)" in data["Moisture Grid"]:
            ResetEntry(res_entry, data["Moisture Grid"]["Resolution (m)"])
        if "Time Step (days)" in data["Moisture Grid"]:
            ResetEntry(timestep_entry, data["Moisture Grid"]["Time Step (days)"])
                
   if "Surface Mesh" in data:
        if "Heightmap" in data["Surface Mesh"]:
            ResetEntry(hm_file_entry, data["Surface Mesh"]["Heightmap"] )
        if "Resolution (m)" in data["Surface Mesh"]:
            ResetEntry(surf_res_entry, data["Surface Mesh"]["Resolution (m)"])
        if "Low Freq Mag (m)" in data["Surface Mesh"]:
            ResetEntry(surf_lfm_entry, data["Surface Mesh"]["Low Freq Mag (m)"])
        if "Low Freq Length (m)" in data["Surface Mesh"]:
            ResetEntry(surf_lfw_entry, data["Surface Mesh"]["Low Freq Length (m)"])
        if "High Freq Mag (m)" in data["Surface Mesh"]:
            ResetEntry(surf_hfm_entry, data["Surface Mesh"]["High Freq Mag (m)"])
        if "High Freq Length (m)" in data["Surface Mesh"]:
            ResetEntry(surf_hfw_entry, data["Surface Mesh"]["High Freq Length (m)"])
        
   if "Masks" in data:
        if "Road Mask" in data["Masks"]:
            ResetEntry(mask_file_entry, data["Masks"]["Road Mask"])
        if "Lower Left (ENU)" in data["Masks"]:
            ResetEntry(mask_llc_x_entry, data["Masks"]["Lower Left (ENU)"][0])
            ResetEntry(mask_llc_y_entry, data["Masks"]["Lower Left (ENU)"][1])
        if "Pixel Dim (m)" in data["Masks"]:
            ResetEntry(mask_res_entry, data["Masks"]["Pixel Dim (m)"])
            
   if "Trail" in data:
        if "Trail Width (m)" in data["Trail"]:
            ResetEntry(trail_width_entry, data["Trail"]["Trail Width (m)"])
        if "Wheelbase (m)" in data["Trail"]:
            ResetEntry(track_width_entry, data["Trail"]["Wheelbase (m)"])
        if "Track Width (m)" in data["Trail"]:
            ResetEntry(rut_width_entry, data["Trail"]["Track Width (m)"] )
        if "Path Type" in data["Trail"]:
            ResetEntry(path_type_entry, data["Trail"]["Path Type"])
           
   if "Weather" in data:
        if "Rain Per Year (mm)" in data["Weather"]:
            ResetEntry(rain_entry, data["Weather"]["Rain Per Year (mm)"])
        if "Rain Days Per Year" in data["Weather"]:
            ResetEntry(rain_days_entry, data["Weather"]["Rain Days Per Year"])            
        if "Temperature (C)" in data["Weather"]:
            ResetEntry(temperature_entry, data["Weather"]["Temperature (C)"])
#--------END OF IMPORT EXISTING  FILE ----------------------------------------------------------#
   

menubar = tk.Menu(root)
filemenu = tk.Menu(menubar, tearoff=0)
filemenu.add_command(label="Import Vehicle File", command=ImportFile)
filemenu.add_command(label="Export Vehicle File", command=ExportFile)
filemenu.add_separator()
filemenu.add_command(label="Exit", command=root.quit)
menubar.add_cascade(label="File", menu=filemenu)

def DisplayAbout():
    tk.messagebox.showinfo("About MAVS.",  "MSU Autonomous Vehicle Simulator\n\nMAVS\n\nDeveloped by Mississippi State University\nCenter for Advanced Vehicular Systems\n\nContact: Chris Goodin (cgoodin@cavs.msstate.edu)") 
def ViewDocs():
    webbrowser.open_new_tab("https://mississippi-state-university-otm.github.io/MAVS/docs/Vehicles/MavsVehicles.html")

helpmenu = tk.Menu(menubar, tearoff=0)
helpmenu.add_command(label="About", command=DisplayAbout)
helpmenu.add_command(label="View Documenation", command=ViewDocs)
menubar.add_cascade(label="Help", menu=helpmenu)

root.config(menu=menubar)

root.mainloop()