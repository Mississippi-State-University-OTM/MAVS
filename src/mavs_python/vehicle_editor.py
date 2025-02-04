import json
import os
import sys
from pathlib import Path
import tkinter as tk
from tkinter import ttk
from tkinter import filedialog
import gui.Tooltip as tt
import webbrowser 
# Load the mavs python modules
import mavs_interface as mavs
import mavs_python_paths
from gui.vehicle_tabs import SuspensionTab, TireTab, VisMeshTab
import matplotlib.pyplot as plt
import matplotlib.patches as patches

mavs_data_path = mavs_python_paths.mavs_data_path

root = tk.Tk()
root.title("MAVS RP3D Vehicle Editor")
root.geometry("600x300")

# recc spring constant = 90* vehicle mass
# recc damping = sqrt(k*m) = 9.5*vehicle_mass

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
tt.Tooltip(sprungmass_label,text=('Chassis and frame mass (kg)'))
tt.Tooltip(sprungmass_entry,text=('Chassis and frame mass (kg)'))

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

tire_tabs = []
axle_tabs = []
# Get desired number of axles
def add_tabs():
    new_tab = SuspensionTab(susp_notebook)
    new_tire_tab = TireTab(tire_notebook)
    susp_notebook.add(new_tab.tab, text=f"Axle {1+susp_notebook.index('end')}")
    tire_notebook.add(new_tire_tab.tab, text=f"Tire, Axle {1+tire_notebook.index('end')}")
    axle_tabs.append(new_tab)
    tire_tabs.append(new_tire_tab)

def remove_tabs():
    if susp_notebook.index('end') > 2:  # Prevent removing the last 2 tabs
        susp_notebook.forget(susp_notebook.index('end')-1)
        axle_tabs.pop()
    if tire_notebook.index('end') > 2:  # Prevent removing the last 2 tabs
        tire_notebook.forget(tire_notebook.index('end')-1)
        tire_tabs.pop()
        
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

# tabs 3 & 4, suspension and tire properties ----------------------------------------------------------------------#
susp_notebook = ttk.Notebook(tab3)
susp_notebook.grid(row=0,column=0,sticky='nw')
tire_notebook = ttk.Notebook(tab4)
tire_notebook.grid(row=0,column=0,sticky='nw')
add_tabs()
add_tabs()
#-------------------------------------------------------------------------------------------------#

# tab 5, meshes -----------------------------------------------------------------#
mesh_notebook = ttk.Notebook(tab5)
mesh_notebook.grid(row=0,column=0,sticky='nw')
veh_mesh_tab = VisMeshTab(mesh_notebook, mavs_data_path)
mesh_notebook.add(veh_mesh_tab.tab, text="Vehicle Mesh")
tire_mesh_tab = VisMeshTab(mesh_notebook, mavs_data_path)
mesh_notebook.add(tire_mesh_tab.tab, text="Tire Mesh")
#-------------------------------------------------------------------------------------------------#
    
def WriteDataDict():
    data={}
    chassis_data = {}
    chassis_data["Sprung Mass"] = float(sprungmass_entry.get())
    chassis_data["CG Offset"] = float(cg_vert_offset_entry.get())
    chassis_data["CG Lateral Offset"] = float(cg_lat_offset_entry.get())
    chassis_data["Dimensions"] = [float(chass_x_entry.get()), float(chass_y_entry.get()), float(chass_z_entry.get())]
    chassis_data["Drag Coefficient"] = float(drag_coeff_entry.get())
    data["Chassis"] = chassis_data
    
    powertrain_data = {}
    powertrain_data["Final Drive Ratio"] = float(fdr_entry.get())
    powertrain_data["Max Engine Torque"] = float(max_torque_entry.get())
    powertrain_data["Max Engine Rpm"] = float(max_rpm_entry.get())
    powertrain_data["Max Braking Torque"] = float(brake_torque_entry.get())
    powertrain_data["Idle Rpm"] = float(idle_rpm_entry.get())
    data["Powertrain"] = powertrain_data
   
    axles =[]
    for i in range(int(num_axle_var.get())):
        ax_data = {}
        ax_data["Longitudinal Offset"] = float(axle_tabs[i].long_offset_entry.get())
        ax_data["Track Width"] = float(axle_tabs[i].track_width_entry.get())
        ax_data["Spring Constant"] = float(axle_tabs[i].spring_const_entry.get())
        ax_data["Spring Length"] = float(axle_tabs[i].spring_len_entry.get())
        ax_data["Max Steer Angle"] = float(axle_tabs[i].steer_angle_entry.get())
        ax_data["Unsprung Mass"] = float(axle_tabs[i].unsprung_entry.get())
        ax_data["Damping Constant"] = float(axle_tabs[i].damp_const_entry.get())
        ax_data["Steered"] = bool(axle_tabs[i].steered_var.get())
        ax_data["Powered"] = bool(axle_tabs[i].powered_var.get())
        tire_data = {}
        tire_data["Spring Constant"] = float(tire_tabs[i].spring_cons_entry.get())
        tire_data["Damping Constant"] = float(tire_tabs[i].damp_const_entry.get())
        tire_data["Radius"] = float(tire_tabs[i].tire_rad_entry.get())
        tire_data["Width"] = float(tire_tabs[i].tire_wid_entry.get())
        tire_data["Section Height"] = float(tire_tabs[i].tire_sh_entry.get())
        tire_data["High Slip Crossover Angle"] = float(tire_tabs[i].hsca_entry.get())
        tire_data["Viscous Friction Coefficient"] = float(tire_tabs[i].vf_entry.get())
        ax_data["Tire"] = tire_data
        axles.append(ax_data)
    data["Axles"] = axles
    
    mesh_data = {}
    mesh_data["File"] = veh_mesh_tab.file_entry.get()
    mesh_data["Rotate Y to Z"] = bool(veh_mesh_tab.rot_y_to_z_var.get())
    mesh_data["Rotate X to Y"] = bool(veh_mesh_tab.rot_x_to_y_var.get())
    mesh_data["Rotate Y to X"] = bool(veh_mesh_tab.rot_y_to_x_var.get())
    mesh_data["Offset"] = [float(veh_mesh_tab.off_x_entry.get()),float(veh_mesh_tab.off_y_entry.get()), float(veh_mesh_tab.off_z_entry.get())]
    mesh_data["Scale"] = [float(veh_mesh_tab.scale_x_entry.get()),float(veh_mesh_tab.scale_y_entry.get()), float(veh_mesh_tab.scale_z_entry.get())]
    data["Mesh"] = mesh_data
    
    tire_mesh_data = {}
    tire_mesh_data["File"] = tire_mesh_tab.file_entry.get()
    tire_mesh_data["Rotate Y to Z"] = bool(tire_mesh_tab.rot_y_to_z_var.get())
    tire_mesh_data["Rotate X to Y"] = bool(tire_mesh_tab.rot_x_to_y_var.get())
    tire_mesh_data["Rotate Y to X"] = bool(tire_mesh_tab.rot_y_to_x_var.get())
    tire_mesh_data["Offset"] = [float(tire_mesh_tab.off_x_entry.get()),float(tire_mesh_tab.off_y_entry.get()), float(tire_mesh_tab.off_z_entry.get())]
    tire_mesh_data["Scale"] = [float(tire_mesh_tab.scale_x_entry.get()),float(tire_mesh_tab.scale_y_entry.get()), float(tire_mesh_tab.scale_z_entry.get())]
    data["Tire Mesh"] = tire_mesh_data

    init_pose_data = {}
    init_pose_data["Position"] = [0.0, 0.0, 0.0]
    init_pose_data["Orientation"] = [1.0, 0.0, 0.0, 0.0]
    data["Initial Pose"] = init_pose_data
    return data

def ExportFile():
    data = WriteDataDict()
    file_out = tk.filedialog.asksaveasfilename(initialdir="./inputs", filetypes=[("JSON","*.json")])
    with open(file_out, 'w') as fout:
        json_dumps_str = json.dumps(data, indent=4)
        print(json_dumps_str, file=fout)
    return file_out

def ResetEntry(entry_field, new_string):
    entry_field.delete(0, tk.END)
    entry_field.insert(0,str(new_string))
    
def ImportFile():
    file = tk.filedialog.askopenfilename(initialdir=mavs_data_path+"/vehicles/rp3d_vehicles", filetypes=[("JSON","*.json")])
    f = open(file)
    data = json.load(f)
    f.close()
   
    if "Chassis" in data:
        if "Sprung Mass" in data["Chassis"]:
            ResetEntry(sprungmass_entry, data["Chassis"]["Sprung Mass"])
        if "CG Offset" in data["Chassis"]:
            ResetEntry(cg_vert_offset_entry, data["Chassis"]["CG Offset"])
        if "CG Lateral Offset" in data["Chassis"]:
            ResetEntry(cg_lat_offset_entry, data["Chassis"]["CG Lateral Offset"])
        if "Dimensions" in data["Chassis"]:
            ResetEntry(chass_x_entry, data["Chassis"]["Dimensions"][0])
            ResetEntry(chass_y_entry, data["Chassis"]["Dimensions"][1])
            ResetEntry(chass_z_entry, data["Chassis"]["Dimensions"][2])
        if "Drag Coefficient" in data["Chassis"]:
            ResetEntry(drag_coeff_entry, data["Chassis"]["Drag Coefficient"])
    
    if "Powertrain" in data:
        if "Final Drive Ratio" in data["Powertrain"]:
            ResetEntry(fdr_entry, data["Powertrain"]["Final Drive Ratio"])
        if "Max Engine Torque" in data["Powertrain"]:
            ResetEntry(max_torque_entry, data["Powertrain"]["Max Engine Torque"])
        if "Max Engine Rpm" in data["Powertrain"]:
            ResetEntry(max_rpm_entry, data["Powertrain"]["Max Engine Rpm"])
        if "Max Braking Torque" in data["Powertrain"]:
            ResetEntry(brake_torque_entry, data["Powertrain"]["Max Braking Torque"])
        if "Idle Rpm" in data["Powertrain"]:
            ResetEntry(idle_rpm_entry, data["Powertrain"]["Idle Rpm"])
            
    if "Axles" in data:
        num_axles = len(data["Axles"])
        num_axle_var.set(str(num_axles))
        num_axles_changed()
        for i in range(num_axles):
            if "Longitudinal Offset" in data["Axles"][i]:
                ResetEntry(axle_tabs[i].long_offset_entry, data["Axles"][i]["Longitudinal Offset"])
            if "Track Width" in data["Axles"][i]:
                ResetEntry(axle_tabs[i].track_width_entry, data["Axles"][i]["Track Width"])
            if "Spring Constant" in data["Axles"][i]:
                ResetEntry(axle_tabs[i].spring_const_entry, data["Axles"][i]["Spring Constant"])
            if "Damping Constant" in data["Axles"][i]:
                ResetEntry(axle_tabs[i].damp_const_entry, data["Axles"][i]["Damping Constant"])
            if "Spring Length" in data["Axles"][i]:
                ResetEntry(axle_tabs[i].spring_len_entry, data["Axles"][i]["Spring Length"])
            if "Max Steer Angle" in data["Axles"][i]:
                ResetEntry(axle_tabs[i].steer_angle_entry, data["Axles"][i]["Max Steer Angle"])
            if "Unsprung Mass" in data["Axles"][i]:
                ResetEntry(axle_tabs[i].unsprung_entry, data["Axles"][i]["Unsprung Mass"])
            if "Steered" in data["Axles"][i]:
                axle_tabs[i].steered_var.set(bool(data["Axles"][i]["Steered"]))
            if "Powered" in data["Axles"][i]:
                axle_tabs[i].powered_var.set(bool(data["Axles"][i]["Powered"]))
            if "Tire" in data["Axles"][i]:
                if "Spring Constant" in data["Axles"][i]["Tire"]:
                    ResetEntry(tire_tabs[i].spring_cons_entry, data["Axles"][i]["Tire"]["Spring Constant"])
                if "Damping Constant" in data["Axles"][i]["Tire"]:
                    ResetEntry(tire_tabs[i].damp_const_entry, data["Axles"][i]["Tire"]["Damping Constant"])
                if "Radius" in data["Axles"][i]["Tire"]:
                    ResetEntry(tire_tabs[i].tire_rad_entry, data["Axles"][i]["Tire"]["Radius"])
                if "Width" in data["Axles"][i]["Tire"]:
                    ResetEntry(tire_tabs[i].tire_wid_entry, data["Axles"][i]["Tire"]["Width"])
                if "Section Height" in data["Axles"][i]["Tire"]:
                    ResetEntry(tire_tabs[i].tire_sh_entry, data["Axles"][i]["Tire"]["Section Height"])
                if "High Slip Crossover Angle" in data["Axles"][i]["Tire"]:
                    ResetEntry(tire_tabs[i].hsca_entry, data["Axles"][i]["Tire"]["High Slip Crossover Angle"])
                if "Viscous Friction Coefficient" in data["Axles"][i]["Tire"]:
                    ResetEntry(tire_tabs[i].vf_entry, data["Axles"][i]["Tire"]["Viscous Friction Coefficient"])
                
        
    if "Mesh" in data:
        if "File" in data["Mesh"]:
            ResetEntry(veh_mesh_tab.file_entry, data["Mesh"]["File"])
        if "Rotate Y to Z" in data["Mesh"]:
            veh_mesh_tab.rot_y_to_z_var.set(bool( data["Mesh"]["Rotate Y to Z"]))
        if "Rotate X to Y" in data["Mesh"]:
            veh_mesh_tab.rot_x_to_y_var.set(bool( data["Mesh"]["Rotate X to Y"]))
        if "Rotate Y to X" in data["Mesh"]:
            veh_mesh_tab.rot_y_to_x_var.set(bool( data["Mesh"]["Rotate Y to X"]))
        if "Offset" in data["Mesh"]:
            ResetEntry(veh_mesh_tab.off_x_entry, data["Mesh"]["Offset"][0])
            ResetEntry(veh_mesh_tab.off_y_entry, data["Mesh"]["Offset"][1])
            ResetEntry(veh_mesh_tab.off_z_entry, data["Mesh"]["Offset"][2])
        if "Scale" in data["Mesh"]:
            ResetEntry(veh_mesh_tab.scale_x_entry, data["Mesh"]["Scale"][0])
            ResetEntry(veh_mesh_tab.scale_y_entry, data["Mesh"]["Scale"][1])
            ResetEntry(veh_mesh_tab.scale_z_entry, data["Mesh"]["Scale"][2])
    
    if "Tire Mesh" in data:
        if "File" in data["Tire Mesh"]:
            ResetEntry(tire_mesh_tab.file_entry, data["Tire Mesh"]["File"])
        if "Rotate Y to Z" in data["Tire Mesh"]:
            tire_mesh_tab.rot_y_to_z_var.set(bool( data["Tire Mesh"]["Rotate Y to Z"]))
        if "Rotate X to Y" in data["Tire Mesh"]:
            tire_mesh_tab.rot_x_to_y_var.set(bool( data["Tire Mesh"]["Rotate X to Y"]))
        if "Rotate Y to X" in data["Tire Mesh"]:
            tire_mesh_tab.rot_y_to_x_var.set(bool( data["Tire Mesh"]["Rotate Y to X"]))
        if "Offset" in data["Tire Mesh"]:
            ResetEntry(tire_mesh_tab.off_x_entry, data["Tire Mesh"]["Offset"][0])
            ResetEntry(tire_mesh_tab.off_y_entry, data["Tire Mesh"]["Offset"][1])
            ResetEntry(tire_mesh_tab.off_z_entry, data["Tire Mesh"]["Offset"][2])
        if "Scale" in data["Tire Mesh"]:
            ResetEntry(tire_mesh_tab.scale_x_entry, data["Tire Mesh"]["Scale"][0])
            ResetEntry(tire_mesh_tab.scale_y_entry, data["Tire Mesh"]["Scale"][1])
            ResetEntry(tire_mesh_tab.scale_z_entry, data["Tire Mesh"]["Scale"][2])
                        
#--------END OF IMPORT EXISTING  FILE ----------------------------------------------------------#


def ViewDebugCallback():
    data = WriteDataDict()
    fname = "rp3d_dbg_tmp_zzz.json"
    with open(fname, 'w') as fout:
        json_dumps_str = json.dumps(data, indent=4)
        print(json_dumps_str, file=fout)
    mavs.ViewRp3dDebug(fname)
    os.remove(fname)
    # fig, ax = plt.subplots()
    # # Create a rectangle
    # lx = float(chass_x_entry.get())
    # ly = float(chass_y_entry.get())
    # lz = float(chass_z_entry.get())
    # z_low = 0.0
    # for i in range(len(axle_tabs)):
    #     tire_radius = float(tire_tabs[i].tire_rad_entry.get())
    #     spring_len = float(axle_tabs[i].spring_len_entry.get())
    #     zc = tire_radius + spring_len
    #     if (zc>z_low):
    #         z_low = zc;
    # x_lo = -0.5*lx
    # x_hi = x_lo + lx
    # rect = patches.Rectangle((-0.5*lx, z_low), lx, lz, color='green')
    # z_hi = z_low + lz
    # ax.add_patch(rect)
    # for i in range(len(axle_tabs)):
    #     tire_radius = float(tire_tabs[i].tire_rad_entry.get())
    #     spring_len = float(axle_tabs[i].spring_len_entry.get())
    #     x_off = float(axle_tabs[i].long_offset_entry.get())
    #     ax.add_patch(plt.Circle((x_off, tire_radius), tire_radius, color='blue'))
    #     ax.add_patch(plt.Rectangle((x_off-0.025, tire_radius), 0.05, spring_len, color='orange'))
    # z_cg = z_low + 0.5*lz +  float(cg_vert_offset_entry.get())   
    # ax.add_patch(plt.Circle((0.0, z_cg), 0.1, color='red'))    
    # plt.xlim(x_lo-0.5, x_hi+0.5)
    # plt.ylim(0, z_hi + 0.5)
    # # Show the plot
    # plt.show()

view_debug_button = tk.Button(root, text ="View Vehicle", command = ViewDebugCallback)
view_debug_button.grid(row=8,column=2,columnspan=3,sticky='ew')
   
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