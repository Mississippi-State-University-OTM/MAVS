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
import math
import time

# Set the path to the mavs python api, mavs_interface.py
# You will have to change this on your system.
sys.path.append(r'C:\Users\cgoodin\Desktop\vm_shared\shared_repos\mavs\src\mavs_python')

# Load the mavs python modules
import mavs_interface
import mavs_python_paths

# Simulation input file, examples in 
# data/sims/sensor_sims. Path must be relative
# to the "data" folder. Defines the scene and
# vehicle. 
#sim_file = 'sims/sensor_sims/halo_city_sim.json'
sim_file = 'sims/sensor_sims/halo_cube_sim.json'

# Create a MAVS simulation and load input file.
simulation = mavs_interface.MavsSimulation()
simulation.Load(mavs_python_paths.mavs_data_path+'/'+sim_file)

# Set whatever environment parameters you want to change,
# if the you don't want the ones in the input file.
simulation.env.year = 2019
simulation.env.hour = 12
simulation.env.minute = 0
simulation.env.second = 0
simulation.env.month = 9
simulation.env.day = 25
simulation.env.SetFog(0.03)
simulation.env.SetSnow(0.0)
simulation.env.SetTurbidity(5.0)
simulation.env.SetAlbedo(0.1)
simulation.env.SetCloudCover(0.0)
simulation.env.SetRainRate(0.0)
simulation.env.SetWind( [0.0, 0.0] )

# To manually drive the vehicle with the W-A-S-D keys, set this to True.
# To have the vehicle automatically follow the waypoints defined in the 
# input file, set to False.
simulation.free_driving = True

# Specify where sensor data will be saved.
# Can be relative or absolute path
simulation.save_location = './'

# Turn on whichever sensors you like, specifying if the output is labeled or raw
# First argument is the sensor number, starting with 0
#simulation.TurnOnSensor(0,display=True,save_raw=False,labeling=False)
#simulation.TurnOnSensor(1,display=True,save_raw=False,labeling=False)
#simulation.TurnOnSensor(2,display=True,save_raw=False,labeling=False)

# Specify the simulation time step, in seconds.
# 30 Hz is the default
dt = 1.0/30.0

# Create a window for driving the vehicle with the W-A-S-D keys
# Window must be heighlighted to input driving commands
drive_cam = mavs_interface.MavsCamera()
drive_cam.Initialize(384,384,0.0035,0.0035,0.0035)
drive_cam.SetOffset([-10.0,0.0,3.0],[1.0,0.0,0.0,0.0])
drive_cam.SetGammaAndGain(0.6,1.0)
drive_cam.RenderShadows(False)

# Define the update function for the driving camera
def UpdateDriveCam():
    drive_cam.SetEnvironmentProperties(simulation.env.obj)
    drive_cam.SetPose(simulation.vehicle.GetPosition(),simulation.vehicle.GetOrientation())
    drive_cam.Update(simulation.env,dt)
    drive_cam.Display()

# Run the simulation loop
while True:
    # Timing info to be sued later
    t0 = time.time()

    # Get the driving command from driving_cam window
    dc = drive_cam.GetDrivingCommand()

    # Update the simulation
    simulation.Update(dt,throttle=dc.throttle, steering=dc.steering, braking=dc.braking, update_actor=True)

    # Grab whatever sensor data you may want to use "in the loop" with autonomy algorithms
    for s in simulation.sensors:
        if s.type=='lidar' and s.is_active:
            # note that because python is really slow at allocating memory
            # calling this at each time step may slow down the sim significantly
            #points = simulation.sensors[2].GetUnRegisteredPointsXYZIL()
            points = s.GetUnRegisteredPointsXYZIL()

    # Update the drive cam
    UpdateDriveCam()

    # If the simulation is running faster than real time, slow it down
    t1 = time.time()
    sleep_time = dt - (time.time()-t0)
    if sleep_time>0.0:
        time.sleep(sleep_time)
