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

# Set the path to the mavs python api, mavs_interface.py
# You will have to change this on your system.
sys.path.append(r'C:\Users\cgoodin\Desktop\vm_shared\shared_repos\mavs\src\mavs_python')

# Load the mavs python modules
import mavs_interface
import mavs_python_paths

# Load a scene
mavs_data_path = mavs_python_paths.mavs_data_path
mavs_scenefile = "/scenes/cube_scene.json"
scene = mavs_interface.MavsEmbreeScene()
scene.Load(mavs_data_path+mavs_scenefile)

# Create the environment and set properties
env = mavs_interface.MavsEnvironment()
env.SetScene(scene.scene)
env.SetCloudCover(0.1)
env.SetTurbidity(4.0)

# Specify the simulation time step, in seconds.
# 30 Hz is the default
dt = 1.0/30.0

# Create a window for driving the vehicle with the W-A-S-D keys
# Window must be heighlighted to input driving commands
free_cam = mavs_interface.MavsCamera()
free_cam.Initialize(384,384,0.0035,0.0035,0.0035)
free_cam.SetOffset([0.0,0.0,0.0],[1.0,0.0,0.0,0.0])
free_cam.SetGammaAndGain(0.6,1.0)
free_cam.FreePose()
free_cam.SetPose([0.0, 0.0, 2.0],[1.0, 0.0, 0.0, 0.0])

# Run the simulation loop
while True:
    # Timing info to be sued later
    t0 = time.time()

    free_cam.Update(env,dt)
    free_cam.Display()

    # If the simulation is running faster than real time, slow it down
    t1 = time.time()
    sleep_time = dt - (time.time()-t0)
    if sleep_time>0.0:
        time.sleep(sleep_time)
