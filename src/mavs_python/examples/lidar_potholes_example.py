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
''' Script for generating lidar data with potholes.'''
import random
import sys

# Set the path to the mavs python api, mavs_interface.py
# you will have to change this on your system
sys.path.append(r'C:\Users\cgoodin\Desktop\vm_shared\shared_repos\mavs\src\mavs_python')

# Load the mavs python modules
import mavs_interface as mavs

#----- Sensor Creation and parameters ------------------#
# Create a MAVS camera, initialize it, and set properties
# This is for visualization purposes only
cam = mavs.MavsCamera()
cam.Initialize(384,384,0.0035,0.0035,0.0035)
cam.SetOffset([-2.18756, 0.0, 1.830],[1.0,0.0,0.0,0.0])
cam.RenderShadows(False)
cam.SetGammaAndGain(0.5,1.0)
# Create the lidar and set the offset
os1_lidar = mavs.MavsLidar('OS2')
os1_lidar.SetOffset([-2.18756, 0.0, 1.830],[1.0,0.0,0.0,0.0])

#----- Scene creation --------------------------------#
# create a randomized mavs scene
random_scene = mavs.MavsRandomScene()
# terrain width and length in meters
random_scene.terrain_width = 100.0
random_scene.terrain_length = 100.0
random_scene.mesh_resolution = 0.25
# magnitude of low frequency roughness, in meters (rolling hills), 0=flat
random_scene.lo_mag = 2.0
# magnitude of hi-freq roughness, [meters] (bumpy terrain), 0 = smooth
random_scene.hi_mag = 0.1
# plant density should be from [0-1]. 0=no plants, 1=most plants
random_scene.plant_density = 0.05
# trail properties
random_scene.trail_width = 2.0
random_scene.track_width = 0.3
random_scene.wheelbase = 1.8
# Add pothole
# x,y, depth, diameter
random_scene.AddPotholeAt(0.0, 0.0, 1.0, 1.5)
random_scene.AddPotholeAt(-20.0, -20.0, 1.0, 0.5)
random_scene.AddPotholeAt(20.0, 20.0, 1.0, 1.0)
random_scene.num_potholes = 3
# Next few lines iniatilze the scene, shouldn't be changed
scene_name = 'pothole_scene'
random_scene.basename = scene_name
random_scene.eco_file = 'american_pine_forest.json'
random_scene.path_type = 'Ridges'
random_scene.CreateScene()
random_scene.TurnOnLabeling()

#--- Load the waypoints that go with this scene ----#
waypoints = mavs.MavsWaypoints()
waypoints.Load('./'+random_scene.basename+'_path.vprp')
# Get the scene geometry and put the waypoints on the ground
scene = mavs.MavsEmbreeScene()
scene.Load(scene_name+'_scene.json')
waypoints.PutWaypointsOnGround(scene)

# Create a MAVS environment and add the scene to it
env = mavs.MavsEnvironment()
env.SetScene(random_scene.scene)
env.SetCloudCover(0.4)
env.SetTurbidity(3.0)
cam.SetEnvironmentProperties(env.obj)

#----- Simulation -------------------------------------#
# loop over all the poses in the waypoints list
for i in range(waypoints.num_waypoints):
    frame_name = 'frame_'+str(i)
    current_position = waypoints.GetWaypoint(i)
    current_orient = waypoints.GetOrientation(i)
    # Set the pose of the camera and render the frame
    cam.SetPose(current_position,current_orient)
    cam.Update(env,0.03)
    cam.Display()
    cam.AnnotateFrame(env)
    cam.SaveAnnotation(env, frame_name+'_labeled')
    cam.SaveCameraImage(frame_name+".bmp")
    #update the lidar and display
    os1_lidar.SetPose(current_position,current_orient)
    os1_lidar.Update(env,0.03)
    os1_lidar.Display()
    # save the point cloud to a file
    os1_lidar.AnnotateFrame(env)
    os1_lidar.SaveLabeledPointCloud(frame_name+".pts")