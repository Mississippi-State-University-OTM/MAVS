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

# Set the path to the mavs python api, mavs_interface.py
# you will have to change this on your system
sys.path.append(r'C:\Users\cgoodin\Desktop\code\mavs\src\mavs_python')

# Load the mavs python modules
import mavs_interface

#----- Sensor Creation and parameters ------------------#
# Create a MAVS camera and initialize it
# camera is for visualization purposes in this simulation
cam = mavs_interface.MavsCamera()
# num horizontal pixels, num vertical pixels, focal_plane_width, focal_plane_height, focal_length
cam.Initialize(224,224,0.0035,0.0035,0.0035)
#offset of the camera from the vehicle cg, [x,y,z] and quaternion
cam.SetOffset([0.0,0.0,1.0],[1.0,0.0,0.0,0.0])
cam.RenderShadows(False)
#Create alidar and set it's offset
lidar = mavs_interface.MavsLidar('HDL-32E')
lidar.SetOffset([0.0,0.0,1.0],[1.0,0.0,0.0,0.0])

#-----------------------------------------------------------#
# The next sections creates a random scene & environment,
# and loop over the scene creating labeled data. To create
# more data, you could put all of the next sections in a
# loop that varied ranges for the plant density, ecosystem
# type, roughness parameters, etc.
#-----------------------------------------------------------#

#----- Scene creation --------------------------------#
# create a randomized mavs scene
random_scene = mavs_interface.MavsRandomScene()
#set the length and width of the terrain (meters)
random_scene.terrain_width = 75.0
random_scene.terrain_length = 75.0
# set the height of the low-frequency roughness (rolling hills)
random_scene.lo_mag = 7.0
# set the height of the high-frequency roughness (rough terrain)
random_scene.hi_mag = 0.1
# set the relative density of othe plants, from 0 to 1 (1 is a lot of plants!)
random_scene.plant_density = 0.25
#select the ecosystem file to use, uncomment the one you want
#'american_southeast_meadow.json', 'american_southwest_desert.json'
random_scene.eco_file = 'american_southeast_forest.json'
# Call this to generate the random scene
random_scene.CreateScene()
# Call this to make sure the output points are labeled
random_scene.TurnOnLabeling()

#--- Load the waypoints that go with this scene ----#
waypoints = mavs_interface.MavsWaypoints()
waypoints.Load('./'+random_scene.basename+'_path.vprp')

#----- Environment Creation ---------------------------#
# Create a MAVS environment and add the scene to it
env = mavs_interface.MavsEnvironment()
env.SetScene(random_scene.scene)
cam.SetEnvironmentProperties(env.obj)

#----- Simulation -------------------------------------#
# loop over all the poses in the waypoints list
for i in range(waypoints.num_waypoints):
    # Set the pose of the camera and lidar
    cam.SetPose(waypoints.GetWaypoint(i),waypoints.GetOrientation(i))
    lidar.SetPose(waypoints.GetWaypoint(i),waypoints.GetOrientation(i))
    # Update the camera and display it
    cam.Update(env,0.03)
    cam.Display()
    # Update the lidar
    lidar.Update(env,0.1)
    lidar.AnnotateFrame(env.obj)
    # Save lidar output
    lidar.SaveLabeledPointCloud('frame_'+str(i)+'.pts')

    

