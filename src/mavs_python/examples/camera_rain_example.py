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
cam = mavs_interface.MavsCamera()
# num horizontal pixels, num vertical pixels, focal_plane_width, focal_plane_height, focal_length
cam.Initialize(224,224,0.0035,0.0035,0.0035)
#offset of the camera from the vehicle cg, [x,y,z] and quaternion
cam.SetOffset([0.0,0.0,1.0],[1.0,0.0,0.0,0.0])
cam.RenderShadows(True)

#-----------------------------------------------------------#
# The next few sections create a random scene & environment,
# and loop over the scene creating labeled data. To create
# more data, you could put all of the next sections in a
# loop that varied ranges for the plant density, ecosystem
# type, rain rate, etc.
#-----------------------------------------------------------#

#----- Scene creation --------------------------------#
# create a randomized mavs scene
random_scene = mavs_interface.MavsRandomScene()
#set the length and width of the terrain (meters)
random_scene.terrain_width = 50.0
random_scene.terrain_length = 50.0
# set the height of the low-frequency roughness (rolling hills)
random_scene.lo_mag = 0.0
# set the height of the high-frequency roughness (rough terrain)
random_scene.hi_mag = 0.0
# set the relative density of othe plants, from 0 to 1 (1 is a lot of plants!)
random_scene.plant_density = 0.1
#select the name of the scene, doesn't really matter too much
random_scene.basename = 'my_scene'
#select the ecosystem file to use, uncomment the one you want
#'american_southeast_meadow.json', 'american_southwest_desert.json'
random_scene.eco_file = 'american_southeast_forest.json'
# Call this to generate the random scene
random_scene.CreateScene()
# Call this to make sure scene is labeled
random_scene.TurnOnLabeling()

#--- Load the waypoints that go with this scene ----#
waypoints = mavs_interface.MavsWaypoints()
waypoints.Load('./'+random_scene.basename+'_path.vprp')

#----- Environment Creation ---------------------------#
# Create a MAVS environment and add the scene to it
env = mavs_interface.MavsEnvironment()
env.SetScene(random_scene.scene)
# Set the rain rate in mm/h
# typical rates are 0-25
if (len(sys.argv)>1):
    rain_rate = float(sys.argv[1])
else:
    rain_rate = 5.0
# cloud cover can be between 0 and 1
cloud_cover = 0.6
# turbidity is 2-10
turbidity = 9.0
env.SetCloudCover(cloud_cover)
env.SetTurbidity(turbidity)

#----- Simulation -------------------------------------#
# loop over all the poses in the waypoints list
for i in range(waypoints.num_waypoints):
    # create a base string for file output
    frame_name = 'frame_'+str(i)
    # Set the pose of the camera and render the frame
    cam.SetPose(waypoints.GetWaypoint(i),waypoints.GetOrientation(i))
    #Render and save the clear image
    env.SetRainRate(0.0)
    cam.SetEnvironmentProperties(env.obj)
    cam.SetDropsOnLens(False)
    # Update the camera with a given environment and time step
    cam.Update(env,0.03)
    cam.SaveCameraImage('clear_'+frame_name+'.bmp')
    #render and save the rainy image
    env.SetRainRate(rain_rate)
    cam.SetEnvironmentProperties(env.obj)
    cam.SetDropsOnLens(True)
    cam.Update(env,0.03)
    cam.Display()
    cam.SaveCameraImage('rain_'+frame_name+'.bmp')
    # Annotate and save the labeled image
    cam.SaveAnnotation(env.obj,('annotated_'+frame_name))
    

