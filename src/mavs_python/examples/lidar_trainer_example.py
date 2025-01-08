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
''' Script for generating labeled lidar and camera data.'''
import random
import sys
# Set the path to the mavs python api, mavs_interface.py
# you will have to change this on your system
sys.path.append(r'C:\Users\cgoodin\Desktop\vm_shared\shared_repos\mavs\src\mavs_python')
# Load the mavs python modules
import mavs_interface as mavs

# Create the lidar and set the offset
lidar = mavs.MavsLidar('OS1')
lidar.SetOffset([0.0, 0.0, 1.830],[1.0,0.0,0.0,0.0])

#----- Scene creation --------------------------------#
# create a randomized mavs scene
random_scene = mavs.MavsRandomScene()
random_scene.terrain_width = 100.0
random_scene.terrain_length = 100.0
random_scene.lo_mag = float(random.randrange(0,120))/10.0
random_scene.hi_mag = 0.0
random_scene.plant_density = float(random.randrange(25,50))/100.0
random_scene.trail_width = 2.0
random_scene.track_width = 0.3
random_scene.wheelbase = 1.8
scene_name = 'mavs_scene'
random_scene.basename = scene_name
random_scene.eco_file = 'american_southeast_forest_obstacles.json'
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

#----- Environment Creation ---------------------------#
# Create a MAVS environment and add the scene to it
env = mavs.MavsEnvironment()
env.SetScene(random_scene.scene)
env.SetTurbidity(float(random.randrange(2,7)))

# loop over all the poses in the waypoints list
for i in range(waypoints.num_waypoints):
    current_position = waypoints.GetWaypoint(i)
    current_orient = waypoints.GetOrientation(i)
    #Set pose of the lidar and scan a frame, saving labeledpoints
    lidar.SetPose(current_position,current_orient)
    lidar.Update(env,0.1)
    lidar.AnnotateFrame(env)
    lidar.AnalyzeCloud('labeled_lidar',i,False)
    lidar.SetDisplayColorType('label')
    lidar.SaveLabeledPcd('labeled_lidar'+str(i).zfill(4)+'.pcd')
    lidar.DisplayPerspective()
    lidar.SaveProjectedLidarImage('labeled_lidar'+str(i).zfill(4)+'.bmp')
