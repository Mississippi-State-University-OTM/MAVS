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
import mavs_python_paths
import math

#----- Sensor Creation and parameters ------------------#
# Create a MAVS camera and initialize it
cam = mavs_interface.MavsCamera()
# num horizontal pixels, num vertical pixels, focal_plane_width, focal_plane_height, focal_length
cam.Initialize(256,256,0.0035,0.0035,0.0035)
#offset of the camera from the vehicle cg, [x,y,z] and quaternion
cam.SetOffset([0.0,0.0,0.0],[1.0,0.0,0.0,0.0])
cam.RenderShadows(False)
# Create a MAVS lidar
#lidar = mavs_interface.MavsLidar('VLP16')
lidar = mavs_interface.MavsLidar('HDL-64E')

# Load a scene
mavs_data_path = mavs_python_paths.mavs_data_path
mavs_scenefile = "/scenes/surface_only_anim.json"
scene = mavs_interface.MavsEmbreeScene()
scene.Load(mavs_data_path+mavs_scenefile)

# Create the environment and set properties
env = mavs_interface.MavsEnvironment()
env.SetScene(scene.scene)
env.SetCloudCover(0.1)
env.SetTurbidity(4.0)

camera_pos = [-10.0,0.0,1.65]
max_range = 10.0;
#----- Simulation -------------------------------------#
while(True):
	# advance the environment, which moves animations
	env.AdvanceTime(0.03)
	
	#calculate the new sensor position / orientation based on the animation location
	anim_pos = env.GetAnimationPosition(0)
	look_to = [anim_pos[0]-camera_pos[0],anim_pos[1]-camera_pos[1],anim_pos[2]-camera_pos[2]]
	d = math.sqrt(look_to[0]*look_to[0]+look_to[1]*look_to[1]+look_to[2]*look_to[2])
	look_to = [look_to[0]/d,look_to[1]/d,look_to[2]/d]
	theta = math.atan2(look_to[1],look_to[0])
	orientation = [math.cos(theta*0.5),0.0,0.0,math.sin(theta*0.5)]
	if d>max_range:
		camera_pos[0] = camera_pos[0] + (d-max_range)*look_to[0]
		camera_pos[1] = camera_pos[1] + (d-max_range)*look_to[1]
	
	# Update the camera and lidar sensor
	cam.SetPose(camera_pos, orientation)
	cam.Update(env,0.03)
	cam.Display()
	lidar.SetPose(camera_pos, orientation)
	lidar.Update(env,0.03)
	lidar.DisplayPerspective()

