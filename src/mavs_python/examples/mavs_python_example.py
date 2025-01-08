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
sys.path.append(r'C:\Users\cgoodin\Desktop\code\mavs\src\mavs_python')
# Load the mavs python modules
import mavs_interface
import mavs_python_paths

# Set the path to the mavs data folder
mavs_data_path = mavs_python_paths.mavs_data_path

# Select a scene and load it
#mavs_scenefile = "/scenes/forest_ecosystem_scene.json"
mavs_scenefile = "/scenes/cube_scene.json"
scene = mavs_interface.MavsEmbreeScene()
scene.Load(mavs_data_path+mavs_scenefile)

# Create a MAVS environment and add the scene to it
env = mavs_interface.MavsEnvironment()
env.SetScene(scene.scene)

# Create a MAVS camera and initialize it
cam = mavs_interface.MavsCamera()
cam.Initialize(1620,1080,0.006784,0.0054272,0.004)

# Set some simulation properties of the camera
cam.SetOffset([0.0,0.0,1.0],[1.0,0.0,0.0,0.0])
cam.RenderShadows(True)
cam.SetAntiAliasingFactor(5)

# Create a MAVS radar and set properties
radar = mavs_interface.MavsRadar()
radar.SetMaxRange(150.0)
radar.SetFieldOfView(15.0)

# Set the pose of the camera and render the frame
cam.SetPose([0.0,0.0,0.0],[1.0,0.0,0.0,0.0])
cam.Update(env,0.03)

# Display the frame and save the image to a file
cam.Display()
cam.SaveCameraImage("mavs_python_image.bmp")

# Update the radar and display the result
radar.SetPose([0.0,0.0,0.0],[1.0,0.0,0.0,0.0])
radar.Update(env,0.03)
radar.Display()
radar.SaveImage("mavs_python_radar_image.bmp")
targets = radar.GetTargets()
print(targets)
