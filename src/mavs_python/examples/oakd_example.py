from PIL import Image
import numpy as np
import sys
# Set the path to the mavs python api, mavs.py
sys.path.append(r'C:/mavs/src/mavs_python')
import mavs_interface as mavs
import mavs_python_paths

def UpdateTime(hour, minute, second):
    second = second + 1
    if (second == 60):
        second = 0
        minute = minute + 1
    if (minute == 60):
        minute = 0
        hour = hour + 1
    if (hour == 24):
        hour = 0
    return hour, minute, second

# Load a MAVS scene
mavs_scenefile = mavs_python_paths.mavs_data_path+"/scenes/rock_wall.json"
scene = mavs.MavsEmbreeScene()
scene.Load(mavs_scenefile)

# Create an environment and add the scene to it
env = mavs.MavsEnvironment()
env.SetScene(scene)

# Creat the OAK-D camera and set the max range to four meters
cam = mavs.MavsOakDCamera()
cam.SetMaxRangeCm(400.0)
cam.SetDisplayType("rgb") # display type can be "rbg", "range", or "both"

# Set the position and orientaiton of the camera in the environment
cam.SetPose([0.0, -2.0, 1.25], [0.70710678, 0.0, 0.0, 0.70710678])

# Update the camera and display the results
cam.Update(env, 0.03)
cam.Display()

# Extract the raw image data from the camera as a numpy array
img_data = cam.GetImage()
# convert it to an image
img = Image.fromarray(img_data, 'RGB')

# Extract the range image as a numpy array
depth_data = cam.GetDepthImage()
depth_img = Image.fromarray(depth_data, 'RGB')
# get the range in meters at a given pixel
center_depth = cam.GetRangeAtPixelMeters(cam.width/2,cam.height/2)
corner_depths = [cam.GetRangeAtPixelMeters(0,0),cam.GetRangeAtPixelMeters(0,cam.height-1), cam.GetRangeAtPixelMeters(cam.width-1,0), cam.GetRangeAtPixelMeters(cam.width-1, cam.height-1)]
print('Range at center (m) = ', center_depth)
print('Range at corners (m) = ', corner_depths)
sys.stdout.flush()

# continue updating and displaying the camera while the window is open
while cam.DisplayOpen():
    cam.Update(env, 0.03)    
    cam.Display()
        
    