'''
MIT License

Copyright (c) 2024 Mississippi State University

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
'''
import sys
import time

# Set the path to the mavs python api, mavs_interface.py
# You will have to change this on your system.
#sys.path.append(r'C:\Users\cgoodin\Desktop\goodin_docs\repos\mavs\src\mavs_python')
sys.path.append(r'/cavs/projects/MAVS/software/mavs/src/mavs_python')

# Load the mavs python modules
import mavs_interface
import mavs_python_paths

# Load a scene
mavs_data_path = mavs_python_paths.mavs_data_path
mavs_scenefile = "/scenes/cavs_proving_ground_powerline_trail.json"
scene = mavs_interface.MavsEmbreeScene()
scene.Load(mavs_data_path+mavs_scenefile)

# Create the environment and set properties
env = mavs_interface.MavsEnvironment()
env.SetScene(scene)
#env.SetCloudCover(0.1)
#env.SetTurbidity(4.0)

# Specify the simulation time step, in seconds.
# Greater than 100 Hz will give a warning
dt = 1.0/200.0

# Create a window for driving the vehicle with the W-A-S-D keys
# Window must be heighlighted to input driving commands
cam = mavs_interface.MavsCamera()
cam.Initialize(1920,1080,0.0035*(1920/1080),0.0035,0.0035)
cam.SetOffset([0.0,0.0,0.0],[1.0,0.0,0.0,0.0])
cam.SetGammaAndGain(0.85,1.0)
cam.SetPose([0.0, 0.0, 4.0],[1.0, 0.0, 0.0, 0.0])

cam.SetSaturationAndTemp(1.0, 6500.0)
cam.Update(env,0.01)
cam.Display()
cam.SaveCameraImage("default.bmp")

cam.SetSaturationAndTemp(1.0, 11200.0)
cam.Update(env,0.01)
cam.Display()
cam.SaveCameraImage("hi_temp.bmp")

cam.SetSaturationAndTemp(1.0, 4700.0)
cam.Update(env,0.01)
cam.Display()
cam.SaveCameraImage("lo_temp.bmp")

cam.SetSaturationAndTemp(4.0, 6500.0)
cam.Update(env,0.01)
cam.Display()
cam.SaveCameraImage("hi_saturation.bmp")

cam.SetSaturationAndTemp(0.0, 6500.0)
cam.Update(env,0.01)
cam.Display()
cam.SaveCameraImage("lo_saturation.bmp")

cam.SetSaturationAndTemp(4.0, 11200.0)
cam.Update(env,0.01)
cam.Display()
cam.SaveCameraImage("hi_hi.bmp")

cam.SetSaturationAndTemp(0.0, 11200.0)
cam.Update(env,0.01)
cam.Display()
cam.SaveCameraImage("lo_hi.bmp")

cam.SetSaturationAndTemp(4.0, 4700.0)
cam.Update(env,0.01)
cam.Display()
cam.SaveCameraImage("hi_lo.bmp")

cam.SetSaturationAndTemp(0.0, 4700.0)
cam.Update(env,0.01)
cam.Display()
cam.SaveCameraImage("lo_lo.bmp")
