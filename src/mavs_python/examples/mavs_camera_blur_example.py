import sys
#Set path to mavs_interface.py and import mavs library
sys.path.append(r'C:/mavs/src/mavs_python')
import mavs_interface as mavs
import mavs_python_paths
mavs_data_path = mavs_python_paths.mavs_data_path

# create an environment and load a scene
mavs_scenefile = "/scenes/forester_scene_less_trees_simple.json" 
env = mavs.MavsEnvironment()
env.LoadScene(mavs_data_path+mavs_scenefile)

# create a camera, initialize it, and set properties
cam = mavs.MavsCamera()
cam.Initialize(640,480,0.0036915,0.00328125,0.0035)
cam.SetOffset([0.0,0.0,0.0],[1.0,0.0,0.0,0.0])
cam.SetGammaAndGain(0.85,2.0)
cam.RenderShadows(True)

# create a camera, initialize it, and set properties
blurry_cam = mavs.MavsCamera()
blurry_cam.Initialize(640,480,0.0036915,0.00328125,0.0035)
blurry_cam.SetOffset([0.0,0.0,0.0],[1.0,0.0,0.0,0.0])
blurry_cam.SetGammaAndGain(0.85,2.0)
blurry_cam.RenderShadows(True)
blurry_cam.UseBlur(True)

blurry_cam2 = mavs.MavsCamera()
blurry_cam2.Initialize(640,480,0.0036915,0.00328125,0.0035)
blurry_cam2.SetOffset([0.0,0.0,0.0],[1.0,0.0,0.0,0.0])
blurry_cam2.SetGammaAndGain(0.85,2.0)
blurry_cam2.RenderShadows(True)
blurry_cam2.UseBlur(True)

x = -100.0
vx = 25.0
velocity = [vx, 0.0, 0.0]
dt = 1.0/30.0
shutter_100 = 1.0/100.0
shutter_500 = 1.0/500.0
n = 0
while (x<10.0):
    position = [x,0.0, 2.0]
    orientation = [1.0,0.0,0.0,0.0]
    cam.SetPose(position, orientation)
    cam.Update(env,shutter_100)
    cam.Display()
    cam.SaveCameraImage(str(n).zfill(3)+"_image.bmp")
    blurry_cam.SetVelocity(velocity[0],velocity[1],velocity[2])
    blurry_cam.SetPose(position, orientation)
    blurry_cam.Update(env,shutter_100)
    blurry_cam.Display()
    blurry_cam.SaveCameraImage(str(n).zfill(3)+"_100_shutter_image.bmp")
    blurry_cam2.SetVelocity(velocity[0],velocity[1],velocity[2])
    blurry_cam2.SetPose(position, orientation)
    blurry_cam2.Update(env,shutter_500)
    blurry_cam2.Display()
    blurry_cam2.SaveCameraImage(str(n).zfill(3)+"_500_shutter_image.bmp")
    x = x + vx*dt
    n = n + 1