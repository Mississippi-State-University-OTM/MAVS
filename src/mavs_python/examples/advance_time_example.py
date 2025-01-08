import sys
# Set the path to the mavs python api, mavs.py
sys.path.append(r'C:/Users/cgoodin/Desktop/goodin_docs/repos/mavs/src/mavs_python')
# Load the mavs python modules
import mavs_interface as mavs
import mavs_python_paths
# Set the path to the mavs data folder
mavs_data_path = mavs_python_paths.mavs_data_path

# create an environment and load a scene
mavs_scenefile = "/scenes/cube_scene.json" 
env = mavs.MavsEnvironment()
env.LoadScene(mavs_data_path+mavs_scenefile)

# create a camera, initialize it, and set properties
cam = mavs.MavsCamera()
cam.Initialize(640,480,0.0036915,0.00328125,0.0035)
cam.SetOffset([0.0,0.0,0.0],[1.0,0.0,0.0,0.0])
cam.SetGammaAndGain(0.85,2.0)
cam.RenderShadows(True)

one_week = 60.0*60.0*24*7
dt = 60.0*5.0 # 5 minutes
elapsed_time = 0.0
n=0
while (elapsed_time<one_week):
    position = [0.0, 0.0, 2.0]
    orientation = [1.0,0.0,0.0,0.0]
    cam.SetPose(position, orientation)
    cam.Update(env,0.05)
    cam.Display()
    #cam.SaveCameraImage(str(n).zfill(3)+"_image.bmp")
    #n = n + 1
    env.AdvanceTime(dt)
    elapsed_time = elapsed_time + dt