import sys
from PIL import Image
#Set path to mavs_interface.py and import mavs library
sys.path.append(r'C:/Users/cgoodin/Desktop/vm_shared/shared_repos/mavs/src/mavs_python')
import mavs_interface as mavs
import mavs_python_paths
mavs_data_path = mavs_python_paths.mavs_data_path

# create an environment and load a scene
mavs_scenefile = "/scenes/forester_scene_less_trees_simple.json" 
scene = mavs.MavsEmbreeScene()
scene.Load(mavs_data_path+mavs_scenefile)
scene.TurnOnLabeling()
env = mavs.MavsEnvironment()
env.SetScene(scene)

# create a camera, initialize it, and set properties
#cam = mavs.MavsCamera()
#cam.Initialize(512,512,0.0035,0.0035,0.0035)
#cam.RenderShadows(True)
#cam = mavs.MavsPathTraceCamera('low', 200, 10, 0.55) 
cam = mavs.MavsPathTraceCamera('custom', 200, 10, 0.55,nx=512,ny=512,h_s=0.0035,v_s=0.0035,flen=0.007,gamma=1.0) 
cam.SetFixPixels(False)
cam.SetOffset([0.0,0.0,0.0],[1.0,0.0,0.0,0.0])
cam.SetGammaAndGain(0.5,2.0)


x = -50.0
frame_num = 0
while (x<=50.0):
    # set the pose of the camera and render an image
    cam.SetPose([x, 0.0, 100.0], [0.7071,0.0,0.7071,0.0])
    cam.Update(env,0.03)

    # Add box labels
    cam.SaveCameraImage(str(frame_num).zfill(4)+"raw_image.bmp")
    cam.SaveBoxAnnotation(env, str(frame_num).zfill(4)+"annotated")

    x = x + 1.0
    frame_num = frame_num + 1
