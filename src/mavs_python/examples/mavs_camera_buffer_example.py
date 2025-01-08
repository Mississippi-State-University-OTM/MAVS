import sys
from PIL import Image
#Set path to mavs_interface.py and import mavs library
sys.path.append(r'C:/Users/cgoodin/Desktop/vm_shared/shared_repos/mavs/src/mavs_python')
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
#cam.Initialize(512,512,0.0035,0.0035,0.0035)
cam.SetOffset([0.0,0.0,0.0],[1.0,0.0,0.0,0.0])
cam.SetGammaAndGain(0.5,2.0)
cam.RenderShadows(True)

# set the pose of the camera and render an image
cam.SetPose([0.0, 0.0, 2.0], [1.0,0.0,0.0,0.0])
cam.Update(env,0.03)

# extract the camera buffer as a numpy array and convert it to an image
img_data = cam.GetNumpyArray()
img = Image.fromarray(img_data, 'RGB')
img.show()
