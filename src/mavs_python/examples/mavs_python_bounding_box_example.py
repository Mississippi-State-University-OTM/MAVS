import sys
# Set the path to the mavs python api, mavs_interface.py
sys.path.append(r'C:/Users/cgoodin/Desktop/vm_shared/shared_repos/mavs/src/mavs_python')
# Load the mavs python modules
import mavs_interface
import mavs_python_paths
# Set the path to the mavs data folder
mavs_data_path = mavs_python_paths.mavs_data_path

def GetObjPositions(env):
    
    n = env.GetNumberOfObjects()
    print("number of boxes = ",n)
    boxes = []
    for i in range(n):
        bb = env.GetObjectBoundingBox(i)
        boxes.append(bb)
    return boxes

# Select a scene and load it
#mavs_scenefile = "/scenes/forester_scene_less_trees_simple.json"
mavs_scenefile = "/scenes/st_line_scene_random.json"
scene = mavs_interface.MavsEmbreeScene()
scene.LoadRandom(mavs_data_path+mavs_scenefile)

# Create a MAVS environment and add the scene to it
env = mavs_interface.MavsEnvironment()
env.SetScene(scene.scene)
boxes = GetObjPositions(env)
n=0
for b in boxes:
    print(n,b,'\n')
    n = n+1