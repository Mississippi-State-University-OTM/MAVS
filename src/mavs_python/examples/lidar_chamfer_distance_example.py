import sys
from PIL import Image
#Set path to mavs_interface.py and import mavs library
sys.path.append(r'C:/mavs/src/mavs_python')
import mavs_interface as mavs
import mavs_python_paths
mavs_data_path = mavs_python_paths.mavs_data_path

# create an environment and load a scene
mavs_scenefile = "/scenes/cube_scene.json" 
scene = mavs.MavsEmbreeScene()
scene.Load(mavs_data_path+mavs_scenefile)
scene.TurnOnLabeling()
env = mavs.MavsEnvironment()
env.SetScene(scene)

lidar = mavs.MavsLidar('OS2')
pos1 = [0.0, 0.0, 1.0]
pos2 = [0.5, 0.5, 1.0]
#pos2 = [50.0, 50.0, 1.0]
ori = [1.0, 0.0, 0.0, 0.0]

lidar.SetPose(pos1, ori)
lidar.Update(env,0.1)
pc1 = lidar.GetPoints()
lidar.Display()

lidar.SetPose(pos2, ori)
lidar.Update(env,0.1)
pc2 = lidar.GetPoints()
lidar.Display()

chamfer_distance = mavs.ChamferDistance(pc1, pc2)
print(chamfer_distance)

pca = [[0.0, 0.0, 1.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, -1.0, 0.0]]
pcb = [[0.0, 0.0, 0.0]]
print(mavs.ChamferDistance(pca, pcb))
print(mavs.ChamferDistance(pcb, pca))

pca = [[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0] ]
pcb = [[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]]
print(mavs.ChamferDistance(pca, pcb))
print(mavs.ChamferDistance(pcb, pca))

