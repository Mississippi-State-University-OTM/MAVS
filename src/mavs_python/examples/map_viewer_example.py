import sys
# Set the path to the mavs python api, mavs.py
sys.path.append(r'C:/mavs/src/mavs_python')
import mavs_interface as mavs
import mavs_python_paths

mavs_data_path = mavs_python_paths.mavs_data_path
mavs_scenefile = "/scenes/proving_ground_southwest_corner.json"

scene = mavs.MavsEmbreeScene()
scene.Load(mavs_data_path+mavs_scenefile)

env = mavs.MavsEnvironment()
env.SetScene(scene)

map_viewer = mavs.MavsMapViewer(-100.0, -100.0, 100.0, 100.0, 0.25)

waypoints = [[-50.0, -50.0], [0.0, 0.0], [75.0, 50.0]]

while map_viewer.IsOpen():
    map_viewer.AddCircle([0.0, 25.0], 2.0)
    map_viewer.AddLine([-25.0, 25.0], [25.0,-25.0])
    map_viewer.AddWaypoints(waypoints)
    map_viewer.Display(env)