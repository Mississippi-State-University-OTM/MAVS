import sys
# Set the path to the mavs python api, mavs_interface.py
#sys.path.append(r'C:/Users/qznksz/msu-autonomous-vehicle-simulator/src/mavs_python')
sys.path.append(r'C:/Users/cgoodin/Desktop/vm_shared/shared_repos/mavs/src/mavs_python')
# Load the mavs python modules
import mavs_interface as mavs
import mavs_python_paths
# Set the path to the mavs data folder
mavs_data_path = mavs_python_paths.mavs_data_path

veh_viewer = mavs.MavsRp3dViewer()
vehicle_file = mavs_data_path+"/vehicles/rp3d_vehicles/hmmwv_rp3d.json"
#vehicle_file = mavs_data_path+"/vehicles/rp3d_vehicles/forester_2017_rp3d"
veh_viewer.LoadVehicle(vehicle_file)
veh_viewer.Display(True) # Do not draw debug info
veh_img_buff = veh_viewer.GetSideImageBuffer()
veh_viewer.SaveSideImage("veh_side.bmp")


env = mavs.MavsEnvironment()
scene = mavs.MavsEmbreeScene()
#scene.Load(mavs_data_path+"/scenes/czarnecki_surface_only.json")
scene.Load(mavs_data_path+"/scenes/cube_scene.json")
env.SetScene(scene.scene)
ortho_viewer = mavs.MavsOrthoViewer()
ortho_viewer.Update(env)
ortho_viewer.Display()
ortho_img_buff = ortho_viewer.GetImageBuffer()
ortho_viewer.SaveImage("ortho_view.bmp")