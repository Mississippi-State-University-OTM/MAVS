import time
import sys
import math
# Set the path to the mavs python api, mavs.py
sys.path.append(r'C:/Users/cgoodin/Desktop/vm_shared/shared_repos/mavs/src/mavs_python')
# Load the mavs python modules
import mavs_interface as mavs
import mavs_python_paths
# Set the path to the mavs data folder
mavs_data_path = mavs_python_paths.mavs_data_path

# This file sets the temperatures and emissivities of everything in the scene
# as well as some environmental conditions like humidity
thermal_input_file = mavs_data_path+'/sims/thermal_sims/pine_forest_hmmwv.json'

# Create a MAVS scene for this test
random_scene = mavs.MavsRandomScene()
random_scene.terrain_width = 350.0
random_scene.terrain_length = 350.0
random_scene.lo_mag = 0.0
random_scene.hi_mag = 0.075
random_scene.mesh_resolution = 0.5 
random_scene.plant_density = 0.01
random_scene.trail_width = 5.0
random_scene.track_width = 0.3
random_scene.wheelbase = 2.0
random_scene.surface_roughness_type = "variable"
scene_name = 'surface'
random_scene.basename = scene_name
random_scene.eco_file = 'american_pine_forest.json'
random_scene.path_type = 'Ridges'
random_scene.CreateScene()
random_scene.TurnOnLabeling()

# Create an environment, add the created scene
env = mavs.MavsEnvironment()
env.SetScene(random_scene)

# Set environment properties
env.SetTime(13) # 0-23
env.SetFog(0.2) # 0.0-100.0
env.SetSnow(0.0) # 0-25
env.SetTurbidity(7.0) # 2-10
env.SetAlbedo(0.1) # 0-1
env.SetCloudCover(0.0) # 0-1
env.SetRainRate(0.0) # 0-25
env.SetWind( [2.5, 1.0] ) # Horizontal windspeed in m/s

# Create the MAVS thermal camera
# NX, NY, dx (m), dy (m), focal length (m)
thermal_cam = mavs.MavsLwirCamera(512, 512, 0.0035, 0.0035, 0.035)
thermal_cam.LoadThermalData(thermal_input_file)
thermal_cam.SetAntiAliasingFactor(3)
thermal_cam.SetOffset([0.0, 0.0,0.0],[1.0,0.0,0.0,0.0])

sensor_elevation = 35.0
sensor_range = 0.75*random_scene.terrain_width
camera_angle = math.atan(sensor_elevation/sensor_range)

thermal_cam.SetPose([-sensor_range,0.0, sensor_elevation],
                [math.cos(0.5*camera_angle), 0.0, math.sin(0.5*camera_angle), 0.0])
thermal_cam.Update(env,0.05)
thermal_cam.Display()
thermal_cam.SaveCameraImage('thermal_image.bmp')

