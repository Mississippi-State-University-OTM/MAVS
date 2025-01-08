import random
import sys

# Set the path to the mavs python api, mavs_interface.py
# you will have to change this on your system
sys.path.append(r'C:\Users\cgoodin\Desktop\vm_shared\shared_repos\mavs\src\mavs_python')

# Load the mavs python modules
import mavs_interface as mavs

#----- Scene creation --------------------------------#
# create a randomized mavs scene
random_scene = mavs.MavsRandomScene()
random_scene.terrain_width = 1000.0
random_scene.terrain_length = 1000.0
random_scene.lo_mag = 10.0
random_scene.hi_mag = 0.5
random_scene.plant_density = 0.0 
random_scene.trail_width = 0.0
random_scene.track_width = 0.0
random_scene.wheelbase = 0.0
scene_name = 'bumpy_surface'
random_scene.basename = scene_name
#random_scene.eco_file = 'american_pine_forest.json'
random_scene.eco_file = 'american_southwest_desert.json'
random_scene.path_type = 'Ridges'
random_scene.CreateScene()