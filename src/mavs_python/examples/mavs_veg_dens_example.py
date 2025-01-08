'''
Example of using the MavsRandomScene to generate a scene
and then measure the vegetation density of the scene with the
"GetVegDensityOnAGrid" method. Shows how to plot the veg density
in elevation slices
'''
import sys
import matplotlib.pyplot as plt
import numpy as np
# Set the path to the mavs python api, mavs.py
sys.path.append(r'C:/Users/cgoodin/Desktop/vm_shared/shared_repos/mavs/src/mavs_python')
# Load the mavs python modules
import mavs_interface as mavs

# Create a MAVS Random Scene
random_scene = mavs.MavsRandomScene()
random_scene.terrain_width = 50.0 # meters
random_scene.terrain_length = 50.0 # meters
# The lowest frequency of noise will be 2/min(terrain_width,terrain_length)
# The highest frequency of noise will be 1/(2*mesh_resolution)
random_scene.mesh_resolution=0.25
# roughness type can be 'variable', 'gaussian', or 'perlin'
random_scene.surface_roughness_type = "variable"
# When surface_roughness_type='variable', this will also generate a file called
# 'rms_truth.txt' that gives the RMS roughness (in meters) on an ENU grid.
random_scene.lo_mag = 0.0 # Always set this to 0 when using 'variable' roughness
random_scene.hi_mag = 0.1 # The magnitude of the highest frequency noise, should be <0.15 in meters
random_scene.plant_density = 0.025 # 0-1
# Set the trail parameters to zero if you don't want a trail
random_scene.trail_width = 0.0 
random_scene.track_width = 0.0
random_scene.wheelbase = 0.0
# The scene name can be whatever you choos
random_scene.basename = 'bumpy_surface'
# Choose from the ecosystem files in mavs/data/ecosystems
random_scene.eco_file = 'american_southeast_forest_brushy.json'
# Have the trail follow 'Ridges', 'Valleys', or 'Loop'
random_scene.path_type = 'Ridges'
# Create the scene with the properties defined above
random_scene.CreateScene()

# Create a MAVS environment
env = mavs.MavsEnvironment()
# Add the scene to the environment
env.SetScene(random_scene.scene)

# Render a top-down view of the scene for reference later
cam = mavs.MavsCamera()
cam.Initialize(512,512,0.0035,0.0035,0.0035)
cam.SetOffset([0.0,0.0,0.0],[1.0,0.0,0.0,0.0])
cam.SetGammaAndGain(0.6,1.0)
cam.SetPose([0.0, 0.0, max(10,1.25*random_scene.terrain_width)],[0.7071, 0.0, 0.7071, 0.0])
cam.Update(env,0.03)
cam.Display()
cam.SaveCameraImage("top_down.bmp")

# Get the density grid
# Arguments are : 
# Lower left corner in ENU
# Upper right corner in ENU
# grid resolution in meters
density_grid = env.GetVegDensityOnAGrid([-0.5*random_scene.terrain_width,-0.5*random_scene.terrain_length,0.0],
                                        [0.5*random_scene.terrain_width,0.5*random_scene.terrain_width,20],1.0)

# convert the python list to a numpy array
veg_dens = np.array(density_grid)

# get the shape of the numpy array
nx,ny,nz = veg_dens.shape

# plot the vegetation density slices, one by one
for k in range(nz):
    slice = veg_dens[:,:,k]
    snx = len(slice)
    sny = len(slice[0])
    plt.xlim((nx,0))
    plt.margins(0,0)
    plt.axis('off')
    plt.imshow(slice,interpolation='none')
    plt.savefig('slice_'+str(k)+'.png',bbox_inches='tight')