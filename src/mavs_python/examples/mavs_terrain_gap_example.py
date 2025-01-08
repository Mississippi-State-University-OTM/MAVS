'''
Example of using the MavsRandomScene to generate a rough surface.
Explains parameters for creating a rough surface and makes a rendering.
'''
import math
import sys
import time
# Set the path to the mavs python api, mavs.py
sys.path.append(r'C:/Users/cgoodin/Desktop/vm_shared/shared_repos/mavs/src/mavs_python')
# Load the mavs python modules
import mavs_interface as mavs
import mavs_python_paths
# Set the path to the mavs data folder
mavs_data_path = mavs_python_paths.mavs_data_path

# Create a MAVS Random Scene
random_scene = mavs.MavsRandomScene()
random_scene.terrain_width = 100.0 # meters
random_scene.terrain_length = 50.0 # meters
# The lowest frequency of noise will be 2/min(terrain_width,terrain_length)
# The highest frequency of noise will be 1/(2*mesh_resolution)
random_scene.mesh_resolution=0.25
# roughness type can be 'variable', 'gaussian', 'gap', or 'perlin'
random_scene.surface_roughness_type = "gap"
# When surface_roughness_type='variable', this will also generate a file called
# 'rms_truth.txt' that gives the RMS roughness (in meters) on an ENU grid.
random_scene.lo_mag = 0.0 # Always set this to 0 when using 'variable' roughness
random_scene.hi_mag = 0.05 # The magnitude of the highest frequency noise, should be <0.15 in meters
random_scene.plant_density = 0.01 # 0-1
# Set the trail parameters to zero if you don't want a trail
random_scene.trail_width = 0.0 
random_scene.track_width = 0.0
random_scene.wheelbase = 0.0
# The scene name can be whatever you choos
random_scene.basename = 'gap_surface'
# Choose from the ecosystem files in mavs/data/ecosystems
#random_scene.eco_file = 'american_southwest_desert.json'
random_scene.eco_file = 'american_southeast_meadow.json'
# Have the trail follow 'Ridges', 'Valleys', or 'Loop'
random_scene.path_type = 'Ridges'

# Create a gap scene with the properties defined above
gap_width = 2.0
gap_depth = 2.0
gap_slope = 15.0
random_scene.CreateGapScene(gap_width,gap_depth, math.radians(gap_slope))

# Create a MAVS environment
env = mavs.MavsEnvironment()
# Set environment properties
env.SetTime(19) # 0-23
env.SetTurbidity(10.0) # 2-10
# Add the scene to the environment
env.SetScene(random_scene)

#Create and load a MAVS vehicle
veh = mavs.MavsRp3d()
veh_file = 'mrzr4_tires.json'
veh.Load(mavs_data_path+'/vehicles/rp3d_vehicles/' + veh_file)
# Starting point for the vehicle
veh.SetInitialPosition(-10.0, 0.0, 0.0) # in global ENU
veh.SetInitialHeading(0.0) # in radians
veh.Update(env, 0.0, 0.0, 1.0, 0.000001)

# Create a window for driving the vehicle with the W-A-S-D keys
# window must be highlighted to input driving commands
drive_cam = mavs.MavsCamera()
# nx,ny,dx,dy,focal_len
drive_cam.Initialize(384,384,0.0035,0.0035,0.0035)
# offset of camera from vehicle CG
#drive_cam.SetOffset([-10.0,0.0,3.0],[1.0,0.0,0.0,0.0])
#drive_cam.SetOffset([-1.0,0.0,0.25],[1.0,0.0,0.0,0.0]
drive_cam.SetOffset([-7.5,0.0,2.0],[1.0,0.0,0.0,0.0])
# Set camera compression and gain
drive_cam.SetGammaAndGain(0.5,2.0)
# Turn off shadows for this camera for efficiency purposes
drive_cam.RenderShadows(True)

# Now start the simulation main loop
dt = 1.0/120.0 # time step, seconds
n = 0 # loop counter
elapsed_time = 0.0
while (True):
    # tw0 is for timing purposes used later
    tw0 = time.time()

    # Get the driving command
    dc = drive_cam.GetDrivingCommand()

    # Update the vehicle with the driving command
    veh.Update(env, dc.throttle, dc.steering, dc.braking, dt)

    if n%5==0:
        # Update the drive camera
        drive_cam.SetPose(veh.GetPosition(),veh.GetOrientation())
        drive_cam.Update(env,0.05)
        drive_cam.Display()
    # Update the loop counter
    n = n+1

    elapsed_time = elapsed_time + dt
    tw1 = time.time()
    wall_dt = tw1-tw0
    if (wall_dt<dt):
        time.sleep(dt-wall_dt)