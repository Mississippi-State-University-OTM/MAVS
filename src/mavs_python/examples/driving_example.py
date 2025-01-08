import time
import sys
import math
# Set the path to the mavs python api, mavs.py
sys.path.append(r'C:/Users/cgoodin/Desktop/goodin_docs/repos/mavs/src/mavs_python')
# Load the mavs python modules
import mavs_interface as mavs
import mavs_python_paths
# Set the path to the mavs data folder
mavs_data_path = mavs_python_paths.mavs_data_path

# Select a scene and load it
#mavs_scenefile = "/scenes/cube_scene.json"
#scene = mavs.MavsEmbreeScene()
#scene.Load(mavs_data_path+mavs_scenefile)

random_scene = mavs.MavsRandomScene()
random_scene.terrain_width = 250.0
random_scene.terrain_length = 250.0
random_scene.lo_mag = 0.0
random_scene.hi_mag = 0.05
random_scene.mesh_resolution=0.3
random_scene.plant_density = 0.0 
random_scene.trail_width = 0.0
random_scene.track_width = 0.0
random_scene.wheelbase = 0.0
random_scene.surface_roughness_type = "variable"
scene_name = 'bumpy_surface'
random_scene.basename = scene_name
random_scene.eco_file = 'american_pine_forest.json'
#random_scene.eco_file = 'american_southwest_desert.json'
random_scene.path_type = 'Ridges'
random_scene.CreateScene()

# Create a MAVS environment and add the scene to it
env = mavs.MavsEnvironment()
#env.SetScene(scene.scene)
env.SetScene(random_scene.scene)

# Set environment properties
env.SetTime(13) # 0-23
env.SetFog(0.0) # 0.0-100.0
env.SetSnow(0.0) # 0-25
env.SetTurbidity(7.0) # 2-10
env.SetAlbedo(0.1) # 0-1
env.SetCloudCover(0.5) # 0-1
env.SetRainRate(0.0) # 0-25
env.SetWind( [2.5, 1.0] ) # Horizontal windspeed in m/s

#Create and load a MAVS vehicle
veh = mavs.MavsRp3d()
# vehicle files are in the mavs "data/vehicles/rp3d_vehicles" folder
#veh_file = 'forester_2017_rp3d.json'
veh_file = 'mrzr4_tires.json'
#veh_file = 'clearpath_warthog.json'
#veh_file = 'hmmwv_rp3d.json'
#veh_file = 'mrzr4.json'
#veh_file = 'sedan_rp3d.json'
#veh_file = 'cucv_laredo_rp3d.json'
veh.Load(mavs_data_path+'/vehicles/rp3d_vehicles/' + veh_file)
# Starting point for the vehicle
#veh.SetInitialPosition(-52.5, 7.5, 0.0) # in global ENU
veh.SetInitialPosition(100.0, 0.0, 0.0) # in global ENU
#veh.SetInitialPosition(65.125, 35.0, 0.0) # in global ENU
# Initial Heading for the vehicle, 0=X, pi/2=Y, pi=-X
veh.SetInitialHeading(0.0) # in radians
#veh.SetInitialHeading(-1.57) # in radians
veh.Update(env, 0.0, 0.0, 1.0, 0.000001)

# Create a window for driving the vehicle with the W-A-S-D keys
# window must be highlighted to input driving commands
drive_cam = mavs.MavsCamera()
# nx,ny,dx,dy,focal_len
drive_cam.Initialize(256,256,0.0035,0.0035,0.0035)
# offset of camera from vehicle CG
drive_cam.SetOffset([-10.0,0.0,3.0],[1.0,0.0,0.0,0.0])
# Set camera compression and gain
#drive_cam.SetGammaAndGain(0.6,1.0)
drive_cam.SetGammaAndGain(0.5,2.0)
# Turn off shadows for this camera for efficiency purposes
drive_cam.RenderShadows(True)

front_cam = mavs.MavsCamera()
# nx,ny,dx,dy,focal_len
front_cam.Initialize(256,256,0.0035,0.0035,0.0035)
# offset of camera from vehicle CG
angle = 135.0
front_cam.SetOffset([3.5,-2.6,0.0],[math.cos(0.5*math.radians(angle)),0.0, 0.0, math.sin(0.5*math.radians(angle))])
#angle = 90.0
#front_cam.SetOffset([1.5,-2.6,0.0],[math.cos(0.5*math.radians(angle)),0.0, 0.0, math.sin(0.5*math.radians(angle))])
# Set camera compression and gain
#drive_cam.SetGammaAndGain(0.6,1.0)
front_cam.SetGammaAndGain(0.5,2.0)
# Turn off shadows for this camera for efficiency purposes
front_cam.RenderShadows(True)

# Now start the simulation main loop
#dt = 1.0/30.0 # time step, seconds
dt = 1.0/100.0 # time step, seconds
n = 0 # loop counter
elapsed_time = 0.0
while (True):
    # tw0 is for timing purposes used later
    tw0 = time.time()

    # Get the driving command
    dc = drive_cam.GetDrivingCommand()

    # Update the vehicle with the driving command
    veh.Update(env, dc.throttle, dc.steering, dc.braking, dt)

    # Get the current vehicle position
    position = veh.GetPosition()
    orientation = veh.GetOrientation()

    # The AdvanceTime method updates the other actors
    env.AdvanceTime(dt)

    # Update the camera sensors at 10 Hz
    # Each sensor calls three functions
    # "SetPose" aligns the sensor with the current vehicle position,
    # the offset is automatically included.
    # "Update" creates new sensor data, point cloud or image
    # "Display" is optional and opens a real-time display window
    if n%4==0:
        # Update the drive camera
        drive_cam.SetPose(position,orientation)
        drive_cam.Update(env,dt)
        drive_cam.Display()
        front_cam.SetPose(position,orientation)
        front_cam.Update(env,dt)
        front_cam.Display()
    if n%10==0:
        #print(veh.GetLongitudinalAcceleration(),veh.GetLateralAcceleration())
        #print(veh.GetTireNormalForce(0),veh.GetTireNormalForce(1),veh.GetTireNormalForce(2),veh.GetTireNormalForce(3))
        print(position)
        #print(elapsed_time,veh.GetTireSlip(0),veh.GetTireAngularVelocity(0),veh.GetTireForces(0)[0],veh.GetVelocity()[0])
        #print(veh.GetTireAngularVelocity(0))
        sys.stdout.flush()
    # Update the loop counter
    n = n+1

    # The following lines ensure that the sim
    # doesn't run faster than real time, which 
    # makes it hard to drive
    elapsed_time = elapsed_time + dt
    tw1 = time.time()
    wall_dt = tw1-tw0
    if (wall_dt<dt):
        time.sleep(dt-wall_dt)