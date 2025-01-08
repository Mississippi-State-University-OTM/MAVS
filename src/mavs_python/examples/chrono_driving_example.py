import time
import sys
# Set the path to the mavs python api, mavs.py
sys.path.append('/your/path/to/mavs/src/mavs_python')
# Load the mavs python modules
import mavs_interface as mavs
import mavs_python_paths
# Set the path to the mavs data folder
mavs_data_path = mavs_python_paths.mavs_data_path

# Select a scene and load it
mavs_scenefile = "/scenes/cube_scene.json"
scene = mavs.MavsEmbreeScene()
scene.Load(mavs_data_path+mavs_scenefile)

# Create a MAVS environment and add the scene to it
env = mavs.MavsEnvironment()
env.SetScene(scene)

# add the rendering to the simulation
env.AddActor(mavs_data_path+'/actors/actors/hmmwv_actor.json',False)
#env.AddActor(mavs_data_path+'/actors/actors/lambo_actor_outer_loop.json')

#Create and load a Chrono vehicle throug the MAVS interface
veh = mavs.ChronoVehicle() 
# chrono vehicle files are in the mavs "data/vehicles/chrono_inputs" folder
veh_file = 'hmmwv.json'
veh.Load(mavs_data_path+'/vehicles/chrono_inputs/' + veh_file)
# Starting point for the vehicle
veh.SetInitialPosition(0.0, 0.0, 0.0) # in global ENU
# Initial Heading for the vehicle, 0=X, pi/2=Y, pi=-X
veh.SetInitialHeading(0.0) # in radians
veh.Update(env, 0.0, 0.0, 0.0, 0.000001)

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
drive_cam.RenderShadows(False)

# Now start the simulation main loop
dt = 1.0/100.0 # time step, seconds
n = 0 # loop counter
while (True):
    # tw0 is for timing purposes used later
    tw0 = time.time()

    # Get the driving command
    dc = drive_cam.GetDrivingCommand()

    # Update the vehicle with the driving command
    # scale the steering because chrono has a tendency to flip
    veh.Update(env, dc.throttle, dc.steering, dc.braking, dt)

    # Get the current vehicle position
    position = veh.GetPosition()
    orientation = veh.GetOrientation()

    # Update the animated vehicle position
    # The Ego-Vehicle is always actor 0
    env.SetActorPosition(0,position,orientation)
    # The AdvanceTime method updates the other actors
    env.AdvanceTime(dt)

    # Update the camera sensors at 10 Hz
    # Each sensor calls three functions
    # "SetPose" aligns the sensor with the current vehicle position,
    # the offset is automatically included.
    # "Update" creates new sensor data, point cloud or image
    # "Display" is optional and opens a real-time display window
    if n%10==0:
        # Update the drive camera
        drive_cam.SetPose(position,orientation)
        drive_cam.Update(env,10*dt)
        drive_cam.Display()
    # Update the loop counter
    n = n+1

    # The following lines ensure that the sim
    # doesn't run faster than real time, which 
    # makes it hard to drive
    tw1 = time.time()
    wall_dt = tw1-tw0
    if (wall_dt<dt):
        time.sleep(dt-wall_dt)