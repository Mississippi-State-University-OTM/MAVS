import time
import sys
# Set the path to the mavs python api, mavs_interface.py
sys.path.append(r'C:/Users/cgoodin/Desktop/vm_shared/shared_repos/mavs/src/mavs_python')

# Load the mavs python modules
import mavs_interface
import mavs_python_paths
# Set the path to the mavs data folder
mavs_data_path = mavs_python_paths.mavs_data_path
    
render = False
if (len(sys.argv)>1):
    arg = int(sys.argv[1])
    if arg>0:
        render = True

# Select a scene and load it
mavs_scenefile = "/scenes/surface_only.json"
#mavs_scenefile = "/scenes/surface_scene.json"
scene = mavs_interface.MavsEmbreeScene()
scene.Load(mavs_data_path+mavs_scenefile)

# Create a MAVS environment and add the scene to it
env = mavs_interface.MavsEnvironment()
env.SetScene(scene.scene)

#Create and load a MAVS vehicle
veh = mavs_interface.MavsRp3d()
veh_file = 'forester_2017_rp3d.json'
veh.Load(mavs_data_path+'/vehicles/rp3d_vehicles/' + veh_file)
veh.SetInitialPosition(0.0, 0.0, 0.0) # in global ENU
veh.SetInitialHeading(0.0) # in radians
veh.Update(env, 0.0, 0.0, 1.0, 0.000001)
veh.SetTerrainProperties(terrain_type='rough',terrain_param1=1.0, terrain_param2=0.0, soil_type='clay', soil_strength=500.0)

# Create a visualization window for driving the vehicle with the W-A-S-D keys
# window must be highlighted to input driving commands
drive_cam = mavs_interface.MavsCamera()
drive_cam.Initialize(384,384,0.0035,0.0035,0.0035)
drive_cam.SetOffset([-10.0,0.0,3.0],[1.0,0.0,0.0,0.0])
drive_cam.SetGammaAndGain(0.6,1.0)
drive_cam.RenderShadows(False)

# Load waypoints and create controller to follow them
waypoints = mavs_interface.MavsWaypoints()
#waypoints_file = 'spa_city_outer_loop.vprp'
waypoints_file = 'x_axis_points.vprp'
waypoints.Load(mavs_data_path+'/waypoints/'+waypoints_file)
waypoints.FillIn(0.5)
controller = mavs_interface.MavsVehicleController()
controller.SetDesiredPath(waypoints.GetWaypoints2D())
desired_speed = 5.0
controller.SetDesiredSpeed(desired_speed) # m/s 
controller.SetSteeringScale(2.35)
controller.SetWheelbase(3.8) # meters
controller.SetMaxSteerAngle(0.855) # radians
controller.TurnOnLooping()

soil_strength_max = 100.0
soil_strength_fade = 500.0

dt = 1.0/30.0 # time step, seconds
time_elapsed = 0.0
n = 0 # loop counter
while (True):
    tw0 = time.time()
    # Get the driving command 
    controller.SetCurrentState(veh.GetPosition()[0],veh.GetPosition()[1],
                                veh.GetSpeed(),veh.GetHeading())
    dc = controller.GetDrivingCommand(dt)

    # Update the vehicle
    veh.Update(env, dc.throttle, dc.steering, dc.braking, dt)

    #update the environment
    p = veh.GetPosition()

    soil_strength_ = (soil_strength_max/soil_strength_fade)*(soil_strength_fade-p[0])
    if (veh.GetSpeed()<0.5*desired_speed and time_elapsed>5.0):
        print('VCI1 = '+str(soil_strength_))
        sys.exit()

    veh.SetTerrainProperties(terrain_type='flat',terrain_param1=0.0, terrain_param2=0.0, soil_type='clay', soil_strength=soil_strength_)

    # Vehicle is always actor 0
    orientation = veh.GetOrientation()
    env.SetActorPosition(0,p,orientation)
    env.AdvanceTime(dt)

    ## Update the camera sensors at 30 Hz
    if n%3==0 and render:
        drive_cam.SetPose(p,orientation)
        drive_cam.Update(env,dt)
        drive_cam.Display()

    n = n+1
    time_elapsed = time_elapsed + dt

    tw1 = time.time()
    wall_dt = tw1-tw0
    if (wall_dt<dt and render):
        time.sleep(dt-wall_dt)

