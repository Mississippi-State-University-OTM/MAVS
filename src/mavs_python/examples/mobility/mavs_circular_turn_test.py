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
    
soil_type = 'paved'
soil_strength = 150.0

radius = 50.0
if (len(sys.argv)>1):
    radius = float(sys.argv[1])

render = True

# Select a scene and load it
mavs_scenefile = "/scenes/surface_only.json"
#mavs_scenefile = "/scenes/surface_scene.json"
#mavs_scenefile = "/scenes/cube_scene.json"
scene = mavs.MavsEmbreeScene()
scene.Load(mavs_data_path+mavs_scenefile)

# Create a MAVS environment and add the scene to it
env = mavs.MavsEnvironment()
env.SetScene(scene)

theta = 0.0
waypoints = []
wp_x = []
wp_y = []
dtheta = 1.0/radius
while theta<2.0*math.pi:
    x = radius*math.cos(theta)
    y = radius*math.sin(theta)
    wp_x.append(x)
    wp_y.append(y)
    waypoints.append([x,y])
    theta = theta + dtheta

#Create and load a MAVS vehicle
veh = mavs.MavsRp3d()
#veh_file = 'forester_2017_rp3d_tires.json'
#veh_file = 'hmmwv_rp3d_tires.json'
veh_file = 'rc_car.json'
#veh_file = 'mrzr4_tires.json'
#veh_file = 'clearpath_warthog_cpu_tires.json'
veh.Load(mavs_data_path+'/vehicles/rp3d_vehicles/' + veh_file)
veh.SetInitialPosition(waypoints[0][0],waypoints[0][1],0.0) # in global ENU
veh.SetInitialHeading(1.57) # in radians
veh.Update(env, 0.0, 0.0, 1.0, 0.000001)
veh.SetTerrainProperties(terrain_type='flat',terrain_param1=0.0, terrain_param2=0.0, soil_type=soil_type, soil_strength=soil_strength)

# Create a visualization window for driving the vehicle with the W-A-S-D keys
# window must be highlighted to input driving commands
drive_cam = mavs.MavsCamera()
drive_cam.Initialize(384,384,0.0035,0.0035,0.0035)
drive_cam.SetOffset([-10.0,0.0,3.0],[1.0,0.0,0.0,0.0])
drive_cam.SetGammaAndGain(0.6,1.0)
drive_cam.RenderShadows(False)

controller = mavs.MavsVehicleController()
controller.SetDesiredPath(waypoints)
desired_speed = 5.0
controller.SetDesiredSpeed(desired_speed) # m/s 
#controller.SetSteeringScale(1.5)
#controller.SetWheelbase(3.8) # meters
#controller.SetMaxSteerAngle(0.855) # radians
#controller.SetMinLookAhead(3.0) # meters
#controller.SetMaxLookAhead(25.0) # meters
# rc car parameters
controller.SetSteeringScale(0.125)
controller.SetWheelbase(0.3) # meters, RC car
controller.SetMaxSteerAngle(0.6) # radians
controller.SetMinLookAhead(1.0) # meters
controller.SetMaxLookAhead(5.0) # meters
# turn on looping so we keep going around and around the circle
controller.TurnOnLooping()

soil_strength_max = 100.0
soil_strength_fade = 500.0

dt = 1.0/100.0 # time step, seconds
time_elapsed = 0.0
n = 0 # loop counter
old_velocity = 0.0
x_his = []
y_his = []
plot = mavs.MavsPlot()
plot.PlotTrajectory(wp_x,wp_y)
outfile  = open("vehicle_path.txt", "w")
while (True):
    tw0 = time.time()
    # Get the driving command 
    controller.SetCurrentState(veh.GetPosition()[0],veh.GetPosition()[1],
                                veh.GetSpeed(),veh.GetHeading())
    dc = controller.GetDrivingCommand(dt)

    # Update the vehicle
    veh.Update(env, dc.throttle, dc.steering, dc.braking, dt)

    #update the environment
    # Vehicle is always actor 0
    p = veh.GetPosition()
    orientation = veh.GetOrientation()
    env.SetActorPosition(0,p,orientation)
    env.AdvanceTime(dt)

    ## Update the camera sensors at 30 Hz
    if n%10==0 and render:
        drive_cam.SetPose(p,orientation)
        drive_cam.Update(env,dt)
        drive_cam.Display()

    x_his.append(veh.GetPosition()[0])
    y_his.append(veh.GetPosition()[1])

    outfile.write(str(veh.GetPosition()[0])+' '+str(veh.GetPosition()[1])+'\n')

    plot.AddToTrajectory(x_his,y_his)

    n = n+1
    time_elapsed = time_elapsed + dt

    #tw1 = time.time()
    #wall_dt = tw1-tw0
    #if (wall_dt<dt and render):
    #    time.sleep(dt-wall_dt)

outfile.close()

