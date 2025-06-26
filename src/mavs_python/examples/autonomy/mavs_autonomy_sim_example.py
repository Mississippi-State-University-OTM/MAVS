import sys
from math import sqrt
import time
from noise import pnoise2
# Set the path to the mavs python api, mavs.py
sys.path.append(r'C:/Users/cgoodin/Desktop/goodin_docs/repos/mavs/src/mavs_python')
# Load the mavs python modules
import mavs_interface as mavs
import mavs_python_paths
# Set the path to the mavs data folder
mavs_data_path = mavs_python_paths.mavs_data_path
# Import the additional autonomy modules
import perception
import astar
import potential_field
import rrt
import rrt_star

# Set some simulation properties
# Set to True to show output on screen
display_debug = True
# Goal coordinate in local ENU
goal_point = [75.0, 75.0]
# start coordinate in locan ENU
start_point = [0.0, 0.0]

# Select a scene to load
mavs_scenefile = "/scenes/cube_scene.json"
# Create a MAVS environment and add the scene to it
env = mavs.MavsEnvironment()
env.LoadScene(mavs_data_path+mavs_scenefile)
env.SetTime(13) # 0-23

# Create a MAVS lidar and set the offset
lidar = mavs.MavsLidar('VLP-16')
lidar.SetOffset([0.0, 0.0, 2.0],[1.0,0.0,0.0,0.0])

# Create a window for viewing the vehicle
drive_cam = mavs.MavsCamera()
drive_cam.Initialize(640,480,0.0036915,0.00328125,0.0035)
# offset of camera from vehicle CG
drive_cam.SetOffset([-9.0,0.0,2.5],[1.0,0.0,0.0,0.0])
# Set camera compression and gain
drive_cam.SetGammaAndGain(0.85,1.25)
# Turn off shadows for this camera for efficiency purposes
drive_cam.RenderShadows(True)

# Load a MAVS vehicle
veh = mavs.MavsRp3d()
veh_file = 'mrzr4_tires_low_gear.json' 
veh.Load(mavs_data_path+'/vehicles/rp3d_vehicles/' + veh_file)
# Starting point for the vehicle
veh.SetInitialPosition(start_point[0], start_point[1], 0.0) # in global ENU
# Initial Heading for the vehicle, 0=X, pi/2=Y, pi=-X
veh.SetInitialHeading(0.0) # in radians
veh.Update(env, 0.0, 0.0, 1.0, 0.000001)

# Create a vehicle controller
controller = mavs.MavsVehicleController()
# Tell the controller how fast to drive the vehicle
controller.SetDesiredSpeed(5.0) # m/s 
# The next three lines set controller parameters that steer the vehicle. 
controller.SetSteeringScale(3.0)
controller.SetWheelbase(3.5) # meters
controller.SetMaxSteerAngle(0.44) # radians

# Set the planner type to 'rrt', 'rrt_star', 'astar' or 'potential'
# Read input from the command line
planner_type = 'astar'
if len(sys.argv)>1:
    planner_type = str(sys.argv[1])
# create the planner based on user input
planner = None
if (planner_type=='astar'):
   planner = astar.AstarPlanner()
elif(planner_type=='potential'):
    planner = potential_field.PotentialFieldPlanner()
elif(planner_type=='rrt'):
    planner =rrt.RRT()
elif(planner_type=='rrt_star'):
    planner =rrt_star.RRTStar()
else:
    print('Error: Planner type '+str(planner_type)+' not recognized.')
    exit()

# Create and occupancy grid and resize it
grid = perception.LidarGrid()
# This is the number of cells in each dimension
grid.resize(400,400)
# Set the resolution (in meters) of the grid
grid.info.resolution = 0.5
# Set the origin of the grid (lower left corner)
# upper right corner = lowerleft + dimension*res
grid.set_origin(-100.0,-100.0)
# Set the height threshold for an obstacle
grid.height_thresh = 0.5
# This will inflate the size of obstacles
# default is zero
grid.inflation = 4;

dt = 1.0/100.0 # time step, seconds
n = 0 # loop counter
total_sim_time = 0.0
goal_thresh_dist = 4.0
dist_to_goal = 100.0
while dist_to_goal>goal_thresh_dist and total_sim_time<100.0:
    
    # Update the driving command using the controller
    controller.SetCurrentState(veh.GetPosition()[0],veh.GetPosition()[1],veh.GetSpeed(),veh.GetHeading())
    dc = controller.GetDrivingCommand(dt)

    # Update the vehicle
    veh.Update(env, dc.throttle, dc.steering, dc.braking, dt)

    # Get the current vehicle position and orientation
    position = veh.GetPosition()
    orientation = veh.GetOrientation()

    # Check to see if we've advanced towards our goal
    dist_to_goal = sqrt(pow(position[0]-goal_point[0],2)+pow(position[1]-goal_point[1],2))

    # Update the sensors and recalculate the path at 10 Hz
    if n%10==0 and n>0:
        # Update and display the drive camera
        # which is for visualization purposes only.
        if (display_debug):
            drive_cam.SetPose(position,orientation)
            drive_cam.Update(env,10.0*dt)
            drive_cam.Display()    
        # Update and display the lidar
        lidar.SetPose(position,orientation)
        lidar.Update(env,10.0*dt)
        if (display_debug):
            lidar.Display()   
        # Get lidar point cloud registered to world coordinates
        registered_points = lidar.GetPoints()

        # add the points to the grid
        grid.add_registered_points(registered_points)

        # get the path from the planner
        path,path_enu = planner.plan(grid,position[0],position[1],goal_point[0],goal_point[1])

        if path:
            # Update the controller path
            controller.SetDesiredPath(path_enu)
            grid.SetCurrentPath(path)
        else:
            print("WARNING: PATH NOT FOUND" )

        # Display the current occupancy grid
        if (display_debug):
            grid.display()
    
    # Update the loop counter
    n = n + 1
    total_sim_time = total_sim_time + dt
