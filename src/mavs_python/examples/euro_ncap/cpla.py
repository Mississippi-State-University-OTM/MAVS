'''
Non-Commercial License - Mississippi State University Autonomous Vehicle Software (MAVS)

ACKNOWLEDGEMENT:
Mississippi State University, Center for Advanced Vehicular Systems (CAVS)

*NOTICE*
Thank you for your interest in MAVS, we hope it serves you well!  We have a few small requests to ask of you that will help us continue to create innovative platforms:
-To share MAVS with a friend, please ask them to visit https://gitlab.com/cgoodin/msu-autonomous-vehicle-simulator to register and download MAVS for free.
-Feel free to create derivative works for non-commercial purposes, i.e. academics, U.S. government, and industry research.  If commercial uses are intended, please contact us for a commercial license at otm@msstate.edu or jonathan.rudd@msstate.edu.
-Finally, please use the ACKNOWLEDGEMENT above in your derivative works.  This helps us both!

Please visit https://gitlab.com/cgoodin/msu-autonomous-vehicle-simulator to view the full license, or email us if you have any questions.

Copyright 2018 (C) Mississippi State University
'''
import sys

#sys.path.append(r'C:\Users\cgoodin\Desktop\code\mavs\src\mavs_python')
sys.path.append(r'C:\Users\cgoodin\Desktop\vm_shared\shared_repos\mavs\src\mavs_python')

# Load the mavs python modules
import mavs_interface
import mavs_python_paths
import math
mavs_data_path = mavs_python_paths.mavs_data_path

# set up test parameters
target_speed = 1.39 
target_dist = 6.0
vehicle_speed = 10.0
wait_time = 1.5
start_line = -30.0 
finish_line = 45.0

# create a path for the vehicle to follow
nx_steps = 100
dx = (finish_line-start_line)/nx_steps
path = []
for i in range(nx_steps):
	x = start_line+i*dx
	y = 0.0
	path.append([x,y])

#Create the vehicle
veh = mavs_interface.MavsRp3d()
veh_file = 'forester_2017_rp3d.json'
veh.Load(mavs_data_path+'/vehicles/rp3d_vehicles/' + veh_file)
veh.SetInitialPosition(start_line, 0.0, 1.0)
veh.SetInitialHeading(0.0)

#----- Sensor Creation and parameters ------------------#
# Create a MAVS camera and initialize it
cam = mavs_interface.MavsCamera()
cam.Initialize(384,384,0.0035,0.0035,0.0035)
#offset of the camera from the vehicle cg, [x,y,z] and quaternion
cam.SetOffset([0.0,0.0,1.5],[1.0,0.0,0.0,0.0])
cam.RenderShadows(False)
cam.SetGammaAndGain(0.5,2.0)
# Create a MAVS lidar
lidar = mavs_interface.MavsLidar('VLP16')
lidar.SetOffset([0.0,0.0,1.5],[1.0,0.0,0.0,0.0])

# Load a scene
mavs_scenefile = "/scenes/concrete_surface.json"
scene = mavs_interface.MavsEmbreeScene()
scene.Load(mavs_data_path+mavs_scenefile)

# Create and animation and add it to the scene
animation = mavs_interface.MavsAnimation()
animation.Load((mavs_data_path+"/scenes/meshes/animations/GenericWalk"),
(mavs_data_path+"/scenes/meshes/animations/GenericWalk/walk_frames.txt"))
animation.SetScale(0.01)
animation.SetRotations(True,False)
animation.SetBehavior('straight')
animation.SetPosition(0.0,0.0)
animation.SetHeading(0.0)
animation.SetSpeed(target_speed) 
scene.AddAnimation(animation)

# Create the environment and set properties
env = mavs_interface.MavsEnvironment()
env.SetScene(scene.scene)
env.SetCloudCover(0.4)
env.SetTurbidity(5.0)
env.SetFog(0.025)

#Create a vehicle controller
controller = mavs_interface.MavsVehicleController()
controller.SetDesiredSpeed(vehicle_speed) 
controller.SetDesiredPath(path)
controller.SetSteeringScale(0.6)
controller.SetWheelbase(1.25)
controller.SetMaxSteerAngle(0.75)
#current_driving_command = mavs_interface.MavsDrivingCommand()

# set up timing variables
dt = 0.01
camera_dt = 0.05
lidar_dt = 0.1
camera_elapsed = 0.0
lidar_elapsed = 0.0
elapsed_time = 0.0
env.AdvanceTime(0.0001)
#----- Simulation -------------------------------------#
while(True):
    # update the vehicle
    if (elapsed_time>wait_time):
        current_pos = veh.GetPosition()
        controller.SetCurrentState(current_pos[0],current_pos[1],veh.GetSpeed(),veh.GetHeading())
        current_driving_command = controller.GetDrivingCommand(dt)
        veh.Update(env,current_driving_command.throttle, current_driving_command.steering,current_driving_command.braking, dt)
    else:
        veh.Update(env, 0.0, 0.0, 1.0, dt)

    # get the current sensor orientation
    camera_pos = veh.GetPosition()
    camera_ori = veh.GetOrientation()
	
    # Move the vehicle actor
    env.SetActorPosition(0,camera_pos,camera_ori)

	# advance the environment, which moves animations
    env.AdvanceTime(dt)
	
    # Update the camera and lidar sensor
    if (lidar_elapsed >=lidar_dt):
        lidar.SetPose(camera_pos, camera_ori)
        lidar.Update(env,dt)
        lidar.DisplayPerspective()
        lidar_elapsed = 0.0
    if (camera_elapsed>=camera_dt):
        cam.SetPose(camera_pos, camera_ori)
        cam.Update(env,dt)
        cam.Display()
        camera_elapsed = 0.0
	# update the timers 
    camera_elapsed = camera_elapsed + dt
    lidar_elapsed = lidar_elapsed + dt
    elapsed_time = elapsed_time + dt

