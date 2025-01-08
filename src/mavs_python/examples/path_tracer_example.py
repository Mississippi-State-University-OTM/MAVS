''' Script for generating labeled lidar and camera data.'''
import random
import sys
import time
import math

# Set the path to the mavs python api, mavs_interface.py
# you will have to change this on your system
sys.path.append(r'C:\Users\cgoodin\Desktop\code\mavs\src\mavs_python')

# Load the mavs python modules
import mavs_interface as mavs
import mavs_python_paths

#----- Sensor Creation and parameters ------------------#
# Create a MAVS camera, initialize it, and set properties
cam = mavs.MavsCamera()
cam.Model('MachineVision')
# resolution values are 'low', 'medium', or 'high'
pt_cam = mavs.MavsPathTraceCamera('low', 250, 15, 0.55) # 0.5-0.8 secs/frame
day_cam = mavs.MavsPathTraceCamera('low', 200, 10, 0.55) # 0.5-0.8 secs/frame
vlp_lidar = mavs.MavsLidar('VLP-16')
vel_lidar = mavs.MavsLidar('HDL-64E')


#offset of the camera from the vehicle cg, [x,y,z] and quaternion
pt_cam.SetOffset([-3.5, 0.0, 1.830],[1.0,0.0,0.0,0.0])
pt_cam.SetGammaAndGain(0.5,1.0)
pt_cam.SetNormalizationType('max')
day_cam.SetOffset([-3.5, 0.0, 1.830],[1.0,0.0,0.0,0.0])
day_cam.SetGammaAndGain(0.8,1.0)
cam.SetOffset([-3.5, 0.0, 1.830],[1.0,0.0,0.0,0.0])
cam.RenderShadows(False)
vlp_lidar.SetOffset([0.0, 0.0, 1.0],[1.0,0.0,0.0,0.0])
vel_lidar.SetOffset([0.0, 0.0, 1.0],[1.0,0.0,0.0,0.0])

#----- Scene creation --------------------------------#
# create a randomized mavs scene
random_scene = mavs.MavsRandomScene()
random_scene.terrain_width = 50.0 #100.0 #150.0
random_scene.terrain_length = 50.0 #100.0 #150.0
random_scene.lo_mag = 5.0 #float(random.randrange(36,120))/10.0
random_scene.hi_mag = 0.0
#random_scene.plant_density = float(random.randrange(95,200))/100.0
random_scene.plant_density = 0.15 #0.35 #0.55 
random_scene.trail_width = 4.0
random_scene.track_width = 0.3
random_scene.wheelbase = 1.8
scene_name = 'my_scene'
random_scene.basename = scene_name
random_scene.eco_file = 'american_pine_forest.json'
#random_scene.eco_file = 'american_southeast_forest.json'
random_scene.path_type = 'Ridges'
random_scene.CreateScene()
random_scene.TurnOnLabeling()
#--- Load the waypoints that go with this scene ----#
waypoints = mavs.MavsWaypoints()
waypoints.Load('./'+random_scene.basename+'_path.vprp')
# Get the scene geometry and put the waypoints on the ground
scene = mavs.MavsEmbreeScene()
scene.Load(scene_name+'_scene.json')
scene.TurnOnLabeling()
waypoints.PutWaypointsOnGround(scene)

controller = mavs.MavsVehicleController()
controller.SetDesiredPath(waypoints.GetWaypoints2D())
controller.SetDesiredSpeed(5.0) # m/s 
controller.SetSteeringScale(2.5)
controller.SetWheelbase(3.1) # meters
controller.SetMaxSteerAngle(0.615) # radians

#----- Environment Creation ---------------------------#
# Create a MAVS environment and add the scene to it
env = mavs.MavsEnvironment()
env.SetScene(random_scene.scene)
env.SetCloudCover(0.0)
env.SetTurbidity(5.0)
env.SetTime(2)
day_env = mavs.MavsEnvironment()
day_env.SetScene(random_scene.scene)
day_env.SetCloudCover(0.0)
day_env.SetTurbidity(5.0)

#Create and load a MAVS vehicle
veh = mavs.MavsRp3d()
veh_file = 'forester_2017_rp3d.json'
wp0 = waypoints.GetWaypoint(0)
wp1 = waypoints.GetWaypoint(1)
veh.Load(mavs_python_paths.mavs_data_path+'/vehicles/rp3d_vehicles/' + veh_file)
veh.SetInitialPosition(wp0[0],wp0[1],wp0[2]+1.0) # in global ENU
heading = math.atan2(wp1[1]-wp0[1],wp1[0]-wp0[0])
veh.SetInitialHeading(heading) # in radians
veh.headlight_offset = 2.0
veh.headlight_width = 1.36
veh.AddHeadlights(env)
veh.Update(env, 0.0, 0.0, 1.0, 0.000001)

last_wp = waypoints.GetWaypoint(waypoints.GetNumWaypoints()-1)

def WpDist(wpa,wpb):
    ddd = math.sqrt(math.pow(wpa[0]-wpb[0],2)+math.pow(wpa[1]-wpb[1],2))
    return ddd

dt = 1.0/100.0

wp_dist = WpDist(veh.GetPosition(),last_wp)
#----- Simulation -------------------------------------#
# loop over all the poses in the waypoints list
i = 0
while (wp_dist>4.0):
#for i in range(waypoints.num_waypoints):

    controller.SetCurrentState(veh.GetPosition()[0],veh.GetPosition()[1],veh.GetSpeed(),veh.GetHeading())
    dc = controller.GetDrivingCommand(dt)

    # Update the vehicle
    veh.Update(env, dc.throttle, dc.steering, dc.braking, dt)

    if (i%10==0):
        position = veh.GetPosition() 
        orient = veh.GetOrientation()  
        env.SetActorPosition(0,position,orient)
        env.AdvanceTime(dt)
        day_env.SetActorPosition(0,position,orient)
        day_env.AdvanceTime(dt)        
        # create a base string for file output
        frame_name = 'frame_'+str(i).zfill(4)
        # Set the pose of the camera and render the frame
        t0 = time.time()
        pt_cam.SetPose(position,orient)
        pt_cam.Update(env,dt)
        pt_cam.Display()
        day_cam.SetPose(position,orient)
        day_cam.Update(day_env,dt)
        day_cam.Display()
        cam.SetPose(position,orient)
        cam.Update(day_env,dt)
        pt_cam.SaveCameraImage('night_'+frame_name+'.bmp')
        day_cam.SaveCameraImage('day_'+frame_name+'.bmp')
        cam.SaveAnnotation(env,('labeled_'+frame_name))
        vlp_lidar.SetPose(position,orient)
        vlp_lidar.Update(day_env,dt)
        vlp_lidar.DisplayPerspective()
        vlp_lidar.SaveLidarImage('lidar_'+frame_name+'.bmp')
        vel_lidar.SetPose(position,orient)
        vel_lidar.Update(day_env,dt)
        vel_lidar.DisplayPerspective()
        vel_lidar.SaveLidarImage('lidar_'+frame_name+'.bmp')
        t1 = time.time()
        print("Timing: "+str(t1-t0))
        sys.stdout.flush()
    i = i+1
    wp_dist = WpDist(veh.GetPosition(),last_wp)
