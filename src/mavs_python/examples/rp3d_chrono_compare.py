# This file runs a MAVS-RP3D vehicle and a MAVS-Chrono vehicle through a 
# double lane change at the same time for purposes of comparison
import matplotlib
import matplotlib.pyplot as plt
import time
import sys
# Set the path to the mavs python api, mavs.py
sys.path.append(r'C:/Users/cgoodin/Desktop/vm_shared/shared_repos/mavs/src/mavs_python')
# Load the mavs python modules
import mavs_interface as mavs
import mavs_python_paths
# Set the path to the mavs data folder
mavs_data_path = mavs_python_paths.mavs_data_path


render_debug = True
test_speed = 5.0
if (len(sys.argv)>1):
    test_speed = float(sys.argv[1])

start_point = -100.0 # Ramp up lane length, course starts at x=0.0

# Define the waypoints for the double lane change maneuver
waypoints =[[0,0],[2,0],[4,0],[6,0],[8,0],[10,0],[12,0],
            [13.5,0.4326],[15,0.8651],[16.5,1.2977],[18,1.7302],
            [19.5,2.1628],[21,2.5953],[22.5,3.0279],[24,3.4604],
            [25.5,3.893],[26.5,3.893],[27.5,3.893],[28.5,3.893],
            [29.5,3.893],[30.5,3.893],[31.5,3.893],[32.5,3.893],
            [33.5,3.893],[34.5,3.893],[35.5,3.893],[36.5,3.893],
            [38,3.42584],[39.5,2.95868],[41,2.49152],[42.5,2.02436],
            [44,1.5572],[45.5,1.09004],[47,0.62288],[48.5,0.15572],
            [49,0],[51,0],[53,0],[55,0],[57,0],[59,0],[61,0]]

# Set the timeout value based on desired speed 
max_time = 2.0*(61.0-start_point)/test_speed

# Select a scene and load it
mavs_scenefile = "/scenes/lane_change_scene.json"
scene = mavs.MavsEmbreeScene()
scene.Load(mavs_data_path+mavs_scenefile)

# Create a MAVS environment and add the scene to it
env = mavs.MavsEnvironment()
env.SetScene(scene)

# add the rendering of the chrono vehicle to the simulation
env.AddActor(mavs_data_path+'/actors/actors/red_hmmwv_actor.json',False)

# Create and load the chrono vehicle
chrono_veh = mavs.ChronoVehicle() 
chrono_veh_file = 'hmmwv.json'
chrono_veh.SetInitialPosition(start_point, 0.0, 1.0) # in global ENU
chrono_veh.SetInitialHeading(0.0) # in radians
chrono_veh.Load(mavs_data_path+'/vehicles/chrono_inputs/' + chrono_veh_file)
chrono_veh.Update(env, 0.0, 0.0, 0.0, 0.000001)

# Create and load a RP3D vehicle
rp3d_veh = mavs.MavsRp3d()
rp3d_veh_file = 'hmmwv_rp3d.json'
#rp3d_veh_file = 'mrzr4.json'
rp3d_veh.Load(mavs_data_path+'/vehicles/rp3d_vehicles/' + rp3d_veh_file)
rp3d_veh.SetInitialPosition(start_point, 0.0, 0.0) # in global ENU
rp3d_veh.SetInitialHeading(0.0) # in radians
rp3d_veh.SetUseDrag(False)
rp3d_veh.Update(env, 0.0, 0.0, 1.0, 0.000001)

# Create a vehicle controller for the RP3D vehicle
rp3d_controller = mavs.MavsVehicleController()
rp3d_controller.SetDesiredSpeed(test_speed) # m/s 
rp3d_controller.SetSteeringScale(1.0)
rp3d_controller.SetWheelbase(2.4) # meters
rp3d_controller.SetMaxSteerAngle(0.855) # radians
rp3d_controller.SetDesiredPath(waypoints)
#rp3d_controller.SetSpeedControllerParams(0.5,0.0, 0.0)
# Create the controller for the Chrono vehicle
chrono_controller = mavs.MavsVehicleController()
chrono_controller.SetDesiredSpeed(test_speed) # m/s 
chrono_controller.SetSteeringScale(1.0)
chrono_controller.SetWheelbase(2.4) # meters
#chrono_controller.SetMaxSteerAngle(0.855) # radians
chrono_controller.SetMaxSteerAngle(0.20) # radians
chrono_controller.SetDesiredPath(waypoints)

# Create cameras for viewing the simulation
rear_cam = mavs.MavsCamera()
rear_cam.Initialize(256,256,0.0035,0.0035,0.0035)
rear_cam.SetOffset([-10.0,0.0,3.0],[1.0,0.0,0.0,0.0])
rear_cam.SetGammaAndGain(0.5,2.0)
rear_cam.RenderShadows(True)
side_cam = mavs.MavsCamera()
side_cam.Initialize(256,256,0.0035,0.0035,0.0035)
side_cam.SetOffset([0.0, 15.0, 3.0],[0.7071,0.0,0.0, -0.7071])
side_cam.SetGammaAndGain(0.5,2.0)
side_cam.RenderShadows(True)

# open files for writing out results
f_rp3d = open("rp3d_out_"+str(test_speed)+".txt","w")
f_chrono = open("chrono_out_"+str(test_speed)+".txt","w")

rp3d_x = []
rp3d_y = []
chrono_x = []
chrono_y = []

# Start the simulation main loop
dt = 1.0/100.0 # time step, seconds
n = 0 # loop counter
elapsed_time = 0.0
while (elapsed_time<max_time):

    # Getdriving commands for the Chrono vehicle
    chrono_controller.SetCurrentState(chrono_veh.GetPosition()[0],chrono_veh.GetPosition()[1],chrono_veh.GetSpeed(),chrono_veh.GetHeading())
    chrono_dc = chrono_controller.GetDrivingCommand(dt)
    chrono_position = chrono_veh.GetPosition()
    chrono_orientation = chrono_veh.GetOrientation()

    # Get driving commands for the rp3d vehicle
    rp3d_controller.SetCurrentState(rp3d_veh.GetPosition()[0],rp3d_veh.GetPosition()[1],rp3d_veh.GetSpeed(),rp3d_veh.GetHeading())
    rp3d_dc = rp3d_controller.GetDrivingCommand(dt)
    rp3d_position = rp3d_veh.GetPosition()
    rp3d_orientation = rp3d_veh.GetOrientation()
        
    # Perform some simulation control checks
    if (n<100): 
        # 1 second of "settle" time
        rp3d_dc.throttle = 0.0
        chrono_dc.throttle = 0.0
        chrono_dc.steering = 0.0
    if (rp3d_position[0]>=0.0): 
        # release throttle at course start
        rp3d_dc.throttle = 0.0
    if (chrono_position[0]>=0.0): 
        # release throttle at course start
        chrono_dc.throttle = 0.0
    if (chrono_veh.GetSpeed()>test_speed): 
        # Chrono vehicle needs brakes or it will go over set speed
        chrono_dc.braking = 0.1
        chrono_dc.throttle = 0.0
    if (chrono_position[0]>=61.0 and rp3d_position[0]>61.0): 
        # end sim when both vehicles reach course end
        break
         
    # Update both vehicles at 100 Hz
    rp3d_veh.Update(env, rp3d_dc.throttle, rp3d_dc.steering, rp3d_dc.braking, dt)
    chrono_veh.Update(env, chrono_dc.throttle, chrono_dc.steering, chrono_dc.braking, dt)

    # Write output and show debug at 10 Hz
    if n%10==0:
        # Write the vehicle position and speed to a file
        f_rp3d.write(str(elapsed_time)+" "+str(rp3d_position[0])+" "+str(rp3d_position[1])+" "+str(rp3d_veh.GetSpeed())+"\n")
        f_chrono.write(str(elapsed_time)+" "+str(chrono_position[0])+" "+str(chrono_position[1])+" "+str(chrono_veh.GetSpeed())+"\n")
            
        # Add data to arrays for plotting later
        rp3d_x.append(rp3d_position[0])
        rp3d_y.append(rp3d_position[1])
        chrono_x.append(chrono_position[0])
        chrono_y.append(chrono_position[1])

        # Write out the vehicle speeds for debugging purposes
        print('rp3d   : ',rp3d_veh.GetSpeed(),rp3d_dc.throttle)
        print('chrono : ',chrono_veh.GetSpeed(),chrono_dc.throttle)
        print(' ')
        sys.stdout.flush()

        if render_debug:
            # Update the animated vehicle position
            env.SetActorPosition(0,chrono_position,chrono_orientation)
            #env.SetActorPosition(1,rp3d_position,rp3d_orientation)

            # Update the camera views
            rear_cam.SetPose(rp3d_position,rp3d_orientation)
            rear_cam.Update(env,10.0*dt)
            rear_cam.Display()
            side_cam.SetPose(rp3d_position,rp3d_orientation)
            side_cam.Update(env,10.0*dt)
            side_cam.Display()

    # Update the loop counter
    n = n+1
    elapsed_time = elapsed_time + dt

# Plot the path taken by each vehicle
plt.plot(chrono_x, chrono_y, label="chrono")
plt.plot(rp3d_x, rp3d_y, label="rp3d")
plt.xlim([0.0,60.0])
plt.ylim([0.0,5.0])
plt.legend()
plt.show()

# Close the data files
f_rp3d.close()
f_chrono.close()