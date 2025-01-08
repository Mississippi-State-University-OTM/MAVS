import time
import sys
import math
import matplotlib.pyplot as plt
# Set the path to the mavs python api, mavs.py
sys.path.append(r'C:/Users/cgoodin/Desktop/vm_shared/shared_repos/mavs/src/mavs_python')
# Load the mavs python modules
import mavs_interface as mavs
import mavs_python_paths
# Set the path to the mavs data folder
mavs_data_path = mavs_python_paths.mavs_data_path

veh_file = 'mrzr4_tires.json'
if (len(sys.argv)>1):
    veh_file = sys.argv[1]

sys.stdout.flush()
random_scene = mavs.MavsRandomScene()
scene = mavs.MavsEmbreeScene()
# Create a MAVS environment and add the scene to it
env = mavs.MavsEnvironment()
mavs_scenefile = "/scenes/surface_only.json"
scene.Load(mavs_data_path+mavs_scenefile)
env.SetScene(scene)

#Create and load a MAVS vehicle
veh = mavs.MavsRp3d()
veh.Load(mavs_data_path+'/vehicles/rp3d_vehicles/' + veh_file)
veh.SetInitialPosition(-2000.0, 0.0, 0.0) # in global ENU
veh.SetInitialHeading(0.0) # in radians
veh.Update(env, 0.0, 0.0, 1.0, 0.000001)
# This has to come after the first update
#veh.SetTerrainProperties(terrain_type='flat',soil_type='clay', soil_strength=200.0)


# Create a window for driving the vehicle with the W-A-S-D keys
# window must be highlighted to input driving commands
drive_cam = mavs.MavsCamera()
# nx,ny,dx,dy,focal_len
drive_cam.Initialize(256,256,0.0035,0.0035,0.0035)
# offset of camera from vehicle CG
drive_cam.SetOffset([-10.0, 0.0, 1.0],[1.0,0.0,0.0,0.0])
# Set camera compression and gain
drive_cam.SetGammaAndGain(0.5,2.0)
# Turn off shadows for this camera for efficiency purposes
drive_cam.RenderShadows(False)

# Now start the simulation main loop
dt = 1.0/120.0 # time step, seconds
n = 0 # loop counter
elapsed_time = 0.0
front_tire_load = 0.0
rear_tire_load = 0.0
front_tire_deflection = 0.0
rear_tire_deflection = 0.0
cg_height = 0.0
load_measured = False
settle_time = 2.0
velocity = 0.0
max_velocity = 0.0
max_vel_found = False
max_vel_time = 0.0
applied_force = 0.0
tf = []
speed = []
while (True):

    # Update the vehicle with the driving command
    throttle = 0.0
    if (elapsed_time>settle_time):
        throttle = min(1.0, 0.05*(elapsed_time-settle_time))
    veh.Update(env, throttle, 0.0, 0.0, dt)

    # check if max speed is reached
    new_velocity = veh.GetSpeed()
    if (new_velocity<=velocity and elapsed_time>5.0+settle_time and not max_vel_found):
        max_velocity = velocity
        max_vel_found = True
        max_vel_time = elapsed_time
        print("found max velocity at time ", elapsed_time, max_velocity)
        sys.stdout.flush()
    velocity = new_velocity
    if (velocity>max_velocity):
        max_velocity = velocity
    #start applying the drawbar force
    if max_vel_found and elapsed_time-max_vel_time>15.0:
        if (n%240==0):
            tf.append(applied_force)
            s = veh.GetSpeed()
            if (s>max_velocity):
                max_velocity = s
            speed.append(s)
            applied_force = applied_force + 500.0 # newtons
            lt = veh.GetLookTo()
            print("Adding external force ",applied_force, speed[-1])
            sys.stdout.flush()
            veh.SetExternalForceOnCg(-lt[0]*applied_force, -lt[1]*applied_force, -lt[2]*applied_force)
        if (veh.GetSpeed()<1.0):
            print("reached max DBP at time ",elapsed_time)
            sys.stdout.flush()
            break
    # Get the current vehicle position
    position = veh.GetPosition()
    orientation = veh.GetOrientation()

    if n%4==0:
        drive_cam.SetPose(position,orientation)
        drive_cam.Update(env,0.05)
        drive_cam.Display()
    
    if (elapsed_time>settle_time and not load_measured):
        front_tire_load = (veh.GetTireNormalForce(0)+veh.GetTireNormalForce(1))*0.5
        rear_tire_load = (veh.GetTireNormalForce(2)+veh.GetTireNormalForce(3))*0.5
        front_tire_deflection = (veh.GetTireDeflection(0)+veh.GetTireDeflection(1))*0.5
        rear_tire_deflection = (veh.GetTireDeflection(2)+veh.GetTireDeflection(3))*0.5
        cg_height = position[2]
        load_measured = True
        print("Found tire loads at time ",elapsed_time)
        sys.stdout.flush()
    # Update the loop counter
    n = n+1
    elapsed_time = elapsed_time + dt


plt.plot(speed, tf, 'k-')
plt.title("Tractive Force vs Speed")
plt.ylabel('TF (N)')
plt.xlabel('Speed (m/s)')
plt.savefig('tf_vs_speed.png')
plt.show()
speed.reverse()
tf.reverse()
f = open("performance_results.json", "w")
f.write("{\n")
f.write("\"Front Tire Load (N)\": "+str(front_tire_load)+",\n")
f.write("\"Rear Tire Load (N)\": "+str(rear_tire_load)+",\n")
f.write("\"Front Tire Deflection (%)\": "+str(front_tire_deflection)+",\n")
f.write("\"Rear Tire Deflection (%)\": "+str(rear_tire_deflection)+",\n")
f.write("\"Vehicle Max Speed (m/s)\": "+str(max_velocity)+",\n")
f.write("\"Tractive Force vs Speed\": [\n")
for i in range(len(tf)):
    ts = "],\n"
    if (i==len(tf)-1):
        ts = "]\n"
    f.write("["+str(speed[i])+","+str(tf[i])+ts)
f.write("]\n")
f.write("}")
f.close()
