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

sys.path.append(r'C:\Users\cgoodin\Desktop\code\mavs\src\mavs_python')
#sys.path.append(r'C:\Users\cgoodin\Desktop\vm_shared\shared_repos\mavs\src\mavs_python')

# Load the mavs python modules
import mavs_interface
import mavs_python_paths
import math
mavs_data_path = mavs_python_paths.mavs_data_path

test_type = 'circular'
if (len(sys.argv)>1):
	test_type = sys.argv[1]

desired_speed = 17.0
if (len(sys.argv)>2):
	desired_speed = float(sys.argv[2])
	
wb_min = 0.5
wb_max = 5.5
wb_step = 0.5
steering_scale_min = 0.5
steering_scale_max = 9.5
steering_scale_step = 1.0
deg_2_rad = math.pi/180.0
steering_angle_min = 15.0*deg_2_rad
steering_angle_max = 75.0*deg_2_rad
steering_angle_step = 10.0*deg_2_rad

path_radius = 20.0
max_error = 2.0

path = []
path_x = []
path_y = []
if (test_type == 'circular'):
	ntheta_steps = 20
	dtheta = 2*math.pi/ntheta_steps
	for i in range(ntheta_steps):
		theta = i*dtheta;
		x = path_radius*math.cos(theta)
		y = path_radius*math.sin(theta)
		path.append([x,y])
		path_x.append(x)
		path_y.append(y)
elif (test_type == 'sine'):
	nx_steps = 100
	dx = 1.0
	for i in range(nx_steps):
		x = i*dx
		y = path_radius*math.sin(x/path_radius)
		path.append([x,y])
		path_x.append(x)
		path_y.append(y)
elif (test_type == 'straight'):
	nx_steps = 100
	dx = 1.0
	for i in range(nx_steps):
		x = i*dx
		y = 0.0
		path.append([x,y])
		path_x.append(x)
		path_y.append(y)
elif (test_type == 'lane'):
	path.append([-50.0, 0.0])
	path.append([-47.5, 0.0])
	path.append([-45.0, 0.0])
	path.append([-42.5, 0.0])
	path.append([-40.0, 0.0])
	path.append([-37.5, 0.0])
	path.append([-35.0, 0.0])
	path.append([-32.5, 0.0])
	path.append([-30.0, 0.0])
	path.append([-27.5, 0.0])
	path.append([-25.0, 0.0])
	path.append([-22.5, 0.0])
	path.append([-20.0, 0.0])
	path.append([-17.5, 0.0])
	path.append([-15.0, 0.0])
	path.append([-12.5, 0.0])
	path.append([-10.0, 0.0])
	path.append([-7.5, 0.0])
	path.append([-5.0, 0.0])
	path.append([-2.5, 0.0])
	path.append([0.0, 0.0])
	path.append([2.5, 0.0])
	path.append([5.0, 0.0])
	path.append([7.5, 0.0])
	path.append([10.0, 0.0])
	path.append([12.0, 0.0])
	path.append([14.5, 0.463])
	path.append([17.0, 0.926])
	path.append([19.5, 1.389])
	path.append([22.0, 1.852])
	path.append([24.5, 2.31])
	path.append([25.5, 2.5])
	path.append([27.0, 2.5])
	path.append([29.5, 2.5])
	path.append([32.0, 2.5])
	path.append([34.5, 2.5])
	path.append([36.5, 2.5])
	path.append([39.0, 2.0])
	path.append([41.5, 1.5])
	path.append([44.0, 1.0])
	path.append([46.5, 0.5])
	path.append([49.0, 0.0])
	path.append([51.0, 0.0])
	path.append([53.5, 0.0])
	path.append([56.0, 0.0])
	path.append([58.5, 0.0])
	path.append([61.0 , 0.0])
	for p in path:
		path_x.append(p[0])
		path_y.append(p[1])

# Load a scene
mavs_scenefile = "/scenes/surface_only.json"
scene = mavs_interface.MavsEmbreeScene()
scene.Load(mavs_data_path+mavs_scenefile)

# Create the environment and set properties
env = mavs_interface.MavsEnvironment()
env.SetScene(scene.scene)
env.SetCloudCover(0.4)
env.SetTurbidity(5.0)
env.SetFog(0.05)
output_file = open("path_following_results.txt","w")
output_file.write('steering_scale wheelbase max_sa path_error elapsed_time \n')

max_time = 2*math.pi*(path_radius+max_error)/desired_speed

steering_scale = steering_scale_min
while steering_scale <= steering_scale_max:
	wb = wb_min
	while wb<=wb_max:
		sa = steering_angle_min
		while sa<=steering_angle_max:
			#Create a vehicle controller
			controller = mavs_interface.MavsVehicleController()
			controller.SetDesiredSpeed(desired_speed)
			controller.SetDesiredPath(path)
			controller.SetSteeringScale(steering_scale)
			controller.SetWheelbase(wb)
			controller.SetMaxSteerAngle(sa)
			
			current_driving_command = mavs_interface.MavsDrivingCommand()
			veh = mavs_interface.ChronoVehicle()
			#Put the vehicle in the starting position
			veh.SetInitialPosition(path[0][0], path[0][1], 1.0)
			init_heading = math.atan2(path[1][1]-path[0][1],path[1][0]-path[0][0])
			veh.SetInitialHeading(init_heading)
			#veh.Load((mavs_data_path+'/vehicles/chrono_inputs/forester_windows.json'))
			#veh.Load((mavs_data_path+'/vehicles/chrono_inputs/forester_windows_alt.json'))
			veh.Load((mavs_data_path+'/vehicles/chrono_inputs/hmmwv_windows_alt.json'))

			dt = 0.01
			camera_dt = 0.1
			camera_elapsed = 0.0
			elapsed_time = 0.0
			x = []
			y = []
			mplotter = mavs_interface.MavsPlot()
			dr_max = 0.0
			#----- Simulation -------------------------------------#
			while(True):
				# advance the environment, which moves animations
				env.AdvanceTime(dt)
				
				# update the vehicle
				if (elapsed_time>1.0):
					current_pos = veh.GetPosition()
					controller.SetCurrentState(current_pos[0],current_pos[1],veh.GetSpeed(),veh.GetHeading())
					current_driving_command = controller.GetDrivingCommand(dt)
					veh.Update(env,current_driving_command.throttle, current_driving_command.steering,current_driving_command.braking, dt)
				else:
					veh.Update(env, 0.0, 0.0, 1.0, dt)

				# get the current sensor orientation
				pos = veh.GetPosition()
				if (test_type=='circular'):
					r = math.sqrt(pos[0]*pos[0] + pos[1]*pos[1])
					dr = abs(r-path_radius)
				elif (test_type=='sine'):
					dr = abs(path_radius*math.sin(pos[0]/path_radius)-pos[1])
				elif (test_type=='straight'):
					dr = abs(pos[1])
				elif(test_type=='lane'):
					if (pos[0]<12.0 or pos[0]>49.0):
						dr = abs(pos[1])
					elif (pos[0]>25.5 and pos[0]<36.5):
						dr = abs(2.5-pos[1])
					else:
						dr = 0.0
							
				if (dr>dr_max):
					dr_max = dr
						
				
				x.append(pos[0])
				y.append(pos[1])
				mplotter.PlotTrajectory(x,y)
				mplotter.AddToTrajectory(path_x,path_y)

				elapsed_time = elapsed_time + dt
				if (dr>max_error):
					break
				dend = math.sqrt((pos[0]-path[-1][0])*(pos[0]-path[-1][0])+(pos[1]-path[-1][1])*(pos[1]-path[-1][1]))
				if (dend<max_error):
					break
			
			output_file.write((str(steering_scale)+' '+str(wb)+' '+str(sa)+' '+str(dr)+' '+str(elapsed_time)+'\n'))
			output_file.flush()
			sa = sa + steering_angle_step
		wb = wb + wb_step
	steering_scale = steering_scale + steering_scale_step

output_file.close()

