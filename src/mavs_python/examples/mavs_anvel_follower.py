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
# path to the anvel python api
sys.path.append('C:\Program Files\Quantum Signal, LLC\ANVEL Academic Edition\API\python\src')
#path to the mavs python api
sys.path.append(r'C:\Users\cgoodin\Desktop\code\mavs\src\mavs_python')
import math
from thrift.transport import TSocket
from thrift.protocol import TBinaryProtocol
from AnvelApi import AnvelControlService
from socket import IPPROTO_TCP, TCP_NODELAY
from AnvelApi.ttypes import *
import time
import mavs_interface
import mavs_python_paths

mavs_data_path = mavs_python_paths.mavs_data_path
mavs_actorfile = "/actors/forester_actor.json"
#mavs_scenefile = "/scenes/grassy_surface.json"
mavs_scenefile = "/scenes/grassy_brownfield.json"
#anvel_scenefile = 'LargeParkingLot.env'
anvel_scenefile = 'brownfield.env'
mavs_pathfile = "/waypoints/small_square.vprp"

scene = mavs_interface.MavsEmbreeScene()
scene.Load(mavs_data_path+mavs_scenefile)
env = mavs_interface.MavsEnvironment()
env.SetScene(scene.scene)
actornum = env.AddActor(mavs_data_path+mavs_actorfile)
env.AddDustToActor(actornum)
cam = mavs_interface.MavsCamera()
cam.Initialize(320,256,0.006784,0.0054272,0.004)
#cam.SetOffset([-7.5,0.0,2.5],[1.0,0.0,0.0,0.0])
cam.SetOffset([0.0,0.0,1.0],[1.0,0.0,0.0,0.0])
lidar = mavs_interface.MavsLidar('HDL-32E')
lidar.SetOffset([0.0,0.0,1.75],[1.0,0.0,0.0,0.0])
waypoints = mavs_interface.MavsWaypoints()
waypoints.Load(mavs_python_paths.mavs_data_path+mavs_pathfile)
anvel_path = []

for i in range(waypoints.GetNumWaypoints()):
    p = waypoints.GetWaypoint(i)
    point = Point3()
    point.x = p[0]
    point.y = p[1]
    point.z = p[2]
    anvel_path.append(point)
np = waypoints.GetNumWaypoints()
reverse_anvel_path = []
for i in range (np):
    j = i+1
    if (j>=np):
        j=0
    reverse_anvel_path.append(anvel_path[j])


def ConnectToANVEL(address='127.0.0.1', port=9094):
    '''Return a connected ANVEL client object.'''
    trans = TSocket.TSocket(address, port)
    proto = TBinaryProtocol.TBinaryProtocol(trans)
    anv = AnvelControlService.Client(proto)
    trans.open()
    trans.handle.setsockopt(IPPROTO_TCP, TCP_NODELAY, 1)
        # disable buffering for congestion control
    return anv

# connect to ANVEL
anv = ConnectToANVEL('127.0.0.1', 9094)  # default loopback configuration for the server

# pause the simulation timer in case it's already running
anv.SetSimulationState(SimulationState.PAUSED)

anv.LoadEnvironment(anvel_scenefile)

vehiclePose = anvel_path[0]
init_yaw = math.atan2(anvel_path[1].y-anvel_path[0].y,anvel_path[1].x-anvel_path[0].x)
vehicle = (anv.CreateObject('Generic 4x4', 'myVeh', 0, vehiclePose, Euler(0, 0, init_yaw), True))
anv.PlaceObjectOnGround(vehicle.objectKey)

reverse_vehiclePose = reverse_anvel_path[0]
reverse_init_yaw = math.atan2(reverse_anvel_path[1].y-reverse_anvel_path[0].y,reverse_anvel_path[1].x-reverse_anvel_path[0].x)
reverse_vehicle = (anv.CreateObject('Generic 4x4', 'myVeh2', 0, reverse_vehiclePose, Euler(0, 0, reverse_init_yaw), True))
anv.PlaceObjectOnGround(reverse_vehicle.objectKey)


main_loop_path = anv.CreatePath('Path',anvel_path)
reverse_main_loop_path = anv.CreatePath('Reverse Path',reverse_anvel_path)

# get the initial simulation time
tInit = anv.GetSimulationTime()
wallInit = time.perf_counter() #process_time()

# advance the simulation in steps
nlidarsteps = 0;
nlidarsaved = 0;
ncamerasteps = 0;
anv.SetSimulationState(SimulationState.RUNNING_EXTERNAL)
#anv.SetSimulationState(SimulationState.RUNNING_CPU)
desired_speed = 3.0
elapsed_time = 0.0
dist_to_goal = 10.0
goal = anvel_path[-1]

anv.SendStringCommandParamList('FollowPath',(str(vehicle.objectKey),str(main_loop_path.objectKey),str(desired_speed)))
anv.SendStringCommandParamList('FollowPath',(str(reverse_vehicle.objectKey),str(reverse_main_loop_path.objectKey),str(desired_speed)))

while(dist_to_goal>3.0 or elapsed_time<5.0):
	anv.StepSimulation()
	vehPoseRecord = (anv.GetPoseExtendedRel(vehicle.objectKey) )
	p = [vehPoseRecord.position.x,vehPoseRecord.position.y,vehPoseRecord.position.z]
	q = [vehPoseRecord.attitude.quaternion.w,vehPoseRecord.attitude.quaternion.x,
		 vehPoseRecord.attitude.quaternion.y,vehPoseRecord.attitude.quaternion.z]
	rev_vehPoseRecord = (anv.GetPoseExtendedRel(reverse_vehicle.objectKey) )
	rev_p = [rev_vehPoseRecord.position.x,rev_vehPoseRecord.position.y,rev_vehPoseRecord.position.z+0.3]
	rev_q = [rev_vehPoseRecord.attitude.quaternion.w,rev_vehPoseRecord.attitude.quaternion.x,
		 rev_vehPoseRecord.attitude.quaternion.y,rev_vehPoseRecord.attitude.quaternion.z]
	
	dist_to_goal = math.sqrt(math.pow(p[0]-goal.x,2)+math.pow(p[1]-goal.y,2))
	if (ncamerasteps==10):
            env.SetActorPosition(actornum,rev_p,rev_q)
            env.UpdateParticleSystems(0.1)
            cam.SetPose(p,q)
            cam.Update(env,0.03)
            cam.Display()
            ncamerasteps=0
	if (nlidarsteps==10):
            lidar.SetPose(p,q)
            lidar.Update(env,0.1)
            lidar.Display()
            #fname = 'lidar_'+str(nlidarsaved).zfill(4)+'.txt'
            #lidar.SaveColorizedPointCloud(fname)
            nlidarsaved = nlidarsaved+1
            nlidarsteps=0
	nlidarsteps = nlidarsteps+1
	ncamerasteps = ncamerasteps+1
	elapsed_time = anv.GetSimulationTime()-tInit

# get the final simulation time
tFinal = anv.GetSimulationTime()
wallFinal = time.perf_counter() #process_time()
print('final time = %f s, elapsed time = %f s, ratio to real time = %f s' % (tFinal, tFinal-tInit,(tFinal-tInit)/(wallFinal-wallInit)))
